#include "motion_ctrl.h"
#include "../live_config.h"
#include "../display/oled.h"
#include "../lidar/obstacle.h"
#include "../log.h"
#include "recorder.h"
#include <math.h>

MotionController gMotionCtrl;

static constexpr float PI_F    = 3.14159265f;
static constexpr float DEG2RAD = PI_F / 180.0f;
static constexpr float RAD2DEG = 180.0f / PI_F;
// Âge moyen de la pose encodeurs lue par la boucle (tâche encodeurs à 200 Hz,
// plus le retard des pas à pas sur la commande) : l'erreur compare la mesure
// à la référence de cet instant-là, pas à celle de maintenant.
static constexpr float MEAS_LAG_S = 0.005f;
// Vitesse envoyée aux moteurs : celle du profil quand ils l'exécuteront
static constexpr float ACT_LAG_S  = MOTION_ACT_LAG_MS / 1000.0f;

static inline float clampf(float v, float lim) { return fmaxf(-lim, fminf(v, lim)); }

// ─── begin ───────────────────────────────────────────────────────────────────

void MotionController::begin(StepControl* motion, QuadEncoder* encL, QuadEncoder* encR) {
    _motion = motion;
    _encL   = encL;
    _encR   = encR;
    xTaskCreatePinnedToCore(_taskTrampoline, "motion", 4096, this, 3, nullptr, 1);
}

void MotionController::_taskTrampoline(void* arg) {
    static_cast<MotionController*>(arg)->_loop();
}

// ─── API non-bloquante ───────────────────────────────────────────────────────

void MotionController::setTarget(const Target& t) {
    portENTER_CRITICAL(&_mux);
    _userTgt = t;
    if (_userTgt.speed   <= 0.0f) _userTgt.speed   = gCalib.defaultSpeed;
    if (_userTgt.accel   <= 0.0f) _userTgt.accel   = gCalib.defaultAccel;
    if (_userTgt.stopMm  <= 0.0f) _userTgt.stopMm  = gCalib.stopMm;
    _userSeq++;
    portEXIT_CRITICAL(&_mux);
    // Empêche waitArrived(blendMm) de sortir avant que la tâche motion ait
    // pris en compte la nouvelle cible (_distMm garde la valeur précédente).
    _distMm = 1e9f;
}

void MotionController::holdPosition() {
    Target t;
    t.x_mm       = gDisplay.enc_pose_x_mm;
    t.y_mm       = gDisplay.enc_pose_y_mm;
    t.theta_deg  = gDisplay.enc_pose_theta_deg;
    t.obstacleEn = false;
    setTarget(t);
}

void MotionController::release() {
    portENTER_CRITICAL(&_mux);
    _state = State::IDLE;
    _phase = Phase::NONE;
    _doneOk = false;
    _doneSeq = _userSeq;   // déverrouille tout waitArrived en cours
    portEXIT_CRITICAL(&_mux);
    obstacleUnwatch();
    if (_motion) _motion->softStop();
    _lastCmdL = 0; _lastCmdR = 0; _refSpeed = 0;
}

bool MotionController::waitArrived(float blendMm) {
    portENTER_CRITICAL(&_mux);
    uint32_t expected = _userSeq;
    portEXIT_CRITICAL(&_mux);

    while (true) {
        portENTER_CRITICAL(&_mux);
        uint32_t curUser = _userSeq;
        uint32_t curDone = _doneSeq;
        portEXIT_CRITICAL(&_mux);

        if (curUser != expected) return false;       // préempté par nouveau setTarget
        if (curDone == expected) return true;        // terminé / release
        if (blendMm > 0.0f && _distMm < blendMm) return true;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ─── Boucle principale (tâche) ───────────────────────────────────────────────

void MotionController::_loop() {
    const TickType_t period = pdMS_TO_TICKS(MOTION_PERIOD_MS);
    TickType_t wake = xTaskGetTickCount();
    uint32_t prevUs = micros();

    while (true) {
        // Cadence fixe. Après un appel bloquant (arrêt, recul obstacle), on
        // repart de maintenant au lieu d'enchaîner les cycles en retard.
        if (xTaskGetTickCount() - wake >= period) wake = xTaskGetTickCount();
        vTaskDelayUntil(&wake, period);

        const uint32_t us = micros();
        float dt = (float)(us - prevUs) * 1e-6f;
        prevUs = us;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.05f)  dt = 0.05f;

        // Nouvelle cible ?
        portENTER_CRITICAL(&_mux);
        uint32_t seq = _userSeq;
        Target   tgtCopy = _userTgt;
        portEXIT_CRITICAL(&_mux);

        if (seq != _handledSeq) {
            _handledSeq = seq;
            const bool wasTranslating = _state == State::MOVING && _phase == Phase::TRANSLATE;
            const float tNow = (float)(us - _t0Us) * 1e-6f;
            _activeTgt = tgtCopy;
            _phaseOk   = true;
            if (!(wasTranslating && _tryBlend(tNow))) {
                if (_state == State::MOVING) {
                    // Changement de cible incompatible : arrêt, puis départ normal
                    obstacleUnwatch();
                    _motion->softStop(fmaxf(_activeTgt.accel, gCalib.defaultAccel));
                }
                _state = State::MOVING;
                if (!gRecorder.recording()) _recordStart();
                _startAlign();
            }
        }

        // release() (autre tâche) a remis IDLE : fin de l'enregistrement
        if (_state == State::IDLE && gRecorder.recording()) gRecorder.stop();

        switch (_state) {
            case State::IDLE:
            case State::HOLD:
                _distMm = 0;
                continue;

            case State::MOVING: {
                const float t = (float)(micros() - _t0Us) * 1e-6f;
                bool phaseDone = (_phase == Phase::TRANSLATE) ? _stepTranslate(t, dt)
                                                              : _stepRotation(t, dt);

                if (_phaseObstacleEn && obstacleDetected()) {
                    _motion->softStop(OBS_STOP_ACCEL_MMS2);
                    _lastCmdL = _lastCmdR = _refSpeed = 0;
                    LOG_W("MOTION", "Obstacle %.0fmm — recul + attente",
                          (double)gDisplay.obs_dist_mm);
                    gDisplay.robot_state = RobotState::OBSTACLE;
                    _state = State::OBSTACLE;
                    _obstacleBackedUp = false;
                    break;
                }

                if (phaseDone && !_phaseOk) {
                    _complete(false);            // phase abandonnée : cible abandonnée
                } else if (phaseDone) {
                    switch (_phase) {
                        case Phase::ALIGN:      _startTranslate(0.0f); break;
                        case Phase::TRANSLATE:  _startFinalTurn();     break;
                        default:                _complete(_phaseOk);   break;
                    }
                }
                break;
            }

            case State::OBSTACLE: {
                // Recul une fois, puis attente du dégagement (ou timeout)
                if (!_obstacleBackedUp) {
                    _motion->go(_activeGoBack ? OBS_BACKUP_MM : -OBS_BACKUP_MM);
                    _obstacleBackedUp = true;
                    _obstacleStartMs  = millis();
                    break;
                }
                const bool clear   = !obstacleDetected();
                const bool timeout = (millis() - _obstacleStartMs) > OBS_WAIT_MS;
                if (clear || timeout) {
                    if (timeout) LOG_W("MOTION", "Timeout obstacle — reprise");
                    else         LOG_I("MOTION", "Obstacle dégagé — reprise");
                    // Reprise depuis la nouvelle position : ALIGN puis TRANSLATE
                    _state = State::MOVING;
                    gDisplay.robot_state = RobotState::GOTO;
                    _startAlign();
                }
                break;
            }
        }

        if (gRecorder.recording() && (++_recDiv & 1) == 0) _recordSample();
    }
}

// ─── Enregistrement ──────────────────────────────────────────────────────────

void MotionController::_recordStart() {
    gRecorder.start();
    _recPrevUs = micros();
    _recPrevL  = _encL->getCount();
    _recPrevR  = _encR->getCount();
    _recDiv = 0;
    _dbgErrD = 0; _dbgErrA = 0;
}

void MotionController::_recordSample() {
    const uint32_t us = micros();
    const int32_t  cL = _encL->getCount();
    const int32_t  cR = _encR->getCount();
    const float    dt = (float)(us - _recPrevUs) * 1e-6f;
    float vL = 0, vR = 0;
    if (dt > 0.001f) {
        vL = (float)(cL - _recPrevL) * gCalib.mmPerCountL / dt;
        vR = (float)(cR - _recPrevR) * gCalib.mmPerCountR / dt;
    }
    _recPrevUs = us; _recPrevL = cL; _recPrevR = cR;

    uint32_t t = millis() - gRecorder.startMs();
    RecSample s;
    s.t     = (uint16_t)(t > 65535 ? 65535 : t);
    s.state = (uint8_t)_state;
    s.phase = (uint8_t)_phase;
    s.x     = recClamp16(gDisplay.enc_pose_x_mm);
    s.y     = recClamp16(gDisplay.enc_pose_y_mm);
    s.th    = recClamp16(gDisplay.enc_pose_theta_deg * 10.0f);
    s.cmdL  = recClamp16(_lastCmdL);
    s.cmdR  = recClamp16(_lastCmdR);
    s.vL    = recClamp16(vL);
    s.vR    = recClamp16(vR);
    s.errD  = recClamp16(_dbgErrD * 10.0f);
    s.errA  = recClamp16(_dbgErrA * 100.0f);
    s.cap   = recClamp16(_refSpeed);
    gRecorder.add(s);
}

// ─── Démarrage des phases ────────────────────────────────────────────────────

void MotionController::_startAlign() {
    const float dx = _activeTgt.x_mm - gDisplay.enc_pose_x_mm;
    const float dy = _activeTgt.y_mm - gDisplay.enc_pose_y_mm;
    const float dist = sqrtf(dx*dx + dy*dy);

    // Déjà au point : seulement le cap final éventuel
    if (dist < _activeTgt.stopMm) {
        _activeGoBack = _activeTgt.backward;
        _startFinalTurn();
        return;
    }

    const float aerr = _normAngle(atan2f(-dy, dx) - gDisplay.enc_pose_theta_rad);
    _activeGoBack = _activeTgt.backward || fabsf(aerr) > PI_F * 0.5f;
    const float turnErr = _activeGoBack ? _normAngle(aerr + PI_F) : aerr;

    // Presque aligné : la boucle de cap de la translation rattrape l'écart
    if (fabsf(turnErr) < 2.0f * DEG2RAD) { _startTranslate(0.0f); return; }

    _startRotation(turnErr, Phase::ALIGN);
}

void MotionController::_startFinalTurn() {
    if (isnanf(_activeTgt.theta_deg)) { _complete(_phaseOk); return; }
    const float delta = _normAngle(_activeTgt.theta_deg * DEG2RAD - gDisplay.enc_pose_theta_rad);
    if (fabsf(delta) < gCalib.stopDeg * DEG2RAD) { _complete(_phaseOk); return; }
    _startRotation(delta, Phase::FINAL_TURN);
}

void MotionController::_startRotation(float deltaRad, Phase ph) {
    const float halfWb = gCalib.wheelbase * 0.5f;
    const float spd = fminf(_activeTgt.speed, TURN_SPEED_MMS);   // vitesse des roues
    const float acc = fminf(_activeTgt.accel, TURN_ACCEL_MMS2);
    _prof.plan(fabsf(deltaRad), spd / halfWb, acc / halfWb);
    _sign   = deltaRad >= 0 ? 1.0f : -1.0f;
    _x0     = gDisplay.enc_pose_x_mm;
    _y0     = gDisplay.enc_pose_y_mm;
    _th0    = gDisplay.enc_pose_theta_rad;
    _prevTh = _th0;
    _rotMes = 0;
    _iD = _iA = 0;
    _settleCnt = 0;
    _vMaxWheel = spd * 1.2f + 50.0f;
    _phaseObstacleEn = false;            // pas de détection en rotation sur place
    obstacleUnwatch();
    _motion->setStepperAccel(fmaxf(acc * MOTION_STEPPER_ACCEL_K, MOTION_STEPPER_ACCEL_MIN));
    _phase = ph;
    _t0Us  = micros();
    gDisplay.robot_state = RobotState::TURNING;
}

void MotionController::_startTranslate(float v0) {
    const float x = gDisplay.enc_pose_x_mm, y = gDisplay.enc_pose_y_mm;
    const float dx = _activeTgt.x_mm - x;
    const float dy = _activeTgt.y_mm - y;
    const float dist = sqrtf(dx*dx + dy*dy);
    if (dist < _activeTgt.stopMm) { _startFinalTurn(); return; }

    _prof.plan(dist, _activeTgt.speed, _activeTgt.accel, v0);
    _x0 = x; _y0 = y;
    _ux = dx / dist; _uy = dy / dist;
    _thRef = _normAngle(atan2f(-dy, dx) + (_activeGoBack ? PI_F : 0.0f));
    _thFrozen = false;
    _iD = _iA = 0;
    _settleCnt = 0;
    _vMaxWheel = fmaxf(_activeTgt.speed, v0) * 1.2f + 50.0f;

    _phaseMoveDir    = _normAngle(atan2f(-dy, dx));   // direction du trajet
    _phaseObstacleEn = _activeTgt.obstacleEn;
    if (_phaseObstacleEn) obstacleWatch(_phaseMoveDir);
    else                  obstacleUnwatch();

    _motion->setStepperAccel(fmaxf(_activeTgt.accel * MOTION_STEPPER_ACCEL_K, MOTION_STEPPER_ACCEL_MIN));
    _phase = Phase::TRANSLATE;
    _t0Us  = micros();
    gDisplay.robot_state = RobotState::MOVING;
}

void MotionController::_complete(bool ok) {
    obstacleUnwatch();
    _motion->softStop();
    _lastCmdL = _lastCmdR = _refSpeed = 0;
    // Pose à l'estime des pas recalée sur la pose encodeurs
    vTaskDelay(pdMS_TO_TICKS(10));
    _motion->setPosition(gDisplay.enc_pose_x_mm,
                         gDisplay.enc_pose_y_mm,
                         gDisplay.enc_pose_theta_deg);

    _phase = Phase::NONE;
    _state = State::IDLE;
    _distMm = 0;
    gDisplay.nav_dist_mm = 0;
    gDisplay.robot_state = RobotState::IDLE;

    if (gRecorder.recording()) { _recordSample(); gRecorder.stop(); }

    portENTER_CRITICAL(&_mux);
    _doneOk  = ok;
    _doneSeq = _handledSeq;
    portEXIT_CRITICAL(&_mux);
}

// Nouvelle cible pendant une translation : enchaînement sans arrêt si elle
// est presque dans l'axe et dans le même sens de marche. Le nouveau profil
// part de la vitesse de référence actuelle ; la boucle de cap infléchit la
// trajectoire. Sinon, arrêt puis départ normal.
bool MotionController::_tryBlend(float tNow) {
    const float dx = _activeTgt.x_mm - gDisplay.enc_pose_x_mm;
    const float dy = _activeTgt.y_mm - gDisplay.enc_pose_y_mm;
    const float dist = sqrtf(dx*dx + dy*dy);
    if (dist < _activeTgt.stopMm) return false;

    const float aerr   = _normAngle(atan2f(-dy, dx) - gDisplay.enc_pose_theta_rad);
    const bool  goBack = _activeTgt.backward || fabsf(aerr) > PI_F * 0.5f;
    const float turnErr = goBack ? _normAngle(aerr + PI_F) : aerr;
    if (goBack != _activeGoBack || fabsf(turnErr) > MOTION_BLEND_MAX_DEG * DEG2RAD) return false;

    float s, v;
    _prof.at(tNow, s, v);
    LOG_I("MOTION", "Enchaînement %.0fmm, écart de cap %.0f°, %.0f mm/s",
          (double)dist, (double)(turnErr * RAD2DEG), (double)v);
    _startTranslate(v);
    return true;
}

// ─── Cycles ──────────────────────────────────────────────────────────────────

bool MotionController::_settle(float t, float err, float tol) {
    if (!_prof.done(t)) { _settleCnt = 0; return false; }
    _settleCnt = fabsf(err) < tol ? _settleCnt + 1 : 0;
    if (_settleCnt >= MOTION_SETTLE_CYCLES) return true;
    if (t > _prof.T + MOTION_SETTLE_MS * 1e-3f) {
        LOG_W("MOTION", "Non convergé : résidu %.2f (tolérance %.2f) — abandon",
              (double)err, (double)tol);
        _phaseOk = false;
        return true;
    }
    return false;
}

bool MotionController::_stepRotation(float t, float dt) {
    const float halfWb = gCalib.wheelbase * 0.5f;
    const float th = gDisplay.enc_pose_theta_rad;
    _rotMes += _normAngle(th - _prevTh);
    _prevTh  = th;

    // Référence : angle à l'instant de la mesure, vitesse à l'instant où les
    // moteurs l'exécuteront
    float sRef, vRef, sNext, wRef;
    _prof.at(t - MEAS_LAG_S, sRef, vRef);
    _prof.at(t + ACT_LAG_S, sNext, wRef);
    sRef *= _sign; wRef *= _sign;

    const float rem = _sign * _prof.D - _rotMes;     // angle restant (rad)
    const float tol = gCalib.stopDeg * DEG2RAD;
    const float e   = sRef - _rotMes;

    // Centre maintenu sur place : écart au point de départ, projeté sur le
    // cap ACTUEL (seul axe sur lequel la vitesse linéaire peut agir)
    const float drift = (gDisplay.enc_pose_x_mm - _x0) * cosf(th)
                      - (gDisplay.enc_pose_y_mm - _y0) * sinf(th);

    const float dxT = _activeTgt.x_mm - gDisplay.enc_pose_x_mm;
    const float dyT = _activeTgt.y_mm - gDisplay.enc_pose_y_mm;
    _distMm = sqrtf(dxT*dxT + dyT*dyT);
    gDisplay.nav_dist_mm   = fabsf(rem) * halfWb;
    gDisplay.nav_delta_deg = rem * RAD2DEG;
    _dbgErrD = -drift;
    _dbgErrA = rem * RAD2DEG;

    if (_settle(t, rem, tol)) {
        _applyWheels(0, 0);
        return true;
    }
    if (fabsf(e) > MOTION_MAX_ERR_DEG * DEG2RAD || fabsf(drift) > MOTION_MAX_ERR_MM * 0.6f) {
        LOG_E("MOTION", "Rotation : écart anormal (angle %.1f°, dérive %.0f mm) — arrêt",
              (double)(e * RAD2DEG), (double)drift);
        _applyWheels(0, 0);
        _phaseOk = false;
        return true;
    }

    _iA = clampf(_iA + gCalib.ki * e * dt, gCalib.iMax / halfWb);
    float w = wRef + gCalib.kpAng * e + _iA;
    if (_prof.done(t)) {
        // Résidu après le profil : correction, au moins à la vitesse mini
        if (fabsf(rem) < tol) w = 0;
        else {
            w = gCalib.kpAng * rem + _iA;
            const float wMin = gCalib.minSpd / halfWb;
            if (fabsf(w) < wMin) w = copysignf(wMin, rem);
        }
    }
    const float v = -gCalib.kpDist * drift;
    _refSpeed = wRef * halfWb;
    _applyWheels(v, w, 0.0f, wRef);
    return false;
}

bool MotionController::_stepTranslate(float t, float dt) {
    const float x = gDisplay.enc_pose_x_mm, y = gDisplay.enc_pose_y_mm;
    const float th = gDisplay.enc_pose_theta_rad;

    // Distance parcourue : projection sur la droite départ → cible
    const float sMes = (x - _x0) * _ux + (y - _y0) * _uy;
    const float rem  = _prof.D - sMes;

    float sRef, vRefNow, sNext, vRef;
    _prof.at(t - MEAS_LAG_S, sRef, vRefNow);
    _prof.at(t + ACT_LAG_S, sNext, vRef);
    const float e = sRef - sMes;

    // Cap visé : vers la cible tant qu'elle est loin, puis figé
    const float dxT = _activeTgt.x_mm - x;
    const float dyT = _activeTgt.y_mm - y;
    const float dGeo = sqrtf(dxT*dxT + dyT*dyT);
    if (!_thFrozen) {
        if (rem > MOTION_HEADING_FREEZE_MM)
            _thRef = _normAngle(atan2f(-dyT, dxT) + (_activeGoBack ? PI_F : 0.0f));
        else
            _thFrozen = true;
    }
    const float eTh = _normAngle(_thRef - th);

    _distMm = dGeo;
    gDisplay.nav_dist_mm   = rem;
    gDisplay.nav_delta_deg = eTh * RAD2DEG;
    _dbgErrD = rem;
    _dbgErrA = eTh * RAD2DEG;

    if (_settle(t, rem, _activeTgt.stopMm)) {
        _applyWheels(0, 0);
        return true;
    }
    if (fabsf(e) > MOTION_MAX_ERR_MM || fabsf(eTh) > MOTION_MAX_HEADING_DEG * DEG2RAD) {
        LOG_E("MOTION", "Translation : écart anormal (distance %.0f mm, cap %.1f°) — arrêt",
              (double)e, (double)(eTh * RAD2DEG));
        _applyWheels(0, 0);
        _phaseOk = false;
        return true;
    }

    const float halfWb = gCalib.wheelbase * 0.5f;
    _iD = clampf(_iD + gCalib.ki * e * dt, gCalib.iMax);
    _iA = clampf(_iA + gCalib.ki * eTh * dt, gCalib.iMax / halfWb);

    float v = vRef + gCalib.kpDist * e + _iD;
    if (_prof.done(t)) {
        if (fabsf(rem) < _activeTgt.stopMm) v = 0;
        else {
            v = gCalib.kpDist * rem + _iD;
            if (fabsf(v) < gCalib.minSpd) v = copysignf(gCalib.minSpd, rem);
        }
    }
    const float w = gCalib.kpAng * eTh + _iA;

    _refSpeed = _activeGoBack ? -vRef : vRef;
    _applyWheels(_activeGoBack ? -v : v, w, _activeGoBack ? -vRef : vRef, 0.0f);
    return false;
}

// v : vitesse linéaire signée (mm/s, + = avant) ; w : vitesse angulaire
// (rad/s, + = gauche). Répartition sur les roues MOTRICES.
// Correction bornée autour du profil : un pas à pas qui décroche (pas perdus
// à forte accélération) ne suit pas une demande plus forte, et la lui faire
// quand même aggrave l'écart (constaté sur la roue droite en marche arrière).
void MotionController::_applyWheels(float v, float w, float vff, float wff) {
    const float halfWb = gCalib.wheelbase * 0.5f;
    const float ffL = vff - wff * halfWb, ffR = vff + wff * halfWb;
    float vL = ffL + clampf(v - w * halfWb - ffL, MOTION_CORR_MAX_MMS);
    float vR = ffR + clampf(v + w * halfWb - ffR, MOTION_CORR_MAX_MMS);
    vL = clampf(vL, _vMaxWheel);
    vR = clampf(vR, _vMaxWheel);
    _motion->setMotorVelocities(vL, vR);
    _lastCmdL = vL;
    _lastCmdR = vR;
}

float MotionController::_normAngle(float rad) {
    while (rad >  PI_F) rad -= 2.0f * PI_F;
    while (rad < -PI_F) rad += 2.0f * PI_F;
    return rad;
}
