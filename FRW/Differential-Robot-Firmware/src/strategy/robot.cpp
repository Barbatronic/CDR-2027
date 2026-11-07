#include "robot.h"
#include "../display/oled.h"
#include "../live_config.h"
#include "../log.h"
#include <math.h>

static constexpr float PI_F = 3.14159265f;

Robot::Robot(StepControl &motion, MotionController &motionCtrl)
    : _motion(motion), _motionCtrl(motionCtrl) {}

// ─── Chrono de match ──────────────────────────────────────────────────────────

void Robot::startMatch() {
    _nearEndMode = false;
    gDisplay.match_start_ms = millis();
}

void Robot::resetMatch() {
    gDisplay.match_start_ms = 0;
    _nearEndMode = false;
}

uint32_t Robot::matchElapsed() const {
    if (gDisplay.match_start_ms == 0) return 0;
    return millis() - gDisplay.match_start_ms;
}

bool Robot::isEndgame()   const { return matchElapsed() >= MATCH_ENDGAME_MS;  }
bool Robot::isMatchOver() const { return matchElapsed() >= MATCH_DURATION_MS; }

void Robot::waitMatchTime(uint32_t target_ms) {
    while (!isMatchOver() && matchElapsed() < target_ms)
        vTaskDelay(pdMS_TO_TICKS(10));
}

// ─── Déplacements ─────────────────────────────────────────────────────────────

bool Robot::_moveAllowed() const {
    return !isMatchOver() && (_nearEndMode || !isEndgame());
}

// ─── Go avec détection de blocage (plaquage bordure) ─────────────────────────
// Démarre le mouvement et surveille les encodeurs. Si les deux roues cessent de
// varier pendant STALL_POLLS polls consécutifs → robot bloqué → stop.
// Retourne true si stall détecté, false si distance atteinte ou timeout.

bool Robot::goStall(float mm, uint32_t timeoutMs, uint32_t stallConfirmMs) {
    if (fabsf(mm) < 0.5f) return false;

    // Direction de déplacement, surveillée par la tâche LIDAR
    const float moveDir = getTheta() + ((mm < 0) ? PI_F : 0.0f);
    if (_obstacleEn) obstacleWatch(moveDir);
    const float sign    = (mm >= 0) ? 1.0f : -1.0f;

    _motion.startGo(mm);

    int32_t  prevL        = _encLeft  ? _encLeft->getCount()  : 0;
    int32_t  prevR        = _encRight ? _encRight->getCount() : 0;
    float    traveled     = 0.0f;
    uint32_t timeoutStart = 0;

    // Blocage = aucune des deux roues codeuses n'a PROGRESSÉ dans le sens du
    // mouvement sur la fenêtre stallConfirmMs (progression nette, signée).
    // Les vibrations des moteurs qui forcent contre la bordure (pas sautés)
    // font osciller les roues codeuses : en valeur absolue cycle par cycle,
    // elles masquaient le blocage ; en progression nette, elles s'annulent.
    // Les deux roues doivent être arrêtées : robot plaqué à plat.
    constexpr int WIN_MAX = 32;
    int nWin = (int)(stallConfirmMs / OBS_POLL_MS);
    if (nWin < 2) nWin = 2;
    if (nWin > WIN_MAX) nWin = WIN_MAX;
    // Seuil : STALL_THRESH_MM par cycle (25 mm/s), ramené à la moitié de la
    // vitesse demandée pour un recul très lent
    const float spd   = _motion.getSpeed(), acc = _motion.getAcceleration();
    const float winS  = nWin * OBS_POLL_MS * 1e-3f;
    const float progMin = fminf(STALL_THRESH_MM * nWin, 0.5f * spd * winS);
    int32_t histL[WIN_MAX], histR[WIN_MAX];
    int     hIdx = 0, hFill = 0;
    float   progL = 0, progR = 0;                   // dernière progression mesurée (mm)

    // Armement : pas de blocage tant que la vitesse COMMANDÉE n'a pas dépassé
    // le double du seuil pendant toute une fenêtre. Pendant l'accélération, le
    // robot avance moins que le seuil sans être bloqué (à 80 mm/s², il roule
    // encore sous 30 mm/s après 5 mm). Collé dès le départ : contact détecté
    // une fenêtre après l'armement.
    const float    vArm  = 2.0f * progMin / winS;   // ≤ spd par construction
    const uint32_t armMs = (uint32_t)(1000.0f * (acc > 1.0f ? vArm / acc : 0.0f)) + stallConfirmMs;
    const uint32_t t0    = millis();

    while (_motion.isMoving()) {
        vTaskDelay(pdMS_TO_TICKS(OBS_POLL_MS));
        _motion.syncPose();

        int32_t curL = _encLeft  ? _encLeft->getCount()  : prevL;
        int32_t curR = _encRight ? _encRight->getCount() : prevR;
        traveled += (float)(abs(curL - prevL) + abs(curR - prevR)) * 0.5f * gCalib.mmPerCount;
        prevL = curL;
        prevR = curR;

        uint32_t now = millis();
        if (timeoutStart == 0 && traveled >= fabsf(mm))
            timeoutStart = now;

        // Détection obstacle dans le sens du mouvement (adversaire, pas le mur cible)
        // Le mur cible est à <LIDAR_BODY_DIST_MM quand plaqué → jamais déclenché
        if (_obstacleEn && obstacleDetected()) {
            _motion.softStop(OBS_STOP_ACCEL_MMS2);
            gDisplay.robot_state = RobotState::OBSTACLE;
            _waitObstacleClear();
            float remaining = fabsf(mm) - traveled;
            if (remaining > 0.5f) _motion.startGo(sign * remaining);
            prevL = _encLeft  ? _encLeft->getCount()  : prevL;
            prevR = _encRight ? _encRight->getCount() : prevR;
            hFill = 0;                               // fenêtre repartie de zéro
            gDisplay.robot_state = RobotState::MOVING;
            continue;
        }

        // Progression nette sur la fenêtre
        const bool armed = now - t0 >= armMs;
        if (armed && hFill >= nWin) {
            const int old = (hIdx - nWin + WIN_MAX) % WIN_MAX;
            progL = sign * (float)(curL - histL[old]) * gCalib.mmPerCountL;
            progR = sign * (float)(curR - histR[old]) * gCalib.mmPerCountR;
            if (progL < progMin && progR < progMin) {
                _motion.stop();
                _motion.syncPose();
                LOG_I("STALL", "Contact : progression G %.1f D %.1f mm / %d ms",
                      (double)progL, (double)progR, nWin * OBS_POLL_MS);
                obstacleUnwatch();
                return true;
            }
        }
        histL[hIdx] = curL;
        histR[hIdx] = curR;
        hIdx = (hIdx + 1) % WIN_MAX;
        if (hFill < nWin) hFill++;

        // Timeout : seulement après avoir atteint la position normale
        if (timeoutStart != 0 && now - timeoutStart >= timeoutMs) {
            _motion.stop();
            _motion.syncPose();
            LOG_W("STALL", "Pas de contact (timeout) : progression G %.1f D %.1f mm, seuil %.1f",
                  (double)progL, (double)progR, (double)progMin);
            obstacleUnwatch();
            return false;
        }

        if (isMatchOver()) {
            _motion.stop(); disableMotors(); obstacleUnwatch();
            gDisplay.robot_state = RobotState::DONE;
            return false;
        }
    }

    obstacleUnwatch();
    _motion.syncPose();
    LOG_W("STALL", "Pas de contact (course finie) : progression G %.1f D %.1f mm, seuil %.1f",
          (double)progL, (double)progR, (double)progMin);
    return false;
}

// ─── API motion non-bloquante (wrappers vers MotionController) ───────────────

bool Robot::waitArrived(float blendMm) {
    // Capturer le seq utilisateur AVANT d'attendre : c'est la cible qu'on suit.
    // Sans ça, l'état IDLE initial (avant que la tâche motion n'ait vu le seq)
    // ferait retourner waitArrived immédiatement.
    uint32_t expected = _motionCtrl.getUserSeq();
    while (true) {
        if (isMatchOver()) {
            _motionCtrl.release();
            _motion.stop();
            disableMotors();
            gDisplay.robot_state = RobotState::DONE;
            return false;
        }
        if (!_nearEndMode && isEndgame()) {
            _motionCtrl.release();
            return false;
        }
        if (_motionCtrl.getUserSeq() != expected) return false;     // préempté
        if (_motionCtrl.getDoneSeq() == expected)                   // terminé
            return _motionCtrl.lastArrivedOk();
        if (blendMm > 0.0f && _motionCtrl.getDistMm() < blendMm) return true;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void Robot::setTarget(float x_mm, float y_mm) {
    setTarget(x_mm, y_mm, NAN, false);
}

void Robot::setTarget(float x_mm, float y_mm, float arrival_deg, bool backward) {
    MotionController::Target t;
    t.x_mm       = x_mm;
    t.y_mm       = y_mm;
    t.theta_deg  = arrival_deg;
    t.speed      = _motion.getSpeed();
    t.accel      = _motion.getAcceleration();
    t.stopMm     = gCalib.stopMm;
    t.backward   = backward;
    t.obstacleEn = _obstacleEn;
    _motionCtrl.setTarget(t);
}

// ─── Ligne droite relative ───────────────────────────────────────────────────

bool Robot::goPID(float mm) {
    if (fabsf(mm) < 0.5f) return true;
    if (!_moveAllowed()) return false;

    // Cible XY = pose courante + (mm) dans la direction theta
    const float curT = getTheta();
    const float tx   = getX() + mm * cosf(curT);
    const float ty   = getY() - mm * sinf(curT);   // Y+ vers le bas

    setTarget(tx, ty, NAN, mm < 0);
    bool ok = waitArrived(0);
    if (!ok) LOG_W("ROBOT", "goPID %.0f mm : non abouti", (double)mm);
    return ok;
}

// ─── Rotation relative ───────────────────────────────────────────────────────

bool Robot::turnPID(float deg) {
    if (fabsf(deg) < 0.1f) return true;
    if (!_moveAllowed()) return false;

    // Cible : pose courante + delta theta (pas de translation)
    setTarget(getX(), getY(), getThetaDeg() + deg, false);
    bool ok = waitArrived(0);
    if (!ok) LOG_W("ROBOT", "turnPID %.1f deg : non abouti", (double)deg);
    return ok;
}

// ─── Aller à un point ────────────────────────────────────────────────────────

bool Robot::gotoXYenc(float tx, float ty) {
    return gotoXYenc(tx, ty, NAN);
}

bool Robot::gotoXYenc(float tx, float ty, float arrival_deg, bool backward) {
    if (!_moveAllowed()) return false;
    gDisplay.robot_state = RobotState::GOTO;
    setTarget(tx, ty, arrival_deg, backward);
    bool ok = waitArrived(0);
    if (!ok) LOG_W("ROBOT", "gotoXY (%.0f, %.0f) : non abouti", (double)tx, (double)ty);
    return ok;
}

// ─── Pose ─────────────────────────────────────────────────────────────────────

void Robot::setPosition(float x_mm, float y_mm, float theta_deg) {
    _motion.setPosition(x_mm, y_mm, theta_deg);
    // Synchronise l'odométrie encodeurs sur la même position
    gDisplay.enc_reset_x         = x_mm;
    gDisplay.enc_reset_y         = y_mm;
    gDisplay.enc_reset_theta_deg = theta_deg;
    gDisplay.enc_reset_pending   = true;
    while (gDisplay.enc_reset_pending) vTaskDelay(1);   // attend que taskEncoders applique
    LOG_I("ROBOT", "Position: x=%.0f y=%.0f theta=%.1f", x_mm, y_mm, theta_deg);
}

// ─── Cinématique ──────────────────────────────────────────────────────────────

void Robot::setSpeed(float mmS) {
    _motion.setSpeed(mmS);
}

void Robot::setSpeedPct(float speedPct, float accelPct) {
    if (accelPct < 0.0f) accelPct = speedPct;   // même % si non spécifié
    _motion.setSpeed(gCalib.defaultSpeed * speedPct / 100.0f);
    _motion.setAcceleration(gCalib.defaultAccel * accelPct / 100.0f);
}

void Robot::resetSpeed() {
    _motion.setSpeed(gCalib.defaultSpeed);
    _motion.setAcceleration(gCalib.defaultAccel);
}

// ─── Obstacle (goStall) ───────────────────────────────────────────────────────

// Attend que la direction surveillée se dégage. false = timeout.
bool Robot::_waitObstacleClear() {
    vTaskDelay(pdMS_TO_TICKS(500));   // attente minimale avant de re-sonder
    uint32_t start = millis();
    while (obstacleDetected()) {
        if (millis() - start > OBS_WAIT_MS) {
            LOG_W("ROBOT", "Timeout obstacle — reprise quand meme");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    LOG_I("ROBOT", "Obstacle degage — reprise");
    return true;
}
