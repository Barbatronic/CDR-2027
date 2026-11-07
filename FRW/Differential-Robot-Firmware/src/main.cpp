#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include "config.h"
#include "live_config.h"
#include "log.h"
#include "wifi/log_server.h"
#include "lidar/ld06.h"
#include "motion/step_control.h"
#include "motion/motion_ctrl.h"
#include "strategy/robot.h"
#include "strategy/strategy.h"
#include "display/oled.h"
#include "io/buttons.h"
#include "io/leds.h"
#include "actuators/actuators.h"
#include "motion/encoder.h"

// ─── Instances globales ───────────────────────────────────────────────────────

static LD06         lidar(Serial1, LIDAR_RX_PIN, LIDAR_PWM_PIN);
static StepControl  motion;
static QuadEncoder  encRight;
static QuadEncoder  encLeft;
static Robot        robot(motion, lidar, gMotionCtrl);

static LidarPoint   scanBuf[LD06_SCAN_BUF_SIZE];

// ─── Tâche encodeurs (Core 0, 200 Hz) ───────────────────────────────────────
// Tâche dédiée pour gérer le rollover PCNT (±32767) indépendamment du reste.

static void taskEncoders(void *) {
    static int32_t lastL   = 0;
    static int32_t lastR   = 0;
    static float   encTheta = 0.0f;
    static constexpr float PI_F = 3.14159265f;

    for (;;) {
        encRight.update();
        encLeft.update();

        int32_t curL = encLeft.getCount();
        int32_t curR = encRight.getCount();

        // Comptes bruts toujours disponibles (affichage WAIT_INIT)
        gDisplay.enc_left_cnt  = curL;
        gDisplay.enc_right_cnt = curR;

        // Reset odométrie si setPosition() l'a demandé
        if (gDisplay.enc_reset_pending) {
            gDisplay.enc_reset_pending  = false;
            gDisplay.enc_pose_x_mm      = gDisplay.enc_reset_x;
            gDisplay.enc_pose_y_mm      = gDisplay.enc_reset_y;
            gDisplay.enc_pose_theta_deg = gDisplay.enc_reset_theta_deg;
            encTheta = gDisplay.enc_reset_theta_deg * (PI_F / 180.0f);
            lastL = curL;
            lastR = curR;
        }

        // Odométrie différentielle encodeurs
        int32_t dL = curL - lastL;
        int32_t dR = curR - lastR;
        lastL = curL;
        lastR = curR;

        float leftMm  = (float)dL * gCalib.mmPerCount;
        float rightMm = (float)dR * gCalib.mmPerCount;
        float dDist   = (leftMm + rightMm) * 0.5f;
        float dTheta  = (rightMm - leftMm) / gCalib.encWheelbase;
        float mid     = encTheta + dTheta * 0.5f;

        gDisplay.enc_pose_x_mm += dDist * cosf(mid);
        gDisplay.enc_pose_y_mm -= dDist * sinf(mid);  // Y+ vers le bas
        encTheta += dTheta;
        while (encTheta >  PI_F) encTheta -= 2.0f * PI_F;
        while (encTheta < -PI_F) encTheta += 2.0f * PI_F;
        gDisplay.enc_pose_theta_rad = encTheta;
        gDisplay.enc_pose_theta_deg = encTheta * (180.0f / PI_F);

        vTaskDelay(pdMS_TO_TICKS(5));   // 200 Hz
    }
}

// ─── Tâche LIDAR (Core 0) ────────────────────────────────────────────────────

static void taskLidar(void *) {
    lidar.begin();
    gDisplay.lidar_ok = true;
    for (;;) {
        lidar.update();
        vTaskDelay(1);
    }
}

// ─── Commandes UI → Strategy ─────────────────────────────────────────────────

#include "state_cmd.h"

QueueHandle_t gStateCmd = nullptr;

// Demandes d'arrêt traitées par taskMatchGuard, y compris pendant la stratégie
// (la file gStateCmd n'est lue qu'avant et après runStrategy*()).
static volatile bool sStopMotorsReq   = false;
static volatile bool sEnableMotorsReq = false;

void sendStateCmd(StateCmd cmd) {
    // Appelé depuis la tâche WiFi : les arrêts sont délégués à taskMatchGuard
    // (Core 1), qui ne s'exécute jamais en même temps que motion / stratégie.
    if (cmd == StateCmd::STOP_MOTORS)   { sStopMotorsReq = true; sEnableMotorsReq = false; }
    if (cmd == StateCmd::ENABLE_MOTORS) { sEnableMotorsReq = true; sStopMotorsReq = false; }
    if (cmd == StateCmd::STOP_MATCH && gDisplay.match_start_ms > 0) {
        uint32_t forced = millis() - MATCH_DURATION_MS;
        gDisplay.match_start_ms = forced ? forced : 1;   // 0 = « pas de match »
    }
    if (gStateCmd) xQueueSend(gStateCmd, &cmd, 0);
}

// ─── Tâche chrono de match (Core 1, prio 5) ──────────────────────────────────
// Garantit l'arrêt à MATCH_DURATION_MS, quoi que fasse la stratégie (tir en
// cours, bras, recul obstacle...). Même cœur que motion et stratégie, priorité
// supérieure : elle les préempte sans jamais s'exécuter en parallèle, comme le
// reste du code qui pilote FastAccelStepper depuis plusieurs tâches.
// À 100 s : arrêt des roues, lanceur et ventilateur à 0, puis verrouillage
// (ESC, ventilateur, activation steppers) pour qu'un appel tardif de la
// stratégie ne relance rien. Le verrou est levé par resetMatchState().

static volatile bool sMatchKilled = false;

static void stopAllActuators() {
    stopFlywheels();
    feeder.stop();
}

static void taskMatchGuard(void *) {
    uint32_t killedAt = 0;
    bool     disabled = false;
    for (;;) {
        uint32_t now   = millis();
        uint32_t start = gDisplay.match_start_ms;

        if (sStopMotorsReq) {
            sStopMotorsReq = false;
            gMotionCtrl.release();
            motion.stop();
            motion.disableMotors();
            LOG_W("GUARD", "Moteurs coupes (UI)");
        }
        if (sEnableMotorsReq) {
            sEnableMotorsReq = false;
            motion.enableMotors();      // sans effet si verrou de fin de match
            LOG_I("GUARD", "Moteurs actives (UI)");
        }

        if (!sMatchKilled && start > 0 && now - start >= MATCH_DURATION_MS) {
            sMatchKilled = true;
            killedAt     = now;
            disabled     = false;
            Esc::setLocked(true);
            PwmFan::setLocked(true);
            motion.setLocked(true);
            gMotionCtrl.release();
            motion.stop();              // arrêt net, moteurs encore alimentés
            stopAllActuators();
            gDisplay.robot_state = RobotState::DONE;
            gDisplay.match_phase = MatchPhase::DONE;
            LOG_W("GUARD", "Fin de match : tout est coupe");
        }

        if (sMatchKilled && Esc::isLocked()) {
            // Coupure des steppers après l'arrêt net (sinon roue libre lancée)
            if (!disabled && now - killedAt >= 300) {
                motion.disableMotors();
                disabled = true;
            }
            // Une consigne écrite juste avant le verrou peut avoir survécu
            if (flywheelLeft.getSpeed() > 0 || flywheelRight.getSpeed() > 0 ||
                feeder.getSpeed() > 0)
                stopAllActuators();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Remise à zéro avant une relance sans redémarrage (retour WAIT_INIT ou
// RESTART_MATCH) : chrono, mode endgame et verrous de fin de match.
// Stratégie terminée (état DONE) : plus aucun appel tardif possible, les
// verrous sont levés pour permettre les essais depuis l'UI (goto, moteurs).
// sMatchKilled reste vrai : le chrono ne recoupe pas.
static void unlockAfterMatch() {
    Esc::setLocked(false);
    PwmFan::setLocked(false);
    motion.setLocked(false);
}

static void resetMatchState() {
    robot.resetMatch();         // match_start_ms = 0 d'abord : le chrono ne recoupe pas
    sMatchKilled = false;
    Esc::setLocked(false);
    PwmFan::setLocked(false);
    motion.setLocked(false);
}

// ─── Tâche Strategy (Core 1, prio 2) ─────────────────────────────────────────

static void taskStrategy(void *) {
    // Encodeurs initialisés une seule fois (PCNT ne supporte pas le re-init)
    encRight.init(ENC_RIGHT_INVERT ? ENC_RIGHT_B_PIN : ENC_RIGHT_A_PIN,
                  ENC_RIGHT_INVERT ? ENC_RIGHT_A_PIN : ENC_RIGHT_B_PIN, PCNT_UNIT_2);
    encLeft.init( ENC_LEFT_INVERT  ? ENC_LEFT_B_PIN  : ENC_LEFT_A_PIN,
                  ENC_LEFT_INVERT  ? ENC_LEFT_A_PIN  : ENC_LEFT_B_PIN,  PCNT_UNIT_3);
    robot.setEncoders(&encLeft, &encRight);

    // taskMotionControl : 50 Hz sur Core 1, prio 3 — démarre une fois que les
    // encodeurs sont prêts (la tâche les lit dès le premier tick).
    gMotionCtrl.begin(&motion, &encLeft, &encRight, &lidar);

#if BOOT_SELFTEST_ENABLED
    runSelfTest();
#endif
#if BOOT_FLYWHEEL_TEST_ENABLED
    runFlywheelTest();
#endif

restart:   // point de retour pour GOTO_WAIT_INIT / RESTART_MATCH
    robot.disableMotors();
    resetMatchState();
    gDisplay.team = teamSwitch();
    ledsSetTeam(gDisplay.team);
    gDisplay.robot_state = RobotState::WAIT_INIT;
    gDisplay.match_phase = MatchPhase::WAIT_INIT;

    // États : WAIT_INIT → WAIT_TIRETTE_IN → WAIT_TIRETTE_OUT → match
    enum class Phase : uint8_t { WAIT_INIT, WAIT_TIRETTE_IN, WAIT_TIRETTE_OUT };
    Phase phase = Phase::WAIT_INIT;

    // Condition de sortie : tirette retirée après init + mise en place
    while (phase != Phase::WAIT_TIRETTE_OUT || !tirette()) {

        // Commandes UI
        StateCmd cmd;
        if (xQueueReceive(gStateCmd, &cmd, 0) == pdTRUE) {
            if (cmd == StateCmd::STOP_MOTORS)    { robot.disableMotors(); }
            if (cmd == StateCmd::ENABLE_MOTORS)  { robot.enableMotors(); }
            if (cmd == StateCmd::GOTO_WAIT_INIT) goto restart;
            if (cmd == StateCmd::START_MATCH)    goto match_start;   // skip tirette
            if (cmd == StateCmd::RUN_TEST && phase == Phase::WAIT_INIT) {
                runSelfTest();
                runFlywheelTest();
            }
            if (cmd == StateCmd::RESTART_INIT)  {
                gDisplay.robot_state = RobotState::INIT;
                gDisplay.match_phase = MatchPhase::CALAGE;
                if (gDisplay.team == Team::YELLOW) runInitYellow(robot);
                else                               runInitBlue(robot);
                phase = Phase::WAIT_TIRETTE_IN;
                gDisplay.robot_state = RobotState::WAIT_TIRETTE_IN;
                gDisplay.match_phase = MatchPhase::WAIT_TIRETTE_IN;
            }
        }

        // Switch d'équipe lu en continu ; changement = init à refaire
        Team t = teamSwitch();
        if (t != gDisplay.team) {
            gDisplay.team = t;
            ledsSetTeam(t);
            phase = Phase::WAIT_INIT;
            gDisplay.robot_state = RobotState::WAIT_INIT;
            gDisplay.match_phase = MatchPhase::WAIT_INIT;
        }

        switch (phase) {
        case Phase::WAIT_INIT:
            if (initPressed()) {
                gDisplay.robot_state = RobotState::INIT;
                gDisplay.match_phase = MatchPhase::CALAGE;
                if (gDisplay.team == Team::YELLOW) runInitYellow(robot);
                else                               runInitBlue(robot);
                phase = Phase::WAIT_TIRETTE_IN;
                gDisplay.robot_state = RobotState::WAIT_TIRETTE_IN;
                gDisplay.match_phase = MatchPhase::WAIT_TIRETTE_IN;
            }
            break;
        case Phase::WAIT_TIRETTE_IN:
            if (!tirette()) {
                phase = Phase::WAIT_TIRETTE_OUT;
                gDisplay.robot_state = RobotState::WAIT_TIRETTE_OUT;
                gDisplay.match_phase = MatchPhase::READY;
            }
            break;
        case Phase::WAIT_TIRETTE_OUT:
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // ── Match ─────────────────────────────────────────────────────────────────
match_start:
    robot.startMatch();
    gDisplay.match_phase = MatchPhase::MATCH;

    if (gDisplay.team == Team::YELLOW) runStrategyYellow(robot);
    else                               runStrategyBlue(robot);

    // ── Attente endgame (vérifie aussi RESTART_MATCH) ────────────────────────
    while (!robot.isEndgame() && !robot.isMatchOver()) {
        StateCmd cmd;
        if (xQueueReceive(gStateCmd, &cmd, 0) == pdTRUE) {
            if (cmd == StateCmd::STOP_MOTORS)    { robot.disableMotors(); }
            if (cmd == StateCmd::ENABLE_MOTORS)  { robot.enableMotors(); }
            // STOP_MATCH : traité dans sendStateCmd (chrono avancé → taskMatchGuard)
            if (cmd == StateCmd::RESTART_MATCH)  { robot.disableMotors(); goto restart; }
            if (cmd == StateCmd::GOTO_WAIT_INIT) { robot.disableMotors(); goto restart; }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // ── Repli fin de match ────────────────────────────────────────────────────
    if (!robot.isMatchOver()) {
        robot.startNearEnd();
        gDisplay.robot_state = RobotState::ENDGAME;
        gDisplay.match_phase = MatchPhase::ENDGAME;
        if (gDisplay.team == Team::YELLOW) runNearEndYellow(robot);
        else                               runNearEndBlue(robot);
    }

    while (!robot.isMatchOver()) vTaskDelay(pdMS_TO_TICKS(50));
    robot.disableMotors();
    actuatorsDisable();
    gDisplay.robot_state = RobotState::DONE;
    gDisplay.match_phase = MatchPhase::DONE;
    unlockAfterMatch();
    LOG_I("MAIN", "Match termine");

    // ── Après match : attente commande UI pour relancer ───────────────────────
    while (true) {
        StateCmd cmd;
        if (xQueueReceive(gStateCmd, &cmd, 0) == pdTRUE) {
            if (cmd == StateCmd::STOP_MOTORS)    { robot.disableMotors(); }
            if (cmd == StateCmd::ENABLE_MOTORS)  { robot.enableMotors(); }
            if (cmd == StateCmd::GOTO_WAIT_INIT) { goto restart; }
            if (cmd == StateCmd::RESTART_MATCH ||
                cmd == StateCmd::START_MATCH)    { resetMatchState(); robot.enableMotors(); goto match_start; }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ─── Setup ───────────────────────────────────────────────────────────────────

void setup() {
    feeder.begin();   // en premier : entrée PWM flottante = ventilateur à fond
    Serial.begin(115200);
    delay(200);
    Serial.println("=== DifferentialRobot ===");

    Wire.begin(DISPLAY_SDA_PIN, DISPLAY_SCL_PIN);
    Wire.setClock(400000);

    if (!motion.begin()) {
        Serial.println("[ERREUR] motion.begin() failed");
        while (true) vTaskDelay(1000);
    }

    gStateCmd = xQueueCreate(4, sizeof(StateCmd));
    buttonsInit();
    ledsInit();
    actuatorsInit();
    wifiLogBegin();

    displayStart();

    xTaskCreatePinnedToCore(taskLidar,    "lidar",    TASK_LIDAR_STACK,
                            nullptr, TASK_LIDAR_PRIO,    nullptr, TASK_LIDAR_CORE);

    xTaskCreatePinnedToCore(taskEncoders, "encoders", 2048,
                            nullptr, 3,                  nullptr, 0);   // Core 0, prio 3

    xTaskCreatePinnedToCore(taskMatchGuard, "guard", 3072,
                            nullptr, 5,                  nullptr, 1);   // Core 1, prio 5

    xTaskCreatePinnedToCore(taskStrategy, "strategy", TASK_STRATEGY_STACK,
                            nullptr, TASK_STRATEGY_PRIO, nullptr, TASK_STRATEGY_CORE);
}

// ─── Loop : LEDs + WiFi telemetry + affichage ────────────────────────────────

void loop() {
    static uint32_t lastLed      = 0;
    static uint32_t lastMatchLog = 0;
    static uint32_t lastWifi     = 0;
    uint32_t now = millis();

    // Mise à jour données écran
    gDisplay.lidar_rpm      = lidar.getRPM();
    gDisplay.pose_x_mm      = motion.getX();
    gDisplay.pose_y_mm      = motion.getY();
    gDisplay.pose_theta_deg = motion.getThetaDeg();

    // Log match — 500 ms pendant le match
    if (LOG_LEVEL >= 3 && gDisplay.match_start_ms > 0 && now - lastMatchLog >= 500) {
        lastMatchLog = now;
        uint32_t elapsed = (now - gDisplay.match_start_ms) / 1000;
        Serial.printf("[%3lus] %s | Est(%5.0f,%5.0f,%+5.1f) Enc(%5.0f,%5.0f,%+5.1f)\n",
                      elapsed,
                      robotStateStr(gDisplay.robot_state),
                      (double)gDisplay.pose_x_mm,      (double)gDisplay.pose_y_mm,
                      (double)gDisplay.pose_theta_deg,
                      (double)gDisplay.enc_pose_x_mm,  (double)gDisplay.enc_pose_y_mm,
                      (double)gDisplay.enc_pose_theta_deg);
    }

    // LEDs + LIDAR WiFi — 100 ms (LIDAR WiFi seulement en pré-match)
    if (now - lastLed >= 100) {
        lastLed = now;
        uint16_t n = lidar.getScan(scanBuf, LD06_SCAN_BUF_SIZE);
        gDisplay.lidar_pts = n;
        ledsUpdateLidar(scanBuf, n);
        RobotState st = gDisplay.robot_state;
        if (st == RobotState::WAIT_INIT      || st == RobotState::INIT ||
            st == RobotState::WAIT_TIRETTE_IN || st == RobotState::WAIT_TIRETTE_OUT)
            wifiLogLidar(scanBuf, n);
        // LIDAR absolu toujours envoyé (coordonnées table, filtré en JS)
        wifiLogLidarAbs(scanBuf, n,
                        gDisplay.enc_pose_x_mm, gDisplay.enc_pose_y_mm,
                        gDisplay.enc_pose_theta_rad);
    }

    // Pose + actionneurs + motion debug + commandes WiFi — 200 ms
    if (now - lastWifi >= 200) {
        lastWifi = now;
        char team = (gDisplay.team == Team::YELLOW) ? 'Y' : 'B';
        // canClick : autorisé après init et hors match en cours
        RobotState rs = gDisplay.robot_state;
        bool postInit  = (rs != RobotState::WAIT_INIT) && (rs != RobotState::INIT);
        bool inMatch   = (gDisplay.match_start_ms > 0) &&
                         (now - gDisplay.match_start_ms < MATCH_DURATION_MS);
        bool canClick  = postInit && !inMatch;
        wifiLogUpdatePose(gDisplay.enc_pose_x_mm, gDisplay.enc_pose_y_mm, gDisplay.enc_pose_theta_deg,
                          robotStateStr(gDisplay.robot_state),
                          gDisplay.nav_dist_mm, gDisplay.nav_delta_deg, team, canClick);
        uint32_t elapsed = gDisplay.match_start_ms ? now - gDisplay.match_start_ms : 0;
        wifiLogUpdateStatus((uint8_t)gDisplay.match_phase, elapsed,
                            motion.isEnabled(), motion.isLocked());
        wifiLogUpdateActuators(flywheelLeft.getSpeed(), flywheelRight.getSpeed(), feeder.getSpeed(),
                               servoBasGauche.getPercent(), servoBasDroit.getPercent());
        wifiLogUpdateMotion((uint8_t)gMotionCtrl.getState(),
                            (uint8_t)gMotionCtrl.getPhase(),
                            gMotionCtrl.getDistMm(),
                            gMotionCtrl.getSpeedCap(),
                            gMotionCtrl.getCmdL(),
                            gMotionCtrl.getCmdR(),
                            gMotionCtrl.getTargetX(),
                            gMotionCtrl.getTargetY(),
                            gMotionCtrl.getTargetTheta());
    }

    // Click utilisateur sur le terrain → goto vers (x, y[, theta]).
    // Autorisé après init (state != WAIT_INIT/INIT) et hors match en cours
    // (match_start_ms == 0 OU match déjà fini).
    float fcX, fcY, fcTheta;
    if (wifiPollFieldClick(fcX, fcY, fcTheta)) {
        RobotState st = gDisplay.robot_state;
        bool postInit = (st != RobotState::WAIT_INIT) && (st != RobotState::INIT);
        bool inMatch  = (gDisplay.match_start_ms > 0) &&
                        (millis() - gDisplay.match_start_ms < MATCH_DURATION_MS);
        if (postInit && !inMatch) {
            robot.enableMotors();
            robot.setTarget(fcX, fcY, fcTheta);   // fcTheta = NAN si pas de drag
            if (isnanf(fcTheta)) LOG_I("UI", "Goto click: %.0f,%.0f", (double)fcX, (double)fcY);
            else                 LOG_I("UI", "Goto click: %.0f,%.0f theta=%.0f", (double)fcX, (double)fcY, (double)fcTheta);
        } else {
            LOG_W("UI", "Goto ignore (etat %d, inMatch=%d)", (int)st, (int)inMatch);
        }
    }

    // Commandes actionneurs depuis l'interface web (seulement avant init)
    ServoCmd cmd;
    while (wifiPollCmd(cmd)) {
        if (gDisplay.robot_state != RobotState::WAIT_INIT) continue;

        if      (strcmp(cmd.id, "seq_stop")      == 0) { feeder.stop(); stopFlywheels(); }
        else if (strcmp(cmd.id, "seq_brasOuvert") == 0) ouvrirBras();
        else if (strcmp(cmd.id, "seq_brasPrise")  == 0) prendreBloc();
        else if (strcmp(cmd.id, "seq_brasRepos")  == 0) reposBras();
        else if (strcmp(cmd.id, "feeder")        == 0) feeder.setSpeed(cmd.val);
        else if (strcmp(cmd.id, "flywheels")     == 0) setFlywheels(cmd.val);
        else if (strcmp(cmd.id, "flywheelLeft")  == 0) flywheelLeft.setSpeed(cmd.val);
        else if (strcmp(cmd.id, "flywheelRight") == 0) flywheelRight.setSpeed(cmd.val);
        else if (strcmp(cmd.id, "servoBasG")     == 0) servoBasGauche.setPercent(cmd.val);
        else if (strcmp(cmd.id, "servoBasD")     == 0) servoBasDroit.setPercent(cmd.val);
    }

    vTaskDelay(10);
}
