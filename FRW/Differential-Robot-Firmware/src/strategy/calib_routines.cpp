#include "calib_routines.h"
#include "../live_config.h"
#include "../display/oled.h"
#include "../wifi/log_server.h"
#include "../log.h"
#include <math.h>

static constexpr float PI_F    = 3.14159265f;
static constexpr float RAD2DEG = 180.0f / PI_F;

static QueueHandle_t sQueue = nullptr;
static volatile bool sAbort = false;

static constexpr float CAL_STALL_PCT  = 10.0f;   // recul contre bordure (comme le calage)
static constexpr float CAL_MOVE_PCT   = 25.0f;   // déplacements entre deux contacts
static constexpr float CAL_TURN_SPEED = 300.0f;  // tours en boucle ouverte (mm/s roue)
static constexpr float CAL_TURN_ACCEL = 500.0f;  // mm/s²
static constexpr float CAL_SQUARE_X0  = 360.0f;  // coin gauche du carré (x)
static constexpr float CAL_SQUARE_Y0  = 200.0f;  // côté haut du carré (y)

bool requestCalibRun(CalibRoutine id, float param) {
    if (!sQueue) return false;               // créée par la tâche stratégie
    if (gDisplay.match_phase != MatchPhase::WAIT_INIT) {
        LOG_W("CALRUN", "Refuse : seulement en attente d'init");
        return false;
    }
    CalibRunReq r{id, param};
    return xQueueSend(sQueue, &r, 0) == pdTRUE;
}

void calibAbort() { sAbort = true; }

// ─── Outils ──────────────────────────────────────────────────────────────────

struct Snap { int32_t l, r; };

static Snap snap(QuadEncoder &L, QuadEncoder &R) { return {L.getCount(), R.getCount()}; }

// Rotation (rad) et avance (mm) odométriques entre deux instantanés
static void odomDelta(Snap a, Snap b, float &rot, float &lin) {
    const float sL = (float)(b.l - a.l) * gCalib.mmPerCountL;
    const float sR = (float)(b.r - a.r) * gCalib.mmPerCountR;
    rot = (sR - sL) / gCalib.encWheelbase;
    lin = (sR + sL) * 0.5f;
}

static float wrapPi(float a) {
    while (a >  PI_F) a -= 2.0f * PI_F;
    while (a < -PI_F) a += 2.0f * PI_F;
    return a;
}

// Recule jusqu'au blocage contre la bordure, puis laisse le robot se poser
static bool backToWall(Robot &robot, float maxMm) {
    if (sAbort) return false;
    robot.setSpeedPct(CAL_STALL_PCT);
    bool ok = robot.goStall(-maxMm);
    vTaskDelay(pdMS_TO_TICKS(300));
    robot.setSpeedPct(CAL_MOVE_PCT);
    if (!ok) LOG_W("CALRUN", "Pas de contact bordure (plus de %.0f mm ?)", (double)maxMm);
    return ok && !sAbort;
}

static void sendResult(const char *json) {
    LOG_I("CALRUN", "%s", json);
    wifiLogCalibResult(json);
}

static void sendFail(const char *id) {
    char buf[96];
    snprintf(buf, sizeof(buf), "{\"type\":\"calres\",\"id\":\"%s\",\"ok\":0,\"abort\":%d}",
             id, sAbort ? 1 : 0);
    sendResult(buf);
}

// Déplacement asservi : au premier échec (non convergé, arrêt), la routine
// s'interrompt au lieu d'enchaîner sur une pose fausse
#define MOVE_OR_FAIL(call, id) do { if (!(call) || sAbort) { sendFail(id); return; } } while (0)

// ─── Ligne droite : diamètre moyen ───────────────────────────────────────────

static void runLine(Robot &robot, QuadEncoder &L, QuadEncoder &R, float mm) {
    robot.setPosition(0, gTable.h() * 0.5f, ANGLE_EAST);
    if (!backToWall(robot, CALAGE_STALL_MM)) { sendFail("line"); return; }
    robot.setPosition(ROBOT_BACK_TO_CENTER_MM, gTable.h() * 0.5f, ANGLE_EAST);

    Snap a = snap(L, R);
    MOVE_OR_FAIL(robot.goPID(mm), "line");
    vTaskDelay(pdMS_TO_TICKS(500));
    if (sAbort) { sendFail("line"); return; }
    Snap b = snap(L, R);

    float rot, lin;
    odomDelta(a, b, rot, lin);
    char buf[200];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"calres\",\"id\":\"line\",\"ok\":1,\"cmd\":%.0f,\"S\":%.2f,"
             "\"rotDeg\":%.2f,\"D\":%.4f}",
             (double)mm, (double)lin, (double)(rot * RAD2DEG), (double)gCalib.encWheelDiamMm);
    sendResult(buf);
}

// ─── Tours sur place : échelle des rotations et entraxe moteur ───────────────

// Sens alterné d'un essai à l'autre : les câbles se déroulent, et la moyenne
// des deux sens compense un éventuel biais propre au sens de rotation.
static int sTurnDir = -1;  // +1 = gauche (CCW), −1 = droite (CW) ; 1er essai à droite

static void runTurns(Robot &robot, QuadEncoder &L, QuadEncoder &R, float turns) {
    const int n   = (int)lroundf(turns);
    const int dir = sTurnDir;
    sTurnDir = -sTurnDir;
    robot.setPosition(0, gTable.h() * 0.5f, ANGLE_EAST);
    if (!backToWall(robot, CALAGE_STALL_MM)) { sendFail("turns"); return; }
    robot.setPosition(ROBOT_BACK_TO_CENTER_MM, gTable.h() * 0.5f, ANGLE_EAST);
    Snap a = snap(L, R);

    MOVE_OR_FAIL(robot.goPID(200.0f), "turns");

    Snap t0 = snap(L, R);
    robot.turnOpen(360.0f * n * dir, CAL_TURN_SPEED, CAL_TURN_ACCEL);
    vTaskDelay(pdMS_TO_TICKS(300));
    Snap t1 = snap(L, R);
    if (sAbort) { sendFail("turns"); return; }

    // Recul direct : les tours moteurs en boucle ouverte ramènent le robot
    // quasiment face à l'Est (constaté sur table), alors que l'odométrie, que
    // l'on calibre ici, peut se tromper de plusieurs degrés. Une correction
    // selon l'odométrie désalignerait le robot avant le contact.
    if (!backToWall(robot, 300.0f)) { sendFail("turns"); return; }
    Snap b = snap(L, R);

    float rot, lin, rotT, linT;
    odomDelta(a, b, rot, lin);
    odomDelta(t0, t1, rotT, linT);
    const float e = wrapPi(2.0f * PI_F * n * dir - rot);
    // Diagnostic par roue pendant les tours : arcs mesurés par chaque codeuse
    // et arc commandé à chaque roue motrice (même valeur absolue des deux côtés)
    const float aL = (float)(t1.l - t0.l) * gCalib.mmPerCountL;
    const float aR = (float)(t1.r - t0.r) * gCalib.mmPerCountR;
    const float aCmd = gCalib.wheelbase * PI_F * n;   // = wheelbase/2 × 2π·n
    char buf[300];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"calres\",\"id\":\"turns\",\"ok\":1,\"n\":%d,\"dir\":%d,\"R\":%.5f,\"S\":%.2f,"
             "\"e\":%.5f,\"Rt\":%.5f,\"b\":%.3f,\"wb\":%.3f,\"aL\":%.1f,\"aR\":%.1f,\"aC\":%.1f}",
             n, dir, (double)rot, (double)lin, (double)e, (double)rotT,
             (double)gCalib.encWheelbase, (double)gCalib.wheelbase,
             (double)aL, (double)aR, (double)aCmd);
    sendResult(buf);
}

// ─── Carré : observation combinée rotation + ligne droite ────────────────────

static void runSquare(Robot &robot, QuadEncoder &L, QuadEncoder &R, float side, bool cw) {
    const char *id = cw ? "sq_cw" : "sq_ccw";
    const float x0 = CAL_SQUARE_X0, y0 = CAL_SQUARE_Y0, x1 = x0 + side, y1 = y0 + side;

    // Calage coin nord-ouest : bordure ouest (x), puis bordure nord (y).
    // Sert seulement à placer le carré sur la table : la mesure se fait entre
    // deux contacts sur la MÊME bordure ouest, l'équerrage de la table
    // n'intervient donc pas.
    robot.setPosition(0, 0, ANGLE_EAST);
    if (!backToWall(robot, CALAGE_STALL_MM)) { sendFail(id); return; }
    robot.setPosition(ROBOT_BACK_TO_CENTER_MM, 0, ANGLE_EAST);
    MOVE_OR_FAIL(robot.goPID(CALAGE_X_MM - ROBOT_BACK_TO_CENTER_MM), id);
    MOVE_OR_FAIL(robot.turnPID(-90.0f), id);
    if (!backToWall(robot, CALAGE_STALL_MM)) { sendFail(id); return; }
    robot.setPosition(robot.getX(), ROBOT_BACK_TO_CENTER_MM, ANGLE_SOUTH);
    MOVE_OR_FAIL(robot.goPID(CALAGE_Y_MM - ROBOT_BACK_TO_CENTER_MM), id);

    // Contact de départ sur la bordure ouest, à la hauteur du coin de départ
    // (haut-gauche en horaire, bas-gauche en anti-horaire) : cap réel = Est
    const float ys = cw ? y0 : y1;
    MOVE_OR_FAIL(robot.gotoXYenc(CALAGE_X_MM, ys, ANGLE_EAST), id);
    if (!backToWall(robot, CALAGE_X_MM - ROBOT_BACK_TO_CENTER_MM + 80.0f)) { sendFail(id); return; }
    robot.setPosition(ROBOT_BACK_TO_CENTER_MM, gDisplay.enc_pose_y_mm, ANGLE_EAST);
    Snap a = snap(L, R);
    MOVE_OR_FAIL(robot.gotoXYenc(x0, ys, ANGLE_EAST), id);

    // Carré, départ et arrivée face à l'Est
    if (cw) {
        MOVE_OR_FAIL(robot.gotoXYenc(x1, y0, ANGLE_SOUTH), id);
        MOVE_OR_FAIL(robot.gotoXYenc(x1, y1, ANGLE_WEST), id);
        MOVE_OR_FAIL(robot.gotoXYenc(x0, y1, ANGLE_NORTH), id);
        MOVE_OR_FAIL(robot.gotoXYenc(x0, y0, ANGLE_EAST), id);
    } else {
        MOVE_OR_FAIL(robot.gotoXYenc(x1, y1, ANGLE_NORTH), id);
        MOVE_OR_FAIL(robot.gotoXYenc(x1, y0, ANGLE_WEST), id);
        MOVE_OR_FAIL(robot.gotoXYenc(x0, y0, ANGLE_SOUTH), id);
        MOVE_OR_FAIL(robot.gotoXYenc(x0, y1, ANGLE_EAST), id);
    }

    // Retour à la bordure ouest : cap réel = Est
    if (!backToWall(robot, x0 - ROBOT_BACK_TO_CENTER_MM + 80.0f)) { sendFail(id); return; }
    Snap b = snap(L, R);
    const float ex = gDisplay.enc_pose_x_mm - ROBOT_BACK_TO_CENTER_MM;

    float rot, lin;
    odomDelta(a, b, rot, lin);
    const float e = wrapPi(-rot);   // Est → Est, même bordure : ±360° réels ≡ 0
    char buf[220];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"calres\",\"id\":\"%s\",\"ok\":1,\"L\":%.0f,\"R\":%.5f,\"S\":%.2f,"
             "\"e\":%.5f,\"ex\":%.2f}",
             id, (double)side, (double)rot, (double)lin, (double)e, (double)ex);
    sendResult(buf);
}

// ─── Exécution ───────────────────────────────────────────────────────────────

bool pollCalibRun(Robot &robot, QuadEncoder &encL, QuadEncoder &encR) {
    if (!sQueue) { sQueue = xQueueCreate(1, sizeof(CalibRunReq)); return false; }
    CalibRunReq r;
    if (xQueueReceive(sQueue, &r, 0) != pdTRUE) return false;

    sAbort = false;
    gDisplay.robot_state = RobotState::INIT;
    gDisplay.match_phase = MatchPhase::CALAGE;   // verrouille calibration et table
    robot.enableMotors();
    robot.disableObstacle();
    robot.setSpeedPct(CAL_MOVE_PCT);

    switch (r.id) {
        case CalibRoutine::LINE:       runLine(robot, encL, encR, r.param);          break;
        case CalibRoutine::TURNS:      runTurns(robot, encL, encR, r.param);         break;
        case CalibRoutine::SQUARE_CW:  runSquare(robot, encL, encR, r.param, true);  break;
        case CalibRoutine::SQUARE_CCW: runSquare(robot, encL, encR, r.param, false); break;
    }

    robot.resetSpeed();
    robot.enableObstacle();
    robot.disableMotors();
    gDisplay.robot_state = RobotState::WAIT_INIT;
    gDisplay.match_phase = MatchPhase::WAIT_INIT;
    return true;
}
