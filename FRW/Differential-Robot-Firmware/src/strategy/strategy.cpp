// ═══════════════════════════════════════════════════════════════════════════════
//  STRATÉGIE ROBOT  —  modifier ce fichier pour définir le comportement
// ═══════════════════════════════════════════════════════════════════════════════
//
//  Commandes disponibles :
//
//  robot.go(mm)                         avance (>0) ou recule (<0)
//  robot.turn(deg)                      tourne à gauche (>0) ou droite (<0)
//  robot.gotoXY(x, y)                   va en (x,y) mm — angle d'arrivée libre
//  robot.gotoXY(x, y, angle_deg)        idem avec orientation finale
//
//  robot.setPosition(x, y, theta_deg)   recalage / datum / position initiale
//  robot.setSpeed(mm_s)                 change la vitesse de déplacement
//
//  robot.disableObstacle()              désactive détection adversaire
//  robot.enableObstacle()               réactive détection adversaire
//
//  Repère table : origine coin haut-gauche
//                X+ = droite (3000 mm), Y+ = bas (2000 mm)
//                angle 0° = droite (+X), 90° = haut (-Y), sens positif = anti-horaire
//  Table  : 3000 mm × 2000 mm
// ═══════════════════════════════════════════════════════════════════════════════

#include "strategy.h"
#include "../live_config.h"
#include "../actuators/actuators.h"
#include "../utils.h"
#include "../log.h"
#include "../io/buttons.h"
#include "../display/oled.h"
#include <Wire.h>
#include <math.h>

// ─── Calibration géométrique ─────────────────────────────────────────────────
void runCalibration(Robot &robot, QuadEncoder &encL, QuadEncoder &encR) {
    robot.enableMotors();
    robot.disableObstacle();

    Serial.println("\n=== CALIBRATION ANGLE (turn 360) ===");
    Serial.printf("WHEELBASE_MM actuel : %.2f\n", (double)gCalib.wheelbase);

    int32_t l0 = encL.getCount(), r0 = encR.getCount();

    robot.turn(720);
    wait(200);

    float arcL = (float)(encL.getCount() - l0) * gCalib.mmPerCount;
    float arcR = (float)(encR.getCount() - r0) * gCalib.mmPerCount;

    float actualAngle  = (arcR - arcL) / gCalib.encWheelbase * (180.0f / 3.14159265f);
    float newWheelbase = gCalib.wheelbase * (720.0f / actualAngle);

    Serial.printf("arcG=%.1fmm  arcD=%.1fmm  angle_reel=%.1f deg\n",
                  (double)arcL, (double)arcR, (double)actualAngle);

    Serial.println("\n=== VALEUR A COPIER DANS config.h ===");
    Serial.printf("#define WHEELBASE_MM  %.2ff\n", (double)newWheelbase);
    Serial.println("=====================================\n");
}

// ─── Calage bordure jaune (coin du fond gauche) ─────────────────────────────
void runInitYellow(Robot &robot) {
    robot.enableMotors();
    robot.disableObstacle();
    initActuators();                     // bras ouverts pendant tout le calage

    robot.setSpeedPct(10); // Change la vitesse et accel pour le calage, afin de limiter les risques de rebonds

    // ── Calage X — arrière plaqué contre la bordure Ouest ──────────────────
    robot.setPosition(0, 0, ANGLE_EAST);
    robot.goStall(-CALAGE_STALL_MM);     // recule jusqu'au contact de la bordure Ouest
    robot.setPosition(ROBOT_BACK_TO_CENTER_MM, 0, ANGLE_EAST);   // X calé
    robot.goPID(CALAGE_X_MM - ROBOT_BACK_TO_CENTER_MM);          // dégage

    // ── Calage Y — arrière plaqué contre la bordure du fond ────────────────
    robot.turnPID(-90.0f);               // s'oriente vers le Sud (public)
    robot.goStall(-CALAGE_STALL_MM);     // recule jusqu'au contact de la bordure du fond
    robot.setPosition(robot.getX(), ROBOT_BACK_TO_CENTER_MM, ANGLE_SOUTH);  // Y calé, X inchangé
    robot.goPID(CALAGE_Y_MM - ROBOT_BACK_TO_CENTER_MM);          // dégage

    // ── Salle du trône, orienté vers la première carrière ──────────────────
    robot.gotoXYenc(POI::startYellow, angleVers(POI::startYellow, POI::carriere_01));

    robot.resetSpeed(); // remet les vitesses par défaut pour la stratégie
}

// ─── Calage bordure bleu (coin du fond droit) — symétrique ──────────────────
void runInitBlue(Robot &robot) {
    robot.enableMotors();
    robot.disableObstacle();
    initActuators();

    robot.setSpeedPct(10);

    // ── Calage X — arrière plaqué contre la bordure Est ────────────────────
    robot.setPosition(TABLE_WIDTH_MM, 0, ANGLE_WEST);
    robot.goStall(-CALAGE_STALL_MM);
    robot.setPosition(TABLE_WIDTH_MM - ROBOT_BACK_TO_CENTER_MM, 0, ANGLE_WEST);
    robot.goPID(CALAGE_X_MM - ROBOT_BACK_TO_CENTER_MM);

    // ── Calage Y — arrière plaqué contre la bordure du fond ────────────────
    robot.turnPID(90.0f);                // s'oriente vers le Sud (180° → 270°)
    robot.goStall(-CALAGE_STALL_MM);
    robot.setPosition(robot.getX(), ROBOT_BACK_TO_CENTER_MM, ANGLE_SOUTH);
    robot.goPID(CALAGE_Y_MM - ROBOT_BACK_TO_CENTER_MM);

    // ── Salle du trône, orienté vers la première carrière ──────────────────
    robot.gotoXYenc(POI::startBlue, angleVers(POI::startBlue, POI::carriere_04));

    robot.resetSpeed();
}

// ─── Outils géométriques ─────────────────────────────────────────────────────
static constexpr float DEG2RAD = 3.14159265f / 180.0f;

// Point à `dist` mm devant `p` dans la direction `deg` (Y+ vers le public)
static Vec2 avancer(Vec2 p, float deg, float dist) {
    return Vec2(p.x + dist * cosf(deg * DEG2RAD), p.y - dist * sinf(deg * DEG2RAD));
}

float angleVers(Vec2 from, Vec2 to) {
    float deg = atan2f(-(to.y - from.y), to.x - from.x) / DEG2RAD;
    return (deg < 0) ? deg + 360.0f : deg;
}

// Approche finale lente en ligne droite, détection obstacle coupée
// (les pierres sont vues par le LIDAR et prises pour un adversaire).
static void approcheLente(Robot &robot, float mm) {
    float spd = robot.getSpeed(), acc = robot.getAcceleration();
    robot.setSpeedPct(APPROCHE_VITESSE_PCT, APPROCHE_VITESSE_PCT);
    robot.goPID(mm);
    robot.setSpeed(spd);
    robot.setAcceleration(acc);
}

// ─── Prise d'une carrière ────────────────────────────────────────────────────
//
//   [amont] ──PRE_APPROCHE──▶ [approche] ──offset──▶ [contact]  |carrière|
//
//  Au contact, l'avant du robot touche la face de la carrière : le centre des
//  pierres est à (avant + épaisseur/2) devant l'axe des roues.
void prendreCarriere(Robot &robot, Vec2 carriere, float approachDeg, float offsetMm) {
    const float contact = ROBOT_FRONT_TO_CENTER_MM + PIERRE_EPAISSEUR_MM / 2;
    Vec2 approche = avancer(carriere, approachDeg + 180.0f, contact + offsetMm);
    Vec2 amont    = avancer(approche, approachDeg + 180.0f, CARRIERE_PRE_APPROCHE_MM);

    LOG_I("STRAT", "Prise carriere (%.0f,%.0f) a %.0f deg", (double)carriere.x, (double)carriere.y, (double)approachDeg);
    ouvrirBras();
    robot.gotoXYenc(amont, approachDeg);            // détection obstacle active

    robot.disableObstacle();
    robot.gotoXYenc(approche, approachDeg);
    approcheLente(robot, offsetMm);                 // avant contre la carrière
    prendreBloc();
    robot.goPID(-CARRIERE_RECUL_MM);
    robot.enableObstacle();
}

// ─── Dépose sur un mur ───────────────────────────────────────────────────────
// Axe de chaque mur (°, repère table). Les murs _2/_4 sont à 45° entre deux tours.
struct MurDef { Vec2 pos; float axeDeg; Vec2 cour; };
static const MurDef MURS[] = {
    {POI::murYellow_1,   0.0f, POI::courYellow},
    {POI::murYellow_2, 135.0f, POI::courYellow},
    {POI::murYellow_3,  90.0f, POI::courYellow},
    {POI::murYellow_4,  45.0f, POI::courYellow},
    {POI::murYellow_5,   0.0f, POI::courYellow},
    {POI::murBlue_1,     0.0f, POI::courBlue},
    {POI::murBlue_2,    45.0f, POI::courBlue},
    {POI::murBlue_3,    90.0f, POI::courBlue},
    {POI::murBlue_4,   135.0f, POI::courBlue},
    {POI::murBlue_5,     0.0f, POI::courBlue},
};

bool deposerMur(Robot &robot, Vec2 mur) {
    const MurDef *def = nullptr;
    for (const MurDef &m : MURS)
        if (m.pos.x == mur.x && m.pos.y == mur.y) def = &m;
    if (!def) {
        LOG_E("STRAT", "deposerMur : (%.0f,%.0f) n'est pas un mur", (double)mur.x, (double)mur.y);
        return false;
    }

    // Robot perpendiculaire au mur (pierres alignées sur l'axe du mur),
    // du côté extérieur : il regarde vers la cour.
    float cap = def->axeDeg + 90.0f;
    float versCour = angleVers(mur, def->cour);
    if (cosf((versCour - cap) * DEG2RAD) < 0) cap += 180.0f;
    if (cap >= 360.0f) cap -= 360.0f;

    const float contact = ROBOT_FRONT_TO_CENTER_MM + PIERRE_EPAISSEUR_MM / 2;
    Vec2 approche = avancer(mur, cap + 180.0f, contact + DEPOSE_APPROCHE_MM);

    LOG_I("STRAT", "Depose mur (%.0f,%.0f) cap %.0f deg", (double)mur.x, (double)mur.y, (double)cap);
    robot.gotoXYenc(approche, cap);

    robot.disableObstacle();
    approcheLente(robot, DEPOSE_APPROCHE_MM);       // pierres au-dessus de la zone
    ouvrirBras();                                   // relâche
    robot.goPID(-DEPOSE_RECUL_MM);
    robot.enableObstacle();
    return true;
}

// ─── Tir depuis la cour ──────────────────────────────────────────────────────
// Entrée dans le château par le mur 3 (face au centre), puis salle du trône.
void tirerDepuisCour(Robot &robot, Team team) {
    const bool jaune = (team == Team::YELLOW);
    Vec2  entree = jaune ? avancer(POI::murYellow_3, ANGLE_EAST, 300.0f)
                         : avancer(POI::murBlue_3,   ANGLE_WEST, 300.0f);
    Vec2  poste  = jaune ? POI::startYellow : POI::startBlue;
    float cap    = jaune ? ANGLE_SOUTH : ANGLE_NORTH;   // flanc gauche vers l'adversaire

    robot.gotoXYenc(entree);
    robot.gotoXYenc(poste, cap);

#if TIR_ATTENDRE_PHASE_ATTAQUE
    robot.waitMatchTime(MATCH_ENDGAME_MS);              // phase d'attaque : 85 s → fin
#endif
    launchBalls(TIR_ROUE_G_PCT, TIR_ROUE_D_PCT, TIR_DUREE_MS);
}

// ─── Self-test matériel ───────────────────────────────────────────────────────
static bool i2cPresent(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

bool runSelfTest() {
    bool ok = true;
    LOG_I("TEST", "=== SELF-TEST ===");

    // Bus I2C
    struct { uint8_t addr; const char *name; bool required; } devs[] = {
        {PCA9685_I2C_ADDR, "PCA9685", true},
        {PCF8574_I2C_ADDR, "PCF8574", false},   // optionnel : non monté actuellement
        {OLED_I2C_ADDR,    "OLED",    true},
    };
    for (auto &d : devs) {
        bool present = i2cPresent(d.addr);
        if (present)          LOG_I("TEST", "I2C 0x%02X %-8s OK",                d.addr, d.name);
        else if (d.required)  LOG_E("TEST", "I2C 0x%02X %-8s ABSENT",            d.addr, d.name);
        else                  LOG_W("TEST", "I2C 0x%02X %-8s absent (optionnel)", d.addr, d.name);
        if (d.required) ok &= present;
    }

    // LIDAR : attend la rotation (max 3 s)
    uint32_t t0 = millis();
    while (gDisplay.lidar_rpm < 1.0f && millis() - t0 < 3000) wait(100);
    if (gDisplay.lidar_rpm >= 1.0f) LOG_I("TEST", "LIDAR OK (%.0f rpm)", (double)gDisplay.lidar_rpm);
    else                          { LOG_E("TEST", "LIDAR pas de rotation"); ok = false; }

    // Encodeurs / IO : simple relevé (vérif visuelle en poussant le robot)
    LOG_I("TEST", "Encodeurs G=%ld D=%ld", (long)gDisplay.enc_left_cnt, (long)gDisplay.enc_right_cnt);
    LOG_I("TEST", "Tirette=%s Equipe=%s",
          tirette() ? "retiree" : "en place",
          teamSwitch() == Team::YELLOW ? "JAUNE" : "BLEU");

    LOG_I("TEST", "=== SELF-TEST %s ===", ok ? "OK" : "ECHEC");
    return ok;
}

// ─── Test lanceur (roues à inertie) ──────────────────────────────────────────
void runFlywheelTest() {
    LOG_I("TEST", "=== TEST LANCEUR ===");
    LOG_I("TEST", "Armement ESC (%d ms gaz mini)", ESC_ARM_MS);
    armFlywheels();

    const float vmax = FLYWHEEL_TEST_PCT;
    LOG_I("TEST", "Rampe -> %.0f%%", (double)(vmax / 2));
    rampFlywheels(vmax / 2, 1000);
    wait(2000);

    LOG_I("TEST", "Rampe -> %.0f%%", (double)vmax);
    rampFlywheels(vmax, 1000);
    wait(2000);

    LOG_I("TEST", "Rampe -> 0%%");
    rampFlywheels(0.0f, 1500);
    stopFlywheels();
    LOG_I("TEST", "=== TEST LANCEUR TERMINE ===");
}

// ─── Lancer de balles ─────────────────────────────────────────────────────────
//  Vitesse réglable par roue : des vitesses différentes donnent de l'effet à la
//  balle et modifient sa trajectoire.
void launchBalls(float pctL, float pctR, uint32_t durationMs, float feedPct) {
    LOG_I("LAUNCH", "Lancer G=%.0f%% D=%.0f%% ventilo=%.0f%% pendant %lu ms",
          (double)pctL, (double)pctR, (double)feedPct, (unsigned long)durationMs);
    rampFlywheels(pctL, pctR, LAUNCH_SPINUP_MS);
    wait(LAUNCH_SETTLE_MS);
    feeder.setSpeed(feedPct);        // pousse les balles vers les roues
    wait(durationMs);
    feeder.stop();
    wait(LAUNCH_TAIL_MS);            // laisse sortir la dernière balle
    stopFlywheels();
}

// ─── Stratégie jaune ──────────────────────────────────────────────────────────
// Essai : carrière 1 → mur 2 → retour dans la cour → tir.
void runStrategyYellow(Robot &robot) {
    robot.enableObstacle();
    robot.setSpeedPct(50, 50);

    prendreCarriere(robot, POI::carriere_01, ANGLE_NORTH);
    deposerMur(robot, POI::murYellow_2);
    tirerDepuisCour(robot, Team::YELLOW);
}

// ─── Stratégie bleue — symétrique ─────────────────────────────────────────────
void runStrategyBlue(Robot &robot) {
    robot.enableObstacle();
    robot.setSpeedPct(50, 50);

    prendreCarriere(robot, POI::carriere_04, ANGLE_NORTH);
    deposerMur(robot, POI::murBlue_2);
    tirerDepuisCour(robot, Team::BLUE);
}

// ─── Repli fin de match jaune ─────────────────────────────────────────────────
void runNearEndYellow(Robot &robot) {
    stopFlywheels();
    robot.disableMotors();
}

// ─── Repli fin de match bleu ──────────────────────────────────────────────────
void runNearEndBlue(Robot &robot) {
    stopFlywheels();
    robot.disableMotors();
}
