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

// ─── Calage bordure jaune ─────────────────────────────────────────────────────
void runInitYellow(Robot &robot) {
    robot.enableMotors();
    robot.disableObstacle();
    initActuators();

    robot.setSpeedPct(10); // Change la vitesse et accel pour le calage, afin de limiter les risques de rebonds

    // ── Calage X — plaquage contre la bordure Ouest ────────────────────────
    robot.setPosition(0, 0, ANGLE_EAST);
    robot.goStall(-150);                 // recule jusqu'au contact de la bordure Ouest
    robot.setPosition(ROBOT_BACK_TO_CENTER_MM, 0, ANGLE_EAST);   // X calé
    robot.goPID(375-ROBOT_BACK_TO_CENTER_MM);                       // dégage

    // ── Calage Y — plaquage contre la bordure Nord ─────────────────────────
    robot.turnPID(-90.0f);               // s'oriente vers le Sud
    robot.goStall(-150);                 // recule jusqu'au contact de la bordure Nord
    robot.setPosition(robot.getX(), ROBOT_BACK_TO_CENTER_MM, ANGLE_SOUTH);  // Y calé, X inchangé
    robot.goPID(225);                       // dégage

    // ── Position de départ ─────────────────────────────────────────────────
    robot.gotoXYenc(POI::startYellow,283);

    robot.resetSpeed(); // remet les vitesses par défaut pour la stratégie
}

// ─── Calage bordure bleu ──────────────────────────────────────────────────────
void runInitBlue(Robot &robot) {
    robot.enableMotors();
    robot.disableObstacle();
    initActuators();

    robot.setSpeedPct(10);

    // ── Calage X — plaquage contre la bordure Est ──────────────────────────
    robot.setPosition(TABLE_WIDTH_MM, 0, ANGLE_WEST);
    robot.goStall(-150);                  // recule (Est) contre la bordure Est
    robot.setPosition(TABLE_WIDTH_MM - ROBOT_BACK_TO_CENTER_MM, 0, ANGLE_WEST);
    robot.goPID(TABLE_WIDTH_MM - ROBOT_BACK_TO_CENTER_MM - POI::startBlue.x);  // dégage vers l'Ouest

    // ── Calage Y — plaquage contre la bordure Nord ─────────────────────────
    robot.turnPID(90.0f);                 // s'oriente vers le Sud (180°→270°)
    robot.goStall(-150);                  // recule (Nord) contre la bordure Nord
    robot.setPosition(robot.getX(), ROBOT_BACK_TO_CENTER_MM, ANGLE_SOUTH);
    robot.goPID(POI::startBlue.y);        // dégage vers le Sud

    // ── Position de départ ─────────────────────────────────────────────────
    robot.gotoXYenc(POI::startBlue, 257);

    robot.resetSpeed();
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
// TODO 2027 : stratégie de match. Pour l'instant : lancer de test sur place.
void runStrategyYellow(Robot &robot) {
    launchBalls(30, 30);
}

// ─── Stratégie bleue ──────────────────────────────────────────────────────────
void runStrategyBlue(Robot &robot) {
    launchBalls(30, 30);
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
