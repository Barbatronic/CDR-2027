#pragma once
#include "robot.h"
#include "../config.h"
#include "../motion/encoder.h"

// ─── Calibration géométrique ─────────────────────────────────────────────────
// Effectue go(1000) + turn(360), mesure les encodeurs et affiche les nouvelles
// valeurs DRIVE_WHEEL_DIAM_MM et WHEELBASE_MM sur le moniteur série.
void runCalibration(Robot &robot, QuadEncoder &encL, QuadEncoder &encR);

// ─── Calage bordure (à appeler avant la mise en place sur table) ─────────────
void runInitYellow(Robot &robot);
void runInitBlue(Robot &robot);

// ─── Tests ────────────────────────────────────────────────────────────────────
// Vérifie I2C (PCA9685, PCF8574, OLED), LIDAR, relève encodeurs / IO.
bool runSelfTest();
// Arme les ESC puis rampe les deux roues à inertie jusqu'à FLYWHEEL_TEST_PCT.
void runFlywheelTest();

// ─── Lanceur ──────────────────────────────────────────────────────────────────
// Monte les roues à inertie à pctL (canal 0) / pctR (canal 1) en LAUNCH_SPINUP_MS,
// attend LAUNCH_SETTLE_MS, lance le ventilateur à feedPct pendant durationMs,
// attend LAUNCH_TAIL_MS puis arrête tout. Bloquant.
// Vitesses de roues différentes → effet sur la balle.
void launchBalls(float pctL, float pctR, uint32_t durationMs = LAUNCH_DURATION_MS,
                 float feedPct = LAUNCH_FEED_PCT);

// ─── Stratégie de match ───────────────────────────────────────────────────────
void runStrategyYellow(Robot &robot);
void runStrategyBlue(Robot &robot);

// ─── Repli fin de match (déclenché à MATCH_ENDGAME_MS) ───────────────────────
void runNearEndYellow(Robot &robot);
void runNearEndBlue(Robot &robot);

