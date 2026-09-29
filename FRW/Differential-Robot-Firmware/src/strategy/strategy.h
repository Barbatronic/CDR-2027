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

// ─── Actions de jeu ──────────────────────────────────────────────────────────
// Angle (°, repère table) pour regarder `to` depuis `from`.
float angleVers(Vec2 from, Vec2 to);

// Prise d'une carrière (pile de 3 pierres couchées) avec les bras bas.
// Déposée telle quelle sur une zone de mur, la pile forme un mur valide.
// approachDeg : orientation du robot face à la carrière (ANGLE_NORTH pour les
// carrières du fond, ANGLE_SOUTH côté public, EAST/WEST pour celles du centre).
// offsetMm    : distance avant du robot ↔ face de la carrière avant d'avancer.
void prendreCarriere(Robot &robot, Vec2 carriere, float approachDeg,
                     float offsetMm = CARRIERE_APPROCHE_MM);

// Dépose des pierres tenues sur une zone de mur (POI::murYellow_x / murBlue_x).
// L'orientation est déduite du mur ; le robot approche depuis l'extérieur du
// château, face à la cour. Retourne false si le POI n'est pas un mur connu.
bool deposerMur(Robot &robot, Vec2 mur);

// Retour dans la salle du trône par l'entrée du château (mur 3), orienté pour
// tirer par le flanc gauche vers l'adversaire, puis tir.
void tirerDepuisCour(Robot &robot, Team team);

// ─── Stratégie de match ───────────────────────────────────────────────────────
void runStrategyYellow(Robot &robot);
void runStrategyBlue(Robot &robot);

// ─── Repli fin de match (déclenché à MATCH_ENDGAME_MS) ───────────────────────
void runNearEndYellow(Robot &robot);
void runNearEndBlue(Robot &robot);

