#pragma once
#include "pca9685.h"
#include "pcf8574.h"
#include "servo.h"
#include "esc.h"
#include "fan.h"
#include "../config.h"
#include "../utils.h"

// ═══════════════════════════════════════════════════════════════════════════════
//  ACTIONNEURS  —  déclarer ici les servos, ESC, pompes et séquences
// ═══════════════════════════════════════════════════════════════════════════════
//
//  Drivers matériel (instances globales) :
//    pca  → PCA9685 : 16 canaux PWM pour servomoteurs / ESC
//    pcf  → PCF8574 : 8 sorties/entrées numériques (pompes, capteurs...)
//
//  Déclarer un servomoteur :
//    extern Servo monServo;                  // dans actuators.h
//    Servo monServo(pca, {canal, minUs, maxUs, minDeg, maxDeg});  // dans actuators.cpp
//
//  Commandes servo :
//    monServo.setAngle(90);                  // va à 90° immédiatement
//    monServo.setPercent(50);                // va à 50% de la course
//    monServo.moveTo(90, 45);               // va à 90° à 45°/s (bloquant)
//    monServo.moveToPercent(100, 30);       // va à 100% à 30%/s (bloquant)
//    monServo.detach();                      // relâche le servo (plus de signal)
//
//  Déclarer un ESC :
//    Esc monEsc(pca, {canal, arretUs, pleinGazUs});
//    monEsc.setSpeed(50);                    // 50 % des gaz
//    monEsc.rampTo(80, 1000);                // rampe vers 80 % en 1 s (bloquant)
//    monEsc.stop();                          // arrêt (gaz mini)
//
//  PCF8574 :
//    pcf.setPin(0, false);                  // active la sortie 0 (LOW)
//    pcf.setPin(0, true);                   // désactive la sortie 0 (HIGH)
//    bool etat = pcf.getPin(3);             // lit l'entrée 3
//
// ═══════════════════════════════════════════════════════════════════════════════

// ─── Instances matériel ──────────────────────────────────────────────────────
extern PCA9685 pca;
extern PCF8574 pcf;

// ─── Init / Shutdown ─────────────────────────────────────────────────────────
bool actuatorsInit();
void actuatorsDisable();   // lanceur + ventilateur à l'arrêt, servos relâchés, PCF au repos

// ─── Lanceur : 2 roues à inertie (ESC) ───────────────────────────────────────
extern Esc flywheelLeft;
extern Esc flywheelRight;

// ─── Ventilateur d'alimentation des balles ──────────────────────────────────
extern PwmFan feeder;

// ─── Servomoteurs ─────────────────────────────────────────────────────────────
extern Servo servoBasGauche;   // canal 2
extern Servo servoBasDroit;    // canal 3

// ─── Séquences d'actionneurs ──────────────────────────────────────────────────
void initActuators();                               // position de repos
void armFlywheels();                                // gaz mini pendant ESC_ARM_MS (bloquant)
void setFlywheels(float pct);                       // même consigne sur les deux roues
void setFlywheels(float pctL, float pctR);          // consigne par roue (canal 0, canal 1)
void rampFlywheels(float pct, uint32_t durationMs); // rampe synchronisée (bloquant)
void rampFlywheels(float pctL, float pctR, uint32_t durationMs);
void stopFlywheels();
