// ═══════════════════════════════════════════════════════════════════════════════
//  ACTIONNEURS  —  définir ici les instances et les séquences
// ═══════════════════════════════════════════════════════════════════════════════

#include "actuators.h"
#include <Arduino.h>
#include <math.h>

// ─── Instances matériel ──────────────────────────────────────────────────────

PCA9685 pca(PCA9685_I2C_ADDR);
PCF8574 pcf(PCF8574_I2C_ADDR);

// ─── Lanceur ──────────────────────────────────────────────────────────────────
// {canal, arrêtUs, pleinGazUs}
Esc flywheelLeft (pca, {FLYWHEEL_L_CHANNEL, ESC_MIN_US, ESC_MAX_US});
Esc flywheelRight(pca, {FLYWHEEL_R_CHANNEL, ESC_MIN_US, ESC_MAX_US});

// ─── Ventilateur d'alimentation ──────────────────────────────────────────────
PwmFan feeder(FEEDER_PWM_PIN, FEEDER_LEDC_CHANNEL, FEEDER_PWM_FREQ_HZ);

// ─── Servomoteurs ─────────────────────────────────────────────────────────────
// {canal, minUs, maxUs, minDeg, maxDeg} — inverser minUs/maxUs pour un servo monté en miroir
// Aucun signal au boot : le servo ne bouge qu'à la première commande.
Servo servoBasGauche(pca, {2, 500, 2500, 0, 180});   // bas gauche
Servo servoBasDroit (pca, {3, 500, 2500, 0, 180});   // bas droit

// ─── Init ────────────────────────────────────────────────────────────────────

bool actuatorsInit() {
    bool ok = true;
    ok &= pca.begin(50.0f);   // 50 Hz standard servo / ESC
    ok &= pcf.begin(0xFF);    // toutes sorties HIGH (repos)
    stopFlywheels();          // gaz mini dès le boot → les ESC peuvent s'armer
    return ok;
}

void actuatorsDisable() {
    feeder.stop();
    stopFlywheels();          // gaz mini plutôt que coupure : arrêt franc des ESC
    servoBasGauche.detach();
    servoBasDroit.detach();
    pcf.writeByte(0xFF);      // toutes sorties PCF8574 au repos (HIGH)
}

// ─── Séquences ───────────────────────────────────────────────────────────────

void initActuators() {
    feeder.stop();
    stopFlywheels();
    ouvrirBras();             // bras ouverts : sinon ils butent contre les bordures
}

void armFlywheels() {
    stopFlywheels();
    wait(ESC_ARM_MS);
}

void setFlywheels(float pct) {
    setFlywheels(pct, pct);
}

void setFlywheels(float pctL, float pctR) {
    flywheelLeft.setSpeed(pctL);
    flywheelRight.setSpeed(pctR);
}

void rampFlywheels(float pct, uint32_t durationMs) {
    rampFlywheels(pct, pct, durationMs);
}

void rampFlywheels(float pctL, float pctR, uint32_t durationMs) {
    constexpr uint32_t STEP_MS = 20;
    float    startL = flywheelLeft.getSpeed();
    float    startR = flywheelRight.getSpeed();
    uint32_t steps  = durationMs / STEP_MS;
    for (uint32_t i = 1; i <= steps; i++) {
        float k = (float)i / (float)steps;
        setFlywheels(startL + (pctL - startL) * k, startR + (pctR - startR) * k);
        wait(STEP_MS);
    }
    setFlywheels(pctL, pctR);
}

void stopFlywheels() {
    setFlywheels(0.0f);
}

// ─── Bras bas ────────────────────────────────────────────────────────────────

// Déplacement simultané interpolé des deux bras.
static void rampBras(float pctG, float pctD, float pctPerSec) {
    constexpr uint32_t STEP_MS = 20;
    float startG = servoBasGauche.getPercent();
    float startD = servoBasDroit.getPercent();
    float dist   = fmaxf(fabsf(pctG - startG), fabsf(pctD - startD));
    uint32_t steps = (uint32_t)(dist / pctPerSec * 1000.0f / STEP_MS);
    for (uint32_t i = 1; i <= steps; i++) {
        float k = (float)i / (float)steps;
        servoBasGauche.setPercent(startG + (pctG - startG) * k);
        servoBasDroit.setPercent(startD + (pctD - startD) * k);
        wait(STEP_MS);
    }
    servoBasGauche.setPercent(pctG);
    servoBasDroit.setPercent(pctD);
}

void moveBras(float pctG, float pctD, float pctPerSec) {
    float curG  = servoBasGauche.getPercent();
    float curD  = servoBasDroit.getPercent();
    bool  known = !isnanf(curG) && !isnanf(curD);   // NAN = jamais commandé

    if (known) {                             // cas normal : les deux ensemble
        rampBras(pctG, pctD, pctPerSec);
        return;
    }

    // Position inconnue : le servo saute à la cible → l'un après l'autre
    bool gFirst = BRAS_INIT_G_EN_PREMIER;
    Servo &first  = gFirst ? servoBasGauche : servoBasDroit;
    Servo &second = gFirst ? servoBasDroit  : servoBasGauche;
    first.moveToPercent(gFirst ? pctG : pctD, pctPerSec);
    wait(BRAS_SEQ_DELAY_MS);
    second.moveToPercent(gFirst ? pctD : pctG, pctPerSec);
    wait(BRAS_SEQ_DELAY_MS);
}

void ouvrirBras()  { moveBras(BRAS_G_OUVERT, BRAS_D_OUVERT); }
void prendreBloc() { moveBras(BRAS_G_PRISE,  BRAS_D_PRISE);  }
void reposBras()   { moveBras(BRAS_G_REPOS,  BRAS_D_REPOS);  }
