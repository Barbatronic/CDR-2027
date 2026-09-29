// ═══════════════════════════════════════════════════════════════════════════════
//  ACTIONNEURS  —  définir ici les instances et les séquences
// ═══════════════════════════════════════════════════════════════════════════════

#include "actuators.h"
#include <Arduino.h>

// ─── Instances matériel ──────────────────────────────────────────────────────

PCA9685 pca(PCA9685_I2C_ADDR);
PCF8574 pcf(PCF8574_I2C_ADDR);

// ─── Lanceur ──────────────────────────────────────────────────────────────────
// {canal, arrêtUs, pleinGazUs}
Esc flywheelLeft (pca, {FLYWHEEL_L_CHANNEL, ESC_MIN_US, ESC_MAX_US});
Esc flywheelRight(pca, {FLYWHEEL_R_CHANNEL, ESC_MIN_US, ESC_MAX_US});

// ─── Ventilateur d'alimentation ──────────────────────────────────────────────
PwmFan feeder(FEEDER_PWM_PIN, FEEDER_LEDC_CHANNEL, FEEDER_PWM_FREQ_HZ);

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
    pcf.writeByte(0xFF);      // toutes sorties PCF8574 au repos (HIGH)
}

// ─── Séquences ───────────────────────────────────────────────────────────────

void initActuators() {
    feeder.stop();
    stopFlywheels();
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
