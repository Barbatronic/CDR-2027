#include "esc.h"
#include <Arduino.h>

Esc::Esc(PCA9685 &pca, EscConfig cfg) : _pca(pca), _cfg(cfg) {}

void Esc::setSpeed(float pct) {
    if (pct > 100.0f) pct = 100.0f;
    if (pct <   0.0f) pct =   0.0f;
    _speedPct = pct;

    float us = (float)_cfg.minUs + pct * (float)(_cfg.maxUs - _cfg.minUs) / 100.0f;
    _pca.setPulseUs(_cfg.channel, (uint16_t)(us + 0.5f));
}

void Esc::rampTo(float pct, uint32_t durationMs) {
    constexpr uint32_t STEP_MS = 20;   // 1 période PWM à 50 Hz
    float    start = _speedPct;
    uint32_t steps = durationMs / STEP_MS;
    for (uint32_t i = 1; i <= steps; i++) {
        setSpeed(start + (pct - start) * (float)i / (float)steps);
        vTaskDelay(pdMS_TO_TICKS(STEP_MS));
    }
    setSpeed(pct);
}

void Esc::detach() {
    _pca.off(_cfg.channel);
    _speedPct = 0.0f;
}
