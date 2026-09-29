#include "fan.h"
#include <Arduino.h>

PwmFan::PwmFan(uint8_t pin, uint8_t ledcChannel, uint32_t freqHz)
    : _pin(pin), _ch(ledcChannel), _freqHz(freqHz) {}

void PwmFan::begin() {
    ledcSetup(_ch, _freqHz, RES_BITS);
    ledcAttachPin(_pin, _ch);
    setSpeed(0.0f);
}

void PwmFan::setSpeed(float pct) {
    if (pct > 100.0f) pct = 100.0f;
    if (pct <   0.0f) pct =   0.0f;
    _speedPct = pct;

    constexpr uint32_t MAX_DUTY = (1u << RES_BITS) - 1;
    ledcWrite(_ch, (uint32_t)(pct * MAX_DUTY / 100.0f + 0.5f));
}
