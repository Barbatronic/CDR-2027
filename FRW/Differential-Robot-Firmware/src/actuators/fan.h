#pragma once
#include <stdint.h>

// Ventilateur PC 4 fils piloté en PWM (norme Intel : ~25 kHz, 0-100 %)
// directement par un GPIO de l'ESP32 via le périphérique LEDC.
// Attention : entrée PWM non pilotée (boot, reset) → le ventilateur part à fond.
// Selon le modèle, 0 % peut ne pas arrêter complètement le ventilateur.
class PwmFan {
public:
    PwmFan(uint8_t pin, uint8_t ledcChannel, uint32_t freqHz);

    void  begin();                // configure le LEDC et met le ventilateur à 0 %
    void  setSpeed(float pct);    // [0, 100] — clampé
    void  stop()                  { setSpeed(0.0f); }
    float getSpeed() const        { return _speedPct; }

private:
    uint8_t  _pin;
    uint8_t  _ch;
    uint32_t _freqHz;
    float    _speedPct = 0.0f;

    static constexpr uint8_t RES_BITS = 8;
};
