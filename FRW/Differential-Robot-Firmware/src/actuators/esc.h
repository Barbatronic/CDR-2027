#pragma once
#include "pca9685.h"

// ESC unidirectionnel piloté comme un servo (50 Hz) via le PCA9685.
//     0 % = minUs → moteur arrêté (aussi utilisé pour l'armement)
//   100 % = maxUs → plein gaz
struct EscConfig {
    uint8_t  channel;     // canal PCA9685 (0-15)
    uint16_t minUs;       // impulsion arrêt (µs)
    uint16_t maxUs;       // impulsion plein gaz (µs)
};

class Esc {
public:
    Esc(PCA9685 &pca, EscConfig cfg);

    void  setSpeed(float pct);    // [0, 100] — clampé
    void  stop()                  { setSpeed(0.0f); }
    float getSpeed() const        { return _speedPct; }

    // Rampe linéaire bloquante de la vitesse courante vers pct en durationMs
    void  rampTo(float pct, uint32_t durationMs);

    void  detach();               // coupe le signal PWM (ESC en failsafe)

    // Verrou commun à tous les ESC : verrouillés, toute consigne devient 0 %.
    // Posé par le chrono de match à 100 s, levé à la relance d'un match.
    static void setLocked(bool locked) { _locked = locked; }
    static bool isLocked()             { return _locked; }

private:
    PCA9685  &_pca;
    EscConfig _cfg;
    float     _speedPct = 0.0f;
    static volatile bool _locked;
};
