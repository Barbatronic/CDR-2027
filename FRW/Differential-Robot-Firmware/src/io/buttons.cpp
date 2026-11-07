#include "buttons.h"
#include "../config.h"
#include <Arduino.h>

void buttonsInit() {
    pinMode(BTN_TIRETTE_PIN, INPUT);         // pull-up externe
    pinMode(BTN_TEAM_PIN,   INPUT_PULLUP);
    pinMode(BTN_INIT_PIN,   INPUT_PULLUP);
}

// État validé seulement après TIRETTE_STABLE_MS sans changement, à
// l'insertion comme au retrait : un rebond ou un parasite ne démarre pas le
// match. Lecture à appeler régulièrement (boucle pré-match à 20 ms).
bool tirette() {
    static bool     stable  = digitalRead(BTN_TIRETTE_PIN) == HIGH;
    static bool     last    = stable;
    static uint32_t since   = millis();

    bool     raw = digitalRead(BTN_TIRETTE_PIN) == HIGH;  // HIGH = retirée
    uint32_t now = millis();
    if (raw != last) {
        last  = raw;
        since = now;
    } else if (raw != stable && now - since >= TIRETTE_STABLE_MS) {
        stable = raw;
    }
    return stable;
}

// Retourne true sur le premier appel détectant un front descendant
// (bouton pressé), puis false jusqu'au relâchement. Anti-rebond 20 ms.
static bool _fallingEdge(uint8_t pin, bool &held, uint32_t &lastMs) {
    bool low = (digitalRead(pin) == LOW);
    uint32_t now = millis();
    if (low && !held && (now - lastMs >= 20)) {
        held   = true;
        lastMs = now;
        return true;
    }
    if (!low) held = false;
    return false;
}

Team teamSwitch() {
    return (digitalRead(BTN_TEAM_PIN) == HIGH) ? Team::YELLOW : Team::BLUE;
}

bool initPressed() {
    static bool     held = false;
    static uint32_t t    = 0;
    return _fallingEdge(BTN_INIT_PIN, held, t);
}
