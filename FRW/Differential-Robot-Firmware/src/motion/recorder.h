#pragma once
#include <stdint.h>

// ─── Enregistreur de mouvement ───────────────────────────────────────────────
//
//  Un échantillon par cycle de la boucle motion (~50 Hz) pendant un mouvement,
//  stocké en RAM puis envoyé en bloc à l'interface une fois le mouvement fini
//  (graphe + export CSV dans l'onglet MOTION).
//
//  Écrit par la seule tâche motion (start / add / stop), lu par la tâche WiFi.
//  Un nouveau mouvement écrase le précédent : `gen` change, et la tâche WiFi
//  abandonne un envoi en cours (l'interface ignore un enregistrement incomplet).
//
//  Unités entières pour tenir 20 s dans ~24 Ko.

struct RecSample {
    uint16_t t;        // ms depuis le début du mouvement
    uint8_t  state;    // MotionController::State
    uint8_t  phase;    // MotionController::Phase
    int16_t  x, y;     // pose encodeurs (mm)
    int16_t  th;       // cap encodeurs (0,1°)
    int16_t  cmdL;     // vitesses commandées aux roues motrices (mm/s)
    int16_t  cmdR;
    int16_t  vL;       // vitesses mesurées aux roues codeuses (mm/s)
    int16_t  vR;
    int16_t  errD;     // erreur de distance (0,1 mm, bornée à ±3276 mm)
    int16_t  errA;     // erreur d'angle (0,01°)
    int16_t  cap;      // plafond de vitesse de la phase (mm/s)
};

#define REC_MAX_SAMPLES  1000   // 20 s à 50 Hz

class MotionRecorder {
public:
    // ── Tâche motion ─────────────────────────────────────────────────────────
    void start();                                // nouveau mouvement
    void add(const RecSample& s);                // ignoré si plein ou arrêté
    void stop();                                 // mouvement fini → prêt à envoyer
    bool recording() const { return _recording; }
    uint32_t startMs() const { return _t0; }

    // ── Tâche WiFi ───────────────────────────────────────────────────────────
    // Prêt à envoyer : gen de l'enregistrement, nombre d'échantillons.
    bool     ready() const     { return _ready; }
    uint32_t gen() const       { return _gen; }
    uint16_t count() const     { return _count; }
    bool     truncated() const { return _truncated; }
    const RecSample& at(uint16_t i) const { return _buf[i]; }
    void     markSent()        { _ready = false; }
    // Renvoi demandé par l'interface (onglet MOTION ouvert après le mouvement)
    void     resend()          { if (_count > 0 && !_recording) _ready = true; }

private:
    RecSample         _buf[REC_MAX_SAMPLES];
    volatile uint16_t _count = 0;
    volatile uint32_t _gen = 0;
    volatile bool     _recording = false;
    volatile bool     _ready = false;
    volatile bool     _truncated = false;
    uint32_t          _t0 = 0;
};

extern MotionRecorder gRecorder;

// Saturation vers int16 (les champs sont bornés plutôt que de déborder)
static inline int16_t recClamp16(float v) {
    if (v >  32767.0f) return  32767;
    if (v < -32767.0f) return -32767;
    return (int16_t)(v >= 0 ? v + 0.5f : v - 0.5f);
}
