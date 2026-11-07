#pragma once
#include <math.h>

// ─── Profil de vitesse trapézoïdal ───────────────────────────────────────────
//
//  Donne, à chaque instant, la position et la vitesse attendues pour parcourir
//  une distance D ≥ 0 : accélération, palier à vmax, freinage jusqu'à l'arrêt
//  pile en D. Unités libres (mm et mm/s, ou rad et rad/s).
//
//      v ▲    ___________
//        │   /           \            v0 > 0 : départ en mouvement
//      v0├──/             \           (enchaînement de deux cibles)
//        │                 \
//        └──┴──────────────┴──▶ t
//           t1      t2      T
//
//  Cas particuliers :
//  • distance trop courte pour atteindre vmax → triangle (pic plus bas) ;
//  • départ au-dessus de vmax → palier à v0 (on ne freine pas pour rien) ;
//  • distance trop courte pour s'arrêter depuis v0 → freinage renforcé, borné
//    à 2 × a (au-delà, l'arrivée dépasse : la boucle corrige ensuite).
//
struct TrapProfile {
    float D  = 0;      // distance totale
    float v0 = 0;      // vitesse de départ
    float vm = 0;      // vitesse de palier (pic)
    float aA = 0;      // accélération de la première rampe (signée)
    float aD = 0;      // décélération finale (> 0)
    float t1 = 0, t2 = 0, T = 0;
    float s1 = 0, s2 = 0;

    void plan(float dist, float vmax, float acc, float vstart = 0.0f) {
        D = fmaxf(0.0f, dist);
        v0 = fmaxf(0.0f, vstart);
        acc = fmaxf(acc, 1e-3f);
        vmax = fmaxf(vmax, 1e-3f);
        aA = 0; aD = acc; t1 = t2 = T = 0; s1 = s2 = 0; vm = v0;
        if (D < 1e-6f) { D = 0; v0 = 0; vm = 0; return; }

        const float sStop0 = v0 * v0 / (2.0f * acc);   // freinage direct depuis v0
        if (v0 >= vmax || sStop0 >= D) {
            // Pas d'accélération : palier à v0 puis freinage (renforcé si besoin)
            vm = v0;
            if (sStop0 >= D) {
                aD = fminf(v0 * v0 / (2.0f * D), 2.0f * acc);
                const float sStop = v0 * v0 / (2.0f * aD);
                s2 = fmaxf(0.0f, D - sStop);
            } else {
                s2 = D - sStop0;
            }
            t2 = s2 / v0;
            T  = t2 + v0 / aD;
            // Freinage borné à 2a : l'arrivée réelle dépasse D (s(T) > D) ;
            // la position renvoyée est plafonnée à D, la boucle finit le travail.
            return;
        }

        // Accélération jusqu'à vmax, palier, freinage
        float vp = vmax;
        float sAcc = (vp * vp - v0 * v0) / (2.0f * acc);
        float sDec = vp * vp / (2.0f * acc);
        if (sAcc + sDec > D) {                           // triangle
            vp   = sqrtf((2.0f * acc * D + v0 * v0) * 0.5f);
            sAcc = (vp * vp - v0 * v0) / (2.0f * acc);
            sDec = D - sAcc;
        }
        vm = vp;
        aA = acc;
        t1 = (vp - v0) / acc;
        s1 = sAcc;
        s2 = D - sDec;
        t2 = t1 + (vp > 1e-6f ? (s2 - s1) / vp : 0.0f);
        T  = t2 + vp / acc;
    }

    // Position et vitesse attendues à l'instant t (s) depuis le départ
    void at(float t, float& s, float& v) const {
        if (t <= 0.0f) { s = 0; v = v0; return; }
        if (t < t1)    { v = v0 + aA * t; s = v0 * t + 0.5f * aA * t * t; return; }
        if (t < t2)    { v = vm; s = s1 + vm * (t - t1); return; }
        if (t < T)     {
            const float u = t - t2;
            v = vm - aD * u;
            s = fminf(D, s2 + vm * u - 0.5f * aD * u * u);
            return;
        }
        s = D; v = 0;
    }

    bool done(float t) const { return t >= T; }
};
