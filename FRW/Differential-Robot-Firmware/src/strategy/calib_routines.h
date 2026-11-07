#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "robot.h"
#include "../motion/encoder.h"

// ─── Routines de calibration de l'odométrie (lot 2) ──────────────────────────
//
//  Lancées depuis l'onglet CALIBRATION, uniquement en attente d'init. La
//  référence est toujours un contact franc contre une bordure (recul jusqu'au
//  blocage) : le robot y est plaqué à plat, son cap réel est donc connu.
//
//  Entre deux contacts, l'erreur de cap de l'odométrie suit (au 1er ordre) :
//
//      e = Δθ_réel − Δθ_odom = u · R + v · S
//
//      R = rotation odométrique totale (rad), S = avance odométrique (mm)
//      u = erreur d'échelle des rotations   → entraxe codeuses / (1 + u)
//      v = (e_droite − e_gauche) / entraxe  → rapport D/G × (1 + v · entraxe)
//
//  Chaque routine envoie une observation {R, S, e} ; l'interface résout u et v
//  sur l'ensemble (tours sur place : R grand, S ≈ 0 ; carrés : S grand).
//
//  LINE    : recul contre une bordure puis ligne droite de `param` mm ; la
//            distance réelle est mesurée au mètre ruban → diamètre moyen.
//  TURNS   : recul, avance de 200 mm, `param` tours moteurs en boucle ouverte,
//            sens alterné à chaque essai, recul direct → u, et entraxe moteur.
//  SQUARE  : calage coin nord-ouest (placement), contact bordure ouest, carré
//            de `param` mm, retour contre la même bordure ouest → observation
//            pour u et v (faire les deux sens). Mesure indépendante de
//            l'équerrage de la table.

enum class CalibRoutine : uint8_t { LINE, TURNS, SQUARE_CW, SQUARE_CCW };

struct CalibRunReq {
    CalibRoutine id;
    float        param;
};

// Depuis la tâche WiFi : met la demande en file (refusée hors attente d'init)
bool requestCalibRun(CalibRoutine id, float param);
// Arrêt des moteurs depuis l'interface : la routine en cours s'interrompt
void calibAbort();

// Tâche stratégie, boucle d'attente d'init : exécute une demande en attente.
// Retourne true si une routine a été exécutée.
bool pollCalibRun(Robot &robot, QuadEncoder &encL, QuadEncoder &encR);
