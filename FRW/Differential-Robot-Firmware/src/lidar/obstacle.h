#pragma once
#include <stdint.h>
#include "ld06.h"

// ─── Détection d'obstacle ────────────────────────────────────────────────────
//
//  Évaluée dans la tâche LIDAR (cœur 0), hors de la boucle d'asservissement :
//  la commande de mouvement et goStall demandent une surveillance dans une
//  direction (repère table), puis lisent simplement le résultat.
//
//  Zone surveillée : rectangle de OBS_WIDTH_MM de large, de OBS_MIN_DIST_MM à
//  OBS_DETECT_DIST_MM devant le robot dans la direction demandée.
//  WALL_FILTERED ignore en plus les points situés dans la marge des bordures
//  (demande une pose recalée).
//
//  Un résultat n'est rendu qu'après une évaluation postérieure à la demande :
//  changer de direction ne laisse pas traîner l'ancien résultat.

enum class ObstacleMode : uint8_t { SIMPLE, WALL_FILTERED };

void obstacleSetMode(ObstacleMode m);
ObstacleMode obstacleMode();

// Surveille la direction dir_rad (repère table, comme le cap du robot)
void obstacleWatch(float dir_rad);
void obstacleUnwatch();
// Obstacle présent dans la direction surveillée (faux sans surveillance active
// ou tant que la première évaluation n'a pas eu lieu)
bool obstacleDetected();

// Tâche LIDAR : évalue la demande en cours (au plus toutes les OBS_POLL_MS)
void obstacleUpdate(LD06& lidar);
