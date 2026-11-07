#pragma once
#include <stdbool.h>
#include "../config.h"

void buttonsInit();

// true = tirette retirée (pin HIGH via pull-up externe).
// Anti-rebond : un changement n'est pris en compte qu'après TIRETTE_STABLE_MS.
bool tirette();

// Lit l'état du switch d'équipe : HIGH = YELLOW, LOW = BLUE
Team teamSwitch();

// true une seule fois par appui (front descendant, anti-rebond 20 ms)
bool initPressed();
