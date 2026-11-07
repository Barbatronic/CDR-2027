# Audit déplacements — Novembre 2026

Audit de la chaîne de déplacement (`MotionController`, `StepControl`, odométrie
`taskEncoders`, API `Robot`) réalisé le 2026-11-07 sur le commit `bc832d3`.
Complète [audit_2026_10.md](audit_2026_10.md) sans en reprendre les points (renvois indiqués).
Les numéros de ligne correspondent à ce commit : ils peuvent avoir bougé depuis.

**Synthèse :** le matériel est bon. Encodeurs à 0,019 mm/incrément sur roues
libres, 200 Hz ; les steppers exécutent fidèlement une vitesse. La loi de commande
entre les deux est le facteur limitant : les déplacements sont **2 à 4× plus lents**
que ce que permet le matériel, et **le cap n'est pas corrigé pendant la translation**.

Cocher les cases au fur et à mesure.

---

## Constats

### 1. 🔴 Fin de mouvement en exponentielle (2–3 s perdues par phase)
- [ ] Corrigé

`_runWheelPidStep` : PID position par roue → consigne de vitesse, KP = 1,8.
Près de la cible, `1,8·e` passe sous la courbe de freinage `√(2·a·e)` dès
e < ~900 mm : la vitesse décroît en exponentielle (constante de temps ~0,5–0,8 s)
au lieu de freiner franchement.

Simulation (2000 mm/s, 1500 mm/s², rampe stepper incluse — modèle idéalisé,
ordres de grandeur seulement) :

| Mouvement     | Actuel | Trapèze idéal |
|---------------|--------|---------------|
| 100 mm        | 2,0 s  | 0,5 s         |
| 300 mm        | 2,7 s  | 0,9 s         |
| 1000 mm       | 3,4 s  | 1,6 s         |
| Rotation 90°  | 2,5 s  | 0,55 s        |

Un `gotoXYenc` = ALIGN + TRANSLATE + FINAL_TURN = ~8 s au lieu de ~3 s.
Monter KP ne règle pas le fond : la consigne est une position fixe, sans profil
temporel, et la rampe FastAccelStepper ajoute du retard dans la boucle.

- Où : [motion_ctrl.cpp:409-461](../src/motion/motion_ctrl.cpp#L409-L461)
- Piste : A (générateur de trajectoire + feedforward).

### 2. 🔴 Cap non corrigé en translation
- [ ] Corrigé

- ALIGN s'arrête à 2° ([motion_ctrl.cpp:230](../src/motion/motion_ctrl.cpp#L230)),
  et est sauté sous 2,9° ([motion_ctrl.cpp:218](../src/motion/motion_ctrl.cpp#L218)).
- TRANSLATE : chaque roue vise « +d counts », rien ne ramène vers le point XY.
- Écart latéral = d·sin(err) : 2° → 35 mm sur 1 m, 70 mm sur 2 m. Le mouvement
  est pourtant déclaré terminé sans vérifier l'arrivée XY.
- Deux PID indépendants par roue : un glissement est rattrapé en counts, mais le
  cap perdu ne l'est jamais.

- Piste : B (asservissement polaire distance + angle).

### 3. 🟠 Erreur finale jamais rattrapée
- [ ] Corrigé

Arrêt dès avg ≤ 5 mm / 2°, puis `softStop` : il reste ~4,9 mm à chaque mouvement
(simulation). KI = 0,01 n'a pas le temps d'agir. FINAL_TURN est sautée sous 0,5°
alors que son seuil d'arrêt est 2° : incohérent.

- Où : [motion_ctrl.cpp:284](../src/motion/motion_ctrl.cpp#L284), [motion_ctrl.cpp:426](../src/motion/motion_ctrl.cpp#L426)

### 4. 🟠 Mélange des empattements
- [ ] Corrigé — **voir audit_2026_10 point 10**

Erreurs mesurées sur roues codeuses (189 mm) envoyées aux roues motrices (145,6 mm) :
gain ×1,3 en rotation, freinage plus brutal que l'accélération réglée. Même défaut
dans `_runPoseStep` ([motion_ctrl.cpp:535](../src/motion/motion_ctrl.cpp#L535)).

### 5. 🟠 Deux systèmes de déplacement coexistent
- [ ] Corrigé

- `go`, `turn`, `gotoXY`, `goStall` : boucle ouverte, pose calculée sur les pas.
- `gotoXYenc`, `goPID`, `turnPID` : encodeurs.
- Détection obstacle de `Robot` sur pose steppers, celle de `MotionController` sur pose encodeurs.
- `_runPoseStep` (le contrôleur le plus abouti) n'est utilisé qu'en blend, donc
  uniquement pour les clics UI : **jamais en match**.

### 6. 🟡 Cadence de boucle
- [ ] Corrigé

`vTaskDelay(20)` après le calcul alors que `dt` est supposé = 20 ms : période réelle
21–22 ms, variable. La détection obstacle (scan LIDAR complet + trigo) tourne dans
la boucle, deux fois par cycle. `softStop()` et `_motion->go()` bloquent la tâche.

- Où : [motion_ctrl.cpp:97-191](../src/motion/motion_ctrl.cpp#L97-L191)

### 7. 🟡 Calibration odométrie
- [ ] Corrigé

- Entraxe roues codeuses 189,0 mm : valeur mesurée, pas étalonnée.
- Un seul diamètre pour les deux roues codeuses (un écart G/D de 0,1 % fait dériver le cap).
- Filtre PCNT à 12,5 µs : **voir audit_2026_10 (mineur)**.

---

## Pistes d'amélioration (par gain décroissant)

### A. Générateur de trajectoire + feedforward — gain principal
- [ ] Fait

Remplacer « consigne = position finale » par un profil trapézoïdal donnant
`x_ref(t)`, `v_ref(t)` à chaque cycle, puis `v = v_ref + Kp·(x_ref − x_mes)`.
Les steppers exécutent `v_ref` presque parfaitement ; l'encodeur ne corrige que le
résidu. Supprime l'exponentielle, autorise un Kp élevé sans dépassement.

### B. Asservissement polaire (distance + angle)
- [ ] Fait

Structure classique CDR (type RCVA) : boucle distance (somme des roues) + boucle
angle (différence). En translation, l'angle consigne est recalculé en continu vers
le point XY tant que dist > quelques cm, puis figé. Corrige le constat 2 et unifie
rotations, translations et courbes. Base existante : `_runPoseStep`, à généraliser
avec A.

### C. Calibration odométrie (UMBmark)
- [ ] Fait

- 10 tours sur place recalés bordure → entraxe roues codeuses.
- Aller-retour 2 m → diamètre moyen.
- Test du carré CW / CCW → séparer diamètre gauche / droit (ajouter `encWheelDiamL/R` dans `gCalib`).

~1 h, améliore toute la précision.

### D. Enchaînement sans arrêt (`followPath`)
- [ ] Fait

Prévu dans la roadmap (1.4). Utile une fois A + B en place : ~1 s gagnée par
point de passage intermédiaire.

### E. Comparaison pas commandés / encodeurs
- [ ] Fait

L'écart mesure directement glissement ou blocage :
- calage bordure plus fiable que le seuil 0,5 mm/poll de `goStall` ;
- détection de poussée / contact adversaire non vu par le LIDAR ;
- mesure de l'accélération max avant glissement (pousser 1500 mm/s² sans risque).

### F. Début d'accélération adouci
- [ ] Fait

`FastAccelStepper::setLinearAcceleration()` : limite glissement et basculement
à forte accélération.

### G. Hygiène de boucle
- [ ] Fait

- `vTaskDelayUntil` à 100 Hz, `dt` mesuré.
- Détection obstacle sortie de la boucle (tâche LIDAR qui publie un flag).
- Aucun appel bloquant dans la boucle.
- Une seule source de pose : les encodeurs.

---

## Ordre suggéré

1. C + correctif empattements (audit_2026_10 #10) : peu de code, fiabilise le reste.
2. A + B ensemble : réécriture de `MotionController` autour d'une seule loi ;
   `go` / `turn` / `gotoXY` / `goStall` basculent dessus.
3. E (calage, détection contact).
4. D, puis F.
