# Audit interface web — Novembre 2026

Audit de l'interface de debug embarquée (`data/index.html`, `data/poi.js`,
`src/wifi/log_server.*`, `src/log.h`, appels depuis `main.cpp`) réalisé le
2026-11-07 sur le commit `bc832d3`.
Complète [audit_2026_10.md](audit_2026_10.md) sans en reprendre les points (renvois indiqués).
Les numéros de ligne correspondent à ce commit : ils peuvent avoir bougé depuis.

**Synthèse :** l'interface est riche et bien organisée (terrain, goto au clic,
calibration live, logs filtrables). Les problèmes sont surtout : un mode
« WiFi désactivé » qui ne compile plus, une calibration modifiable pendant le match,
des constantes dupliquées entre C++ et JS qui ont déjà divergé, et un radar dont
les statistiques avant/arrière regardent les mauvais côtés.

Cocher les cases au fur et à mesure.

---

## 🔴 Critique

### 1. `WIFI_LOG_ENABLED 0` ne compile pas
- [ ] Corrigé

Vérifié par compilation d'une copie avec `WIFI_LOG_ENABLED 0` :

```
src/wifi/log_server.h:121:13: error: redefinition of 'void wifiLogBegin()'
src/main.cpp:311:9: error: 'wifiLogLidarAbs' was not declared in this scope
```

Le mode « compétition — zéro overhead WiFi » annoncé dans `config.h` est donc
inutilisable : en match, on est obligé de garder le WiFi.

- Où : [log.h:20-23](../src/log.h#L20-L23), [log_server.h:119-130](../src/wifi/log_server.h#L119-L130)
- Correctif : un seul jeu de stubs (dans `log_server.h`), ajouter `wifiLogLidarAbs`,
  supprimer la déclaration périmée à 7 arguments de `wifiLogUpdatePose` dans `log.h`.
  Ajouter ce cas à l'environnement `[env:match]` proposé dans audit_2026_10 #6 pour
  qu'il soit compilé régulièrement.

### 2. Calibration modifiable à tout moment, sans validation
- [ ] Corrigé

Contrairement aux actionneurs (verrouillés hors WAIT_INIT) et au goto (verrouillé
pendant le match), `_handleCalibMsg` accepte toute valeur, à tout moment, y compris
en plein match. Les `min`/`max` HTML ne sont pas appliqués sur `onchange` (une valeur
tapée hors plage est envoyée). Diamètre à 0 → division par zéro (déjà noté dans
audit_2026_10, mineur).

- Où : [log_server.cpp:417-452](../src/wifi/log_server.cpp#L417-L452)
- Correctif : refuser hors pré-match, valider les plages côté firmware, renvoyer
  l'état réel (`calib_state`) après un refus.

### 3. WebSocket ouvert à toute page web
- [ ] Corrigé — **auth : voir audit_2026_10 #6**

En plus de l'absence d'authentification, le serveur ne vérifie pas l'en-tête
`Origin` : n'importe quelle page ouverte dans un navigateur connecté au réseau du
robot peut ouvrir `ws://192.168.4.1:81` et envoyer `start_match`, `goto`, `calib`.

- Correctif : vérifier `Origin` à la connexion + jeton simple dans l'URL de la page.

---

## 🟠 Important

### 4. Radar : stats « Avant » / « Arrière » sur les mauvais côtés
- [ ] Corrigé

Le firmware envoie l'angle **brut** du LD06. Le tracé est juste (avant = haut),
mais les stats testent l'angle brut :
- « Avant (±30°) » (brut ≈ 0°) mesure en réalité le **flanc droit** du robot
  (repère robot = 270 − brut) ;
- « Arrière (±30°) » (brut ≈ 180°) mesure le **flanc gauche**.

`lidarOffsetDeg` est ignoré, les zones aveugles ne sont pas affichées, et le
commentaire de convention ([index.html:609](../data/index.html#L609)) est faux.

- Où : [index.html:658-677](../data/index.html#L658-L677), [log_server.cpp:241-243](../src/wifi/log_server.cpp#L241-L243)
- Correctif : envoyer l'angle en **repère robot** depuis le firmware
  (`270 − a + offset`), une seule convention partout.

### 5. Constantes dupliquées C++ / JS (déjà divergées)
- [ ] Corrigé

| JS | Valeur JS | Source C++ | Valeur réelle |
|---|---|---|---|
| `OBS_DETECT_MM` | 400 | `OBS_DETECT_DIST_MM` | **500** |
| `CALIB_DEFAULTS` | copie | `config.h` | à resynchroniser à chaque changement |
| `MOT_STATE` / `MOT_PHASE` | tableaux | enums `MotionController` | ordre à maintenir |
| `canControl()` | `state === 'APPUYER INIT'` | texte OLED | changer le texte de l'écran casse le déverrouillage des actionneurs |

« Reset défauts » renverra des valeurs périmées dès que `config.h` évolue.

- Correctif : même principe que `poi.js` — le firmware envoie à la connexion un
  message `config` (défauts calib, dimensions détection, zones aveugles, enums), ou
  `sync_poi.py` génère aussi un `config.js` depuis `config.h`. Le verrou actionneurs
  doit utiliser un booléen envoyé par le firmware (comme `cc`), pas un libellé.

### 6. Trames partagées entre cœurs sans protection
- [ ] Corrigé — **LIDAR : voir audit_2026_10 (mineur)**

`loop()` (cœur 1) écrit `_lidarJson`, `_lidarAbsJson`, `_pstate` pendant que la tâche
WiFi (cœur 0) les envoie → trames tronquées ou mélangées, ignorées par
`JSON.parse`. Même course sur `pollFieldClick` (x/y/θ lus pendant qu'un nouveau
clic les écrit).

- Correctif : double tampon avec index atomique, ou `portMUX` autour de la copie.

### 7. Charge inutile pendant le match
- [ ] Corrigé

`loop()` calcule le JSON `lidar_abs` (trigo + `snprintf` sur ~150 points) toutes
les 100 ms **même sans client connecté**, sur le cœur 1 (celui de la tâche motion),
puis la tâche WiFi diffuse ~5 Ko × 10/s.

- Où : [main.cpp:311-313](../src/main.cpp#L311-L313)
- Correctif : ne rien construire si `_ws.connectedClients() == 0` ; en match,
  réduire la fréquence ou couper.

### 8. Pas de détection de données figées
- [ ] Corrigé

Si le WiFi tombe sans fermeture propre, le navigateur peut rester « Connecté »
longtemps avec une pose figée. Rien n'indique l'âge des données.

- Correctif : afficher l'âge de la dernière trame `pose` (rouge si > 1 s) ; ping
  applicatif périodique.

### 9. Inutilisable sur téléphone / tablette
- [ ] Corrigé

Colonne latérale fixe de 440 px, `overflow: hidden`, goto uniquement à la souris
(pas d'événements tactiles). Au stand ou au bord de la table, le téléphone est
pourtant l'appareil naturel.

- Correctif : disposition en une colonne sous ~900 px, Pointer Events à la place
  des événements souris.

---

## 🟡 Mineur

- [ ] **Verrou actionneurs** : `lock.textContent = '&#x1F513; …'` affiche l'entité
  en clair au lieu de l'icône ([index.html:1176](../data/index.html#L1176)).
- [ ] **Logs, DOM jamais purgé** : le tableau est limité à 800 entrées mais les
  lignes HTML s'accumulent sans fin → page de plus en plus lente sur une longue session.
- [ ] **Logs, JSON fragile** : seuls les `"` sont échappés (pas `\`, ni les
  caractères de contrôle), et le tampon de 140 octets peut tronquer (jusqu'à 149
  octets avec tag de 15 caractères) → ligne silencieusement perdue
  ([log_server.cpp:329-332](../src/wifi/log_server.cpp#L329-L332)).
- [ ] **Sliders** : `oninput` envoie à chaque pixel ; la file de commandes (8)
  déborde silencieusement et la dernière valeur peut être perdue. Limiter la
  cadence et envoyer la valeur finale sur `change`.
- [ ] **Goto non borné côté firmware** : une cible hors table (macro FreeCAD,
  client tiers) est acceptée. Borner aux limites table − demi-largeur robot.
- [ ] **Robot dessiné comme un point de 8 px** : dessiner l'emprise réelle
  (avant/arrière/largeur) pour juger les passages serrés (cf. audit_2026_10 #8).
- [ ] **Animation du clic** : chaque message reçu pendant le flash lance une
  nouvelle chaîne `requestAnimationFrame` (rendus multipliés pendant 600 ms).
- [ ] **Code mort** : `needRender` inutilisé.
- [ ] **HTTP** : pas d'en-tête de cache sur `field.jpg` (195 Ko renvoyés à chaque
  rechargement).
- [ ] **Calibration en RAM uniquement** : pas de sauvegarde (NVS) ni d'export
  « copier pour config.h ».

---

## Pistes d'amélioration

Utiles en priorité pour le travail sur les déplacements
([audit_motion_2026_11.md](audit_motion_2026_11.md)) :

- [ ] **Télémétrie motion à la cadence de la boucle** : aujourd'hui 5 Hz pour une
  boucle à 50 Hz, trop lent pour régler un asservissement. Enregistrer chaque cycle
  dans un tampon pendant le mouvement, puis l'envoyer en un bloc à la fin.
- [ ] **Graphe temps réel** (roadmap 2.5) : consigne vs mesure (vitesse, erreur
  distance, erreur angle) par mouvement.
- [ ] **Trace de trajectoire** : traînée des poses passées sur le terrain, superposée
  à la ligne robot → cible.
- [ ] **Export CSV** d'un mouvement ou d'une session.
- [ ] **Boutons de calibration** : lancer depuis l'UI les routines UMBmark
  (N tours sur place, aller-retour 2 m, carré CW/CCW).
- [ ] **Source unique de configuration** (cf. point 5).

---

## Ordre suggéré

1. Stubs `WIFI_LOG_ENABLED 0` (1) : quelques lignes, débloque le mode match.
2. Verrou et validation de la calibration (2), angle LIDAR en repère robot (4).
3. Message `config` à la connexion pour supprimer les doublons JS (5).
4. Double tampon (6), pas de JSON sans client (7), âge des données (8).
5. Télémétrie haute cadence + graphe, en même temps que la refonte motion.
6. Version mobile (9) et points mineurs.
