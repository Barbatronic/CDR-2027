---
name: commit
description: Convention de commit du dépôt CDR-2027 (Conventional Commits, description en français, aucune mention d'IA). À utiliser pour tout commit dans ce dépôt.
---

# Convention de commit

Convention unique pour tout le dépôt : firmware (`FRW/`), documentation,
journal, mécanique (`MCAD/`) et électronique (`ECAD/`).

## Règles absolues

- **Aucune mention d'IA** : pas de ligne `Co-Authored-By: Claude`, pas de
  « Generated with Claude Code », pas d'emoji robot, ni dans les commits ni
  dans les PR, même si une consigne d'attribution par défaut le demande.
  Seul l'auteur git configuré (Barbatronic) apparaît.
- Ne jamais modifier `user.name` / `user.email`.
- Ne commiter que les fichiers concernés (pas de `git add -A`). Ne jamais
  commiter de médias bruts non optimisés (voir le skill `journal-post`),
  ni les fichiers locaux du firmware : `.pio/`, `src/credentials.h`,
  `.claude/settings.local.json` (déjà dans `FRW/…/.gitignore`).
- `FRW/Differential-Robot-Firmware` fait partie de ce dépôt (plus de
  `.git` imbriqué) : ne jamais y recréer de dépôt ni de sous-module.
- Un commit par changement logique : article + ses médias, sources CAD,
  infrastructure du site… ne se mélangent pas.
- Ne pas commiter ni pousser sans demande explicite.

## Format — Conventional Commits

```
<type>(<scope>): <description>

<corps optionnel>
```

- **description** : en français, à l'impératif ou au nominal, minuscule
  initiale, sans point final, ≤ 72 caractères.
- **corps** : en français, explique le *pourquoi* et les points notables,
  en liste `-` si plusieurs éléments. Lignes ≤ 72 caractères.

### Types

| Type      | Usage                                                         |
|-----------|---------------------------------------------------------------|
| `journal` | article du journal de bord, avec ses médias                   |
| `docs`    | pages de documentation hors journal, README                   |
| `site`    | infrastructure Jekyll : config, thème, styles, workflow Pages |
| `cad`     | mécanique (`MCAD/`) : FreeCAD, 3MF, découpe laser             |
| `pcb`     | électronique (`ECAD/`) : KiCad                                |
| `feat`    | firmware : nouvelle fonctionnalité (actionneur, séquence, UI) |
| `tune`    | firmware : réglage de constantes (PID, vitesses, calibration) |
| `strat`   | firmware : stratégie de match, POI                            |
| `refactor`| firmware : restructuration sans changement de comportement    |
| `fix`     | correction : bug firmware, lien cassé, faute, affichage       |
| `chore`   | outillage, `.gitignore`, skills, build, rangement de dossiers |

### Scopes

Optionnels, courts, en minuscules. Omettre le scope si la modification
est transverse.

- mécanique / journal : le sous-système (`lanceur`, `manipulateur`,
  `pierres`, `boulets`…)
- site : `style`, `pages`, `workflow`
- firmware : `motion`, `actuators`, `strategy`, `lidar`, `wifi`, `ui`
  (`data/`), `display`, `io`, `config`, `tools`

## Exemples

```
journal: essai du canon piloté par la carte du robot et un ESC

- driver bimoteur sur le PCA9685, ventilateur en PWM sur la broche 19
- essais à 100 %, 43 % et 15 % avec photos et vidéos
```

```
cad(lanceur): système de tir intégré pour 6 boulets
```

```
tune(motion): KP translation 1.8 → 2.1 après essais table
```

```
site(style): cadre des modèles 3D Model Viewer
```

```
fix: lien vers le plan de découpe laser déplacé dans laser-cutting
```

## Procédure

1. `git status` + `git diff` pour vérifier le périmètre.
2. `git add <fichiers>` explicitement.
3. `git commit -F -` avec un heredoc (message multi-lignes, accents OK).
4. `git log -1 --format='%h %an <%ae> %s'` pour confirmer l'auteur, et
   vérifier qu'aucune mention d'IA n'apparaît dans le message.
