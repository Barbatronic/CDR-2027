---
name: journal-post
description: Rédaction d'un article du journal de bord (DOCS/_posts) à partir de la dictée de l'utilisateur, avec intégration, optimisation et nettoyage des photos, vidéos et modèles 3D. À utiliser pour tout nouvel article ou ajout de médias au journal.
---

# Article du journal de bord

Site Jekyll + Just the Docs dans `DOCS/`, publié sur
https://barbatronic.fr/CDR-2027/ par GitHub Actions à chaque push sur `main`.

## 1. Créer l'article et son dossier

- Article : `DOCS/_posts/AAAA-MM-JJ-slug.md` (date du jour, slug court en
  minuscules sans accents, ex. `2026-09-29-canon-pilote-robot`).
- Médias : `DOCS/assets/img/AAAA-MM-JJ-slug/` (même nom que l'article).
- Créer les deux tout de suite, même vides, pour que l'utilisateur puisse
  déposer ses photos pendant qu'il dicte.

Front matter (le layout est appliqué par défaut, ne pas l'écrire) :

```yaml
---
title: "Titre de l'article"
date: AAAA-MM-JJ
tags: [mécanique, test]
description: "Une phrase de résumé, affichée dans la liste du journal."
---
```

- `title` et `description` **toujours entre guillemets** (un « : » casse
  le build sinon).
- Tags autorisés uniquement : `mécanique`, `électronique`, `logiciel`,
  `test`. Pas de tag `décision`.

## 2. Rédiger à partir de la dictée

L'utilisateur dicte à l'oral, avec des hésitations et des mots mal
transcrits. Le texte doit rester **le sien** :

- première personne, ton naturel et parlé, passé composé pour les essais ;
- ne rien inventer : pas de chiffre, de cause ou de conclusion qu'il n'a
  pas dite. Signaler les trous dans la réponse plutôt que les combler ;
- structure habituelle : `## Objectif`, sections par étape dans l'ordre
  chronologique réel, `## Conclusion` ;
- lien vers l'article précédent quand il y a une suite :
  `[l'essai d'hier]({{ '/journal/AAAA-MM-JJ-slug/' | relative_url }})`.

Vocabulaire du projet :

- nomenclature Coupe 2027 : balles de 45 mm = **boulets**, cartons
  32 × 11 × 11 cm = **pierres** ;
- transcriptions fréquentes à corriger : « Sibel » → 6 balles, « tiers »
  → tir, « SP32 » → ESP32, « IT60 » → XT60, « les 10 minutes » → les
  métadonnées, « A1 mini / P1P » → imprimantes Bambu Lab ;
- valeurs de mesure : reprendre la valeur **lue sur l'instrument** dans
  la photo (pied à coulisse, balance), pas la valeur arrondie dictée.

Lister en fin de réponse les interprétations faites sur des passages
ambigus, pour que l'utilisateur corrige.

## 3. Regarder chaque média avant de le placer

- Lire chaque photo. Pour les vidéos, extraire quelques images
  (`ffmpeg -ss <t> -frames:v 1`) et les assembler en planche contact
  (`montage`) dans le scratchpad.
- Placer chaque média là où il illustre le texte, pas dans l'ordre des
  fichiers. En cas de doute sur ce qu'une photo montre, le dire.
- Un doublon quasi identique : n'en garder qu'un, sortir l'autre du
  dossier (l'original reste dans la sauvegarde) et le signaler.
- Ne pas prétendre avoir vu un tir ou un résultat qu'aucune image
  extraite ne montre.

## 4. Optimiser et renommer

Pour chaque fichier déposé, avec un nom lisible préfixé par l'article
(ex. `cp-driver-dessus`, `t6-essai-1`) :

```bash
.claude/skills/journal-post/scripts/optimize-media.sh <source> <dossier>/<nouveau-nom>
```

Le script réduit (photos 1600 px, captures 1400 px, vidéos H.264 720p),
efface **toutes** les métadonnées (GPS compris) et sauvegarde l'original
hors du dépôt. Les photos de téléphone contiennent toujours des
coordonnées GPS : aucun média brut ne doit être commité.

## 5. Intégrer dans l'article

Photo, avec légende en italique sur la ligne suivante :

```markdown
![Texte alternatif descriptif]({{ '/assets/img/AAAA-MM-JJ-slug/nom.jpg' | relative_url }})
*Légende courte.*
```

Vidéo :

```html
<video controls preload="metadata" playsinline width="100%">
  <source src="{{ '/assets/img/AAAA-MM-JJ-slug/nom.mp4' | relative_url }}" type="video/mp4">
  Votre navigateur ne sait pas lire cette vidéo.
</video>

*Légende courte.*
```

Modèle 3D (Model Viewer est chargé sur tout le site) : demander un
**`.glb`** (un `.gltf` seul dépend d'un `.bin` souvent oublié).

```html
<model-viewer src="{{ '/assets/img/AAAA-MM-JJ-slug/modele.glb' | relative_url }}"
  alt="Description du modèle" camera-controls auto-rotate shadow-intensity="1">
</model-viewer>
*Légende.*
```

Fichiers sources CAD : lien direct vers le dépôt, et les commiter en même
temps sinon le lien est mort :
`https://github.com/Barbatronic/CDR-2027/blob/main/MCAD/<chemin>`.

Le firmware est dans ce dépôt, dans `FRW/Differential-Robot-Firmware` :
`https://github.com/Barbatronic/CDR-2027/tree/main/FRW/Differential-Robot-Firmware`
(vérifier que les modifications citées sont poussées avant d'y renvoyer).

## 6. Vérifier

```bash
.claude/skills/journal-post/scripts/check-post.sh DOCS/_posts/AAAA-MM-JJ-slug.md
```

Doit afficher `OK` : build sans erreur, tous les médias trouvés, aucun
fichier inutilisé, aucune métadonnée. Les avertissements MD033 de
l'éditeur sur `<video>` et `<model-viewer>` sont normaux.

## 7. Commiter et publier

Utiliser le skill `commit` (convention du dépôt, aucune mention d'IA).
Ne jamais commiter ni pousser sans demande explicite de l'utilisateur.
Après un push, la publication prend 1 à 2 minutes
(`gh run watch` pour suivre).
