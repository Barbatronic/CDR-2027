---
title: "Essai d'un lanceur de boulets à deux moteurs"
date: 2026-09-22
tags: [mécanique, électronique, test]
description: "Premier essai rapide d'un lanceur de boulets à deux roues motorisées, en boucle ouverte à 12 V."
---

## Objectif

Ce soir, je teste un premier lanceur de boulets à deux moteurs, pour voir si ce principe fonctionne avant d'aller plus loin.

## La maquette

J'ai fait une maquette rapide ce soir, en impression 3D et découpe laser. Le modèle est dans le dépôt du projet : [ball-launcher-test.FCStd](https://github.com/Barbatronic/CDR-2027/blob/main/MCAD/actuator-parts/ball-launcher-test.FCStd) pour le fichier FreeCAD, et [ball-launcher-test_side.svg](https://github.com/Barbatronic/CDR-2027/blob/main/MCAD/laser/ball-launcher-test_side.svg) pour le plan de découpe laser.

### Modèle 3D

<model-viewer
  src="{{ '/assets/img/2026-09-22-lanceur-deux-moteurs/ball-launcher-test.glb' | relative_url }}"
  alt="Modèle 3D du lanceur de boulets à deux moteurs"
  camera-controls
  auto-rotate
  shadow-intensity="1">
</model-viewer>
*Modèle 3D du lanceur, cliquer-glisser pour tourner autour.*

![Les deux moteurs MF360S, avec leur support imprimé]({{ '/assets/img/2026-09-22-lanceur-deux-moteurs/lanceur-deux-moteurs.jpg' | relative_url }})
*Les deux moteurs MF360S, avec leur support imprimé.*

Le lanceur utilise deux moteurs MF360S, alimentés en 12 V. Chaque moteur entraîne une roue d'envoi de 56 mm de diamètre, les deux roues étant placées de part et d'autre du boulet. Les roues sont imprimées en TPU, pour permettre un emmanchement en force sur l'axe du moteur, pour le moment.

![Le guide imprimé qui canalise le boulet entre les deux roues]({{ '/assets/img/2026-09-22-lanceur-deux-moteurs/lanceur-guide-impression.jpg' | relative_url }})
*Le guide imprimé qui canalise le boulet entre les deux roues.*

![Une des deux roues d'envoi imprimées en TPU]({{ '/assets/img/2026-09-22-lanceur-deux-moteurs/lanceur-roue-impression.jpg' | relative_url }})
*Une des deux roues d'envoi, imprimées en TPU.*

Je ne pilote pas la vitesse : je suis en boucle ouverte, les moteurs sont simplement alimentés en 12 V via mon alimentation stabilisée. Je ne connais donc pas la vitesse de sortie des roues.

![Le lanceur assemblé, moteurs branchés avec des pinces crocodile]({{ '/assets/img/2026-09-22-lanceur-deux-moteurs/lanceur-assemblage-1.jpg' | relative_url }})
*Le lanceur assemblé, moteurs branchés avec des pinces crocodile.*

![Le lanceur assemblé, vu depuis l'autre côté]({{ '/assets/img/2026-09-22-lanceur-deux-moteurs/lanceur-assemblage-2.jpg' | relative_url }})
*Le lanceur assemblé, vu de dessus.*

![Un boulet chargé entre les deux roues du lanceur]({{ '/assets/img/2026-09-22-lanceur-deux-moteurs/lanceur-boulet-charge.jpg' | relative_url }})
*Un boulet chargé entre les deux roues, prêt à être envoyé.*

## Essai de tir

Je me place à 3 m du lanceur, pour simuler un tir depuis l'autre côté du terrain.

<video controls preload="metadata" playsinline width="100%">
  <source src="{{ '/assets/img/2026-09-22-lanceur-deux-moteurs/lanceur-essai-tir.mp4' | relative_url }}" type="video/mp4">
  Votre navigateur ne sait pas lire cette vidéo.
</video>

*Essai de tir du lanceur à deux moteurs.*

Résultat : le lanceur est efficace, même un peu limité par la hauteur de mon plafond. Je peux faire des tirs en cloche pour passer au-dessus des murs de 3 cartons.

## Conclusion

Premier test prometteur : on peut visiblement construire un lanceur assez facilement avec ce type de système, deux roues motorisées en boucle ouverte.
