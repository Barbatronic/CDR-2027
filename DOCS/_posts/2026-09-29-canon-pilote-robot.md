---
title: "Canon intégré au robot, piloté par la carte du robot et un ESC"
date: 2026-09-29
tags: [mécanique, électronique, test]
description: "Le canon à 6 boulets est piloté par la carte du robot : un ESC bimoteur via le PCA9685 pour les roues à inertie, et une sortie PWM directe pour le ventilateur."
---

## Objectif

Après [l'essai d'hier]({{ '/journal/2026-09-28-tir-integre-6-boulets/' | relative_url }}), où le canon était alimenté directement, je teste ce soir le canon intégré au robot, piloté par la carte interne du robot et un ESC.

## Le driver des roues à inertie

J'utilise un simple driver unidirectionnel bimoteur, pour piloter deux moteurs à courant continu. L'idée est de piloter les deux moteurs des roues à inertie avec ce système, et de pouvoir moduler la vitesse de sortie des roues droite et gauche, afin de pouvoir potentiellement orienter un petit peu le tir.

![Le driver bimoteur, côté marquage : Dual Brushed, 2/3S, 6 A × 2]({{ '/assets/img/2026-09-29-canon-pilote-robot/cp-driver-dessus.jpg' | relative_url }})
*Le driver bimoteur, côté marquage : « Dual Brushed, 2/3S, 6A×2 ».*

![Le driver bimoteur, côté composants]({{ '/assets/img/2026-09-29-canon-pilote-robot/cp-driver-dessous.jpg' | relative_url }})
*Le même driver, côté composants.*

Il est branché directement sur le PCA9685, un étendeur I2C qui génère des signaux PWM. Ça me permet de piloter ces deux moteurs comme si c'étaient des servomoteurs, tout en continuant à utiliser le PCA9685 pour piloter des servomoteurs classiques. Le driver est alimenté directement par le 12 V de la batterie.

![Le driver monté sur le robot]({{ '/assets/img/2026-09-29-canon-pilote-robot/cp-driver-monte.jpg' | relative_url }})
*Le driver, monté sur le robot.*

Après plusieurs essais, je me suis rendu compte que le driver est en mode mix. J'ai donc dû modifier un peu la manière de le piloter, pour pouvoir commander chaque roue indépendamment.

![Le canon monté sur le robot, avec les deux moteurs des roues à inertie]({{ '/assets/img/2026-09-29-canon-pilote-robot/cp-canon-sur-robot.jpg' | relative_url }})
*Le canon monté sur le robot, avec les deux moteurs des roues à inertie.*

## Le ventilateur

J'ai également branché le ventilateur, mais de manière un peu « sale », entre guillemets : vite fait, avec des fils Dupont plantés dans un connecteur XT60, parce que je voulais tester. Pour mes essais ça a fonctionné, mais je vais faire quelque chose de plus propre dans la soirée.

![Deux fils Dupont plantés directement dans un connecteur XT60]({{ '/assets/img/2026-09-29-canon-pilote-robot/cp-xt60-dupont.jpg' | relative_url }})
*Deux Dupont plantés dans un XT60 : à ne pas refaire chez vous 😅*

Le pilotage PWM du ventilateur se fait sur la broche 19, qui est en fait la broche D- de l'USB sur l'ESP32-S3. Je n'avais pas envie d'utiliser cette broche en particulier, je voulais la garder pour l'USB. Mais je ne l'utilise pas pour l'instant, et c'est la dernière broche qui me reste. Ça me permet quand même de faire des essais, avant de remplacer potentiellement la carte de l'année dernière par une nouvelle version, un peu plus moderne.

## Firmware et interface web

J'ai mis à jour le firmware, que vous pouvez retrouver dans le dépôt du projet : [FRW/Differential-Robot-Firmware](https://github.com/Barbatronic/CDR-2027/tree/main/FRW/Differential-Robot-Firmware).

J'ai aussi mis à jour l'interface web intégrée à l'ESP32, qui sert à piloter le robot depuis un navigateur, pour y ajouter ces trois nouveaux éléments : les deux moteurs et le ventilateur. Il y a un curseur pour le moteur 0, un curseur pour le moteur 1, et un curseur pour la vitesse du ventilateur. On peut tout activer d'un coup, et il y a un bouton d'arrêt et un bouton de lancement, qui met tout à fond.

## Essais de tir

Pour le premier essai, je suis à 100 %. J'attends la montée en vitesse des roues à inertie, puis je lance le ventilateur. Ça règle le problème vu [hier]({{ '/journal/2026-09-28-tir-integre-6-boulets/' | relative_url }}), où la première balle partait trop vite. Ça fonctionne extrêmement bien, c'est même beaucoup trop rapide.

<video controls preload="metadata" playsinline width="100%">
  <source src="{{ '/assets/img/2026-09-29-canon-pilote-robot/cp-essai-100.mp4' | relative_url }}" type="video/mp4">
  Votre navigateur ne sait pas lire cette vidéo.
</video>

*Premier essai, roues à inertie à 100 %.*

Pour un autre essai, j'ai diminué la vitesse des roues à inertie à 43 % de la vitesse maximale. Je ne connais pas cette vitesse maximale pour l'instant : elle correspond à la tension maximale de la batterie, ou en tout cas à ce que le driver peut fournir au maximum. Le tir reste assez puissant.

<video controls preload="metadata" playsinline width="100%">
  <source src="{{ '/assets/img/2026-09-29-canon-pilote-robot/cp-essai-43.mp4' | relative_url }}" type="video/mp4">
  Votre navigateur ne sait pas lire cette vidéo.
</video>

*Deuxième essai, roues à inertie à 43 %.*

Pour le troisième essai, je suis descendu à 15 %, et le tir reste encore assez puissant.

<video controls preload="metadata" playsinline width="100%">
  <source src="{{ '/assets/img/2026-09-29-canon-pilote-robot/cp-essai-15.mp4' | relative_url }}" type="video/mp4">
  Votre navigateur ne sait pas lire cette vidéo.
</video>

*Troisième essai, roues à inertie à 15 %.*

## Conclusion

Le canon est maintenant piloté par la carte du robot : chaque roue à inertie indépendamment, et le ventilateur à part, lancé une fois les roues en vitesse. Même à 15 %, le tir reste puissant.

Je n'arrive pas à me rendre compte de ce que ça représente sur une table : encore une fois, je tape dans le plafond, je tape un peu partout, je n'ai pas beaucoup de place. L'idée est de faire des tests demain sur une table de mon école, dans une salle avec un peu de place, pour voir à quel point c'est efficace, ou pas.
