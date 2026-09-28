---
title: "Essai du tir intégré complet, avec 6 boulets stockés"
date: 2026-09-28
tags: [mécanique, test]
description: "Un système de tir intégré, imprimé en une seule pièce, qui stocke et tire 6 boulets sans blocage, testé seul puis monté sur le robot."
---

## Objectif

Après [l'essai d'hier]({{ '/journal/2026-09-27-systeme-tir-integre/' | relative_url }}), où les boulets se bloquaient dans le coude, j'ai imprimé ce soir un système minimal, prévu pour 6 boulets, afin de vérifier qu'il n'y a plus de blocage.

## Pourquoi 6 boulets

On ne peut donc pas stocker les 10 boulets prévus au départ. Mais on peut stocker un boulet par PAMI, et j'ai 4 PAMI récupérés de l'année dernière. On peut donc stocker 6 boulets dans le robot et les tirer, puis envoyer les 4 derniers avec les PAMI. Au minimum, ça peut fonctionner.

## Le système

Dans cette version, le ventilateur est en bas, et non plus en haut. L'expulsion des boulets se fait toujours par le haut, au même endroit que le système testé hier soir.

La pièce est imprimée en une seule fois, à plat, sur une Bambu Lab P1P.

![Le système imprimé, vu de dessus, avec les logements des deux moteurs au bout]({{ '/assets/img/2026-09-28-tir-integre-6-boulets/t6-piece-dessus.jpg' | relative_url }})
*La pièce imprimée, avec les logements des deux moteurs au bout.*

![Le système imprimé, vu de l'autre côté, avec le canal ouvert]({{ '/assets/img/2026-09-28-tir-integre-6-boulets/t6-piece-interieur.jpg' | relative_url }})
*La même pièce, vue de l'autre côté.*

![Le système avec les deux moteurs montés et branchés]({{ '/assets/img/2026-09-28-tir-integre-6-boulets/t6-piece-moteurs.jpg' | relative_url }})
*Les deux moteurs montés et branchés.*

## Essais du système seul

Dans la première vidéo, je mets six boulets à l'intérieur, et le tir va super vite.

<video controls preload="metadata" playsinline width="100%">
  <source src="{{ '/assets/img/2026-09-28-tir-integre-6-boulets/t6-essai-1.mp4' | relative_url }}" type="video/mp4">
  Votre navigateur ne sait pas lire cette vidéo.
</video>

*Premier essai : six boulets chargés, puis tirés.*

Deuxième vidéo, pareil : les six boulets, et je tire.

<video controls preload="metadata" playsinline width="100%">
  <source src="{{ '/assets/img/2026-09-28-tir-integre-6-boulets/t6-essai-2.mp4' | relative_url }}" type="video/mp4">
  Votre navigateur ne sait pas lire cette vidéo.
</video>

*Deuxième essai, avec six boulets.*

C'est très efficace. La première balle sort un peu trop vite, parce que les moteurs sont alimentés en même temps que le ventilateur : ils n'ont pas le temps de monter en vitesse et d'atteindre le régime souhaité. Il faudra régler ça dans le code, en attendant que les moteurs aient atteint un certain régime avant de lancer le ventilateur pour envoyer les boulets.

## Essai monté sur le robot

Pour le deuxième essai, j'ai monté le système complet sur le robot, avec une plaque découpée en acrylique. Le montage est un peu bancal, mais pour un test, c'est largement suffisant pour le moment.

![Le système de tir monté sur le robot]({{ '/assets/img/2026-09-28-tir-integre-6-boulets/t6-sur-robot.jpg' | relative_url }})
*Le système de tir monté sur le robot.*

J'ai fait un essai, et on voit bien que le tir part quasiment directement, sans être gêné par l'écran du robot ou autre chose : c'est fonctionnel.

<video controls preload="metadata" playsinline width="100%">
  <source src="{{ '/assets/img/2026-09-28-tir-integre-6-boulets/t6-essai-robot.mp4' | relative_url }}" type="video/mp4">
  Votre navigateur ne sait pas lire cette vidéo.
</video>

*Essai de tir avec le système monté sur le robot.*

## Conclusion

Le système à 6 boulets fonctionne sans blocage, et le tir est très efficace, seul comme monté sur le robot. Il reste à attendre la montée en régime des moteurs avant de lancer le ventilateur, pour que la première balle ne sorte pas trop vite.

En revanche, je vais avoir un problème pour tester le tir réel : il est en cloche, et la hauteur de mon plafond ne suffit pas pour un tir en cloche à 3 mètres sur une table. Il va falloir que je teste ça à un autre endroit.
