---
title: "Vers un système de tir intégré : ventilateur et lanceur en une seule pièce"
date: 2026-09-27
tags: [mécanique, test]
description: "Premiers essais de modélisation et d'impression d'un système intégrant le ventilateur de poussée et les deux roues de tir en une seule pièce, et découverte d'un blocage des boulets dans les coudes."
---

## Objectif

Ce soir, j'essaie d'aller un peu plus loin qu'un simple assemblage : intégrer à la fois le ventilateur, qui pousse les boulets, et les deux moteurs, qui les tirent, dans une seule et même pièce imprimée en 3D. La modélisation se fait sous FreeCAD.

## Le lanceur vertical à 46 mm

Juste avant de commencer, j'ai terminé un lanceur vertical imprimé en 3D, pour tester le diamètre de tube. Avec un diamètre de 46 mm, ça fonctionne bien.

![Le tube imprimé de 46 mm de diamètre]({{ '/assets/img/2026-09-27-systeme-tir-integre/sti-tube-46mm.jpg' | relative_url }})
*Le tube imprimé, 46 mm de diamètre.*

![Un boulet chargé dans le tube de 46 mm]({{ '/assets/img/2026-09-27-systeme-tir-integre/sti-tube-46mm-boulet.jpg' | relative_url }})
*Un boulet chargé dans le tube.*

<video controls preload="metadata" playsinline width="100%">
  <source src="{{ '/assets/img/2026-09-27-systeme-tir-integre/sti-tube-46mm-essai.mp4' | relative_url }}" type="video/mp4">
  Votre navigateur ne sait pas lire cette vidéo.
</video>

*Essai du tube à 46 mm : les boulets sont éjectés correctement.*

## Premier corps imprimé : le canal coudé

J'ai d'abord modélisé le canal coudé, sans les mécanismes de poussée et de tir, pour que ce soit plus simple et plus rapide à imprimer.

L'impression devait durer environ 3 heures, et j'ai dû séparer la pièce en deux pour qu'elle passe sur mes imprimantes, des Bambu Lab A1 mini : une partie coudée, et une autre partie qui va sur le dessus.

![La partie coudée du canal, imprimée]({{ '/assets/img/2026-09-27-systeme-tir-integre/sti-coude-piece.jpg' | relative_url }})
*La partie coudée du canal.*

![Les deux parties du canal : la partie coudée et la partie qui va sur le dessus]({{ '/assets/img/2026-09-27-systeme-tir-integre/sti-coude-deux-parties.jpg' | relative_url }})
*Les deux parties du canal : la partie coudée, et la partie qui va sur le dessus.*

## Modélisation du support des rouleaux et du ventilateur

Pendant que le canal s'imprimait, j'ai continué la modélisation : d'abord le support des deux rouleaux, puis le support du ventilateur. Pour ce système complet, je n'ai imprimé que la partie de tir.

Le modèle FreeCAD est dans le dépôt du projet : [ball-launcher.FCStd](https://github.com/Barbatronic/CDR-2027/blob/main/MCAD/actuator-parts/ball-launcher.FCStd).

![Modèle 3D du système complet sous FreeCAD : canal coudé, support du ventilateur et support des rouleaux]({{ '/assets/img/2026-09-27-systeme-tir-integre/sti-systeme-modele-3d.png' | relative_url }})
*Le système complet modélisé sous FreeCAD : le canal coudé, le support du ventilateur en haut et le support des rouleaux.*

![Le support des deux rouleaux imprimé, avant le montage des moteurs]({{ '/assets/img/2026-09-27-systeme-tir-integre/sti-support-rouleaux-1.jpg' | relative_url }})
*Le support des deux rouleaux imprimé, avant le montage des moteurs.*

![Le support des rouleaux vu sous un autre angle, avec les logements des moteurs]({{ '/assets/img/2026-09-27-systeme-tir-integre/sti-support-rouleaux-2.jpg' | relative_url }})
*Le même support, avec les logements des moteurs.*

## Test du circuit coudé

Une fois l'impression du canal terminée, j'ai testé le circuit. Il se bloque assez facilement : des boulets viennent se coincer à certains endroits, ce qui empêche le mécanisme de bien fonctionner. Le coude semble un peu trop marqué, et les boulets s'y bloquent.

![Un boulet bloqué à l'intérieur du canal coudé]({{ '/assets/img/2026-09-27-systeme-tir-integre/sti-coude-boulet-bloque.jpg' | relative_url }})
*Un boulet bloqué à l'intérieur du canal.*

Ce qui se passe : une fois qu'un boulet touche un autre, ils adhèrent l'un à l'autre et se frottent entre eux. Si la descente est directe, ça passe, mais s'il faut qu'ils puissent rouler l'un sur l'autre, ils se bloquent complètement. Un canal coudé comme celui-là comporte donc un vrai risque de blocage. Il faudra que je fasse d'autres essais, mais un redimensionnement est à envisager.

Ça confirme ce que j'avais déjà observé : les boulets, très légers, ont tendance à se déformer l'un contre l'autre et à bien adhérer entre eux, au point de se bloquer dans un tube.

## Essai de tir du système intégré

J'ai quand même fait un essai de tir avec le système intégré, partiellement imprimé : uniquement la partie de tir. Il est plus intégré que le dispositif précédent, avec des roues de 44 mm de diamètre, contre 56 mm avant.

![Le canon intégré, avec ses deux roues et l'entrée du tube sur le dessus]({{ '/assets/img/2026-09-27-systeme-tir-integre/sti-canon-integre-1.jpg' | relative_url }})
*Le canon intégré : les deux roues et l'entrée du tube sur le dessus.*

![Le canon intégré, vu sous un autre angle]({{ '/assets/img/2026-09-27-systeme-tir-integre/sti-canon-integre-2.jpg' | relative_url }})
*Le canon intégré, vu sous un autre angle.*

![Comparaison entre l'ancien lanceur (roues de 56 mm) et le nouveau canon intégré (roues de 44 mm)]({{ '/assets/img/2026-09-27-systeme-tir-integre/sti-comparaison-roues-1.jpg' | relative_url }})
*L'ancien lanceur, avec des roues de 56 mm, à côté du nouveau canon intégré, avec des roues de 44 mm.*

Résultat : ça fonctionne très bien.

<video controls preload="metadata" playsinline width="100%">
  <source src="{{ '/assets/img/2026-09-27-systeme-tir-integre/sti-canon-integre-essai.mp4' | relative_url }}" type="video/mp4">
  Votre navigateur ne sait pas lire cette vidéo.
</video>

*Essai de tir avec le canon intégré.*

## Conclusion

Le tir fonctionne, et j'ai validé le fait que je peux imprimer des pièces de ce type sans souci particulier.

En revanche, les boulets se bloquent complètement dans le système dès qu'il y a un coude. Je réfléchis maintenant à une autre séquence de tir, en envisageant peut-être, dans un premier temps, de ne pas stocker l'intégralité des boulets dans le système.
