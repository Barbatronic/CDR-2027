---
title: "Essai d'un ventilateur pour alimenter le lanceur en boulets"
date: 2026-09-25
tags: [mécanique, électronique, test]
description: "Un ventilateur 12 V de 40 × 40 mm pousse les boulets à la verticale dans un tube, pour alimenter le lanceur à deux roues."
---

## Objectif

Ce soir, j'ai reçu un ventilateur 12 V de 40 × 40 mm, assez puissant. L'idée est de m'en servir pour pousser les boulets dans un tube, qui pourrait être vertical pour les stocker, puis de les envoyer avec les deux roues testées dans [l'article précédent]({{ '/journal/2026-09-22-lanceur-deux-moteurs/' | relative_url }}). Le tout serait placé sur le dessus du robot.

## Le ventilateur

C'est un Arctic S4028-15K : [fiche du fabricant](https://www.arctic.de/en/S4028-15K/ACFAN00264A) et [lien d'achat](https://www.amazon.fr/dp/B09RK64BQH). Sa plaque indique 12 V et 0,47 A. Pour mes essais, je l'ai branché directement en 12 V.

![La plaque du ventilateur Arctic S4028-15K : DC 12 V, 0,47 A]({{ '/assets/img/2026-09-25-ventilateur-boulets/ventilateur-plaque.jpg' | relative_url }})
*Le ventilateur Arctic S4028-15K et sa plaque : 12 V, 0,47 A.*

## Premier essai : un tube en papier

Pour voir ce que ça donne, j'ai commencé par un tube en papier, d'environ 45 mm de diamètre, avec le ventilateur scotché à une extrémité. Il était compliqué à faire proprement, et les boulets stockés à l'intérieur frottent sur le coin du tube. Je suis en train d'imprimer un tube de 46 mm de diamètre, plus propre, mais en attendant je voulais déjà tester comme ça.

![Le tube en papier fermé au ruban adhésif, avec le ventilateur à une extrémité]({{ '/assets/img/2026-09-25-ventilateur-boulets/tube-papier-ventilateur.jpg' | relative_url }})
*Le tube en papier, avec le ventilateur scotché à une extrémité.*

![Le ventilateur scotché au bout du tube, tenu en main]({{ '/assets/img/2026-09-25-ventilateur-boulets/ventilateur-scotche-tube.jpg' | relative_url }})
*Le ventilateur, scotché au bout du tube.*

Je stocke des boulets par le dessus, je lance le ventilateur à fond, et on voit bien que toutes les balles sortent. Le ventilateur semble donc assez puissant pour propulser un certain nombre de balles purement à la verticale, et c'est une bonne nouvelle.

<video controls preload="metadata" playsinline width="100%">
  <source src="{{ '/assets/img/2026-09-25-ventilateur-boulets/ventilateur-test-vertical.mp4' | relative_url }}" type="video/mp4">
  Votre navigateur ne sait pas lire cette vidéo.
</video>

*Test à la verticale : le ventilateur à fond fait sortir toutes les balles du tube.*

## Essai avec le lanceur

J'ai ensuite scotché le tube au dispositif testé précédemment.

Avec un tir quasiment à l'horizontale, le résultat est très positif : ça fonctionne extrêmement bien.

<video controls preload="metadata" playsinline width="100%">
  <source src="{{ '/assets/img/2026-09-25-ventilateur-boulets/ventilateur-tir-horizontal.mp4' | relative_url }}" type="video/mp4">
  Votre navigateur ne sait pas lire cette vidéo.
</video>

*Tir quasiment à l'horizontale.*

J'ai aussi testé un tir avec un peu plus d'angle, et c'est pareil, ça fonctionne extrêmement bien.

<video controls preload="metadata" playsinline width="100%">
  <source src="{{ '/assets/img/2026-09-25-ventilateur-boulets/ventilateur-tir-incline.mp4' | relative_url }}" type="video/mp4">
  Votre navigateur ne sait pas lire cette vidéo.
</video>

*Tir avec un peu plus d'angle.*

## Conclusion

Le système peut donc fonctionner avec un stockage inversé : le ventilateur pousse les boulets contre le dispositif de lancer, qui sera sur le dessus du robot.
