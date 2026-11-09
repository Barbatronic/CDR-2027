BOB-12009 SparkFun — assemblage STEP avec connecteurs 2 x 1x6 P2,54 mm
=============================================================================
Fichiers :
- BOB-12009_assemblage_headers_males_femelles.step
  Convertisseur BOB-12009 original + headers MALES sur module + headers
  FEMELLES sur carte d'accueil. Fichier a utiliser dans KiCad pour afficher
  l'ensemble depuis l'empreinte de la carte d'accueil (plan PCB : XY, Z>0).

- BOB-12009_avec_headers_males.step
  Module BOB-12009 d'origine avec deux barres de headers MALES sous le PCB.
  Dessous du PCB du module place a Z=0. Les broches sont a Z<0.

- Headers_femelles_2x_1x06_P2.54.step
  Deux supports FEMELLES seuls, a poser sur la carte d'accueil Z=0.

- creer_BOB12009_headers.py + BOB-12009.step
  Sources parametriques CadQuery et STEP original fourni pour regenerer les fichiers.

GEOMETRIE
---------
- Module BOB-12009 original conserve : 16,05 x 13,33 mm, epaisseur 1,60 mm.
- 12 trous : (X = -5,08 ou +5,08 mm ; Y = -6,35 / -3,81 / -1,27 /
  +1,27 / +3,81 / +6,35 mm).
- Pas longitudinal : 2,54 mm ; entraxe entre rangs : 10,16 mm.
- Supports femelles generiques : hauteur 8,50 mm (au-dessus du PCB d'accueil).
- Embases males generiques : corps plastique epaisseur 2,54 mm, queue 6 mm.
- Jeu de 1,00 mm entre le support male et l'embase femelle.
- Dessous du PCB BOB-12009 a Z=+12,04 mm par rapport au PCB d'accueil Z=0.
- Encombrement approximatif total hors echelle : Z=-3,00 a +14,84 mm.

INTEGRATION DANS KICAD
----------------------
1. Monter sur la carte d'accueil deux connecteurs femelles 1x06, pas 2,54 mm.
2. Les centres des deux rangs sont separes de 10,16 mm.
3. Positionner le modele 3D complet dans les proprietes de l'empreinte :
   ${KIPRJMOD}/3dmodels/BOB-12009_assemblage_headers_males_femelles.step
4. Aligner l'origine du STEP (milieu des 12 trous) sur le milieu des 12 pads.
5. L'orientation du modele est deja adaptee au PCB KiCad : XY plan, +Z haut.
6. Ne pas ajouter en meme temps les modeles 3D des headers femelles dans
   KiCad si tu affiches le STEP complet, sous peine de les dupliquer.
7. Verifier le pinout, le sens LV/HV et les cotes du connecteur physique reel.

LIMITES
-------
Les connecteurs sont generiques et simplifies pour la visualisation et
l'implantation preliminaire. Ils ne sont PAS des references constructeur.
Verifier la hauteur et la longueur des broches des connecteurs achetes,
ainsi que l'alignement reel, avant validation mecanique de l'assemblage.
