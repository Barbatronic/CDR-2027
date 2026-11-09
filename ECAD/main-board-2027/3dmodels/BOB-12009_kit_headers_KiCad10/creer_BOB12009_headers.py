"""BOB-12009 SparkFun : modele 3D avec 2x6 headers male/femelle (pas 2.54 mm).

Prend le STEP de l'utilisateur sans le modifier et le repositionne dans le repere KiCad :
  XY = plan du circuit imprimé d'accueil ; Z = perpendiculaire a la carte, vers le haut.
  Le dessus du PCB d'accueil est z = 0.

Les pin headers sont des modeles generiques, PAS des composants fabricants certifies.

Dependance : pip install cadquery
"""
from pathlib import Path
import cadquery as cq

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / 'BOB-12009.step'
PITCH = 2.54
HOLES = [(x, y) for x in (-5.08, 5.08) for y in (-6.35, -3.81, -1.27, 1.27, 3.81, 6.35)]

# Dimensions generiques, a adapter au materiel reel.
FEMALE_HEIGHT = 8.50       # boitier femelle sur la carte mere, au dessus de z=0
FEMALE_TAIL = 3.00         # broche femelle sous la carte mere
MALE_BODY_HEIGHT = 2.54   # barre plastique cote module
MALE_GAP = 1.00           # jeu entre tete femelle et support plastique male
MALE_PIN_LENGTH = 6.00    # broches males libres en dessous du support
MALE_PIN_TOP = 0.75       # portion visible au-dessus du PCB fille
BOARD_THICKNESS = 1.60
FEMALE_WIDTH = 2.54
PIN_SQUARE = 0.64
FEMALE_HOLE_SQUARE = 0.94
FEMALE_SOCKET_INNER = 0.72

# Position exacte du dessous du PCB BOB-12009 par rapport au PCB mere (z=0).
Z_DAUGHTER_BOT = FEMALE_HEIGHT + MALE_GAP + MALE_BODY_HEIGHT  # 12.04 mm
Z_DAUGHTER_TOP = Z_DAUGHTER_BOT + BOARD_THICKNESS

BLACK = cq.Color(0.105, 0.115, 0.12)
RED = cq.Color(0.79, 0.11, 0.13)
GOLD = cq.Color(0.83, 0.67, 0.27)
SILVER = cq.Color(0.75, 0.77, 0.77)
DARK = cq.Color(0.13, 0.135, 0.145)

# Constructeur de solides par blocs centres en XY, avec base z donnee
# Utiliser des formes disjointes dans le STEP, comme un vrai assemblage.
def rect(x, y, z0, dx, dy, dz):
    return cq.Workplane('XY').box(dx, dy, dz, centered=(True, True, False)).val().translate((x, y, z0))


def make_body_with_holes(x):
    housing = rect(x, 0, 0, FEMALE_WIDTH, 15.24, FEMALE_HEIGHT)
    for y in (-6.35, -3.81, -1.27, 1.27, 3.81, 6.35):
        # Cavite carree : l'embase femelle s'ouvre vers le haut
        cavity = rect(x, y, 2.20, FEMALE_HOLE_SQUARE, FEMALE_HOLE_SQUARE,
                      FEMALE_HEIGHT - 2.20 + .05)
        housing = housing.cut(cavity)
    return housing


def make_socket_ring(x, y):
    # Fourreau conducteur creux interne a la cavite plastique
    outer = rect(x, y, 2.28, .84, .84, 5.95)
    inner = rect(x, y, 2.26, FEMALE_SOCKET_INNER, FEMALE_SOCKET_INNER, 6.05)
    return outer.cut(inner)


def add_female_connectors(assembly):
    for n, x in enumerate((-5.08, 5.08), 1):
        assembly.add(make_body_with_holes(x), name=f'female_1x6_housing_{n}', color=BLACK)
    for i, (x, y) in enumerate(HOLES, 1):
        assembly.add(make_socket_ring(x, y), name=f'female_contact_{i}', color=GOLD)
        # Queue de connexion a souder dans la carte mere
        assembly.add(rect(x, y, -FEMALE_TAIL, PIN_SQUARE, PIN_SQUARE, FEMALE_TAIL + 2.3),
                     name=f'female_solder_tail_{i}', color=SILVER)


def add_male_connectors(assembly, board_bottom_z):
    for n, x in enumerate((-5.08, 5.08), 1):
        assembly.add(rect(x, 0, board_bottom_z-MALE_BODY_HEIGHT,
                          2.54, 15.24, MALE_BODY_HEIGHT),
                     name=f'male_1x6_housing_{n}', color=BLACK)
    for i, (x, y) in enumerate(HOLES, 1):
        # Le corps plastique s'appuie sous le PCB fille; chaque pin passe le trou de dia. 1.016mm.
        assembly.add(rect(x, y, board_bottom_z-MALE_BODY_HEIGHT-MALE_PIN_LENGTH,
                          PIN_SQUARE, PIN_SQUARE,
                          MALE_PIN_LENGTH + MALE_BODY_HEIGHT + BOARD_THICKNESS + MALE_PIN_TOP),
                     name=f'male_pin_{i}', color=GOLD)


def add_daughter_board(assembly, board_bottom_z):
    # Dans le fichier fourni, le PCB est dans le plan XZ, entre Y=0 et Y=1.6.
    # Rotation de 90 deg autour de X => pcb en XY, composants vers +Z.
    rot = cq.Location(cq.Vector(0, 0, board_bottom_z), cq.Vector(1, 0, 0), 90)
    solids = cq.importers.importStep(str(SOURCE)).val().Solids()
    for i, s in enumerate(solids):
        bb=s.BoundingBox()
        if s.Volume() > 200:  # substrat PCB
            color = RED
        elif bb.ymax <= 1.601 and s.Volume() > 6:  # couches PCB
            color = RED
        elif bb.xmin < -4.0 or bb.xmax > 4.0:
            color = GOLD if s.Volume() > 0.8 else SILVER
        elif s.Volume() > 0.4:
            color = DARK
        else:
            color = SILVER
        assembly.add(s, name=f'original_BOB_12009_solid_{i+1}', loc=rot, color=color)


def export_all():
    # 1. Assemblage complet, pret a etre affiche sur le PCB d'accueil KiCad.
    full = cq.Assembly(name='SparkFun_BOB_12009_male_female_headers_2x6')
    add_female_connectors(full)
    add_male_connectors(full, Z_DAUGHTER_BOT)
    add_daughter_board(full, Z_DAUGHTER_BOT)
    p1=ROOT / 'BOB-12009_assemblage_headers_males_femelles.step'
    print('Export',p1,flush=True)
    full.save(str(p1), exportType='STEP')

    # 2. Module avec broches males seulement, base du PCB fille a z=0.
    single = cq.Assembly(name='SparkFun_BOB_12009_male_headers_only')
    add_male_connectors(single, 0)
    add_daughter_board(single, 0)
    p2=ROOT / 'BOB-12009_avec_headers_males.step'
    print('Export',p2,flush=True)
    single.save(str(p2), exportType='STEP')

    # 3. Deux connecteurs femelles seuls, sur le PCB mere.
    female = cq.Assembly(name='Two_female_header_1x6_pitch_2p54')
    add_female_connectors(female)
    p3=ROOT / 'Headers_femelles_2x_1x06_P2.54.step'
    print('Export',p3,flush=True)
    female.save(str(p3), exportType='STEP')

    return [p1,p2,p3]

if __name__=='__main__':
    for p in export_all():
        print(p.name, 'octets', p.stat().st_size)
