#pragma once

struct Vec2 {
    float x, y;
    constexpr Vec2(float x, float y) : x(x), y(y) {}
};

// Points d'intérêt — règlement Eurobot 2027 « The legend of Camelot » (BETA 0.4)
// Repère firmware : origine coin arrière gauche (vu du public), X → droite
// (3000 mm), Y → vers le public (2000 mm). Jaune à gauche, bleu à droite.
// Relevés sur le plan de l'annexe G.1 (précision ±5 mm), centres des zones.
namespace POI{

    // ------------------------------------------
    // Salle du trône — aire de départ/arrivée robot (500 × 500)
    const Vec2 startYellow = Vec2(250,1000);
    const Vec2 startBlue   = Vec2(2750,1000);

    // ------------------------------------------
    // Écuries — aires de départ PAMI (200 × 300)
    const Vec2 ecurieYellow_1 = Vec2(100,150);
    const Vec2 ecurieYellow_2 = Vec2(100,1850);
    const Vec2 ecurieBlue_1   = Vec2(2900,150);
    const Vec2 ecurieBlue_2   = Vec2(2900,1850);

    // ------------------------------------------
    // Cour du château — cible des boulets (point de visée approximatif)
    const Vec2 courYellow = Vec2(450,1000);
    const Vec2 courBlue   = Vec2(2550,1000);

    // ------------------------------------------
    // Remparts — tours (Ø200, 1 pierre verticale)
    const Vec2 tourYellow_1 = Vec2(500,400);
    const Vec2 tourYellow_2 = Vec2(850,750);
    const Vec2 tourYellow_3 = Vec2(850,1250);
    const Vec2 tourYellow_4 = Vec2(500,1600);

    const Vec2 tourBlue_1 = Vec2(2500,400);
    const Vec2 tourBlue_2 = Vec2(2150,750);
    const Vec2 tourBlue_3 = Vec2(2150,1250);
    const Vec2 tourBlue_4 = Vec2(2500,1600);

    // ------------------------------------------
    // Remparts — murs (150 × 400, mur 3 pierres ou porte)
    //   _1 / _5 : le long de X   _2 / _4 : diagonale 45°   _3 : le long de Y (face au centre)
    const Vec2 murYellow_1 = Vec2(250,400);
    const Vec2 murYellow_2 = Vec2(675,575);
    const Vec2 murYellow_3 = Vec2(850,1000);
    const Vec2 murYellow_4 = Vec2(675,1425);
    const Vec2 murYellow_5 = Vec2(250,1600);

    const Vec2 murBlue_1 = Vec2(2750,400);
    const Vec2 murBlue_2 = Vec2(2325,575);
    const Vec2 murBlue_3 = Vec2(2150,1000);
    const Vec2 murBlue_4 = Vec2(2325,1425);
    const Vec2 murBlue_5 = Vec2(2750,1600);

    // ------------------------------------------
    // Douves — cible des PAMI adverses
    const Vec2 douveYellow_1 = Vec2(820,430);
    const Vec2 douveYellow_2 = Vec2(1050,1000);
    const Vec2 douveYellow_3 = Vec2(820,1570);

    const Vec2 douveBlue_1 = Vec2(2180,430);
    const Vec2 douveBlue_2 = Vec2(1950,1000);
    const Vec2 douveBlue_3 = Vec2(2180,1570);

    // ------------------------------------------
    // Carrières de pierre — pile de 3 pierres couchées (emprise 320 × 110)
    // Le texte BETA dit « verticalement » : coquille, elles sont bien à l'horizontale.
    // Attention : le plan BETA place la 4e carrière à X=2300 (pas 2250, non symétrique)
    const Vec2 carriere_01 = Vec2(750,55);      // bord du fond
    const Vec2 carriere_02 = Vec2(1250,55);
    const Vec2 carriere_03 = Vec2(1750,55);
    const Vec2 carriere_04 = Vec2(2300,55);
    const Vec2 carriere_05 = Vec2(1500,500);    // centre, alignée selon Y
    const Vec2 carriere_06 = Vec2(1500,1500);   // centre, alignée selon Y
    const Vec2 carriere_07 = Vec2(750,1945);    // bord côté public
    const Vec2 carriere_08 = Vec2(1250,1945);
    const Vec2 carriere_09 = Vec2(1750,1945);
    const Vec2 carriere_10 = Vec2(2300,1945);

}
