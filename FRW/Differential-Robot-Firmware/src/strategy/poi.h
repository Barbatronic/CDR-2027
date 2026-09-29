#pragma once

struct Vec2 {
    float x, y;
    constexpr Vec2(float x, float y) : x(x), y(y) {}
};

namespace POI{

    // ------------------------------------------
    // Departure Areas - Center
    // TODO 2027 : reprendre les zones de départ du nouveau règlement
    const Vec2 startYellow = Vec2(375,225);
    const Vec2 startBlue = Vec2(2625,225);

}
