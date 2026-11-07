#include "live_config.h"
#include <Preferences.h>
#include <math.h>
#include <string.h>

Calib gCalib;
Table gTable;

static constexpr uint32_t CALIB_NVS_VERSION = 1;   // à incrémenter si un champ change de sens

void Calib::recompute() {
    const float PI_F = 3.14159265f;
    const float cpr  = ENC_PPR * 4.0f;
    mmPerCount  = PI_F * encWheelDiamMm / cpr;
    mmPerCountL = PI_F * encWheelDiamMm * 2.0f / (1.0f + encWheelRatio) / cpr;
    mmPerCountR = PI_F * encWheelDiamMm * 2.0f * encWheelRatio / (1.0f + encWheelRatio) / cpr;
    stepsPerMm  = (float)STEPPER_STEPS_REV / (PI_F * driveWheelDiamMm);
}

static bool allFinite(const Calib& c) {
    const float* f = reinterpret_cast<const float*>(&c);
    for (size_t i = 0; i < sizeof(Calib) / sizeof(float); i++)
        if (!isfinite(f[i])) return false;
    return true;
}

bool calibLoad() {
    Preferences p;
    if (!p.begin("calib", true)) return false;
    Calib c;
    bool ok = p.getUInt("ver", 0) == CALIB_NVS_VERSION &&
              p.getBytesLength("data") == sizeof(Calib) &&
              p.getBytes("data", &c, sizeof(Calib)) == sizeof(Calib) &&
              allFinite(c) && c.encWheelDiamMm > 0 && c.encWheelRatio > 0 &&
              c.encWheelbase > 0 && c.wheelbase > 0 && c.driveWheelDiamMm > 0;
    p.end();
    if (!ok) return false;
    c.recompute();
    gCalib = c;
    return true;
}

bool calibSave() {
    Preferences p;
    if (!p.begin("calib", false)) return false;
    bool ok = p.putBytes("data", &gCalib, sizeof(Calib)) == sizeof(Calib) &&
              p.putUInt("ver", CALIB_NVS_VERSION) > 0;
    p.end();
    return ok;
}

void calibRestoreDefaults() {
    gCalib = Calib();
}

bool calibIsSaved() {
    Preferences p;
    if (!p.begin("calib", true)) return false;
    Calib c;
    bool same = p.getUInt("ver", 0) == CALIB_NVS_VERSION &&
                p.getBytes("data", &c, sizeof(Calib)) == sizeof(Calib) &&
                memcmp(&c, &gCalib, sizeof(Calib)) == 0;
    p.end();
    return same;
}
