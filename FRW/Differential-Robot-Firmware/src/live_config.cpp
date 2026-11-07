#include "live_config.h"
#include <Preferences.h>
#include <math.h>
#include <string.h>

Calib gCalib;
Table gTable;

static constexpr uint32_t CALIB_NVS_VERSION = 2;   // à incrémenter si un champ change de sens

// Version 1 (avant le lot 3) : même mécanique, anciens réglages du PID par
// roue. Au chargement, seules la mécanique et les vitesses par défaut sont
// reprises ; l'asservissement repart des valeurs de config.h.
struct CalibV1 {
    float encWheelDiamMm, encWheelRatio, encWheelbase, wheelbase, driveWheelDiamMm, lidarOffsetDeg;
    float mmPerCount, mmPerCountL, mmPerCountR, stepsPerMm;
    float kp, ki, kd, iMax, stopMm, minSpd, defaultSpeed, defaultAccel;
};

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
    const uint32_t ver = p.getUInt("ver", 0);
    bool ok = false;
    if (ver == CALIB_NVS_VERSION) {
        ok = p.getBytesLength("data") == sizeof(Calib) &&
             p.getBytes("data", &c, sizeof(Calib)) == sizeof(Calib);
    } else if (ver == 1) {
        CalibV1 o;
        ok = p.getBytesLength("data") == sizeof(CalibV1) &&
             p.getBytes("data", &o, sizeof(CalibV1)) == sizeof(CalibV1);
        if (ok) {
            c.encWheelDiamMm   = o.encWheelDiamMm;
            c.encWheelRatio    = o.encWheelRatio;
            c.encWheelbase     = o.encWheelbase;
            c.wheelbase        = o.wheelbase;
            c.driveWheelDiamMm = o.driveWheelDiamMm;
            c.lidarOffsetDeg   = o.lidarOffsetDeg;
            c.defaultSpeed     = o.defaultSpeed;
            c.defaultAccel     = o.defaultAccel;
        }
    }
    ok = ok && allFinite(c) && c.encWheelDiamMm > 0 && c.encWheelRatio > 0 &&
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
