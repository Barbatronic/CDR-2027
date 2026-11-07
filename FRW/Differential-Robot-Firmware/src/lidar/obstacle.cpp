#include "obstacle.h"
#include <Arduino.h>
#include <math.h>
#include "../config.h"
#include "../live_config.h"
#include "../display/oled.h"

static constexpr float PI_F    = 3.14159265f;
static constexpr float DEG2RAD = PI_F / 180.0f;
static constexpr float RAD2DEG = 180.0f / PI_F;

// Demande (écrite par les tâches mouvement / stratégie) et résultat (tâche LIDAR).
// Lectures et écritures de 32 bits isolées : atomiques sur ESP32.
static volatile float        sDir      = 0.0f;
static volatile bool         sActive   = false;
static volatile uint32_t     sReqSeq   = 0;
static volatile uint32_t     sResSeq   = 0;
static volatile bool         sFound    = false;
static volatile ObstacleMode sMode     = ObstacleMode::SIMPLE;
static portMUX_TYPE          sMux      = portMUX_INITIALIZER_UNLOCKED;

static LidarPoint sScan[LD06_SCAN_BUF_SIZE];   // tâche LIDAR uniquement

void obstacleSetMode(ObstacleMode m) { sMode = m; }
ObstacleMode obstacleMode()          { return sMode; }

void obstacleWatch(float dir_rad) {
    portENTER_CRITICAL(&sMux);
    sDir    = dir_rad;
    sActive = true;
    sReqSeq = sReqSeq + 1;
    portEXIT_CRITICAL(&sMux);
}

void obstacleUnwatch() {
    portENTER_CRITICAL(&sMux);
    sActive = false;
    sReqSeq = sReqSeq + 1;
    portEXIT_CRITICAL(&sMux);
}

bool obstacleDetected() {
    portENTER_CRITICAL(&sMux);
    bool r = sActive && sResSeq == sReqSeq && sFound;
    portEXIT_CRITICAL(&sMux);
    return r;
}

// Un point du scan dans la zone surveillée ? Renvoie la distance projetée.
static bool scanZone(uint16_t n, float dir_rad, bool wallFilter, float& minFwd, float& minAngleDeg) {
    const float rTheta = gDisplay.enc_pose_theta_rad;
    const float rx     = gDisplay.enc_pose_x_mm;
    const float ry     = gDisplay.enc_pose_y_mm;
    const float delta  = dir_rad - rTheta;
    const float cd = cosf(delta), sd = sinf(delta);
    bool found = false;
    minFwd = OBS_DETECT_DIST_MM;
    minAngleDeg = 0;

    for (uint16_t i = 0; i < n; i++) {
        if (sScan[i].distance_mm == 0) continue;
        if (sScan[i].confidence < OBS_CONFIDENCE_MIN) continue;
        const float d = (float)sScan[i].distance_mm;
        if (d < LIDAR_BODY_DIST_MM) continue;

        // Zones aveugles (poteaux structurels)
        const float rDeg = fmodf(270.0f - sScan[i].angle_deg + gCalib.lidarOffsetDeg + 360.0f, 360.0f);
        if ((rDeg >= LIDAR_BLIND_L_START && rDeg <= LIDAR_BLIND_L_END) ||
            (rDeg >= LIDAR_BLIND_R_START && rDeg <= LIDAR_BLIND_R_END)) continue;

        const float la = (270.0f - sScan[i].angle_deg + gCalib.lidarOffsetDeg) * DEG2RAD;
        const float px = d * cosf(la);
        const float py = d * sinf(la);

        if (wallFilter) {
            // Point en coordonnées table (Y+ vers le bas) : bordures ignorées
            const float a  = la + rTheta;
            const float wx = rx + d * cosf(a);
            const float wy = ry - d * sinf(a);
            if (wx < TABLE_MARGIN_MM || wx > gTable.w() - TABLE_MARGIN_MM) continue;
            if (wy < TABLE_MARGIN_MM || wy > gTable.h() - TABLE_MARGIN_MM) continue;
        }

        const float fwd =  px * cd + py * sd;
        const float lat = -px * sd + py * cd;
        if (fwd > OBS_MIN_DIST_MM && fwd < OBS_DETECT_DIST_MM && fabsf(lat) < OBS_WIDTH_MM * 0.5f) {
            if (fwd < minFwd) { minFwd = fwd; minAngleDeg = atan2f(py, px) * RAD2DEG; }
            found = true;
        }
    }
    return found;
}

void obstacleUpdate(LD06& lidar) {
    static uint32_t last = 0;
    const uint32_t now = millis();
    if (now - last < OBS_POLL_MS) return;
    last = now;

    portENTER_CRITICAL(&sMux);
    const bool     active = sActive;
    const float    dir    = sDir;
    const uint32_t seq    = sReqSeq;
    portEXIT_CRITICAL(&sMux);
    if (!active) return;

    const uint16_t n = lidar.getScan(sScan, LD06_SCAN_BUF_SIZE);
    float fwd = 0, ang = 0;
    const bool found = n > 0 &&
        scanZone(n, dir, sMode == ObstacleMode::WALL_FILTERED, fwd, ang);
    if (found) {
        gDisplay.obs_dist_mm   = fwd;
        gDisplay.obs_angle_deg = ang;
    }

    portENTER_CRITICAL(&sMux);
    if (sReqSeq == seq) {       // demande inchangée pendant l'évaluation
        sFound  = found;
        sResSeq = seq;
    }
    portEXIT_CRITICAL(&sMux);
}
