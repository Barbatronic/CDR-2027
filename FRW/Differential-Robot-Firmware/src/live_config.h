#pragma once
#include "config.h"

// Paramètres de calibration modifiables en RAM via l'interface web.
// Initialisés depuis config.h, puis remplacés au démarrage par la dernière
// sauvegarde en mémoire non volatile (NVS) si elle existe : calibLoad().
// Thread safety : écritures isolées et rares (calibration seulement).
// Les float 32-bit sur ESP32 sont alignés → lecture/écriture atomiques.
struct Calib {
    // ── Mécanique ────────────────────────────────────────────────────────────
    float encWheelDiamMm   = ENC_WHEEL_DIAM_MM;   // diamètre moyen des roues codeuses
    float encWheelRatio    = ENC_WHEEL_RATIO;     // diamètre droit / gauche
    float encWheelbase     = ENC_WHEELBASE_MM;
    float wheelbase        = WHEELBASE_MM;
    float driveWheelDiamMm = DRIVE_WHEEL_DIAM_MM;
    float lidarOffsetDeg   = LIDAR_OFFSET_DEG;

    // Dérivés — recalculés par recompute() quand un diamètre change.
    // Gauche / droite : D_G = 2·D / (1 + r), D_D = 2·D·r / (1 + r)
    float mmPerCount  = 3.14159265f * ENC_WHEEL_DIAM_MM / (ENC_PPR * 4.0f);   // moyenne
    float mmPerCountL = 3.14159265f * ENC_WHEEL_DIAM_MM * 2.0f / (1.0f + ENC_WHEEL_RATIO)
                        / (ENC_PPR * 4.0f);
    float mmPerCountR = 3.14159265f * ENC_WHEEL_DIAM_MM * 2.0f * ENC_WHEEL_RATIO
                        / (1.0f + ENC_WHEEL_RATIO) / (ENC_PPR * 4.0f);
    float stepsPerMm  = (float)STEPPER_STEPS_REV / (3.14159265f * DRIVE_WHEEL_DIAM_MM);

    // ── PID ──────────────────────────────────────────────────────────────────
    float kp           = ENC_P1_KP;
    float ki           = ENC_P1_KI;
    float kd           = ENC_P1_KD;
    float iMax         = ENC_P1_I_MAX;
    float stopMm       = ENC_P1_STOP_MM;
    float minSpd       = ENC_P1_MIN_SPD;
    float defaultSpeed = DEFAULT_SPEED_MMS;
    float defaultAccel = DEFAULT_ACCEL_MMS2;

    void recompute();
};

extern Calib gCalib;

// ── Sauvegarde NVS ───────────────────────────────────────────────────────────
// Format : la structure entière + une version. Une sauvegarde d'une autre
// version ou d'une autre taille (champ ajouté) est ignorée au chargement.
bool calibLoad();            // au démarrage ; true si une sauvegarde a été appliquée
bool calibSave();            // RAM → NVS
void calibRestoreDefaults(); // RAM ← config.h (la sauvegarde NVS reste)
bool calibIsSaved();         // RAM identique à la sauvegarde NVS

// Dimensions de la table en cours : match (3000 × 2000) ou essai (1000 × 700).
// Basculé depuis l'interface, uniquement avant ou après un match.
struct Table {
    volatile bool essai = TERRAIN_ESSAI;
    float w() const { return essai ? ESSAI_WIDTH_MM  : TABLE_WIDTH_MM;  }
    float h() const { return essai ? ESSAI_HEIGHT_MM : TABLE_HEIGHT_MM; }
};

extern Table gTable;
