#pragma once
#include "../motion/step_control.h"
#include "../motion/motion_ctrl.h"
#include "../motion/encoder.h"
#include "../lidar/obstacle.h"
#include "../config.h"
#include "poi.h"
#include "../display/oled.h"

// Direction de déplacement pour gotoXYenc
constexpr bool FORWARD = false;
constexpr bool REAR    = true;

// Interface haut niveau du robot.
// Toutes les méthodes de déplacement sont BLOQUANTES : elles rendent la main
// quand le mouvement est terminé. Elles passent par MotionController (profil +
// asservissement sur les roues codeuses), sauf goStall et turnOpen, en boucle
// ouverte volontairement.
// Retour true = arrivé dans la tolérance ; false = fin de match, repli de fin
// de match, mouvement préempté ou non convergé.
class Robot {
public:
    Robot(StepControl &motion, MotionController &motionCtrl);

    // ── Déplacements ─────────────────────────────────────────────────────────
    // Recul / avance en boucle ouverte jusqu'au blocage contre une bordure :
    // sans asservissement de cap, pour que le robot se plaque à plat.
    // timeoutMs   : durée max totale avant abandon (sécurité)
    // stallConfirmMs : durée min de non-mouvement pour valider le stall (anti-frottement)
    bool goStall(float mm, uint32_t timeoutMs = STALL_TIMEOUT_MS, uint32_t stallConfirmMs = STALL_CONFIRM_MS);
    bool goPID(float mm);                           // ligne droite relative (mm)
    bool turnPID(float deg);                        // rotation relative, positif = gauche
    // Rotation des moteurs en boucle ouverte, vitesse imposée (calibration)
    void turnOpen(float deg, float speedMmS, float accelMmS2) { _motion.turnAt(deg, speedMmS, accelMmS2); }
    bool gotoXYenc(float x_mm, float y_mm);
    bool gotoXYenc(float x_mm, float y_mm, float arrival_deg, bool backward = false);
    bool gotoXYenc(Vec2 poi)                                        { return gotoXYenc(poi.x, poi.y); }
    bool gotoXYenc(Vec2 poi, float arrival_deg, bool backward = false) { return gotoXYenc(poi.x, poi.y, arrival_deg, backward); }
    void setEncoders(QuadEncoder *l, QuadEncoder *r) { _encLeft = l; _encRight = r; }

    // ── Pose ─────────────────────────────────────────────────────────────────
    void setPosition(float x_mm, float y_mm, float theta_deg);

    // ── Cinématique ───────────────────────────────────────────────────────────
    void  setSpeed(float mmS);
    void  setAcceleration(float mmS2)   { _motion.setAcceleration(mmS2); }
    void  setSpeedPct(float speedPct, float accelPct = -1.0f);
    void  resetSpeed();
    float getSpeed()        const { return _motion.getSpeed(); }
    float getAcceleration() const { return _motion.getAcceleration(); }

    // ── API motion non-bloquante (taskMotionControl) ─────────────────────────
    // Met à jour la cible courante de la tâche motion (retour immédiat).
    void setTarget(float x_mm, float y_mm);
    void setTarget(float x_mm, float y_mm, float arrival_deg, bool backward = false);
    void setTarget(Vec2 poi)                                   { setTarget(poi.x, poi.y); }
    void setTarget(Vec2 poi, float arrival_deg, bool backward = false) {
        setTarget(poi.x, poi.y, arrival_deg, backward);
    }
    // Maintient la pose courante en mode HOLD.
    void holdPosition() { _motionCtrl.holdPosition(); }
    // Libère la tâche motion → IDLE (moteurs en roue libre).
    void releaseMotion() { _motionCtrl.release(); }
    // Bloque jusqu'à convergence (ou distance < blendMm si > 0).
    // Surveille aussi match over / endgame et libère la tâche motion.
    bool waitArrived(float blendMm = 0.0f);
    MotionController::State getMotionState() const { return _motionCtrl.getState(); }

    // ── Détection obstacle ───────────────────────────────────────────────────
    enum class DetectMode : uint8_t {
        SIMPLE,
        WALL_FILTERED,
    };
    void enableObstacle()             { _obstacleEn = true;  }
    void disableObstacle()            { _obstacleEn = false; }
    bool obstacleEnabled() const      { return _obstacleEn; }
    void setDetectMode(DetectMode m)  {
        obstacleSetMode(m == DetectMode::WALL_FILTERED ? ObstacleMode::WALL_FILTERED
                                                       : ObstacleMode::SIMPLE);
    }

    // ── Moteurs ──────────────────────────────────────────────────────────────
    void disableMotors()  { _motionCtrl.release(); _motion.disableMotors(); }
    void enableMotors()   { _motion.enableMotors();  }

    // ── Chrono de match ──────────────────────────────────────────────────────
    void     startMatch();
    // Remet le chrono à zéro (pas de match en cours) et réarme les interruptions
    // endgame : à appeler avant toute relance sans redémarrage de la carte.
    void     resetMatch();
    void     startNearEnd() { _nearEndMode = true; }  // désactive les interruptions isEndgame()
    uint32_t matchElapsed() const;
    bool     isEndgame()    const;
    bool     isMatchOver()  const;
    // Bloque jusqu'à ce que le chrono de match atteigne target_ms (ou fin de match).
    // Usage dans runNearEndXxx : robot.waitMatchTime(95000); // attend la 95e seconde
    void     waitMatchTime(uint32_t target_ms);

    // ── Accesseurs état : pose des roues codeuses (la seule pose de référence)
    float getX()        const { return gDisplay.enc_pose_x_mm; }
    float getY()        const { return gDisplay.enc_pose_y_mm; }
    float getTheta()    const { return gDisplay.enc_pose_theta_rad; }
    float getThetaDeg() const { return gDisplay.enc_pose_theta_deg; }

private:
    StepControl       &_motion;
    MotionController  &_motionCtrl;
    bool               _obstacleEn = true;

    bool          _nearEndMode = false;
    QuadEncoder  *_encLeft  = nullptr;
    QuadEncoder  *_encRight = nullptr;

    bool _moveAllowed() const;          // false en fin de match / repli
    bool _waitObstacleClear();
};
