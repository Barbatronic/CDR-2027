#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "step_control.h"
#include "encoder.h"
#include "profile.h"
#include "../config.h"

// ─── MotionController ────────────────────────────────────────────────────────
//
//  Tâche FreeRTOS à cadence fixe (MOTION_PERIOD_MS, 100 Hz) qui amène le robot
//  à une cible (X, Y, θ) en trois phases : ALIGN (rotation vers la cible),
//  TRANSLATE (ligne droite), FINAL_TURN (cap final, optionnel).
//
//  Chaque phase suit un profil trapézoïdal calculé dans le temps (profile.h).
//  À chaque cycle, deux boucles corrigent l'écart entre la valeur attendue
//  et la mesure des roues codeuses :
//
//      v = v_ref + KP_DIST · e_dist (+ KI)      ω = ω_ref + KP_ANG · e_angle (+ KI)
//
//  En rotation, ω_ref vient du profil d'angle et la boucle de distance
//  maintient le centre sur place. En ligne droite, v_ref vient du profil de
//  distance (mesurée en projection sur la droite départ → cible) et le cap
//  visé est recalculé vers la cible tant qu'il reste plus de
//  MOTION_HEADING_FREEZE_MM, puis figé.
//
//  Fin de phase : profil terminé et erreur dans la tolérance pendant
//  MOTION_SETTLE_CYCLES cycles. Le résidu est corrigé à vitesse réduite.
//  Abandon (mouvement non convergé) MOTION_SETTLE_MS après la fin du profil.
//
//  Parcours (setPath) : jusqu'à PATH_PTS_MAX points enchaînés. À un point de
//  passage où le cap change de moins de MOTION_PASS_MAX_DEG, le robot ne
//  s'arrête pas : le profil y arrive à une vitesse limitée par l'angle, le
//  segment suivant repart de là avec un virage planifié (profil d'angle
//  comme une rotation sur place, superposé à l'avance).
//  Sinon arrêt, rotation, puis segment suivant. Cap final au dernier point.
//
//  Glissement : à chaque cycle, le déplacement des roues codeuses est
//  comparé à celui que donnent les pas émis aux moteurs. Un écart de plus de
//  MOTION_SLIP_MM sur MOTION_SLIP_CYCLES cycles (roue qui patine, robot qui
//  pousse un obstacle non vu, moteur qui décroche) arrête le mouvement.
//
//  Obstacle : pendant TRANSLATE, la tâche LIDAR surveille la direction du
//  trajet (lidar/obstacle.h) ; la boucle ne fait que lire le résultat.
//
//  Usage stratégie :
//    MotionController::Target t;
//    t.x_mm = 1000; t.y_mm = 500; t.theta_deg = 90;
//    motionCtrl.setTarget(t);
//    motionCtrl.waitArrived();   // bloquant
//
//  Thread safety : la cible est protégée par portMUX ; les compteurs de
//  séquence permettent à waitArrived de détecter l'arrivée ou la préemption.
//
class MotionController {
public:
    enum class State : uint8_t {
        IDLE,       // moteurs libres, tâche inactive
        MOVING,     // mouvement en cours
        HOLD,       // réservé (maintien de position)
        OBSTACLE,   // arrêté, attente dégagement
    };

    enum class Phase : uint8_t {
        NONE,
        ALIGN,          // rotation sur place vers la direction de la cible
        TRANSLATE,      // translation vers (x, y)
        FINAL_TURN,     // rotation finale vers theta
    };

    // Raison d'un mouvement non abouti (lastFail)
    enum class Fail : uint8_t {
        NONE,           // arrivé dans la tolérance
        NOT_CONVERGED,  // résidu hors tolérance 1,5 s après la fin du profil
        FOLLOW_ERR,     // écart anormal à la référence (garde-fou)
        SLIP,           // glissement / blocage détecté
        RELEASED,       // interrompu par release()
    };

    static constexpr uint8_t PATH_PTS_MAX = 8;

    struct Target {
        float x_mm     = 0.0f;
        float y_mm     = 0.0f;
        float theta_deg = NAN;       // NaN = pas de contrainte d'orientation finale
        float speed    = 0.0f;       // mm/s ; 0 = utiliser gCalib.defaultSpeed
        float accel    = 0.0f;       // mm/s² ; 0 = utiliser gCalib.defaultAccel
        float stopMm   = 0.0f;       // mm ; 0 = utiliser gCalib.stopMm
        bool  backward = false;      // forcer approche en marche arrière
        bool  obstacleEn = true;     // activer détection pendant TRANSLATE
    };

    void begin(StepControl* motion, QuadEncoder* encL, QuadEncoder* encR);

    // ── API non-bloquante ────────────────────────────────────────────────────
    void setTarget(const Target& t);
    // Parcours de n points (1 ≤ n ≤ PATH_PTS_MAX) ; theta_deg du dernier seulement
    void setPath(const Target* pts, uint8_t n);
    void holdPosition();
    void release();                  // → IDLE, moteurs libres

    // ── API bloquante ────────────────────────────────────────────────────────
    // Bloque jusqu'à la fin du mouvement (ou distance < blendMm si > 0).
    // Retourne false si préempté par un nouveau setTarget ou release().
    bool waitArrived(float blendMm = 0.0f);

    // Dernier mouvement terminé : arrivé dans la tolérance (false si abandonné
    // faute de convergence, ou interrompu par release)
    bool lastArrivedOk() const { return _doneFail == Fail::NONE; }
    Fail lastFail() const      { return _doneFail; }

    // ── État (lecture) ───────────────────────────────────────────────────────
    State    getState() const     { return _state; }
    Phase    getPhase() const     { return _phase; }
    float    getDistMm() const    { return _distMm; }
    uint32_t getUserSeq() const   { return _userSeq; }
    uint32_t getDoneSeq() const   { return _doneSeq; }
    // Debug : vitesses commandées aux roues (mm/s, signées) et vitesse de
    // référence du profil (mm/s ; en rotation, vitesse équivalente des roues)
    float    getCmdL() const      { return _lastCmdL; }
    float    getCmdR() const      { return _lastCmdR; }
    float    getRefSpeed() const  { return _refSpeed; }
    // Cible courante (valide quand state != IDLE)
    float    getTargetX() const     { return _activeTgt.x_mm; }
    float    getTargetY() const     { return _activeTgt.y_mm; }
    float    getTargetTheta() const { return _activeTgt.theta_deg; }

private:
    static void _taskTrampoline(void* arg);
    void        _loop();

    // Démarrage des phases
    void _startAlign();
    void _startTranslate(float v0);
    void _startFinalTurn();
    void _startRotation(float deltaRad, Phase ph);
    void _complete(Fail f);
    // Fin d'un segment : point suivant du parcours, ou cap final
    void _segmentDone();
    // Vitesse de passage au point courant vers le suivant (0 = arrêt) et
    // distance avant le point où commence le virage
    float _passSpeed(float& lead) const;
    // Glissement : remise à zéro de la fenêtre, contrôle d'un cycle
    void _slipReset();
    bool _slipCheck();
    bool _tryBlend(float tNow);

    // Un cycle de la phase en cours ; true = phase terminée
    bool _stepRotation(float t, float dt);
    bool _stepTranslate(float t, float dt);
    // true = phase terminée (convergée, ou abandon après MOTION_SETTLE_MS)
    bool _settle(float t, float err, float tol);
    // v, w : commande ; vff, wff : partie prévue par le profil. Chaque roue est
    // bornée à ± MOTION_CORR_MAX_MMS autour de la vitesse prévue.
    void _applyWheels(float v, float w, float vff = 0.0f, float wff = 0.0f);

    void _recordStart();
    void _recordSample();

    static float _normAngle(float rad);

    StepControl* _motion = nullptr;
    QuadEncoder* _encL = nullptr;
    QuadEncoder* _encR = nullptr;

    // Parcours demandé (partagé, protégé par mux)
    Target            _userPath[PATH_PTS_MAX];
    uint8_t           _userN = 0;
    volatile uint32_t _userSeq = 0;
    portMUX_TYPE      _mux = portMUX_INITIALIZER_UNLOCKED;

    // État interne (tâche motion)
    volatile State    _state = State::IDLE;
    volatile Phase    _phase = Phase::NONE;
    uint32_t          _handledSeq = 0;
    volatile uint32_t _doneSeq = 0;
    volatile Fail     _doneFail = Fail::NONE;

    // Phase en cours
    Target      _path[PATH_PTS_MAX];     // parcours en cours
    uint8_t     _pathN = 0, _pathIdx = 0;
    float       _segVEnd = 0;        // vitesse de passage en fin de segment (0 = arrêt)
    Target      _activeTgt;          // = _path[_pathIdx]
    bool        _activeGoBack = false;
    TrapProfile _prof;
    uint32_t    _t0Us = 0;           // départ du profil
    float       _x0 = 0, _y0 = 0, _th0 = 0;   // pose au départ de la phase
    float       _ux = 0, _uy = 0;    // translation : direction départ → cible
    float       _sign = 1;           // rotation : sens (+1 = gauche)
    float       _rotMes = 0;         // rotation : angle parcouru mesuré (rad)
    float       _prevTh = 0;
    float       _thRef = 0;          // translation : cap visé
    bool        _thFrozen = false;
    float       _tProf = 0;          // translation : horloge du profil (s), ralentie en virage
    float       _headGuard = 0;      // translation : écart de cap maxi toléré (rad)
    float       _wCmd = 0;           // translation : dernière correction de cap (rad/s)
    // Virage planifié au départ d'un segment enchaîné
    TrapProfile _turnProf;
    bool        _turning = false;
    float       _turnSign = 1, _turnTh0 = 0, _tTurn = 0, _turnV0 = 0;
    float       _lastVRef = 0;       // translation : vitesse de référence suivie au dernier cycle
    float       _vMaxWheel = 0;      // borne des commandes de roue (mm/s)
    float       _iD = 0, _iA = 0;    // intégrales distance (mm/s) et angle (rad/s)
    int         _settleCnt = 0;
    Fail        _phaseFail = Fail::NONE;   // phase abandonnée → cible abandonnée
    bool        _phaseObstacleEn = false;
    float       _phaseMoveDir = 0;

    // Glissement : écarts roue codeuse − pas émis, fenêtre glissante (mm)
    float    _slipBufL[MOTION_SLIP_CYCLES] = {}, _slipBufR[MOTION_SLIP_CYCLES] = {};
    float    _slipSumL = 0, _slipSumR = 0;
    uint8_t  _slipIdx = 0, _slipFill = 0;
    float    _slipML = 0, _slipMR = 0;     // position moteurs au cycle précédent (mm)
    int32_t  _slipCL = 0, _slipCR = 0;     // comptes codeuses au cycle précédent

    // OBSTACLE
    uint32_t _obstacleStartMs = 0;
    bool     _obstacleBackedUp = false;

    // Affichage debug
    volatile float _distMm = 0;
    volatile float _lastCmdL = 0;
    volatile float _lastCmdR = 0;
    volatile float _refSpeed = 0;
    float _dbgErrD = 0, _dbgErrA = 0;   // restant : distance (mm), angle (°)
    // Enregistreur : vitesses mesurées et décimation (50 Hz)
    int32_t  _recPrevL = 0, _recPrevR = 0;
    uint32_t _recPrevUs = 0;
    uint8_t  _recDiv = 0;
};

extern MotionController gMotionCtrl;
