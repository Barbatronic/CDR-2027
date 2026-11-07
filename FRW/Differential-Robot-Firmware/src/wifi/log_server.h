#pragma once
#include <Arduino.h>
#include "../config.h"

#if WIFI_LOG_ENABLED

#include <WebServer.h>
#include <WebSocketsServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "../lidar/ld06.h"

// ─── Structures ──────────────────────────────────────────────────────────────

struct LogEntry {
    char     level;
    char     tag[16];
    char     msg[96];
    uint32_t ts;
};

struct ServoCmd {
    char  id[24];   // "flywheels" | "flywheelLeft" | "flywheelRight" | "feeder" | "servoBasG" | "servoBasD" | "seq_*"
    float val;      // 0-100 (%)
};

// ─── LogServer ───────────────────────────────────────────────────────────────

class LogServer {
public:
    void begin();
    void push(char level, const char* tag, const char* msg);
    void updatePose(float x, float y, float theta_deg, const char* state,
                    float nav_dist_mm, float nav_delta_deg, char team, bool canClick);
    void updateLidar(const LidarPoint* buf, uint16_t n);
    void updateLidarAbs(const LidarPoint* buf, uint16_t n,
                        float robot_x, float robot_y, float robot_theta_rad);
    void updateActuators(float fwL, float fwR, float fd, float sbg, float sbd);
    // État lisible pour l'UI, envoyé avec la pose : phase (MatchPhase),
    // chrono de match, moteurs alimentés, moteurs verrouillés (fin de match).
    void updateStatus(uint8_t phase, uint32_t elapsedMs, bool motorsOn, bool motorsLocked);
    void calibResult(const char* json);
    // Motion debug : état/phase + cible courante + vitesses commandées.
    // state/phase codés en uint8_t (cf. MotionController::State/Phase).
    void updateMotion(uint8_t state, uint8_t phase,
                      float distMm, float speedCap,
                      float vL, float vR,
                      float tgtX, float tgtY, float tgtTheta);
    bool pollCmd(ServoCmd& out);
    bool pollFieldClick(float& x_mm, float& y_mm, float& theta_deg);
    // Parcours cliqué dans l'interface (mode Parcours) : n ≤ 8 points
    bool pollPath(float* xs, float* ys, uint8_t& n, float& theta_deg);
    // Nombre de pages connectées : sans client, inutile de construire les
    // trames LIDAR (main.cpp)
    uint8_t clientCount() const { return _clients; }

private:
    static void _task(void* arg);
    void        _loop();
    void        _handleWsMsg(uint8_t* payload, size_t len);
    void        _handleCalibMsg(const char* p);
    void        _buildCalibJson();
    void        _sendConfig(uint8_t num);
    bool        _originOk(const String& origin) const;

    WebServer        _http{80};
    WebSocketsServer _ws{81};
    QueueHandle_t    _logQueue{nullptr};
    QueueHandle_t    _cmdQueue{nullptr};
    volatile uint8_t _clients{0};

    // Les données ci-dessous sont écrites par la boucle Arduino ou la tâche
    // stratégie (cœur 1) et envoyées par la tâche WiFi (cœur 0) : chaque
    // écriture et chaque copie avant envoi se font sous ce verrou, pour ne
    // jamais envoyer une trame à moitié mise à jour.
    portMUX_TYPE     _mux = portMUX_INITIALIZER_UNLOCKED;
    char             _sendBuf[5120];   // copie d'une trame LIDAR avant envoi (tâche WiFi)

    // Pose
    volatile float _px{0}, _py{0}, _ptheta{0};
    volatile float _navDist{0}, _navDelta{0};
    volatile bool  _poseNew{false};
    volatile bool  _canClick{false};
    volatile char  _team{'?'};
    char           _pstate[16]{"?"};
    volatile uint8_t  _phase{0};
    volatile uint32_t _elapsedMs{0};
    volatile bool     _motorsOn{false}, _motorsLocked{false};

    // LIDAR radar (relatif robot)
    char          _lidarJson[5120];
    volatile bool _lidarNew{false};

    // LIDAR absolu (coordonnées table)
    char          _lidarAbsJson[5120];
    volatile bool _lidarAbsNew{false};

    // Actionneurs
    volatile float _actFwL{0}, _actFwR{0}, _actFd{0}, _actSbg{-1}, _actSbd{-1};
    volatile bool  _actNew{false};

    // Motion debug
    volatile uint8_t _motSt{0}, _motPh{0};
    volatile float   _motDist{0}, _motCap{0}, _motVL{0}, _motVR{0};
    volatile float   _motTgtX{0}, _motTgtY{0}, _motTgtTheta{0};
    volatile bool    _motNew{false};

    // Field click (utilisateur clique sur le terrain dans l'UI).
    // _fcTheta = NAN si l'utilisateur n'a pas drag (pas de contrainte orientation).
    volatile float _fcX{0}, _fcY{0}, _fcTheta{0};
    volatile bool  _fcHasTheta{false};
    volatile bool  _fcPending{false};

    // Parcours cliqué (8 points max), même protection que le clic simple
    float          _pathX[8]{}, _pathY[8]{};
    uint8_t        _pathN{0};
    float          _pathTheta{NAN};
    volatile bool  _pathPending{false};

    // Calibration
    char          _calibJson[512];
    volatile bool _calibNew{false};

    // Résultat de routine de calibration en attente d'envoi
    char          _calresJson[320];
    volatile bool _calresNew{false};

    // Envoi du dernier mouvement enregistré (motion/recorder.h), par blocs
    void     _sendRecChunk();
    uint32_t _recGen{0};      // enregistrement en cours d'envoi
    uint16_t _recIdx{0};      // prochain échantillon à envoyer
};

extern LogServer logServer;

// ─── Fonctions libres ────────────────────────────────────────────────────────
void wifiLogBegin();
void wifiLogPush(char level, const char* tag, const char* msg);
void wifiLogUpdatePose(float x, float y, float theta_deg, const char* state,
                       float nav_dist_mm, float nav_delta_deg, char team, bool canClick);
void wifiLogLidar(const LidarPoint* buf, uint16_t n);
void wifiLogLidarAbs(const LidarPoint* buf, uint16_t n,
                     float robot_x, float robot_y, float robot_theta_rad);
void wifiLogUpdateActuators(float fwL, float fwR, float fd, float sbg, float sbd);
void wifiLogUpdateStatus(uint8_t phase, uint32_t elapsedMs, bool motorsOn, bool motorsLocked);
// Résultat d'une routine de calibration (JSON complet, type "calres")
void wifiLogCalibResult(const char* json);
void wifiLogUpdateMotion(uint8_t state, uint8_t phase,
                         float distMm, float speedCap,
                         float vL, float vR,
                         float tgtX, float tgtY, float tgtTheta);
bool wifiPollCmd(ServoCmd& out);
// true si au moins une page est connectée
bool wifiLogHasClient();

// Click utilisateur sur le terrain. Renvoie true si une commande était en
// attente (et remet le flag à false). x/y en mm coordonnées table.
// theta_deg = NAN si pas de contrainte orientation (click sans drag).
bool wifiPollFieldClick(float& x_mm, float& y_mm, float& theta_deg);
// Parcours cliqué : xs/ys de 8 places ; theta_deg = NAN sans cap final
bool wifiPollPath(float* xs, float* ys, uint8_t& n, float& theta_deg);

#else

#include "../lidar/ld06.h"

inline void wifiLogBegin() {}
inline void wifiLogPush(char, const char*, const char*) {}
inline void wifiLogUpdatePose(float, float, float, const char*, float, float, char, bool) {}
inline void wifiLogLidar(const LidarPoint*, uint16_t) {}
inline void wifiLogLidarAbs(const LidarPoint*, uint16_t, float, float, float) {}
inline void wifiLogUpdateActuators(float, float, float, float, float) {}
inline void wifiLogUpdateStatus(uint8_t, uint32_t, bool, bool) {}
inline void wifiLogCalibResult(const char*) {}
inline void wifiLogUpdateMotion(uint8_t, uint8_t, float, float, float, float, float, float, float) {}
struct ServoCmd { char id[24]; float val; };
inline bool wifiPollCmd(ServoCmd&) { return false; }
inline bool wifiLogHasClient() { return false; }
inline bool wifiPollFieldClick(float&, float&, float&) { return false; }
inline bool wifiPollPath(float*, float*, uint8_t&, float&) { return false; }

#endif // WIFI_LOG_ENABLED
