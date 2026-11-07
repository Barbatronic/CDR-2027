#include "recorder.h"
#include <Arduino.h>

MotionRecorder gRecorder;

void MotionRecorder::start() {
    _ready     = false;      // un envoi en cours de l'ancien est abandonné
    _gen       = _gen + 1;
    _count     = 0;
    _truncated = false;
    _t0        = millis();
    _recording = true;
}

void MotionRecorder::add(const RecSample& s) {
    if (!_recording) return;
    if (_count >= REC_MAX_SAMPLES) { _truncated = true; return; }
    _buf[_count] = s;
    _count = _count + 1;
}

void MotionRecorder::stop() {
    if (!_recording) return;
    _recording = false;
    if (_count > 0) _ready = true;
}
