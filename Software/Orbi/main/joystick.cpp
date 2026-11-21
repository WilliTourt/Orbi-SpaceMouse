#include "joystick.h"

// #include "esp_log.h"

bool Joystick::begin() {
    const bool ok1 = _s1.begin();
    const bool ok2 = _s2.begin();
    const bool ok3 = _s3.begin();

    if (!(ok1 && ok2 && ok3)) return false;
    return true;
}

bool Joystick::sensorsOk() const {
    return _s1.isReady() && _s2.isReady() && _s3.isReady();
}

bool Joystick::_sample(float out[9]) {
    TMAG3001 *s[3] = { &_s1, &_s2, &_s3 };

    for (uint8_t i = 0; i < 3; i++) {
        if (!s[i]->read()) return false;

        const TMAG3001::Sample m = s[i]->getMt();
        out[i * 3 + 0] = m.x_mT;
        out[i * 3 + 1] = m.y_mT;
        out[i * 3 + 2] = m.z_mT;
    }
    return true;
}

bool Joystick::calibrate() {
    float avg[9] = {};

    for (uint16_t n = 0; n < CALIB_SAMPLES; n++) {
        float sample[9];
        if (!_sample(sample)) return false;
        for (int i = 0; i < 9; i++) avg[i] += sample[i];
    }

    const float inv = 1.0f / (float)CALIB_SAMPLES;
    for (int i = 0; i < 9; i++) {
        avg[i] *= inv;
        _b[i] = avg[i];
    }

    // Br, bias and the resting pose are all from this sample frame
    _motion.begin(avg);
    _pose = _motion.getPose();
    _calibrated = true;

    // ESP_LOGI("joystick", "calibrated: rest u1 %.3f/%.3f/%.3f mT",
    //          avg[0], avg[1], avg[2]);
    return true;
}

bool Joystick::update() {
    if (!_sample(_b)) return false;

    _motion.update(_b);
    _pose = _motion.getPose();
    return true;
}
