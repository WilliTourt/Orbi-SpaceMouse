#pragma once

#include "config.h"
#include "motion.h"
#include "tmag3001.h"

class Joystick {
    public:

        bool begin();
        bool calibrate(); // must be called with the stick untouched

        // one sample + one LM step
        // returns false if a sensor read failed
        bool update();

        bool isCalibrated() const { return _calibrated; }
        bool sensorsOk() const;

        const Motion::Pose& getPose() const { return _pose; }
        const float* getSample() const { return _b; }

    private:
        static constexpr uint16_t CALIB_SAMPLES = 50;

        bool _sample(float out[9]);
        
        TMAG3001 _s1{ CFG_IIC_A_SDA, CFG_IIC_A_SCL, TMAG3001::AddrPin::toGND };
        TMAG3001 _s2{ CFG_IIC_A_SDA, CFG_IIC_A_SCL, TMAG3001::AddrPin::toVCC };
        TMAG3001 _s3{ CFG_IIC_B_SDA, CFG_IIC_B_SCL, TMAG3001::AddrPin::toVCC };

        Motion _motion;
        Motion::Pose _pose{}; // latest pose
        float _b[9] = {}; // latest raw values, 9 floats (u1 xyz, u2 xyz, u3 xyz)

        bool _calibrated = false;
};
