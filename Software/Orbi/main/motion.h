#pragma once

#include <cstdint>

class Motion {
    public:
        struct Pose {
            float x, y, z;
            float rx, ry, rz;
        };

        Motion();
        ~Motion();

        Pose getPose() { return _joystick_pose; };

        // void begin(float b_stable_meas[9]);
        void calibrate(float b_stable_meas[9]);
        void update(float b_meas[9]);

    private:
        struct Magnet {
            /*** Magnetic ***/

            float br; // mT
            float b_meas[3]; // bx, by, bz
            float b_sign[3]; // sign of the position and orientation

            /*** Position and orientation ***/

            struct MagPose {
                float x, y, z;
                float rx, ry, rz;
            };
            MagPose pose, pose_prev;
            MagPose base; // base position and orientation
            float vel[6]; // vx, vy, vz, vrx, vry, vrz
        } _mag[3];

        struct JacobianMatrix {
            float j[9][6]; // 9 rows, 6 columns
            float j_transpose[6][9]; // 6 rows, 9 columns
            float jtj[6][6];
            float jtj_inv[6][6];
        } _j_mats;

        Pose _joystick_pose;

        float _calcBr(float bz, float R, float L);
        void _solveForward(const uint8_t mag_idx, Magnet::MagPose pose, float out[3]); // Calculate single magnet's magnetic field
        void _solveLM(float b_meas[9], float out_delta[6]);
};
