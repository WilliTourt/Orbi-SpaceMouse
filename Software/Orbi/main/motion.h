#pragma once

#include <cstdint>

class Motion {
    public:
        struct Pose { // DO NOT MODIFY STRUCTURE EVER
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
        // for better readability
        typedef float vector[3];
        typedef float point[3];
        typedef float field[3];

        struct Magnet {
            /*** Magnetic ***/

            float br; // mT
            field b_meas; // bx, by, bz
            float b_sign[3]; // sign of the position and orientation

            /*** Position and orientation ***/

            struct MagPose {
                float x, y, z;
                float rx, ry, rz;
            };
            MagPose base; // base position and orientation
            vector pm;    // p is pivot, m is magnet base xyz point
            // float vel[6]; // vx, vy, vz, vrx, vry, vrz
        } _mag[3];

        struct JacobianMatrix {
            float j[9][6]; // 9 rows, 6 columns
            float j_transpose[6][9]; // 6 rows, 9 columns
            float jtj[6][6];
            // float jtj_inv[6][6];

            void solve(float lambda); // solve jt, jtj after j has been filled
        } _j_mats;

        Pose _joystick_pose, _joystick_pose_prev;

        float _calcBr(float bz, float R, float L);

        // single magnet to single sensor
        void _solveForward(const uint8_t mag_idx, const uint8_t sensor_idx,
                           Magnet::MagPose pose, field out_predicted);
        // three magnets to one sensor
        void _solveForward(const uint8_t sensor_idx, Magnet::MagPose pose[3], field sensor_field);

        // convert a joystick pose to a single magnet pose
        Magnet::MagPose _getMagPose(const uint8_t mag_idx, const Pose &joystick_pose);

        // update _j_mats.j (NOTE: only fit for nearby joystick pose)
        void _solveJacobianMat(const Pose &joystick_pose, field b_predict[3]);

        // solve (JtJ + λI) delta = Jt r. return false if A is not positive definite
        bool _cholesky(const float A[6][6], const float b[6], Pose &delta);

        void _solveLM(field b_meas[3], Pose &out_delta);
};
