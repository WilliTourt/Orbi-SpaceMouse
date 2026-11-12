#include "motion.h"
#include "config.h"
#include <cmath>

static constexpr float MU0 = 1.25663706127e-6f; // Vacuum permeability
static constexpr float MUR = 1.00000037f;       // Relative permeability of air
static constexpr float MU  = MU0 * MUR;

static constexpr float PI = 3.14159265359f;
static constexpr float PI2 = 6.28318530718f;

Motion::Motion() {
    _joystick_pose = Pose();
    _mag[0] = Magnet(); _mag[1] = Magnet(); _mag[2] = Magnet();

    for (int i = 0; i < 3; i++) {
        _mag[i].base = { *CFG_SENSOR_COORDS[i],
                         *(CFG_SENSOR_COORDS[i]+1),
                         *(CFG_SENSOR_COORDS[i]+2) + CFG_HALL_TO_MAGNET_BOTTOM_Z,
                         0.0f, 0.0f, 0.0f };
        _mag[i].pose = _mag[i].base;
        _mag[i].pose_prev = _mag[i].base;
    }
}

// void Motion::begin(float b_stable_meas[9]) {
    
// }

void Motion::calibrate(float b_stable_meas[9]) {
    const float br[3] = {
        _calcBr(b_stable_meas[2], CFG_MAGNET_RADIUS, CFG_MAGNET_LENGTH),
        _calcBr(b_stable_meas[5], CFG_MAGNET_RADIUS, CFG_MAGNET_LENGTH),
        _calcBr(b_stable_meas[8], CFG_MAGNET_RADIUS, CFG_MAGNET_LENGTH)
    };
    for (int i = 0; i < 3; i++) _mag[i].br = br[i];


}

void Motion::update(float b_meas[9]) {

}

float Motion::_calcBr(float bz, float R, float L) {
    const float t = CFG_HALL_TO_MAGNET_BOTTOM_Z / R;
    const float lambda = L / R;

    const float tl = t + lambda;
    const float k = tl / sqrtf(1.0f + tl * tl) - (t / sqrtf(1.0f + t * t));

    return (2.0f * fabsf(bz) / (MUR * k));
}

void Motion::_solveForward(const uint8_t mag_idx, Magnet::MagPose pose, float out[3]) {

    // for better readability
    typedef float vector[3];
    typedef float point[3];

    constexpr uint32_t NUM_DR = 8;
    constexpr uint32_t NUM_DPHI = 16;

    constexpr float dr = CFG_MAGNET_RADIUS / static_cast<float>(NUM_DR);
    constexpr float dphi = PI2 / static_cast<float>(NUM_DPHI);

    constexpr float sin[NUM_DPHI] = {
        0.000000f, 0.382683f, 0.707107f, 0.923880f,
        1.000000f, 0.923880f, 0.707107f, 0.382683f,
        0.000000f,-0.382683f,-0.707107f,-0.923880f,
       -1.000000f,-0.923880f,-0.707107f,-0.382683f
    };
    constexpr float cos[NUM_DPHI] = {
        1.000000f, 0.923880f, 0.707107f, 0.382683f,
        0.000000f,-0.382683f,-0.707107f,-0.923880f,
       -1.000000f,-0.923880f,-0.707107f,-0.382683f,
        0.000000f, 0.382683f, 0.707107f, 0.923880f
    };

    // constexpr vector u = { 1.0f, 0.0f, 0.0f };
    // constexpr vector v = { 0.0f, 1.0f, 0.0f };
    // constexpr vector n = { 0.0f, 0.0f, 1.0f };

    const float cosx = cosf(pose.rx), sinx = sinf(pose.rx);
    const float cosy = cosf(pose.ry), siny = sinf(pose.ry);
    const float cosz = cosf(pose.rz), sinz = sinf(pose.rz);
    const float ROT_MAT[3][3] = {
        {cosy*cosz, cosz*sinx*siny-sinz*cosx, cosz*cosx*siny+sinz*sinx},
        {cosy*sinz, sinz*sinx*siny+cosz*cosx, sinz*cosx*siny-cosz*sinx},
        {-siny    , cosy*sinx               , cosy*cosx               }
    }; // ROT_MAT * UT = U_rotated, V,N as the same

    const vector u_rot = { ROT_MAT[0][0], ROT_MAT[1][0], ROT_MAT[2][0] };
    const vector v_rot = { ROT_MAT[0][1], ROT_MAT[1][1], ROT_MAT[2][1] };
    const vector n_rot = { ROT_MAT[0][2], ROT_MAT[1][2], ROT_MAT[2][2] };

    // s is sensor point
    const float* sensor_coord = CFG_SENSOR_COORDS[mag_idx];
    point s = { sensor_coord[0], sensor_coord[1], sensor_coord[2] };

    // results
    float b[3] = { 0.0f, 0.0f, 0.0f };
    for (int i = 1; i <= NUM_DR; i++) { // integrate over radius

        const float da = 0.5f * static_cast<float>(2 * i - 1) * dr * dr * dphi; // r*dr*dphi
        const float r = static_cast<float>(i - 0.5) * dr; // Center radius of the ring element

        for (int j = 0; j < NUM_DPHI; j++) { // integrate over phi

            // Position of the ring element (center point)
            point qn, qs; // qn is the point on N plane, qs for S plane
            qn[0] = pose.x + r * cos[j] * u_rot[0] + r * sin[j] * v_rot[0];
            qn[1] = pose.y + r * cos[j] * u_rot[1] + r * sin[j] * v_rot[1];
            qn[2] = pose.z + r * cos[j] * u_rot[2] + r * sin[j] * v_rot[2];

            qs[0] = qn[0] + CFG_MAGNET_LENGTH * n_rot[0];
            qs[1] = qn[1] + CFG_MAGNET_LENGTH * n_rot[1];
            qs[2] = qn[2] + CFG_MAGNET_LENGTH * n_rot[2];

            const vector qns = { s[0] - qn[0], s[1] - qn[1], s[2] - qn[2] };
            const vector qss = { s[0] - qs[0], s[1] - qs[1], s[2] - qs[2] };

            const float dist_qn_s = sqrtf(qns[0]*qns[0] + qns[1]*qns[1] + qns[2]*qns[2]);
            const float dist_qs_s = sqrtf(qss[0]*qss[0] + qss[1]*qss[1] + qss[2]*qss[2]);
            const float dist_qn_s3 = dist_qn_s * dist_qn_s * dist_qn_s;
            const float dist_qs_s3 = dist_qs_s * dist_qs_s * dist_qs_s;

            const float field_atten[3] = { da * ((qns[0] / dist_qn_s3)
                                              -  (qss[0] / dist_qs_s3)),
                                           da * ((qns[1] / dist_qn_s3)
                                              -  (qss[1] / dist_qs_s3)),
                                           da * ((qns[2] / dist_qn_s3)
                                              -  (qss[2] / dist_qs_s3)) };
            for (int k = 0; k < 3; k++) b[k] += field_atten[k];
        }
    }

    const float k = MUR * _mag[mag_idx].br / (4.0f * PI);
    for (int i = 0; i < 3; i++) out[i] = k * b[i];
}

void Motion::_solveLM(float b_meas[9], float out_delta[6]) {
    constexpr uint32_t MAX_ITERATIONS = 15;
}
