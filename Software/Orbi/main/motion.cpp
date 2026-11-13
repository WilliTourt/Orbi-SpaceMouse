#include "motion.h"
#include "config.h"
#include <cmath>

static constexpr float MU0 = 1.25663706127e-6f; // Vacuum permeability
static constexpr float MUR = 1.00000037f;       // Relative permeability of air
static constexpr float MU  = MU0 * MUR;

static constexpr float PI = 3.14159265359f;
static constexpr float PI2 = 6.28318530718f;

Motion::Motion() {
    _joystick_pose = Pose(); _joystick_pose_prev = Pose();
    _mag[0] = Magnet(); _mag[1] = Magnet(); _mag[2] = Magnet();

    for (int i = 0; i < 3; i++) {
        _mag[i].base = { *CFG_SENSOR_COORDS[i],
                         *(CFG_SENSOR_COORDS[i]+1),
                         *(CFG_SENSOR_COORDS[i]+2) + CFG_HALL_TO_MAGNET_BOTTOM_Z,
                         0.0f, 0.0f, 0.0f };
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
    field b_meas_field[3];
    for (int i = 0; i < 3; i++) {
        b_meas_field[i][X] = b_meas[i * 3 + 0];
        b_meas_field[i][Y] = b_meas[i * 3 + 1];
        b_meas_field[i][Z] = b_meas[i * 3 + 2];
    }

    Pose delta;

    _solveLM(b_meas_field, delta);
    _joystick_pose.x += delta.x;
    _joystick_pose.y += delta.y;
    _joystick_pose.z += delta.z;
    _joystick_pose.rx += delta.rx;
    _joystick_pose.ry += delta.ry;
    _joystick_pose.rz += delta.rz;
}

float Motion::_calcBr(float bz, float R, float L) {
    const float t = CFG_HALL_TO_MAGNET_BOTTOM_Z / R;
    const float lambda = L / R;

    const float tl = t + lambda;
    const float k = tl / sqrtf(1.0f + tl * tl) - (t / sqrtf(1.0f + t * t));

    return (2.0f * fabsf(bz) / (MUR * k));
}

void Motion::_solveForward(const uint8_t mag_idx, const uint8_t sensor_idx, 
                           Magnet::MagPose pose, field out_predicted) {
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
        { cosy*cosz, cosz*sinx*siny-sinz*cosx, cosz*cosx*siny+sinz*sinx },
        { cosy*sinz, sinz*sinx*siny+cosz*cosx, sinz*cosx*siny-cosz*sinx },
        { -siny    , cosy*sinx               , cosy*cosx                }
    }; // ROT_MAT * UT = U_rotated, V,N as the same

    const vector u_rot = { ROT_MAT[0][0], ROT_MAT[1][0], ROT_MAT[2][0] };
    const vector v_rot = { ROT_MAT[0][1], ROT_MAT[1][1], ROT_MAT[2][1] };
    const vector n_rot = { ROT_MAT[0][2], ROT_MAT[1][2], ROT_MAT[2][2] };

    // s is sensor point
    const float* sensor_coord = CFG_SENSOR_COORDS[sensor_idx];
    point s = { sensor_coord[0], sensor_coord[1], sensor_coord[2] };

    // results
    field b = { 0.0f, 0.0f, 0.0f };
    for (int i = 1; i <= NUM_DR; i++) { // integrate over radius

        const float da = 0.5f * static_cast<float>(2 * i - 1) * dr * dr * dphi; // r*dr*dphi
        const float r = static_cast<float>(i - 0.5) * dr; // Center radius of the ring element

        for (int j = 0; j < NUM_DPHI; j++) { // integrate over phi

            // Position of the ring element (center point)
            point qn, qs; // qn is the point on N plane, qs for S plane
            qn[X] = pose.x + r * cos[j] * u_rot[X] + r * sin[j] * v_rot[X];
            qn[Y] = pose.y + r * cos[j] * u_rot[Y] + r * sin[j] * v_rot[Y];
            qn[Z] = pose.z + r * cos[j] * u_rot[Z] + r * sin[j] * v_rot[Z];

            qs[X] = qn[X] + CFG_MAGNET_LENGTH * n_rot[X];
            qs[Y] = qn[Y] + CFG_MAGNET_LENGTH * n_rot[Y];
            qs[Z] = qn[Z] + CFG_MAGNET_LENGTH * n_rot[Z];

            const vector qns = { s[X] - qn[X], s[Y] - qn[Y], s[Z] - qn[Z] };
            const vector qss = { s[X] - qs[X], s[Y] - qs[Y], s[Z] - qs[Z] };

            const float dist_qn_s = sqrtf(qns[X]*qns[X] + qns[Y]*qns[Y] + qns[Z]*qns[Z]);
            const float dist_qs_s = sqrtf(qss[X]*qss[X] + qss[Y]*qss[Y] + qss[Z]*qss[Z]);
            const float dist_qn_s3 = dist_qn_s * dist_qn_s * dist_qn_s;
            const float dist_qs_s3 = dist_qs_s * dist_qs_s * dist_qs_s;

            // field attenuation (distance related)
            const float field_atten[3] = { da * ((qns[X] / dist_qn_s3)
                                              -  (qss[X] / dist_qs_s3)),
                                           da * ((qns[Y] / dist_qn_s3)
                                              -  (qss[Y] / dist_qs_s3)),
                                           da * ((qns[Z] / dist_qn_s3)
                                              -  (qss[Z] / dist_qs_s3)) };
            for (int k = 0; k < 3; k++) b[k] += field_atten[k];
        }
    }

    const float k = MUR * _mag[mag_idx].br / (4.0f * PI);
    for (int i = 0; i < 3; i++) out_predicted[i] = k * b[i];
}

void Motion::_mergeFieldToSensor(field to_merge[3], field sensor_field) {
    for (int i = 0; i < 3; i++) {
        sensor_field[i] = to_merge[0][i] + to_merge[1][i] + to_merge[2][i];        
    }
}

void Motion::_solveLM(field b_meas[3], Pose &out_delta) {
    constexpr uint32_t MAX_ITERATIONS = 15;
    constexpr float DEADZONE = 0.1f;

    constexpr float DISPLACEMENT_UNIT = 0.001f; // mm
    constexpr float ROTATION_UNIT = 0.0001f;    // Rad

    Pose delta = Pose();
    Magnet::MagPose magpose[3] = { Magnet::MagPose() }; // only for lm solver

    // to get every magnet's x,y,z pose, we need to rotate the vector from pivot to magnet base (pm0~2)
    const vector pm0 = { _mag[0].base.x - CFG_JOYSTICK_PIVOT_COORD[X],
                         _mag[0].base.y - CFG_JOYSTICK_PIVOT_COORD[Y],
                         _mag[0].base.z - CFG_JOYSTICK_PIVOT_COORD[Z] };
    const vector pm1 = { _mag[1].base.x - CFG_JOYSTICK_PIVOT_COORD[X],
                         _mag[1].base.y - CFG_JOYSTICK_PIVOT_COORD[Y],
                         _mag[1].base.z - CFG_JOYSTICK_PIVOT_COORD[Z] };
    const vector pm2 = { _mag[2].base.x - CFG_JOYSTICK_PIVOT_COORD[X],
                         _mag[2].base.y - CFG_JOYSTICK_PIVOT_COORD[Y],
                         _mag[2].base.z - CFG_JOYSTICK_PIVOT_COORD[Z] };
    vector pm0_rot, pm1_rot, pm2_rot;

    // because every iteration changes magpose, calc magnet xyz in while()

    // iters ctrl
    float residual_err = FLT_MAX;
    uint32_t iters = 0;

    field b_predict[3] = { {0}, {0}, {0} }; // sensors' predicted b field
    field b_magnet_temp[3] = { {0}, {0}, {0} }; // temp field for merging

    while ((residual_err > DEADZONE) && (iters++ < MAX_ITERATIONS)) {
        for (int i = 0; i < 3; i++) {
            magpose[i].rx = _joystick_pose.rx;
            magpose[i].ry = _joystick_pose.ry;
            magpose[i].rz = _joystick_pose.rz;
        }
        
        const float cosx = cosf(magpose[0].rx), sinx = sinf(magpose[0].rx);
        const float cosy = cosf(magpose[0].ry), siny = sinf(magpose[0].ry);
        const float cosz = cosf(magpose[0].rz), sinz = sinf(magpose[0].rz);
        const float ROT_MAT[3][3] = {
            { cosy*cosz, cosz*sinx*siny-sinz*cosx, cosz*cosx*siny+sinz*sinx },
            { cosy*sinz, sinz*sinx*siny+cosz*cosx, sinz*cosx*siny-cosz*sinx },
            { -siny    , cosy*sinx               , cosy*cosx                }
        };

        pm0_rot[X] = ROT_MAT[0][0] * pm0[X] + ROT_MAT[0][1] * pm0[Y] + ROT_MAT[0][2] * pm0[Z];
        pm0_rot[Y] = ROT_MAT[1][0] * pm0[X] + ROT_MAT[1][1] * pm0[Y] + ROT_MAT[1][2] * pm0[Z];
        pm0_rot[Z] = ROT_MAT[2][0] * pm0[X] + ROT_MAT[2][1] * pm0[Y] + ROT_MAT[2][2] * pm0[Z];

        pm1_rot[X] = ROT_MAT[0][0] * pm1[X] + ROT_MAT[0][1] * pm1[Y] + ROT_MAT[0][2] * pm1[Z];
        pm1_rot[Y] = ROT_MAT[1][0] * pm1[X] + ROT_MAT[1][1] * pm1[Y] + ROT_MAT[1][2] * pm1[Z];
        pm1_rot[Z] = ROT_MAT[2][0] * pm1[X] + ROT_MAT[2][1] * pm1[Y] + ROT_MAT[2][2] * pm1[Z]; 

        pm2_rot[X] = ROT_MAT[0][0] * pm2[X] + ROT_MAT[0][1] * pm2[Y] + ROT_MAT[0][2] * pm2[Z];
        pm2_rot[Y] = ROT_MAT[1][0] * pm2[X] + ROT_MAT[1][1] * pm2[Y] + ROT_MAT[1][2] * pm2[Z];
        pm2_rot[Z] = ROT_MAT[2][0] * pm2[X] + ROT_MAT[2][1] * pm2[Y] + ROT_MAT[2][2] * pm2[Z];

        // here we got magnet base coords after rotation
        point m0 = { pm0_rot[X] + CFG_JOYSTICK_PIVOT_COORD[X], 
                    pm0_rot[Y] + CFG_JOYSTICK_PIVOT_COORD[Y], 
                    pm0_rot[Z] + CFG_JOYSTICK_PIVOT_COORD[Z] };
        point m1 = { pm1_rot[X] + CFG_JOYSTICK_PIVOT_COORD[X], 
                    pm1_rot[Y] + CFG_JOYSTICK_PIVOT_COORD[Y], 
                    pm1_rot[Z] + CFG_JOYSTICK_PIVOT_COORD[Z] };
        point m2 = { pm2_rot[X] + CFG_JOYSTICK_PIVOT_COORD[X], 
                    pm2_rot[Y] + CFG_JOYSTICK_PIVOT_COORD[Y], 
                    pm2_rot[Z] + CFG_JOYSTICK_PIVOT_COORD[Z] };

        // should add up joystick's displacement too
        // and write xyz back to magpose
        point* m[3] = { &m0, &m1, &m2 };
        for (int i = 0; i < 3; i++) {
            (*m[i])[X] += _joystick_pose.x;
            (*m[i])[Y] += _joystick_pose.y;
            (*m[i])[Z] += _joystick_pose.z;
            magpose[i].x = (*m[i])[X];
            magpose[i].y = (*m[i])[Y];
            magpose[i].z = (*m[i])[Z];
        }
        
        residual_err = 0.0f;
        for (int sen_idx = 0; sen_idx < 3; sen_idx++) {
            for (int mag_idx = 0; mag_idx < 3; mag_idx++) {
                _solveForward(mag_idx, sen_idx, magpose[mag_idx], b_magnet_temp[mag_idx]);
            }
            _mergeFieldToSensor(b_magnet_temp, b_predict[sen_idx]);

            // add every sensor's x,y,z error
            for (int i = 0; i < 3; i++) residual_err += fabsf(b_meas[sen_idx][i] - b_predict[sen_idx][i]);
        }

        for (int sen_idx = 0; sen_idx < 3; sen_idx++) {
            for (int mag_idx = 0; mag_idx < 3; mag_idx++) {
                magpose[mag_idx].x +=
            }
        }

    }

    // for (int i = 0; i < 3; i++) _mag[i].pose_prev = _mag[i].pose;
}
