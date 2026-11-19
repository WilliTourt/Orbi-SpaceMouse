#include "motion.h"
#include "config.h"
#include <cmath>
#include <cfloat>

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
        _mag[i].pm[X] = _mag[i].base.x - CFG_JOYSTICK_PIVOT_COORD[X];
        _mag[i].pm[Y] = _mag[i].base.y - CFG_JOYSTICK_PIVOT_COORD[Y];
        _mag[i].pm[Z] = _mag[i].base.z - CFG_JOYSTICK_PIVOT_COORD[Z];
    }
}

void Motion::begin(float b_stable_meas[9]) {
    calibrate(b_stable_meas);
}

void Motion::calibrate(float b_stable_meas[9]) {
    const float br[3] = {
        _calcBr(b_stable_meas[2], CFG_MAGNET_RADIUS, CFG_MAGNET_LENGTH),
        _calcBr(b_stable_meas[5], CFG_MAGNET_RADIUS, CFG_MAGNET_LENGTH),
        _calcBr(b_stable_meas[8], CFG_MAGNET_RADIUS, CFG_MAGNET_LENGTH)
    };
    for (int i = 0; i < 3; i++) {
        _mag[i].br = br[i];
        _mag[i].b_sign = (b_stable_meas[i * 3 + 2] > 0.0f) ? -1.0f : 1.0f;
    }

    // calc bias
    const Pose zero = Pose();
    Magnet::MagPose rest_pose[3];
    field b_rest_predict[3]; // forward solver calculated resting vals
    for (int i = 0; i < 3; i++) rest_pose[i] = _getMagPose(i, zero);

    for (int s = 0; s < 3; s++) _solveForward(s, rest_pose, b_rest_predict[s]);

    for (int i = 0; i < 3; i++) {
        for (int k = 0; k < 3; k++) {
            _b_bias[i][k] = b_stable_meas[i * 3 + k] - b_rest_predict[i][k];
        }
    }
}

void Motion::update(float b_meas[9]) {
    field b_meas_field[3];
    for (int i = 0; i < 3; i++) {
        b_meas_field[i][X] = b_meas[i * 3 + 0] - _b_bias[i][X];
        b_meas_field[i][Y] = b_meas[i * 3 + 1] - _b_bias[i][Y];
        b_meas_field[i][Z] = b_meas[i * 3 + 2] - _b_bias[i][Z];
    }

    Pose delta = Pose();
    _solveLM(b_meas_field, delta);
    for (int i = 0; i < 6; i++) ((float*)&_joystick_pose)[i] += ((float*)&delta)[i];
}



void Motion::JacobianMatrix::solve() {
    // Jt
    for (int row = 0; row < 9; row++) {
        for (int col = 0; col < 6; col++) {
            j_transpose[col][row] = j[row][col];
        }
    }

    // JtJ (symmetric)
    for (int row = 0; row < 6; row++) {
        for (int col = row; col < 6; col++) {
            float sum = 0.0f;
            for (int i = 0; i < 9; i++) sum += j_transpose[row][i] * j[i][col];
            jtj[row][col] = sum;
            jtj[col][row] = jtj[row][col];
        }
    }

    // not going to fill JtJ_inv, using (JtJ + λI)delta = Jt(b_measurement − f(p)) to solve
}

void Motion::JacobianMatrix::damp(float lambda) {
    for (int row = 0; row < 6; row++)
        for (int col = 0; col < 6; col++) jtj_damped[row][col] = jtj[row][col];
    // JtJ + λI, I is Marquardt scaling
    for (int i = 0; i < 6; i++) jtj_damped[i][i] += (lambda * jtj[i][i]);
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

    const float k = (MUR * _mag[mag_idx].br * _mag[mag_idx].b_sign) / (4.0f * PI);
    for (int i = 0; i < 3; i++) out_predicted[i] = k * b[i];
}

void Motion::_solveForward(const uint8_t sensor_idx, Magnet::MagPose pose[3], field sensor_field) {
    field to_merge[3] = { {0}, {0}, {0} };
    for (int mag_idx = 0; mag_idx < 3; mag_idx++) {
        _solveForward(mag_idx, sensor_idx, pose[mag_idx], to_merge[mag_idx]);
    }
    for (int i = 0; i < 3; i++) {
        sensor_field[i] = to_merge[0][i] + to_merge[1][i] + to_merge[2][i];        
    }
}

Motion::Magnet::MagPose Motion::_getMagPose(const uint8_t mag_idx, const Pose &joystick_pose) {
    // to get a magnet's x,y,z pose, we need to rotate the vector from pivot to magnet base (pm)
    vector pm_rot;

    Magnet::MagPose magpose = Magnet::MagPose();

    // magnet rotation is joystick rotation
    magpose.rx = joystick_pose.rx;
    magpose.ry = joystick_pose.ry;
    magpose.rz = joystick_pose.rz;
    
    // building rotation matrix
    const float cosx = cosf(magpose.rx), sinx = sinf(magpose.rx);
    const float cosy = cosf(magpose.ry), siny = sinf(magpose.ry);
    const float cosz = cosf(magpose.rz), sinz = sinf(magpose.rz);
    const float ROT_MAT[3][3] = {
        { cosy*cosz, cosz*sinx*siny-sinz*cosx, cosz*cosx*siny+sinz*sinx },
        { cosy*sinz, sinz*sinx*siny+cosz*cosx, sinz*cosx*siny-cosz*sinx },
        { -siny    , cosy*sinx               , cosy*cosx                }
    };

    pm_rot[X] = ROT_MAT[0][0] * _mag[mag_idx].pm[X] +
                ROT_MAT[0][1] * _mag[mag_idx].pm[Y] +
                ROT_MAT[0][2] * _mag[mag_idx].pm[Z];
    pm_rot[Y] = ROT_MAT[1][0] * _mag[mag_idx].pm[X] + 
                ROT_MAT[1][1] * _mag[mag_idx].pm[Y] + 
                ROT_MAT[1][2] * _mag[mag_idx].pm[Z];
    pm_rot[Z] = ROT_MAT[2][0] * _mag[mag_idx].pm[X] + 
                ROT_MAT[2][1] * _mag[mag_idx].pm[Y] + 
                ROT_MAT[2][2] * _mag[mag_idx].pm[Z];

    // here we got magnet base coords after rotation
    point m = { pm_rot[X] + CFG_JOYSTICK_PIVOT_COORD[X], 
                pm_rot[Y] + CFG_JOYSTICK_PIVOT_COORD[Y], 
                pm_rot[Z] + CFG_JOYSTICK_PIVOT_COORD[Z] };

    // should add up joystick's displacement too
    // and write xyz back to magpose
    m[X] += joystick_pose.x;
    m[Y] += joystick_pose.y;
    m[Z] += joystick_pose.z;
    magpose.x = m[X];
    magpose.y = m[Y];
    magpose.z = m[Z];

    return magpose;
}

void Motion::_solveJacobianMat(const Pose &joystick_pose, field b_predict[3]) {
    constexpr float DISPLACEMENT_UNIT = 0.001f; // mm
    constexpr float ROTATION_UNIT = 0.0001f;    // Rad

    Pose pose_disturbed = joystick_pose;
    Magnet::MagPose magpose_disturbed[3] = { Magnet::MagPose() };

    field b_disturbed[3] = { {0}, {0}, {0} };

    for (int col = 0; col < 6; col++) { // calc delta xyz on 1..3 columns, rx,ry,rz on 4..6
        pose_disturbed = joystick_pose;
        ((float*)&(pose_disturbed))[col] += (col < 3) ? DISPLACEMENT_UNIT : ROTATION_UNIT;
        for (int mag_idx = 0; mag_idx < 3; mag_idx++) {
            magpose_disturbed[mag_idx] = _getMagPose(mag_idx, pose_disturbed);
        }
        for (int sen_idx = 0; sen_idx < 3; sen_idx++) {
            _solveForward(sen_idx, magpose_disturbed, b_disturbed[sen_idx]);
        }

        for (int row = 0; row < 9; row++) {
            _j_mats.j[row][col] = b_disturbed[row / 3][row % 3] - b_predict[row / 3][row % 3];    
            _j_mats.j[row][col] /= (col < 3) ? DISPLACEMENT_UNIT : ROTATION_UNIT; // delta / unit is derivative
        }
    }
}

// the only code that not written by myself, i am stressful about linear algebra
bool Motion::_cholesky(const float A[6][6], const float b[6], Pose &delta) {
    float L[6][6] = {0}; // lower triangle L

    for (int row = 0; row < 6; row++) {
        for (int col = 0; col <= row; col++) {
            float sum = A[row][col];
            for (int i = 0; i < col; i++) sum -= L[row][i] * L[col][i];

            if (col == row) {
                if (sum <= 0.0f) return false;
                L[row][col] = sqrtf(sum);
            } else {
                L[row][col] = sum / L[col][col];
            }
        }
    }

    // dont know what the fuxk is going on
    float y[6];
    for (int i = 0; i < 6; i++) {
        float s = b[i];
        for (int k = 0; k < i; k++) s -= L[i][k] * y[k];
        y[i] = s / L[i][i];
    }

    for (int i = 5; i >= 0; i--) {
        float s = y[i];
        for (int k = i + 1; k < 6; k++) s -= L[k][i] * ((float*)&delta)[k];
        ((float*)&delta)[i] = s / L[i][i];
    }

    return true;
}

void Motion::_solveLM(field b_meas[3], Pose &out_delta) {
    constexpr uint32_t MAX_ITERATIONS = 15;
    constexpr float    DEADZONE = 1.0f;

    constexpr float    LAMBDA_DEFAULT = 0.1f;
    constexpr float    LAMBDA_STEP = 3.0f;
    constexpr float    LAMBDA_MAX = 1000.0f;
    constexpr float    LAMBDA_MIN = 0.02f;

    constexpr float    MAX_STEP_MM = 1.0f;
    constexpr float    MAX_STEP_RAD = 0.05f;

    // iters ctrl
    float residual_err = FLT_MAX;
    uint32_t iters = 0;
    bool pose_changed = true; // to decide whether to calc residual_err and j mat again
    // true on first round because nothing is computed yet

    Magnet::MagPose magpose[3] = { Magnet::MagPose() }; // only for lm solver
    // calc magnet xyz in every iteration

    float lambda = LAMBDA_DEFAULT;
    Pose iter_pose = _joystick_pose;
    Pose iter_delta = Pose();
    field b_predict[3] = { {0}, {0}, {0} }; // sensors' predicted b field
    while ((residual_err > DEADZONE) && (iters++ < MAX_ITERATIONS)) {
         // if pose unchanged, predictions and J mat are still valid so no need to calc them again
        if (pose_changed) {
            for (int mag_idx = 0; mag_idx < 3; mag_idx++) {
                magpose[mag_idx] = _getMagPose(mag_idx, iter_pose);
            }

            // step 1: residual error
            residual_err = 0.0f;
            for (int sen_idx = 0; sen_idx < 3; sen_idx++) {
                _solveForward(sen_idx, magpose, b_predict[sen_idx]);

                // add every sensor's x,y,z error
                for (int i = 0; i < 3; i++) residual_err += fabsf(b_meas[sen_idx][i] - b_predict[sen_idx][i]);
            }

            // step 2: get j mat and JtJ
            _solveJacobianMat(iter_pose, b_predict);
            _j_mats.solve();
        }

        _j_mats.damp(lambda);

        // step 3: calc Jtr ( Jt(b_meas - b_predict) )
        float jtr[6] = {0};
        for (int row = 0; row < 6; row++) {
            for (int col = 0; col < 9; col++) {
                jtr[row] += _j_mats.j_transpose[row][col]
                          * (b_meas[col / 3][col % 3] - b_predict[col / 3][col % 3]);
            }
        }

        // step 4: solve delta (Cholesky decomposition A(SPD) = LLt)
        while (!_cholesky(_j_mats.jtj_damped, jtr, iter_delta) && (lambda < LAMBDA_MAX)) {
            lambda *= LAMBDA_STEP;
            _j_mats.solve();
            _j_mats.damp(lambda);
        }
        // if lambda is very high, but still unsolved, discard this iteration
        if (!_cholesky(_j_mats.jtj_damped, jtr, iter_delta)) break;

        // step 5: clamp delta
        for (int i = 0; i < 6; i++) {
            const float clamp = (i < 3) ? MAX_STEP_MM : MAX_STEP_RAD;
            if (((float*)&iter_delta)[i] > clamp) {
                ((float*)&iter_delta)[i] = clamp;
            } else if (((float*)&iter_delta)[i] < -clamp) {
                ((float*)&iter_delta)[i] = -clamp;
            }
        }

        // step 6: update pose, lambda and start next iteration
        // if pose_preview shows a lower residual error, then update pose and lambda
        Pose pose_preview = iter_pose;
        Magnet::MagPose magpose_preview[3] = { Magnet::MagPose() };
        field b_predict_preview[3] = { {0}, {0}, {0} };
        float residual_err_preview = 0.0f;

        for (int i = 0; i < 6; i++) ((float*)&pose_preview)[i] += ((float*)&iter_delta)[i];
        for (int mag_idx = 0; mag_idx < 3; mag_idx++) {
            magpose_preview[mag_idx] = _getMagPose(mag_idx, pose_preview);
        }
        for (int sen_idx = 0; sen_idx < 3; sen_idx++) {
            _solveForward(sen_idx, magpose_preview, b_predict_preview[sen_idx]);
            for (int i = 0; i < 3; i++) {
                residual_err_preview += fabsf(b_meas[sen_idx][i] - b_predict_preview[sen_idx][i]);
            }
        }

        if (residual_err_preview < residual_err) {
            lambda /= LAMBDA_STEP;
            lambda = (lambda < LAMBDA_MIN) ? LAMBDA_MIN : lambda;

            iter_pose = pose_preview;
            residual_err = residual_err_preview;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    b_predict[i][j] = b_predict_preview[i][j];
                }
            }
            pose_changed = true;
        } else {
            lambda *= LAMBDA_STEP;
            pose_changed = false;
        }
    }

    // apply overall delta
    for (int i = 0; i < 6; i++) {
        ((float*)&out_delta)[i] = ((float*)&iter_pose)[i] - ((float*)&_joystick_pose)[i];
    }
}
