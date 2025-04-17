#include "AltimeterFilter.h"
#include <math.h>
#include "Eigen/Dense"
#include <iostream>

using Eigen::Matrix;
using Eigen::Vector;

constexpr int STATE_LEN = 4;
constexpr int OBSERVATION_LEN = 2;
const float deltaT = 0.010f;
const float g = 9.81f;

#define STAGE_GROUND 0
#define STAGE_BURNOUT 1
#define STAGE_APOGEE 2

// Vectors in Eigen are column vectors
struct AltimeterFilter
{
    Vector<float, STATE_LEN> state;                    // [altitude_m, vel_z, acc_z, jerk_z]
    Matrix<float, STATE_LEN, STATE_LEN> P;             // Noise covariance
    Matrix<float, OBSERVATION_LEN, STATE_LEN> H;       // State to observation mapping
    Matrix<float, STATE_LEN, STATE_LEN> Phi;           // State transition matrix
    Vector<float, STATE_LEN> B;                        // State transition bias
    Matrix<float, STATE_LEN, STATE_LEN> Q;             // Process noise covariance
    Matrix<float, OBSERVATION_LEN, OBSERVATION_LEN> R; // Observation noise covariance

    float max_accel;
    int flight_stage;
};

struct AltimeterFilter f;

static void AltimeterFilterPrint()
{
    std::cout << "\nstate:\n"
              << f.state << std::endl;
    std::cout << "\nP:\n"
              << f.P << std::endl;
    std::cout << "\nH:\n"
              << f.H << std::endl;
    std::cout << "\nPhi:\n"
              << f.Phi << std::endl;
    std::cout << "\nB:\n"
              << f.B << std::endl;
    std::cout << "\nQ:\n"
              << f.Q << std::endl;
    std::cout << "\nR:\n"
              << f.R << std::endl;
}

void AltimeterFilterInit(float altitude_m, float vel_z)
{

    f.state << altitude_m, vel_z, 0, 0;
    f.P.setZero();
    f.H << 1, 0, 0, 0,
        0, 0, 0, 0;
    f.Phi << 1, deltaT, 0, 0,
        0, 1, deltaT, 0,
        0, 0, 1, deltaT,
        0, 0, 0, 1;
    f.B << 0, 0, 0, 0;

    f.Q << 0.100, 0, 0, 0,
        0, 0.01, 0, 0,
        0, 0, 0.01, 0,
        0, 0, 0, 0.001;
    f.R << 100, 0,
        0, 100;

    f.max_accel = 0;
    f.flight_stage = STAGE_GROUND;
}

float AltimeterFilterProcess(float altitude_m)
{
    // Kalman gain
    auto K = f.P * f.H.transpose() * (f.H * f.P * f.H.transpose() + f.R).inverse();
    // std::cout << "\nK:\n"
    //   << K << std::endl;

    // AltimeterFilterPrint();
    Vector<float, OBSERVATION_LEN> z; // [pressure_altitude, acceleration]
    z(0) = altitude_m;                // Observation
    z(1) = 0;

    // Scale acceleration variance according to current acceleration
    // f.Q(2, 2) = fabs(AltimeterFilterGetAcceleration()) / 100;

    const float ACCEL_THRESHOLD = 20; // 20 m/s^2

    if (f.max_accel < AltimeterFilterGetAcceleration())
    {
        f.max_accel = AltimeterFilterGetAcceleration();
    }

    switch (f.flight_stage)
    {
    case STAGE_GROUND:
        // Detect motor burnout
        if (f.max_accel > ACCEL_THRESHOLD && AltimeterFilterGetJerk() < 0)
        {
            f.flight_stage = STAGE_BURNOUT;
        }
        break;
    case STAGE_BURNOUT:
        // In burnout:
        // Keep "observation" of acceleration at -g
        z(1) = -g;
        // Set acceleration state to relate to acceleration "observation"
        f.H(1, 2) = 1;

        if (AltimeterFilterGetVelocity() < 0)
        {
            // TODO: FIRE DA EJECTION CHARGEEEEEE
            f.flight_stage = STAGE_APOGEE;
        }
        break;
    case STAGE_APOGEE:
        // After apogee:
        // Rely entirely now on smoothed predictions, no more gravity modeling
        // Set acceleration state to no longer relate to acceleration "observation"
        f.H(1, 2) = 0;
        break;
    }

    auto state_update = f.state + K * (z - f.H * f.state);
    Matrix<float, STATE_LEN, STATE_LEN> I;
    I.setIdentity();
    auto P_update = (I - K * f.H) * f.P;

    f.state = f.Phi * state_update + f.B;
    f.P = f.Phi * P_update * f.Phi.transpose() + f.Q;

    return f.state(0);
}

float AltimeterFilterGetVelocity()
{
    return f.state(1);
}

float AltimeterFilterGetAcceleration()
{
    return f.state(2);
}

float AltimeterFilterGetJerk()
{
    return f.state(3);
}

float pressure_mbar_to_ft(float pressure_mbar)
{
    // TODO: Adjustable QNH
    return 145366.45 * (1.0 - powf(pressure_mbar / 1013.25, 0.190284));
}
