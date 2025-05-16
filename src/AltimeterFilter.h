#ifndef ALTIMETER_FILTER_H
#define ALTIMETER_FILTER_H

constexpr int STATE_LEN = 4;
constexpr int OBSERVATION_LEN = 2;

struct AltimeterFilterOutput {
    float altitude_m;
    float velocity_mps;
    float acceleration_mps2;
    float jerk_mps3;
    float time_to_apogee_s;
};

#ifdef __cplusplus
extern "C"
{
#endif
    float pressure_mbar_to_ft(float mbar);
    void AltimeterFilterInit(float vel_z, float altitude_m);
    struct AltimeterFilterOutput AltimeterFilterProcess(float altitude_m);
    void AltimeterFilterSetProcessVariancePreApogee(int i, float val);
    void AltimeterFilterSetProcessVariancePostApogee(int i, float val);
    void AltimeterFilterSetObservationVariance(int i, float val);
#ifdef __cplusplus
}
#endif

#endif