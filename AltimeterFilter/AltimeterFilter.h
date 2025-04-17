#ifndef ALTIMETER_FILTER_H
#define ALTIMETER_FILTER_H

#ifdef __cplusplus
extern "C"
{
#endif
    float pressure_mbar_to_ft(float mbar);
    void AltimeterFilterInit(float vel_z, float altitude_m);
    float AltimeterFilterProcess(float altitude_m);
    float AltimeterFilterGetVelocity(void);
    float AltimeterFilterGetAcceleration(void);
    float AltimeterFilterGetJerk(void);
    void AltimeterFilterSetProcessVariancePreApogee(int i, float val);
    void AltimeterFilterSetProcessVariancePostApogee(int i, float val);
    void AltimeterFilterSetObservationVariance(int i, float val);
#ifdef __cplusplus
}
#endif

#endif