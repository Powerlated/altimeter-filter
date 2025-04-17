#include <math.h>

float pressure_mbar_to_ft(float pressure_mbar) {
    // TODO: Adjustable QNH
    return 145366.45 * (1.0 - powf(pressure_mbar / 1013.25, 0.190284));
}