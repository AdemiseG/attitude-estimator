#include "attitude.h"
#include <math.h>

#define RAD_TO_DEG 57.29577951308232f

void attitude_init(attitude_t *f, float alpha)
{
    f->roll_deg = 0.0f;
    f->pitch_deg = 0.0f;
    f->alpha = alpha;
}

void attitude_update(attitude_t *f,
                     float gx, float gy,
                     float ax, float ay, float az,
                     float dt)
{
    /* 1. predict: carry the current estimate forward with the gyro */
    const float roll_pred  = f->roll_deg  + gx * RAD_TO_DEG * dt;
    const float pitch_pred = f->pitch_deg + gy * RAD_TO_DEG * dt;

    /* 2. measure: ask gravity which way is down.
     *
     * Signs were determined empirically against simulator ground truth,
     * not from a frame diagram - both were inverted on the first attempt,
     * which made the filter score worse than a constant. These must
     * correlate positively with true roll and pitch.
     *
     * atan2f/sqrtf, not atan2/sqrt: the double versions force every value
     * through software emulation on a single-precision FPU.
     */
    const float acc_roll  = atan2f(-ay, -az) * RAD_TO_DEG;
    const float acc_pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

    /* 3. blend */
    const float a = f->alpha;
    f->roll_deg  = a * roll_pred  + (1.0f - a) * acc_roll;
    f->pitch_deg = a * pitch_pred + (1.0f - a) * acc_pitch;
}
