/*
 * Complementary filter for roll and pitch.
 *
 * No allocation, no I/O, no libc beyond math.h. The caller owns the state,
 * so this compiles unchanged for a host replay harness and for a Teensy.
 */
#ifndef ATTITUDE_H
#define ATTITUDE_H

typedef struct {
    float roll_deg;
    float pitch_deg;
    float alpha;       /* gyro weight per step, 0..1 */
} attitude_t;

/* alpha near 1 trusts the gyro (smooth, drifts);
   near 0 trusts the accelerometer (no drift, noisy). 0.95f at 25 Hz. */
void attitude_init(attitude_t *f, float alpha);

/* gx, gy  body angular rate, rad/s
   ax, ay, az  body specific force, m/s^2, az negative when level
   dt  seconds since the previous call */
void attitude_update(attitude_t *f,
                     float gx, float gy,
                     float ax, float ay, float az,
                     float dt);

#endif /* ATTITUDE_H */
