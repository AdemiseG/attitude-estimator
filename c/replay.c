/*
 * Host harness: replay a recorded flight through the filter and score it.
 *
 * This exists to prove the C filter produces the same answer as the Python
 * reference before any of it runs on hardware. It is not part of the
 * firmware - attitude.c is.
 *
 *   ./replay ../data/flight2.csv [alpha]
 */
#include "attitude.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_ROWS 20000

typedef struct {
    float t, gx, gy, gz, ax, ay, az, truth_roll, truth_pitch, truth_yaw;
} sample_t;

static sample_t rows[MAX_ROWS];

static int load(const char *path)
{
    FILE *f = fopen(path, "r");
    char line[512];
    int n = 0;

    if (!f) {
        fprintf(stderr, "cannot open %s\n", path);
        return -1;
    }
    if (!fgets(line, sizeof line, f)) {   /* discard header */
        fclose(f);
        return -1;
    }
    while (n < MAX_ROWS && fgets(line, sizeof line, f)) {
        sample_t *r = &rows[n];
        if (sscanf(line, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                   &r->t, &r->gx, &r->gy, &r->gz,
                   &r->ax, &r->ay, &r->az,
                   &r->truth_roll, &r->truth_pitch, &r->truth_yaw) == 10) {
            n++;
        }
    }
    fclose(f);
    return n;
}

int main(int argc, char **argv)
{
    const char *path = (argc > 1) ? argv[1] : "../data/flight2.csv";
    const float alpha = (argc > 2) ? (float)atof(argv[2]) : 0.95f;

    const int n = load(path);
    if (n <= 0) {
        return 1;
    }

    attitude_t f;
    attitude_init(&f, alpha);

    /* gyro-only integration, for the same comparison the Python makes */
    float gyro_roll = 0.0f, gyro_pitch = 0.0f;

    double se_roll = 0.0, se_pitch = 0.0;
    double se_gyro_roll = 0.0, se_gyro_pitch = 0.0;
    float prev_t = rows[0].t;

    for (int i = 0; i < n; i++) {
        const sample_t *r = &rows[i];
        const float dt = r->t - prev_t;
        prev_t = r->t;

        attitude_update(&f, r->gx, r->gy, r->ax, r->ay, r->az, dt);

        gyro_roll  += r->gx * 57.29577951308232f * dt;
        gyro_pitch += r->gy * 57.29577951308232f * dt;

        /* accumulate error in double: 4409 squared terms in float loses
           digits we care about. Explicit casts, so -Wdouble-promotion
           still protects the filter itself. */
        const double er = (double)f.roll_deg - (double)r->truth_roll;
        const double ep = (double)f.pitch_deg - (double)r->truth_pitch;
        se_roll += er * er;
        se_pitch += ep * ep;

        const double gr = (double)gyro_roll - (double)r->truth_roll;
        const double gp = (double)gyro_pitch - (double)r->truth_pitch;
        se_gyro_roll += gr * gr;
        se_gyro_pitch += gp * gp;
    }

    printf("%s: %d samples, %.1f s, alpha %.3f\n\n",
           path, n, (double)(rows[n - 1].t - rows[0].t), (double)alpha);
    printf("%-16s roll RMS %9.5f deg   pitch RMS %9.5f deg\n", "gyro only",
           sqrt(se_gyro_roll / n), sqrt(se_gyro_pitch / n));
    printf("%-16s roll RMS %9.5f deg   pitch RMS %9.5f deg\n", "complementary",
           sqrt(se_roll / n), sqrt(se_pitch / n));
    return 0;
}
