"""
Attitude estimation from IMU data, validated against ground truth.

Data: ArduCopter SITL flight, 25 Hz, 223 s.
  gyro_*  rad/s
  acc_*   m/s^2
  truth_* degrees (simulator ground truth - the answer)

Run:  python estimator.py
"""
import csv
import math
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load(path="data/flight1.csv"):
    with open(path) as f:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]


def gyro_only(rows):
    """Integrate gyro rates into angles. Correct instant to instant, drifts over time."""
    roll = pitch = 0.0
    out = []
    prev_t = rows[0]["t"]
    for r in rows:
        dt = r["t"] - prev_t
        prev_t = r["t"]
        roll += math.degrees(r["gyro_x"]) * dt
        pitch += math.degrees(r["gyro_y"]) * dt
        out.append((roll, pitch))
    return out


def complementary(rows, alpha=0.98):
    """Fuse gyro and accelerometer into a drift-free attitude estimate.

    Two sensors with opposite failure modes:
      gyro  - smooth and accurate instant to instant, but integrating its
              bias (~0.195 deg/s here) accumulates unbounded drift
      accel - noisy, and wrong whenever the vehicle accelerates, but it
              measures gravity, which never drifts

    So: trust the gyro over short intervals, and let gravity slowly pull the
    estimate back toward truth. alpha sets where that line is drawn -
    near 1 is smooth but drifts, near 0 is drift-free but noisy.

    alpha=0.98 at 25 Hz gives a time constant of about
    dt * alpha / (1 - alpha) = 0.04 * 49 = ~2 s.
    """
    roll = pitch = 0.0
    out = []
    prev_t = rows[0]["t"]
    for r in rows:
        dt = r["t"] - prev_t
        prev_t = r["t"]

        # 1. predict: carry the current estimate forward with the gyro
        roll_pred = roll + math.degrees(r["gyro_x"]) * dt
        pitch_pred = pitch + math.degrees(r["gyro_y"]) * dt

        # 2. measure: ask gravity which way is down.
        #    Signs here were determined empirically against simulator ground
        #    truth, not from a frame diagram - both were backwards at first,
        #    which made the filter score worse than outputting a constant.
        #    Check: these must correlate POSITIVELY with truth_*_deg.
        acc_roll = math.degrees(math.atan2(-r["acc_y"], -r["acc_z"]))
        acc_pitch = math.degrees(math.atan2(r["acc_x"],
                                            math.hypot(r["acc_y"], r["acc_z"])))

        # 3. blend the two
        roll = alpha * roll_pred + (1.0 - alpha) * acc_roll
        pitch = alpha * pitch_pred + (1.0 - alpha) * acc_pitch

        out.append((roll, pitch))
    return out


def rms(est, rows, key, idx):
    errs = [e[idx] - r[key] for e, r in zip(est, rows)]
    return math.sqrt(sum(x * x for x in errs) / len(errs))


def zero(rows):
    """Baseline worth keeping honest about: output level, always. On the first
    dataset (a pure hover) this beat every real estimator."""
    return [(0.0, 0.0) for _ in rows]


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "data/flight2.csv"
    rows = load(path)
    t = [r["t"] - rows[0]["t"] for r in rows]
    print("%s: %d samples, %.1f s, peak roll %.1f deg, peak pitch %.1f deg\n"
          % (path, len(rows), t[-1],
             max(abs(r["truth_roll_deg"]) for r in rows),
             max(abs(r["truth_pitch_deg"]) for r in rows)))

    # Choosing alpha is the real decision: too high and gyro bias leaks
    # through, too low and accelerometer noise does.
    print("alpha sweep (RMS error vs. ground truth):")
    print("  %-8s %10s %10s %10s" % ("alpha", "roll", "pitch", "time const"))
    best = None
    dt = (t[-1] - t[0]) / (len(t) - 1)
    for a in [0.90, 0.95, 0.98, 0.99, 0.995, 0.999]:
        est = complementary(rows, alpha=a)
        r_err = rms(est, rows, "truth_roll_deg", 0)
        p_err = rms(est, rows, "truth_pitch_deg", 1)
        print("  %-8.3f %9.2f%s %9.2f%s %8.1f s"
              % (a, r_err, "", p_err, "", dt * a / (1 - a)))
        if best is None or r_err + p_err < best[1]:
            best = (a, r_err + p_err)
    alpha = best[0]
    print("\nbest alpha = %.3f\n" % alpha)

    runs = {
        "always level": zero(rows),
        "gyro only": gyro_only(rows),
        "complementary": complementary(rows, alpha=alpha),
    }
    for name, est in runs.items():
        print("%-16s roll RMS %7.2f deg   pitch RMS %7.2f deg"
              % (name, rms(est, rows, "truth_roll_deg", 0),
                 rms(est, rows, "truth_pitch_deg", 1)))

    fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for i, (key, label) in enumerate([("truth_roll_deg", "roll"),
                                      ("truth_pitch_deg", "pitch")]):
        ax[i].plot(t, [r[key] for r in rows], "k", lw=1.8, label="ground truth")
        for name, est in runs.items():
            if name == "always level":
                continue
            ax[i].plot(t, [e[i] for e in est], lw=1.1, label=name)
        ax[i].set_ylabel("%s (deg)" % label)
        ax[i].grid(alpha=0.3)
        ax[i].legend(fontsize=9, loc="upper right")
    ax[1].set_xlabel("time (s)")
    ax[0].set_title("Attitude estimation vs. simulator ground truth "
                    "(complementary filter, alpha=%.3f)" % alpha)
    plt.tight_layout()
    plt.savefig("result.png", dpi=130)
    print("\nwrote result.png")
