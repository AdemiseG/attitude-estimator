"""Build a legible profile-sized thumbnail.

The full two-panel result.png is unreadable when GitHub scales it into a
profile table cell, so this renders one panel, over the window where the
aircraft is actually maneuvering, with type large enough to survive it.
"""
import sys
sys.path.insert(0, ".")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from estimator import load, complementary, rms

rows = load("data/flight2.csv")
est = complementary(rows, alpha=0.95)
t0 = rows[0]["t"]
t = [r["t"] - t0 for r in rows]
err = rms(est, rows, "truth_pitch_deg", 1)

lo, hi = 30, 112
idx = [i for i, x in enumerate(t) if lo <= x <= hi]

fig, ax = plt.subplots(figsize=(9.0, 3.8))
ax.plot([t[i] for i in idx], [rows[i]["truth_pitch_deg"] for i in idx],
        "k", lw=3.0, label="ground truth")
ax.plot([t[i] for i in idx], [est[i][1] for i in idx],
        color="#e8710a", lw=2.4, label="complementary filter")
ax.set_title("Pitch estimate vs. simulator truth  —  %.2f° RMS" % err,
             fontsize=21, pad=12)
ax.set_xlabel("time (s)", fontsize=16)
ax.set_ylabel("pitch (deg)", fontsize=16)
ax.tick_params(labelsize=14)
ax.legend(fontsize=15, loc="upper right", framealpha=0.95)
ax.grid(alpha=0.3)
plt.tight_layout()
plt.savefig("thumb.png", dpi=120)
print("wrote thumb.png")
