"""Fit the unknown masses (booster dry, sustainer dry) and body Cd of the Minerva preset to the
CONOPS trajectory points (team slides). Re-run after correcting any target below.

    python calibrate_minerva.py [olympus|motor_o] [--apogee 10000]
"""
from dataclasses import replace
import numpy as np
from scipy.optimize import least_squares

import sys
from rocketfusion.config import minerva_rocket
from rocketfusion.dynamics import simulate_flight

PAD = 10.0
# (time since ignition [s], quantity, target, tolerance).  Read from CONOPS slide: some digits are blurry,
# so tolerances are generous.  quantity: "h" altitude [m], "v" velocity [m/s]
import argparse
_ap = argparse.ArgumentParser()
_ap.add_argument("booster", nargs="?", default="motor_o", choices=["olympus", "motor_o"])
_ap.add_argument("--apogee", type=float, default=10000.0,
                 help="target apogee [m]: 10000 ~ RASAero (10 001 m @ 5 m/s wind), 9490 = CONOPS slide")
_ap.add_argument("--apogee-tol", type=float, default=50.0)
_ap.add_argument("--no-plot", action="store_true")
_ARGS = _ap.parse_args()
BOOSTER, APOGEE = _ARGS.booster, _ARGS.apogee
T_BO1 = 3.45 if BOOSTER == "olympus" else 4.53
TARGETS = [
    (0.44, "v", 30.74, 2.0),        # rail exit
    (T_BO1, "h", 796.94, 40.0),     # booster burnout (CONOPS time digit is blurry in the slide)
    (T_BO1, "v", 317.28, 12.0),
    (9.53, "h", 2142.2, 50.0),      # interstage separation
    (9.53, "v", 227.63, 12.0),
    (16.68, "h", 4021.0, 80.0),     # sustainer burnout
    (16.68, "v", 498.25, 15.0),
    ("apogee", "h", APOGEE, _ARGS.apogee_tol),
    ("apogee", "t", 46.78, 4.0),
]


def predictions(r):
    tr = simulate_flight(r, dt=0.004)
    out = []
    for when, q, tgt, tol in TARGETS:
        if when == "apogee":
            val = (tr.h.max() if q == "h" else tr.events["APOGEE"] - PAD)
        else:
            i = np.searchsorted(tr.t, PAD + when)
            val = tr.h[i] if q == "h" else tr.v[i]
        out.append(val)
    return np.array(out)


def residuals(x, base):
    r = replace(base, stage1_dry=x[0], stage2_dry=x[1], cd0=x[2])
    p = predictions(r)
    return np.array([(pv - t[2]) / t[3] for pv, t in zip(p, TARGETS)])


if __name__ == "__main__":
    base = minerva_rocket(BOOSTER)
    sol = least_squares(residuals, [base.stage1_dry, base.stage2_dry, base.cd0], args=(base,),
                        bounds=([15, 12, 0.15], [60, 50, 0.9]), diff_step=0.02)
    r = replace(base, stage1_dry=sol.x[0], stage2_dry=sol.x[1], cd0=sol.x[2])
    p = predictions(r)
    print(f"stage1_dry = {sol.x[0]:.2f} kg   stage2_dry = {sol.x[1]:.2f} kg   cd0 = {sol.x[2]:.3f}")
    print(f"liftoff mass = {r.stage1_dry + r.stage1_prop + r.stage2_dry + r.stage2_prop:.1f} kg")
    print(f"{'when':>8s} {'qty':>3s} {'target':>9s} {'model':>9s} {'err/tol':>8s}")
    for (w, q, t, tol), v in zip(TARGETS, p):
        print(f"{str(w):>8s} {q:>3s} {t:9.2f} {v:9.2f} {(v - t) / tol:8.2f}")

    if _ARGS.no_plot:
        raise SystemExit
    # ---- validation figure against the CONOPS points ----
    import os
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs("reference_results", exist_ok=True)
    tr = simulate_flight(r, dt=0.004)
    t = tr.t - PAD
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.8))
    ax[0].plot(t, tr.h, "k"); ax[1].plot(t, tr.v, "k")
    for (w, q, tg, tol) in TARGETS:
        tt = tr.events["APOGEE"] - PAD if w == "apogee" else w
        if q == "h":
            ax[0].errorbar(tt, tg, yerr=tol, fmt="ro", ms=4)
        elif q == "v":
            ax[1].errorbar(tt, tg, yerr=tol, fmt="ro", ms=4)
    ax[0].set_xlim(0, 60); ax[1].set_xlim(0, 60); ax[0].set_ylim(0, 10500)
    ax[0].set_ylabel("altitude [m]"); ax[1].set_ylabel("velocity [m/s]")
    for a_ in ax: a_.set_xlabel("time since ignition [s]"); a_.grid(alpha=.3)
    ax[0].set_title(f"model (line) vs CONOPS points (red, +/- tolerance) - booster: {BOOSTER}", fontsize=8)
    fig.tight_layout(); fig.savefig(f"reference_results/minerva_validation_{BOOSTER}_{int(APOGEE)}m.png", dpi=150)
    print(f"Mach max {tr.mach.max():.2f},  peak specific force {tr.f.max() / 9.80665:.1f} g")
