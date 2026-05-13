#!/usr/bin/env python3
"""
plot_all_attacks.py
Automatically scans a sweep results folder and produces:
  1. drift_landing_comparison.png   – no CM vs best clamp vs reject
  2. oscillation_comparison.png     – no CM vs best slew_gate vs best robust_fallback
  3. spike_comparison.png           – no CM vs best slew_gate vs best robust_fallback
  4. constant_comparison.png        – no CM vs best slew_gate vs best robust_fallback
  5. countermeasure_activity.png    – bar chart of clamped/rejected/faults per mode
"""

import csv
import re
import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np


# ---------------------------------------------------------------------------
def load_altitude(csv_path):
    """Return (times, altitudes) from a monitor.csv."""
    times, alts = [], []
    try:
        with open(csv_path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                t = row.get("t_elapsed_s")
                alt = row.get("ekf_altitude_m")
                if t and alt:
                    try:
                        times.append(float(t))
                        alts.append(float(alt))
                    except ValueError:
                        pass
    except Exception:
        pass
    return times, alts


def load_injector_stats(csv_path):
    """Return (total, rejected, clamped, faults) from injector.csv."""
    total = rejected = clamped = faults = 0
    try:
        with open(csv_path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                total += 1
                action = row.get("countermeasure_action", "")
                if "rejected" in action:
                    rejected += 1
                if "clamped" in action:
                    clamped += 1
                if "fault" in action:
                    faults += 1
    except Exception:
        pass
    return total, rejected, clamped, faults


def parse_folder_name(name):
    """Extract mode, countermeasure, and key params from folder name."""
    parts = name.split("____")
    mode = parts[0] if len(parts) > 0 else "unknown"

    cm = "none"
    params = {}

    if len(parts) >= 2:
        rest = parts[1]
        if "countermeasure_slew_gate" in rest:
            cm = "slew_gate"
        elif "countermeasure_robust_fallback" in rest:
            cm = "robust_fallback"
        elif "no_countermeasure" in rest:
            cm = "none"

    full = name
    # slew_gate params
    for key in ["max_rate_mps", "max_jump_m"]:
        m = re.search(rf"{key}_([\d.]+)", full)
        if m:
            params[key] = float(m.group(1))
    if "reject_instead_of_clamp" in full:
        params["reject_mode"] = "reject"
    elif cm == "slew_gate":
        params["reject_mode"] = "clamp"

    # robust_fallback params
    for key in ["fault_threshold", "window_size", "hold_seconds", "residual_threshold_m"]:
        m = re.search(rf"{key}_([\d.]+)", full)
        if m:
            val = m.group(1)
            params[key] = float(val) if '.' in val else int(val)
    m = re.search(r"min_quality_(\d+)", full)
    if m:
        params["min_quality"] = int(m.group(1))

    return mode, cm, params


# ---------------------------------------------------------------------------
def find_best_experiment(folders, mode, cm, prefer_clamp=True, prefer_hold_long=False):
    """
    Find the "best" experiment for a given mode and countermeasure.
    Criteria (in order):
      - If prefer_clamp: exclude reject_mode='reject'.
      - If prefer_hold_long: prefer larger hold_seconds.
      - Choose the one with smallest absolute ekf_drift (from summary if available,
        otherwise choose the one with most clamped/faults).
    """
    candidates = []
    for folder in folders:
        m, c, params = parse_folder_name(folder.name)
        if m != mode or c != cm:
            continue
        if prefer_clamp and params.get("reject_mode") == "reject":
            continue
        mon_csv = folder / "monitor.csv"
        inj_csv = folder / "injector.csv"
        if not mon_csv.exists():
            continue
        times, alts = load_altitude(mon_csv)
        if len(alts) < 2:
            continue
        drift = alts[-1] - alts[0]
        total, rej, clamp, faults = load_injector_stats(inj_csv)
        # Score: combine clamped/faults count and absolute drift
        score = abs(drift) - 0.001 * (clamp + faults)  # more activity = better
        candidates.append((score, folder, params, times, alts, clamp, faults, drift))
    if not candidates:
        return None
    # Sort by score (smallest drift wins, boosted by activity)
    candidates.sort(key=lambda x: x[0])
    return candidates[0]


def find_reject_experiment(folders, mode):
    """Find the reject-mode experiment for a given mode."""
    for folder in folders:
        m, c, params = parse_folder_name(folder.name)
        if m == mode and c == "slew_gate" and params.get("reject_mode") == "reject":
            mon_csv = folder / "monitor.csv"
            if mon_csv.exists():
                times, alts = load_altitude(mon_csv)
                if len(alts) >= 2:
                    return folder.name, times, alts
    return None, [], []


def find_no_cm(folders, mode):
    """Find the no-countermeasure baseline for a mode."""
    for folder in folders:
        m, c, _ = parse_folder_name(folder.name)
        if m == mode and c == "none":
            mon_csv = folder / "monitor.csv"
            if mon_csv.exists():
                times, alts = load_altitude(mon_csv)
                if len(alts) >= 2:
                    return folder.name, times, alts
    return None, [], []


# ---------------------------------------------------------------------------
def plot_comparison(sweep_dir, mode, curves, title, filename):
    """Plot multiple altitude traces on one axes."""
    fig, ax = plt.subplots(figsize=(10, 4.5))
    colours = ['black', 'green', 'blue', 'red', 'orange']
    for i, (label, times, alts) in enumerate(curves):
        if len(times) == 0:
            continue
        ax.plot(times, alts, label=label, linewidth=1.0, color=colours[i % len(colours)])
    ax.axhline(y=0.0, color='red', linestyle='--', alpha=0.3, label='Ground')
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("EKF Altitude (m)")
    ax.set_title(title)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    out_path = sweep_dir / filename
    fig.savefig(out_path)
    plt.close()
    print(f"  Saved {out_path}")


def plot_activity_bar(sweep_dir, folders):
    """Bar chart of countermeasure activity per mode."""
    modes = ["drift", "oscillation", "spike", "constant"]
    cm_types = ["none", "slew_gate", "robust_fallback"]

    # Aggregate
    data = {mode: {cm: [0, 0, 0] for cm in cm_types} for mode in modes}  # [rej, clamp, faults]
    for folder in folders:
        m, c, _ = parse_folder_name(folder.name)
        if m not in modes or c not in cm_types:
            continue
        inj_csv = folder / "injector.csv"
        if inj_csv.exists():
            _, rej, clamp, faults = load_injector_stats(inj_csv)
            data[m][c][0] += rej
            data[m][c][1] += clamp
            data[m][c][2] += faults

    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5), sharey=False)
    titles = ["Rejected", "Clamped", "Faults"]
    x = np.arange(len(modes))
    width = 0.25

    for ax_idx, (col_idx, title) in enumerate(zip([0, 1, 2], titles)):
        ax = axes[ax_idx]
        for i, cm in enumerate(cm_types):
            vals = [data[m][cm][col_idx] for m in modes]
            ax.bar(x + i * width, vals, width, label=cm)
        ax.set_title(title)
        ax.set_xticks(x + width)
        ax.set_xticklabels(modes, rotation=30, ha='right')
        if ax_idx == 0:
            ax.legend(fontsize=8)

    fig.suptitle("Countermeasure Activity per Attack Mode")
    fig.tight_layout()
    out_path = sweep_dir / "countermeasure_activity.png"
    fig.savefig(out_path)
    plt.close()
    print(f"  Saved {out_path}")


# ---------------------------------------------------------------------------
def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_all_attacks.py <sweep_results_folder>")
        sys.exit(1)

    sweep_dir = Path(sys.argv[1])
    if not sweep_dir.is_dir():
        print(f"ERROR: {sweep_dir} not found")
        sys.exit(1)

    # Gather all experiment folders
    folders = sorted([f for f in sweep_dir.iterdir() if f.is_dir()])
    print(f"Found {len(folders)} experiment folders.\n")

    # ---- 1. Drift: No CM vs Best Clamp vs Reject ----
    print("=== Drift Attack ===")
    _, no_cm_t, no_cm_a = find_no_cm(folders, "drift")
    best_clamp = find_best_experiment(folders, "drift", "slew_gate", prefer_clamp=True)
    _, rej_t, rej_a = find_reject_experiment(folders, "drift")

    curves = []
    if no_cm_a:
        curves.append(("No Countermeasure", no_cm_t, no_cm_a))
    if best_clamp:
        curves.append((f"Best Clamp (rate={best_clamp[2].get('max_rate_mps','?')})", best_clamp[3], best_clamp[4]))
    if rej_a:
        curves.append(("Reject Mode (rate=0.4)", rej_t, rej_a))

    plot_comparison(sweep_dir, "drift", curves,
                    "Drift Attack: No CM vs Best Clamp vs Reject",
                    "drift_landing_comparison.png")

    # ---- 2. Oscillation: No CM vs Best Slew vs Best Robust ----
    print("=== Oscillation Attack ===")
    _, no_cm_t, no_cm_a = find_no_cm(folders, "oscillation")
    best_slew = find_best_experiment(folders, "oscillation", "slew_gate", prefer_clamp=True)
    best_rob = find_best_experiment(folders, "oscillation", "robust_fallback", prefer_hold_long=True)

    curves = []
    if no_cm_a:
        curves.append(("No Countermeasure", no_cm_t, no_cm_a))
    if best_slew:
        curves.append((f"Best Slew Gate (jump={best_slew[2].get('max_jump_m','?')})", best_slew[3], best_slew[4]))
    if best_rob:
        curves.append((f"Best Robust Fallback (hold={best_rob[2].get('hold_seconds','?')}s)", best_rob[3], best_rob[4]))

    plot_comparison(sweep_dir, "oscillation", curves,
                    "Oscillation Attack: No CM vs Slew Gate vs Robust Fallback",
                    "oscillation_comparison.png")

    # ---- 3. Spike: No CM vs Best Slew vs Best Robust ----
    print("=== Spike Attack ===")
    _, no_cm_t, no_cm_a = find_no_cm(folders, "spike")
    best_slew_sp = find_best_experiment(folders, "spike", "slew_gate", prefer_clamp=True)
    best_rob_sp = find_best_experiment(folders, "spike", "robust_fallback")

    curves = []
    if no_cm_a:
        curves.append(("No Countermeasure", no_cm_t, no_cm_a))
    if best_slew_sp:
        curves.append((f"Best Slew Gate (rate={best_slew_sp[2].get('max_rate_mps','?')})", best_slew_sp[3], best_slew_sp[4]))
    if best_rob_sp:
        curves.append((f"Best Robust Fallback (fault_thr={best_rob_sp[2].get('fault_threshold','?')})", best_rob_sp[3], best_rob_sp[4]))

    plot_comparison(sweep_dir, "spike", curves,
                    "Spike Attack: No CM vs Slew Gate vs Robust Fallback",
                    "spike_comparison.png")

    # ---- 4. Constant ----
    print("=== Constant Attack ===")
    _, no_cm_t, no_cm_a = find_no_cm(folders, "constant")
    best_slew_co = find_best_experiment(folders, "constant", "slew_gate", prefer_clamp=True)
    best_rob_co = find_best_experiment(folders, "constant", "robust_fallback")

    curves = []
    if no_cm_a:
        curves.append(("No Countermeasure", no_cm_t, no_cm_a))
    if best_slew_co:
        curves.append((f"Best Slew Gate", best_slew_co[3], best_slew_co[4]))
    if best_rob_co:
        curves.append((f"Best Robust Fallback", best_rob_co[3], best_rob_co[4]))

    plot_comparison(sweep_dir, "constant", curves,
                    "Constant Attack: No CM vs Slew Gate vs Robust Fallback",
                    "constant_comparison.png")

    # ---- 5. Activity Bar Chart ----
    print("=== Countermeasure Activity ===")
    plot_activity_bar(sweep_dir, folders)

    print("\nAll plots generated in", sweep_dir)


if __name__ == "__main__":
    main()
