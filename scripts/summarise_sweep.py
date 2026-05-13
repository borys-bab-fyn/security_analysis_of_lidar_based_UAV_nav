#!/usr/bin/env python3
"""
summarise_sweep.py
Reads all experiment folders from a sweep and produces a summary CSV.
"""
import os, sys, csv
from pathlib import Path

def summarise_injector(csv_path):
    total = rejected = clamped = faults = 0
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            total += 1
            # countermeasure_action column
            action = row.get("countermeasure_action", "")
            if "rejected" in action:
                rejected += 1
            elif "clamped" in action:
                clamped += 1
            if "fault" in action:
                faults += 1
    return total, rejected, clamped, faults

def summarise_monitor(csv_path):
    # Extract altitude drift: difference between first and last ekf_alt
    ekf_alts = []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            val = row.get("ekf_altitude_m", "")
            if val:
                try:
                    ekf_alts.append(float(val))
                except ValueError:
                    pass
    if len(ekf_alts) < 2:
        return None, None
    drift = ekf_alts[-1] - ekf_alts[0]
    return drift, ekf_alts[0] if ekf_alts else None

sweep_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("results_sweep_20260509_120000")
out_rows = []
for exp_dir in sorted(sweep_dir.iterdir()):
    if not exp_dir.is_dir():
        continue
    tag = exp_dir.name
    inj_csv = exp_dir / "injector.csv"
    mon_csv = exp_dir / "monitor.csv"
    if not inj_csv.exists() or not mon_csv.exists():
        continue

    total, rej, clamp, faults = summarise_injector(inj_csv)
    drift, start_alt = summarise_monitor(mon_csv)
    out_rows.append({
        "experiment": tag,
        "samples": total,
        "rejected": rej,
        "clamped": clamp,
        "faults": faults,
        "ekf_drift_m": f"{drift:.3f}" if drift is not None else "N/A",
        "ekf_start_m": f"{start_alt:.2f}" if start_alt is not None else "N/A"
    })

out_path = sweep_dir / "summary.csv"
with open(out_path, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=["experiment","samples","rejected","clamped","faults","ekf_drift_m","ekf_start_m"])
    writer.writeheader()
    writer.writerows(out_rows)

print(f"Summary written to {out_path}")
