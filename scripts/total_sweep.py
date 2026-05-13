#!/usr/bin/env python3
"""
summarise_sweep.py
Reads all experiment folders from a countermeasure sweep and produces:
  1. summary.csv           — one row per experiment
  2. summary_by_attack.csv — pivot: attack vs countermeasure vs ekf_drift
  3. Prints a readable table to stdout
"""
import csv
import re
import sys
from pathlib import Path


# ---------------------------------------------------------------------------
def summarise_injector(csv_path: Path):
    """Extract countermeasure statistics from injector.csv."""
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
        return total, rejected, clamped, faults
    except Exception as e:
        print(f"  WARNING: could not read {csv_path}: {e}", file=sys.stderr)
        return 0, 0, 0, 0


# ---------------------------------------------------------------------------
def summarise_monitor(csv_path: Path):
    """Extract altitude drift and max deviation from monitor.csv."""
    ekf_alts = []
    reported_lidar = []
    try:
        with open(csv_path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                # EKF altitude
                val = row.get("ekf_altitude_m", "")
                if val:
                    try:
                        ekf_alts.append(float(val))
                    except ValueError:
                        pass
                # Reported LiDAR distance (what the injector sent)
                val2 = row.get("reported_lidar_m", "")
                if val2:
                    try:
                        reported_lidar.append(float(val2))
                    except ValueError:
                        pass
    except Exception:
        return None, None, None, None, None

    if len(ekf_alts) < 2:
        return None, None, None, None, None

    start_alt = ekf_alts[0]
    end_alt = ekf_alts[-1]
    drift = end_alt - start_alt
    max_deviation = max(abs(a - start_alt) for a in ekf_alts)
    ekf_std = (sum((a - sum(ekf_alts)/len(ekf_alts))**2 for a in ekf_alts) / len(ekf_alts)) ** 0.5

    # How much did the reported LiDAR deviate?
    lidar_range = max(reported_lidar) - min(reported_lidar) if len(reported_lidar) >= 2 else 0

    return round(drift, 4), round(start_alt, 2), round(max_deviation, 4), round(ekf_std, 4), round(lidar_range, 2)


# ---------------------------------------------------------------------------
def parse_folder_name(folder_name: str):
    """
    Parse a folder name like:
      drift____countermeasure_slew_gate___max_rate_mps_0.4___reject_instead_of_clamp
    into:
      mode, countermeasure, dict of params
    """
    parts = folder_name.split("____")  # four underscores = separator between sections
    if len(parts) < 1:
        return "unknown", "unknown", {}

    mode = parts[0]

    countermeasure = "none"
    params = {}

    if len(parts) >= 2:
        rest = parts[1]
        # Detect countermeasure type
        if "countermeasure_slew_gate" in rest:
            countermeasure = "slew_gate"
        elif "countermeasure_robust_fallback" in rest:
            countermeasure = "robust_fallback"
        elif "no_countermeasure" in rest:
            countermeasure = "none"

    # Extract numeric parameters from the full folder name
    full_name = folder_name

    # ---- slew_gate params ----
    for key in ["max_rate_mps", "max_jump_m"]:
        m = re.search(rf"{key}_([\d.]+)", full_name)
        if m:
            params[key] = float(m.group(1))

    if "reject_instead_of_clamp" in full_name:
        params["reject_mode"] = "reject"
    elif countermeasure == "slew_gate":
        params["reject_mode"] = "clamp"

    # ---- robust_fallback params ----
    for key in ["fault_threshold", "window_size", "hold_seconds", "residual_threshold_m"]:
        m = re.search(rf"{key}_([\d.]+)", full_name)
        if m:
            val = m.group(1)
            params[key] = float(val) if '.' in val else int(val)

    # min_quality
    m = re.search(r"min_quality_(\d+)", full_name)
    if m:
        params["min_quality"] = int(m.group(1))

    return mode, countermeasure, params


# ---------------------------------------------------------------------------
def main():
    sweep_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(".")

    if not sweep_dir.is_dir():
        print(f"ERROR: {sweep_dir} is not a directory", file=sys.stderr)
        sys.exit(1)

    rows = []
    for exp_dir in sorted(sweep_dir.iterdir()):
        if not exp_dir.is_dir():
            continue
        tag = exp_dir.name
        inj_csv = exp_dir / "injector.csv"
        mon_csv = exp_dir / "monitor.csv"

        if not inj_csv.exists():
            print(f"  SKIP {tag} — no injector.csv", file=sys.stderr)
            continue
        if not mon_csv.exists():
            print(f"  SKIP {tag} — no monitor.csv", file=sys.stderr)
            continue

        mode, cm, params = parse_folder_name(tag)
        total, rej, clamp, faults = summarise_injector(inj_csv)
        drift, start_alt, max_dev, ekf_std, lidar_range = summarise_monitor(mon_csv)

        row = {
            "experiment": tag,
            "mode": mode,
            "countermeasure": cm,
            "max_rate_mps": params.get("max_rate_mps", ""),
            "max_jump_m": params.get("max_jump_m", ""),
            "reject_mode": params.get("reject_mode", ""),
            "window_size": params.get("window_size", ""),
            "fault_threshold": params.get("fault_threshold", ""),
            "hold_seconds": params.get("hold_seconds", ""),
            "residual_threshold_m": params.get("residual_threshold_m", ""),
            "min_quality": params.get("min_quality", ""),
            "total_samples": total,
            "rejected": rej,
            "clamped": clamp,
            "faults": faults,
            "ekf_drift_m": drift if drift is not None else "N/A",
            "ekf_start_m": start_alt if start_alt is not None else "N/A",
            "ekf_max_deviation_m": max_dev if max_dev is not None else "N/A",
            "ekf_std_m": ekf_std if ekf_std is not None else "N/A",
            "lidar_range_m": lidar_range if lidar_range is not None else "N/A",
        }
        rows.append(row)

    # ---- Write full summary CSV ----
    fieldnames = [
        "experiment", "mode", "countermeasure",
        "max_rate_mps", "max_jump_m", "reject_mode",
        "window_size", "fault_threshold", "hold_seconds",
        "residual_threshold_m", "min_quality",
        "total_samples", "rejected", "clamped", "faults",
        "ekf_drift_m", "ekf_start_m", "ekf_max_deviation_m",
        "ekf_std_m", "lidar_range_m"
    ]

    out_path = sweep_dir / "summary.csv"
    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)

    print(f"[✓] Full summary written to {out_path}")
    print(f"    {len(rows)} experiments summarised\n")

    # ---- Print readable table: per attack, best countermeasure ----
    print("=" * 110)
    print(f"{'Attack':<14} {'CM':<16} {'Best Config':<35} {'Drift(m)':>10} {'MaxDev(m)':>10} {'Clamped':>8} {'Faults':>7}")
    print("-" * 110)

    for mode in ["drift", "oscillation", "spike", "constant"]:
        mode_rows = [r for r in rows if r["mode"] == mode]
        if not mode_rows:
            continue

        # Find baseline (no countermeasure)
        baseline = [r for r in mode_rows if r["countermeasure"] == "none"]
        base_drift = baseline[0]["ekf_drift_m"] if baseline else "N/A"
        base_dev = baseline[0]["ekf_max_deviation_m"] if baseline else "N/A"

        print(f"{mode:<14} {'none':<16} {'(baseline)':<35} {str(base_drift):>10} {str(base_dev):>10} {'-':>8} {'-':>7}")

        # Find best for each countermeasure (smallest absolute drift)
        for cm in ["slew_gate", "robust_fallback"]:
            cm_rows = [r for r in mode_rows if r["countermeasure"] == cm]
            if not cm_rows:
                continue
            # Sort by absolute drift
            cm_rows_sorted = sorted(
                [r for r in cm_rows if isinstance(r["ekf_drift_m"], (int, float))],
                key=lambda r: abs(r["ekf_drift_m"])
            )
            if cm_rows_sorted:
                best = cm_rows_sorted[0]
                # Build short config string
                config_parts = []
                if best["max_rate_mps"] != "":
                    config_parts.append(f"rate={best['max_rate_mps']}")
                if best["max_jump_m"] != "":
                    config_parts.append(f"jump={best['max_jump_m']}")
                if best["fault_threshold"] != "":
                    config_parts.append(f"fault_thr={best['fault_threshold']}")
                if best["hold_seconds"] != "":
                    config_parts.append(f"hold={best['hold_seconds']}s")
                if best["window_size"] != "":
                    config_parts.append(f"win={best['window_size']}")
                if best["reject_mode"] == "reject":
                    config_parts.append("REJECT")
                config_str = ", ".join(config_parts)

                print(f"{'':<14} {cm:<16} {config_str:<35} {str(best['ekf_drift_m']):>10} {str(best['ekf_max_deviation_m']):>10} {str(best['clamped']):>8} {str(best['faults']):>7}")

        print("-" * 110)

    print(f"\n[✓] Summary by attack printed above")
    print(f"[✓] Open {out_path} in Excel/LibreOffice for full data")


if __name__ == "__main__":
    main()
