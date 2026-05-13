#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_log.py
==============

Post-experiment analysis for the LiDAR-injection PX4 SITL experiments.

The script ingests either (or both):

1. A CSV log written by ``lidar_monitor.py``  (--csv <file.csv>)
2. A PX4 ULog file produced by the SITL autopilot (--ulog <file.ulg>)

   If --ulog is omitted but --ulog-dir is given (or defaults to
   ``~/CS/final_project/PX4-Autopilot/build/px4_sitl_default/rootfs/log/``)
   the most recent ``*.ulg`` in that directory is used.

For each available source, the following plots are rendered into a
``results/`` folder (configurable with --outdir):

    Plot 1 — True altitude vs LiDAR reported vs EKF2 estimate
    Plot 2 — EKF2 innovations and test-ratios for the range sensor
    Plot 3 — Active height source (baro vs range vs GPS vs EV) over time

This script is deliberately defensive: every topic / column is looked
up and the plot is simply skipped (with a warning) if the underlying
data is absent — this way it works both on monitor-only runs and on
full ULog runs.

Author : dissertation student
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional

# ---------------------------------------------------------------------------
# ANSI colours
# ---------------------------------------------------------------------------
class C:
    R = "\033[91m"
    G = "\033[92m"
    Y = "\033[93m"
    B = "\033[94m"
    CY = "\033[96m"
    W = "\033[97m"
    BOLD = "\033[1m"
    DIM = "\033[2m"
    END = "\033[0m"


def _info(msg: str) -> None:
    print(f"{C.B}[i]{C.END} {msg}")


def _warn(msg: str) -> None:
    print(f"{C.Y}[!]{C.END} {msg}")


def _ok(msg: str) -> None:
    print(f"{C.G}[✓]{C.END} {msg}")


def _err(msg: str) -> None:
    print(f"{C.R}[x]{C.END} {msg}")


# ---------------------------------------------------------------------------
# Lazy imports (matplotlib + pandas + pyulog + numpy are heavy)
# ---------------------------------------------------------------------------
def _import_deps():
    """Import heavy deps lazily so --help works without them."""
    try:
        import numpy as np            # noqa: F401
        import pandas as pd           # noqa: F401
        import matplotlib             # noqa: F401
        matplotlib.use("Agg")         # headless-friendly
        import matplotlib.pyplot as plt  # noqa: F401
    except ImportError as e:
        _err(f"missing plotting dependency: {e}. "
             f"Install with:  pip install -r requirements.txt")
        sys.exit(1)

    try:
        from pyulog import ULog       # noqa: F401
    except ImportError:
        _warn("pyulog not installed — ULog analysis will be disabled. "
              "Install with:  pip install pyulog")
    return


# ---------------------------------------------------------------------------
# Data containers
# ---------------------------------------------------------------------------
@dataclass
class CsvData:
    """Tidy dataframe loaded from lidar_monitor.py output."""
    df: "pd.DataFrame"
    path: Path


@dataclass
class UlogData:
    """Subset of PX4 ULog topics relevant to this study."""
    path: Path
    vehicle_local_position: Optional["pd.DataFrame"] = None
    distance_sensor: Optional["pd.DataFrame"] = None
    estimator_innovations: Optional["pd.DataFrame"] = None
    estimator_innovation_test_ratios: Optional["pd.DataFrame"] = None
    estimator_status_flags: Optional["pd.DataFrame"] = None
    vehicle_air_data: Optional["pd.DataFrame"] = None
    estimator_status: Optional["pd.DataFrame"] = None


# ---------------------------------------------------------------------------
# Loaders
# ---------------------------------------------------------------------------
def load_csv(path: Path) -> Optional[CsvData]:
    """Load a lidar_monitor.py CSV log into a pandas DataFrame."""
    import pandas as pd
    if not path.exists():
        _err(f"CSV not found: {path}")
        return None
    try:
        df = pd.read_csv(path)
    except Exception as e:
        _err(f"failed to read CSV {path}: {e}")
        return None
    if "t_elapsed_s" not in df.columns:
        _err(f"CSV does not look like a lidar_monitor log (missing "
             f"t_elapsed_s column): {path}")
        return None
    # Coerce numerics — CSV writer stores floats as strings.
    for col in ("true_altitude_m", "reported_lidar_m", "ekf_altitude_m",
                "vertical_velocity_mps", "lidar_quality"):
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    _ok(f"loaded {len(df):,} rows from {path}")
    return CsvData(df=df, path=path)


def _latest_ulog(directory: Path) -> Optional[Path]:
    """Return the most recent *.ulg under ``directory`` (recursive)."""
    if not directory.exists():
        return None
    candidates = sorted(directory.rglob("*.ulg"),
                        key=lambda p: p.stat().st_mtime,
                        reverse=True)
    return candidates[0] if candidates else None


def load_ulog(path: Path) -> Optional[UlogData]:
    """Load selected topics from a PX4 ULog file."""
    try:
        from pyulog import ULog
        import pandas as pd
    except ImportError:
        _warn("pyulog unavailable — skipping ULog analysis.")
        return None

    if not path.exists():
        _err(f"ULog not found: {path}")
        return None

    topics_of_interest = [
        "vehicle_local_position",
        "distance_sensor",
        "estimator_innovations",
        "estimator_innovation_test_ratios",
        "estimator_status_flags",
        "estimator_status",
        "vehicle_air_data",
    ]
    try:
        ulog = ULog(str(path), message_name_filter_list=topics_of_interest)
    except Exception as e:
        _err(f"failed to parse ULog {path}: {e}")
        return None

    def to_df(topic_name: str) -> Optional["pd.DataFrame"]:
        for d in ulog.data_list:
            if d.name == topic_name:
                df = pd.DataFrame(d.data)
                # All PX4 topics carry a 'timestamp' field in microseconds.
                if "timestamp" in df.columns:
                    df["t_s"] = (df["timestamp"] -
                                 df["timestamp"].iloc[0]) / 1e6
                return df
        return None

    data = UlogData(
        path=path,
        vehicle_local_position=to_df("vehicle_local_position"),
        distance_sensor=to_df("distance_sensor"),
        estimator_innovations=to_df("estimator_innovations"),
        estimator_innovation_test_ratios=to_df(
            "estimator_innovation_test_ratios"),
        estimator_status_flags=to_df("estimator_status_flags"),
        estimator_status=to_df("estimator_status"),
        vehicle_air_data=to_df("vehicle_air_data"),
    )
    present = [k for k, v in data.__dict__.items()
               if k != "path" and v is not None]
    _ok(f"loaded ULog {path}  [topics: {', '.join(present) or 'none'}]")
    return data


# ---------------------------------------------------------------------------
# Plot helpers
# ---------------------------------------------------------------------------
def _style():
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "figure.figsize": (11, 5.5),
        "figure.dpi": 110,
        "savefig.dpi": 150,
        "axes.grid": True,
        "grid.alpha": 0.3,
        "axes.titlesize": 12,
        "axes.labelsize": 10,
        "legend.fontsize": 9,
        "lines.linewidth": 1.3,
    })


def plot_altitude_sources(csv: Optional[CsvData], ulog: Optional[UlogData],
                          outdir: Path, stem: str) -> Optional[Path]:
    """Plot 1 — True altitude vs LiDAR reading vs EKF estimate."""
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots()
    has_any = False

    # ---- CSV monitor traces (wall-clock elapsed) ----
    if csv is not None and not csv.df.empty:
        df = csv.df
        if "true_altitude_m" in df and df["true_altitude_m"].notna().any():
            ax.plot(df["t_elapsed_s"], df["true_altitude_m"],
                    label="True altitude (LOCAL_POSITION_NED, -z)",
                    color="#1f77b4")
            has_any = True
        if "reported_lidar_m" in df and df["reported_lidar_m"].notna().any():
            ax.plot(df["t_elapsed_s"], df["reported_lidar_m"],
                    label="Reported LiDAR (DISTANCE_SENSOR)",
                    color="#d62728", alpha=0.85)
            has_any = True
        if "ekf_altitude_m" in df and df["ekf_altitude_m"].notna().any():
            ax.plot(df["t_elapsed_s"], df["ekf_altitude_m"],
                    label="EKF2 altitude (ALTITUDE.altitude_local)",
                    color="#2ca02c", linestyle="--")
            has_any = True

    # ---- ULog traces overlayed (independent time base) ----
    if ulog is not None:
        if ulog.vehicle_local_position is not None and \
                "z" in ulog.vehicle_local_position.columns:
            df = ulog.vehicle_local_position
            ax.plot(df["t_s"], -df["z"],
                    label="ULog vehicle_local_position (-z)",
                    color="#17becf", alpha=0.6)
            has_any = True
        if ulog.distance_sensor is not None and \
                "current_distance" in ulog.distance_sensor.columns:
            df = ulog.distance_sensor
            ax.plot(df["t_s"], df["current_distance"],
                    label="ULog distance_sensor.current_distance",
                    color="#ff7f0e", alpha=0.6)
            has_any = True

    if not has_any:
        plt.close(fig)
        _warn("plot_altitude_sources: no data available — skipped.")
        return None

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Altitude / distance (m)")
    ax.set_title("True altitude vs LiDAR reported vs EKF2 estimate")
    ax.legend(loc="best")
    fig.tight_layout()
    out = outdir / f"{stem}_01_altitude_sources.png"
    fig.savefig(out)
    plt.close(fig)
    _ok(f"wrote {out}")
    return out


def plot_ekf_innovations(ulog: Optional[UlogData],
                         outdir: Path, stem: str) -> Optional[Path]:
    """Plot 2 — EKF2 range-finder innovation and test ratio."""
    import matplotlib.pyplot as plt
    import pandas as pd  # noqa: F401

    if ulog is None or (
        ulog.estimator_innovations is None
        and ulog.estimator_innovation_test_ratios is None
    ):
        _warn("plot_ekf_innovations: no estimator_innovations topic — "
              "skipped (need a ULog).")
        return None

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(11, 7))
    plotted = False

    # ---- Innovation (residual) ----
    inno = ulog.estimator_innovations
    if inno is not None:
        for col in ("hagl", "rng_hagl", "height_rng", "rng_vpos"):
            if col in inno.columns:
                ax1.plot(inno["t_s"], inno[col], label=f"innovation: {col}",
                         color="#d62728")
                plotted = True
                break
        else:
            _warn("no HAGL/range innovation column found in "
                  "estimator_innovations — (normal for some PX4 versions)")

    ax1.axhline(0.0, color="#999", linewidth=0.8)
    ax1.set_ylabel("Range innovation (m)")
    ax1.set_title("EKF2 range-finder innovation (residual: measurement − prediction)")
    if plotted:
        ax1.legend(loc="best")

    # ---- Test ratio (>1 indicates rejection) ----
    tr = ulog.estimator_innovation_test_ratios
    plotted2 = False
    if tr is not None:
        for col in ("hagl", "rng_hagl", "height_rng", "rng_vpos"):
            if col in tr.columns:
                ax2.plot(tr["t_s"], tr[col],
                         label=f"test ratio: {col}", color="#2ca02c")
                plotted2 = True
                break
        else:
            _warn("no HAGL test-ratio column found in "
                  "estimator_innovation_test_ratios.")

    ax2.axhline(1.0, color="#cc3333", linestyle="--", linewidth=0.8,
                label="rejection threshold (ratio=1)")
    ax2.set_ylabel("Innovation test ratio")
    ax2.set_xlabel("Time (s)")
    ax2.set_title("EKF2 range-finder innovation test ratio "
                  "(>1 ⇒ measurement rejected)")
    if plotted2:
        ax2.legend(loc="best")

    if not plotted and not plotted2:
        plt.close(fig)
        _warn("plot_ekf_innovations: nothing to plot — skipped.")
        return None

    fig.tight_layout()
    out = outdir / f"{stem}_02_ekf_innovations.png"
    fig.savefig(out)
    plt.close(fig)
    _ok(f"wrote {out}")
    return out


def plot_height_source(ulog: Optional[UlogData],
                       outdir: Path, stem: str) -> Optional[Path]:
    """Plot 3 — height-source activity (baro / range / gps / ev)."""
    import matplotlib.pyplot as plt

    if ulog is None or ulog.estimator_status_flags is None:
        _warn("plot_height_source: estimator_status_flags unavailable — "
              "skipped (need a ULog).")
        return None

    df = ulog.estimator_status_flags
    # Different PX4 versions expose the same information under different
    # column names. Try a list of candidates for each source.
    candidates: Dict[str, list[str]] = {
        "baro":  ["cs_baro_hgt", "cs_baro_fault"],
        "range": ["cs_rng_hgt", "cs_rng_fault"],
        "gps":   ["cs_gps_hgt", "cs_gps_hgt_fusion"],
        "ev":    ["cs_ev_hgt", "cs_ev_hgt_fusion"],
    }

    resolved: Dict[str, str] = {}
    for src, cand_list in candidates.items():
        for c in cand_list:
            if c in df.columns:
                resolved[src] = c
                break

    if not resolved:
        _warn("plot_height_source: no cs_*_hgt columns recognised — "
              "skipped.")
        return None

    fig, ax = plt.subplots(figsize=(11, 5))
    colours = {"baro": "#1f77b4", "range": "#d62728",
               "gps": "#2ca02c", "ev": "#9467bd"}

    # Stacked step-plot: each source sits at a y-offset, 1 = active.
    for i, (src, col) in enumerate(resolved.items()):
        y = df[col].astype(float).values + i * 1.4
        ax.step(df["t_s"], y, where="post",
                label=f"{src} active ({col})",
                color=colours.get(src, None))
        ax.axhline(i * 1.4, color="#eee", linewidth=0.5)

    ax.set_yticks([i * 1.4 + 0.5 for i in range(len(resolved))])
    ax.set_yticklabels(list(resolved.keys()))
    ax.set_xlabel("Time (s)")
    ax.set_title("EKF2 active height sources over time "
                 "(step up = source active / fused)")
    ax.legend(loc="upper right", ncol=2)

    fig.tight_layout()
    out = outdir / f"{stem}_03_height_source.png"
    fig.savefig(out)
    plt.close(fig)
    _ok(f"wrote {out}")
    return out


# ---------------------------------------------------------------------------
# Orchestrator
# ---------------------------------------------------------------------------
def run_analysis(csv_path: Optional[Path],
                 ulog_path: Optional[Path],
                 outdir: Path,
                 tag: str) -> list[Path]:
    _import_deps()
    _style()

    outdir.mkdir(parents=True, exist_ok=True)

    csv = load_csv(csv_path) if csv_path else None
    ulog = load_ulog(ulog_path) if ulog_path else None

    if csv is None and ulog is None:
        _err("no valid input — need --csv and/or --ulog.")
        return []

    stamp = time.strftime("%Y%m%d_%H%M%S")
    stem = f"{stamp}_{tag}" if tag else stamp

    produced = []
    p1 = plot_altitude_sources(csv, ulog, outdir, stem)
    if p1: produced.append(p1)
    p2 = plot_ekf_innovations(ulog, outdir, stem)
    if p2: produced.append(p2)
    p3 = plot_height_source(ulog, outdir, stem)
    if p3: produced.append(p3)

    # Short numeric summary written alongside the plots.
    summary = outdir / f"{stem}_summary.txt"
    with summary.open("w") as f:
        f.write(f"LiDAR injection analysis summary\n")
        f.write(f"generated : {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        if csv is not None:
            f.write(f"csv       : {csv.path}\n")
            df = csv.df
            for col in ("true_altitude_m", "reported_lidar_m",
                        "ekf_altitude_m"):
                if col in df and df[col].notna().any():
                    s = df[col].describe()
                    f.write(f"  {col:<22s}  "
                            f"n={int(s['count'])}  "
                            f"mean={s['mean']:.3f}  "
                            f"std={s['std']:.3f}  "
                            f"min={s['min']:.3f}  "
                            f"max={s['max']:.3f}\n")
        if ulog is not None:
            f.write(f"ulog      : {ulog.path}\n")
            for name, df in ulog.__dict__.items():
                if name == "path" or df is None:
                    continue
                f.write(f"  topic {name:<38s}  rows={len(df)}\n")
    _ok(f"wrote {summary}")
    produced.append(summary)
    return produced


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
DEFAULT_ULOG_DIR = Path(os.path.expanduser(
    "~/CS/final_project/PX4-Autopilot/build/px4_sitl_default/rootfs/log/"))


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="analyze_log",
        description="Plot altitude sources, EKF innovations and height-"
                    "source switching for a LiDAR injection experiment.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--csv", type=Path, default=None,
        help="Path to a lidar_monitor.py CSV log.",
    )
    parser.add_argument(
        "--ulog", type=Path, default=None,
        help="Path to a PX4 ULog (.ulg) file.",
    )
    parser.add_argument(
        "--ulog-dir", type=Path, default=DEFAULT_ULOG_DIR,
        help=f"Directory to search for latest *.ulg when --ulog not given "
             f"(default: {DEFAULT_ULOG_DIR}).",
    )
    parser.add_argument(
        "--outdir", type=Path,
        default=Path(__file__).resolve().parent / "results",
        help="Directory to write PNG plots into (default: ./results).",
    )
    parser.add_argument(
        "--tag", default="",
        help="Optional suffix added to all generated files "
             "(e.g. attack mode).",
    )
    parser.add_argument(
        "--no-auto-ulog", action="store_true",
        help="Do not auto-pick the latest ULog from --ulog-dir.",
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = _parse_args(argv)

    ulog_path = args.ulog
    if ulog_path is None and not args.no_auto_ulog:
        picked = _latest_ulog(args.ulog_dir)
        if picked is not None:
            _info(f"auto-selected latest ULog: {picked}")
            ulog_path = picked
        else:
            _warn(f"no ULog found under {args.ulog_dir} — skipping ULog "
                  "plots. Pass --ulog explicitly if needed.")

    produced = run_analysis(
        csv_path=args.csv,
        ulog_path=ulog_path,
        outdir=args.outdir,
        tag=args.tag,
    )

    if not produced:
        _err("no output was produced.")
        return 1
    _ok(f"analysis complete — {len(produced)} artefacts in {args.outdir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
