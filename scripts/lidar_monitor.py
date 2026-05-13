#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lidar_monitor.py
================

Real-time MAVLink observer for the dissertation LiDAR-injection
experiments.  It connects to a PX4 SITL instance and records the
quantities needed to diagnose whether (and how) the spoofed
DISTANCE_SENSOR data produced by ``lidar_injector.py`` is being
accepted by the EKF2 height estimator.

Captured telemetry
------------------
* LOCAL_POSITION_NED ..... ground truth-ish altitude (z, vz)
* DISTANCE_SENSOR ........ the LiDAR reading PX4 currently sees
                           (real or spoofed, depending on experiment)
* ALTITUDE ............... PX4's fused altitude estimate
                           (altitude_relative / altitude_local)
* GLOBAL_POSITION_INT ..... fallback altitude source
* VFR_HUD ................ airspeed/climb fallback
* STATUSTEXT .............. autopilot warnings (useful around EKF
                            rejections / failsafes)

CSV columns (one row per monitor tick)
--------------------------------------
    wall_time_iso, monotonic_s, t_elapsed_s,
    true_altitude_m,          # -LOCAL_POSITION_NED.z
    reported_lidar_m,         # DISTANCE_SENSOR.current_distance
    lidar_quality,            # DISTANCE_SENSOR.signal_quality
    ekf_altitude_m,           # ALTITUDE.altitude_local (fallback chain)
    vertical_velocity_mps     # LOCAL_POSITION_NED.vz

Usage
-----
    ./lidar_monitor.py
    ./lidar_monitor.py --endpoint udpin:127.0.0.1:14550 --log mylog.csv

Run until Ctrl+C; the live table is printed once per second.
"""

from __future__ import annotations

import argparse
import csv
import os
import signal
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# Force MAVLink v2 + 'common' dialect before importing pymavlink, so the
# extended DISTANCE_SENSOR fields (signal_quality in particular) can be
# decoded when present.
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


# ---------------------------------------------------------------------------
# ANSI colours
# ---------------------------------------------------------------------------
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


# ---------------------------------------------------------------------------
# State container — the latest value of every telemetry item of interest
# ---------------------------------------------------------------------------
@dataclass
class TelemetryState:
    true_altitude_m: Optional[float] = None        # -LOCAL_POSITION_NED.z
    reported_lidar_m: Optional[float] = None       # DISTANCE_SENSOR
    lidar_quality: Optional[int] = None
    lidar_last_seen: float = 0.0                   # monotonic
    ekf_altitude_m: Optional[float] = None         # ALTITUDE.altitude_local
    altitude_relative_m: Optional[float] = None    # ALTITUDE.altitude_relative
    global_alt_m: Optional[float] = None           # GLOBAL_POSITION_INT.alt
    vertical_velocity_mps: Optional[float] = None  # LOCAL_POSITION_NED.vz
    last_statustext: str = ""
    statustext_history: list[str] = field(default_factory=list)

    def ekf_or_fallback(self) -> Optional[float]:
        """Return the best available estimator altitude."""
        if self.ekf_altitude_m is not None:
            return self.ekf_altitude_m
        if self.altitude_relative_m is not None:
            return self.altitude_relative_m
        if self.global_alt_m is not None:
            return self.global_alt_m
        return None


# ---------------------------------------------------------------------------
# Monitor
# ---------------------------------------------------------------------------
class LidarMonitor:
    """Listens to PX4 MAVLink and logs the quantities required for
    post-hoc analysis of the LiDAR-injection experiments."""

    MESSAGE_TYPES = {
        "LOCAL_POSITION_NED",
        "DISTANCE_SENSOR",
        "ALTITUDE",
        "GLOBAL_POSITION_INT",
        "VFR_HUD",
        "STATUSTEXT",
        "HEARTBEAT",
    }

    def __init__(
        self,
        endpoint: str = "udpin:127.0.0.1:14550",
        log_path: Optional[Path] = None,
        print_hz: float = 1.0,
    ) -> None:
        self.endpoint = endpoint
        self.print_period = 1.0 / max(0.1, print_hz)

        # ------------- CSV log setup -------------
        if log_path is None:
            log_dir = Path(__file__).resolve().parent / "logs"
            log_dir.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            log_path = log_dir / f"monitor_{stamp}.csv"
        self.log_path = Path(log_path)
        self._log_fh = self.log_path.open("w", newline="")
        self._log = csv.writer(self._log_fh)
        self._log.writerow([
            "wall_time_iso",
            "monotonic_s",
            "t_elapsed_s",
            "true_altitude_m",
            "reported_lidar_m",
            "lidar_quality",
            "ekf_altitude_m",
            "vertical_velocity_mps",
            "last_statustext",
        ])

        # ------------- runtime --------------------
        self.state = TelemetryState()
        self._stop = False
        self._master: Optional[mavutil.mavfile] = None
        self._heartbeat_seen = False

    # ------------------------------------------------------------------
    def _connect(self) -> None:
        print(f"{C.B}[*]{C.END} listening on {self.endpoint} …")
        self._master = mavutil.mavlink_connection(self.endpoint, input=True)
        # Do NOT wait_heartbeat() — on udpin:14550 we may not always see one
        # immediately depending on routing.  We just start reading.
        print(f"{C.G}[+]{C.END} MAVLink listener open, waiting for messages.")

    # ------------------------------------------------------------------
    def _handle(self, msg) -> None:
        """Dispatch a single MAVLink message to state update."""
        msg_type = msg.get_type()
        now = time.monotonic()

        if msg_type == "HEARTBEAT":
            if not self._heartbeat_seen:
                self._heartbeat_seen = True
                print(f"{C.G}[+]{C.END} first HEARTBEAT received "
                      f"(sys={msg.get_srcSystem()}, "
                      f"comp={msg.get_srcComponent()}).")

        elif msg_type == "LOCAL_POSITION_NED":
            # NED frame: z is down-positive, so altitude AGL ≈ -z.
            self.state.true_altitude_m = -float(msg.z)
            self.state.vertical_velocity_mps = -float(msg.vz)

        elif msg_type == "DISTANCE_SENSOR":
            self.state.reported_lidar_m = float(msg.current_distance) / 100.0
            quality = getattr(msg, "signal_quality", None)
            if quality is not None:
                try:
                    self.state.lidar_quality = int(quality)
                except (TypeError, ValueError):
                    self.state.lidar_quality = None
            self.state.lidar_last_seen = now

        elif msg_type == "ALTITUDE":
            # ALTITUDE.altitude_local is the EKF's estimate in local frame.
            self.state.ekf_altitude_m = float(msg.altitude_local)
            self.state.altitude_relative_m = float(msg.altitude_relative)

        elif msg_type == "GLOBAL_POSITION_INT":
            # alt is in mm AMSL; relative_alt is in mm above home.
            self.state.global_alt_m = float(msg.relative_alt) / 1000.0

        elif msg_type == "VFR_HUD":
            # climb is m/s — use as fallback for vertical velocity.
            if self.state.vertical_velocity_mps is None:
                self.state.vertical_velocity_mps = float(msg.climb)

        elif msg_type == "STATUSTEXT":
            text = msg.text.strip() if isinstance(msg.text, str) \
                   else bytes(msg.text).rstrip(b"\x00").decode(
                       "utf-8", errors="replace").strip()
            if text:
                self.state.last_statustext = text
                self.state.statustext_history.append(
                    f"{time.strftime('%H:%M:%S')} {text}"
                )
                # Highlight error-class messages on console.
                print(f"  {C.Y}[STATUSTEXT]{C.END} {text}")

    # ------------------------------------------------------------------
    def _write_row(self, t_elapsed: float) -> None:
        s = self.state
        self._log.writerow([
            time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()) +
            f".{int((time.time() % 1) * 1e6):06d}",
            f"{time.monotonic():.6f}",
            f"{t_elapsed:.6f}",
            "" if s.true_altitude_m is None else f"{s.true_altitude_m:.4f}",
            "" if s.reported_lidar_m is None else
                f"{s.reported_lidar_m:.4f}",
            "" if s.lidar_quality is None else str(s.lidar_quality),
            "" if s.ekf_or_fallback() is None else
                f"{s.ekf_or_fallback():.4f}",
            "" if s.vertical_velocity_mps is None else
                f"{s.vertical_velocity_mps:.4f}",
            s.last_statustext,
        ])
        self._log_fh.flush()

    # ------------------------------------------------------------------
    def _print_table(self, t_elapsed: float) -> None:
        s = self.state
        now = time.monotonic()
        lidar_age = (now - s.lidar_last_seen) if s.lidar_last_seen else float("inf")

        def fmt(val, unit="", fmt_spec=".2f") -> str:
            if val is None:
                return f"{C.DIM}n/a{C.END}"
            return f"{val:{fmt_spec}}{unit}"

        # stale/inject warning
        if s.reported_lidar_m is None:
            lidar_disp = f"{C.DIM}no DISTANCE_SENSOR{C.END}"
        elif lidar_age > 2.0:
            lidar_disp = f"{C.R}{s.reported_lidar_m:.2f} m (stale {lidar_age:.1f}s){C.END}"
        else:
            lidar_disp = f"{C.Y}{s.reported_lidar_m:.2f} m{C.END}"

        ekf_best = s.ekf_or_fallback()
        print(
            f"{C.DIM}t={t_elapsed:7.2f}s{C.END}  "
            f"true_alt={C.W}{fmt(s.true_altitude_m, ' m')}{C.END}  "
            f"lidar={lidar_disp}  "
            f"q={fmt(s.lidar_quality, '', 'd') if s.lidar_quality is not None else f'{C.DIM}n/a{C.END}'}  "
            f"ekf_alt={C.CY}{fmt(ekf_best, ' m')}{C.END}  "
            f"vz={fmt(s.vertical_velocity_mps, ' m/s')}"
        )

    # ------------------------------------------------------------------
    def run(self) -> None:
        self._connect()
        signal.signal(signal.SIGINT, self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

        start = time.monotonic()
        next_print = start
        last_log = start

        print(f"{C.M}[>]{C.END} monitoring… Ctrl+C to stop. "
              f"log: {self.log_path}")

        try:
            assert self._master is not None
            while not self._stop:
                # recv_match is non-blocking when timeout is small
                msg = self._master.recv_match(blocking=True, timeout=0.1)
                if msg is not None and msg.get_type() in self.MESSAGE_TYPES:
                    self._handle(msg)

                now = time.monotonic()
                t_elapsed = now - start

                # 1 Hz console tick
                if now >= next_print:
                    self._print_table(t_elapsed)
                    next_print = now + self.print_period

                # 2 Hz log tick — gives us a regular time base even if
                # a particular message happens to be slow.
                if now - last_log >= 0.5:
                    self._write_row(t_elapsed)
                    last_log = now
        finally:
            self._shutdown(time.monotonic() - start)

    # ------------------------------------------------------------------
    def _on_signal(self, signum, _frame):
        print(f"\n{C.Y}[!]{C.END} signal {signum} received — stopping.")
        self._stop = True

    def _shutdown(self, elapsed: float) -> None:
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
        print(f"{C.G}[✓]{C.END} monitor stopped after {elapsed:.2f}s. "
              f"log: {self.log_path}")
        if self.state.statustext_history:
            print(f"{C.CY}STATUSTEXT summary:{C.END}")
            for line in self.state.statustext_history[-15:]:
                print(f"  {line}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="lidar_monitor",
        description="Real-time PX4 telemetry logger for LiDAR injection "
                    "experiments.",
    )
    parser.add_argument(
        "--endpoint", default="udpin:127.0.0.1:14550",
        help="pymavlink listen endpoint (default: udpin:127.0.0.1:14550).",
    )
    parser.add_argument(
        "--log", default=None,
        help="CSV log path (default: ./logs/monitor_<ts>.csv).",
    )
    parser.add_argument(
        "--print-hz", type=float, default=1.0,
        help="Console print rate in Hz (default: 1.0).",
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = _parse_args(argv)
    mon = LidarMonitor(
        endpoint=args.endpoint,
        log_path=Path(args.log) if args.log else None,
        print_hz=args.print_hz,
    )
    try:
        mon.run()
    except Exception as exc:  # pragma: no cover - defensive
        print(f"{C.R}[FATAL]{C.END} {type(exc).__name__}: {exc}")
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
