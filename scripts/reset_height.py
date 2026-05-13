#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
reset_lidar_height.py

Continuously publishes a sensible downward DISTANCE_SENSOR height to PX4 SITL.

Use this when PX4/EKF2 is rejecting a fake LiDAR value such as 2.5 m while the
vehicle is still on the ground. Default output is 0.10 m.

Example:
    python3 reset_lidar_height.py
    python3 reset_lidar_height.py --height 0.10 --endpoint udpout:127.0.0.1:14580
    python3 reset_lidar_height.py --height 0.15 --rate 50 --duration 120
"""

import argparse
import os
import signal
import sys
import time

os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "common")

try:
    from pymavlink import mavutil
except ImportError:
    print("[FATAL] pymavlink is not installed. Install with: pip install pymavlink")
    sys.exit(1)


MIN_DISTANCE_CM = 5
MAX_DISTANCE_CM = 4000

# MAV_DISTANCE_SENSOR_LASER
SENSOR_TYPE = 0

# Downward-facing sensor orientation.
# 25 = MAV_SENSOR_ROTATION_PITCH_270
SENSOR_ORIENTATION = 25

SENSOR_ID = 0

# Use a companion/onboard component id.
SPOOF_SYSID = 1
SPOOF_COMPID = 191


stop = False


def on_signal(signum, frame):
    global stop
    print(f"\n[INFO] Signal {signum} received, stopping.")
    stop = True


def clamp_cm(value_cm: float) -> int:
    value = int(round(value_cm))
    return max(MIN_DISTANCE_CM, min(MAX_DISTANCE_CM, value))


def send_distance_sensor(master, height_m: float, signal_quality: int, t_elapsed: float):
    distance_cm = clamp_cm(height_m * 100.0)

    # PX4 accepts this as time_boot_ms. Keep it monotonic and 32-bit.
    time_boot_ms = int(t_elapsed * 1000.0) & 0xFFFFFFFF

    covariance = 1 if signal_quality > 0 else 255

    try:
        master.mav.distance_sensor_send(
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
            signal_quality=signal_quality,
        )
    except TypeError:
        # Older pymavlink fallback.
        master.mav.distance_sensor_send(
            time_boot_ms,
            MIN_DISTANCE_CM,
            MAX_DISTANCE_CM,
            distance_cm,
            SENSOR_TYPE,
            SENSOR_ID,
            SENSOR_ORIENTATION,
            covariance,
        )


def main():
    parser = argparse.ArgumentParser(
        description="Continuously publish a sane downward LiDAR height to PX4 SITL."
    )

    parser.add_argument(
        "--height",
        type=float,
        default=0.10,
        help="Height/range above ground in metres. Default: 0.10.",
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=50.0,
        help="Publish rate in Hz. Default: 50.",
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Duration in seconds. 0 means run forever. Default: 0.",
    )

    parser.add_argument(
        "--endpoint",
        default="udpout:127.0.0.1:14580",
        help="MAVLink endpoint. Default: udpout:127.0.0.1:14580.",
    )

    parser.add_argument(
        "--quality",
        type=int,
        default=100,
        help="Signal quality 0-100. Default: 100.",
    )

    args = parser.parse_args()

    if args.height < 0.05:
        print("[WARN] Height below sensor minimum. Clamping to 0.05 m.")
        args.height = 0.05

    if args.height > 40.0:
        print("[WARN] Height above sensor maximum. Clamping to 40.0 m.")
        args.height = 40.0

    args.quality = max(0, min(100, args.quality))

    print("[INFO] Opening MAVLink endpoint:", args.endpoint)
    master = mavutil.mavlink_connection(
        args.endpoint,
        source_system=SPOOF_SYSID,
        source_component=SPOOF_COMPID,
        input=False,
    )

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    period = 1.0 / args.rate
    start = time.monotonic()
    next_tick = start
    sent = 0

    print(
        f"[INFO] Publishing DISTANCE_SENSOR height={args.height:.2f} m "
        f"rate={args.rate:.1f} Hz quality={args.quality} endpoint={args.endpoint}"
    )
    print("[INFO] Press Ctrl+C to stop.")

    try:
        while not stop:
            now = time.monotonic()
            elapsed = now - start

            if args.duration > 0 and elapsed >= args.duration:
                break

            send_distance_sensor(master, args.height, args.quality, elapsed)
            sent += 1

            if sent % int(max(1, args.rate)) == 0:
                print(
                    f"[INFO] sent={sent} t={elapsed:.1f}s "
                    f"height={args.height:.2f} m"
                )

            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

    finally:
        try:
            master.close()
        except Exception:
            pass

        elapsed = time.monotonic() - start
        eff_rate = sent / elapsed if elapsed > 0 else 0.0
        print(f"[INFO] Done. Sent {sent} messages in {elapsed:.2f}s ({eff_rate:.1f} Hz).")


if __name__ == "__main__":
    main()
