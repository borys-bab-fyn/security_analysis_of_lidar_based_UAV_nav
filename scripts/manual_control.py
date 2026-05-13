#!/usr/bin/env python3

import time
from pymavlink import mavutil

m = mavutil.mavlink_connection(
    "udpout:127.0.0.1:14580",
    source_system=2,
    source_component=1,
)

while True:
    m.mav.manual_control_send(
        1,      # target system
        0,      # pitch
        0,      # roll
        500,    # throttle neutral
        0,      # yaw
        0,
    )
    time.sleep(0.05)
