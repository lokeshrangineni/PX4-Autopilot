#!/usr/bin/env python3
"""Send raw Gazebo horizontal LiDAR to the pod-local demo planner.

PX4's MAVLink OBSTACLE_DISTANCE stream uses the fused Position-mode topic,
which is not a source of fresh scans during Offboard flight. Read Gazebo
Transport directly, independently of the selected flight mode.
"""

import json
import math
import os
import socket
import subprocess
import time


def main():
    world = os.environ["PX4_GZ_WORLD"]
    topic = f"/world/{world}/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan"
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    with subprocess.Popen(
        ["gz", "topic", "-e", "-t", topic, "--json-output"],
        stdout=subprocess.PIPE, text=True,
    ) as process:
        try:
            for line in process.stdout:
                if not line.strip():
                    continue
                scan = json.loads(line)
                ranges = [float(value) for value in scan["ranges"]]
                # Preserve each ray: the planner checks the travel cone and
                # treats NaN as missing coverage, not as clear space.
                message = {
                    "observed": time.monotonic(),
                    "angle_min": float(scan["angleMin"]),
                    "angle_step": float(scan["angleStep"]),
                    "range_max": float(scan["rangeMax"]),
                    "ranges": [value if math.isfinite(value) else
                               (float(scan["rangeMax"]) if value > 0 else None)
                               for value in ranges],
                }
                sock.sendto(json.dumps(message, allow_nan=False).encode(),
                            ("127.0.0.1", 14541))
            raise RuntimeError("Gazebo LiDAR stream closed")
        finally:
            process.terminate()
            process.wait(timeout=5)


if __name__ == "__main__":
    main()
