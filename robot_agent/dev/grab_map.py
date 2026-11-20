#!/usr/bin/env python3
"""Save the robot's current learned map (/map from slam_toolbox) as a PGM image + YAML, for viewing on the PC.

Run on the robot:  python3 grab_map.py ~/robot_agent/maps/snapshot
Writes snapshot.pgm (free 254, occupied 0, unknown 205; row 0 = top = max y) and snapshot.yaml (resolution, origin).
Read-only: it only listens to /map.
"""
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "map_snapshot"
    rclpy.init()
    node = rclpy.create_node("grab_map")
    got = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, "/map", got.append, qos)
    end = time.monotonic() + 15
    while not got and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.5)
    if not got:
        raise SystemExit("no /map received in 15 s (is slam_toolbox running and has the robot left the dock?)")
    m = got[-1]
    w, h = m.info.width, m.info.height
    rows = []
    for r in range(h - 1, -1, -1):
        row = m.data[r * w:(r + 1) * w]
        rows.append(bytes(205 if v < 0 else (0 if v >= 65 else (254 if v < 25 else 205)) for v in row))
    with open(out + ".pgm", "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode())
        f.write(b"".join(rows))
    o = m.info.origin.position
    with open(out + ".yaml", "w") as f:
        f.write(f"image: {out.split('/')[-1]}.pgm\nresolution: {m.info.resolution}\norigin: [{o.x}, {o.y}, 0.0]\n"
                f"size_m: [{w * m.info.resolution:.2f}, {h * m.info.resolution:.2f}]\n")
    print(f"{w}x{h} cells at {m.info.resolution} m = {w * m.info.resolution:.1f} x {h * m.info.resolution:.1f} m")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
