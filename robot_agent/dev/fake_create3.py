#!/usr/bin/env python3
"""Fake TurtleBot 4 for testing the agent without the robot (development only).

Provides what agent.py uses from the real robot, with the same names and types:
  actions /drive_distance /rotate_angle /dock /undock (irobot_create_msgs), cancelable;
  TF odom -> base_link (30 Hz) and a static base_link -> rplidar_link rotated 90 deg (as on the TurtleBot 4);
  /scan in rplidar_link from a 6 x 6 m square room (+ an optional box obstacle), silent while docked like the real lidar;
  /battery_state, /dock_status, /oakd_rgbd/rgb/image_raw (bgr8 640x360), /oakd_rgbd/stereo/image_raw (16UC1).

The dock sits at the origin: docked pose is (0, 0, yaw 0) with the dock just ahead in +x. Undock backs up 0.35 m and
turns 180 deg. Dock succeeds only from within 1 m of the dock while facing it (within 30 deg), like needing to see it.

Run (WSL/Linux with ROS 2 Jazzy + irobot_create_msgs):  python3 fake_create3.py [--obstacle X Y]
"""
import argparse
import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from irobot_create_msgs.action import Dock, DriveDistance, RotateAngle, Undock
from irobot_create_msgs.msg import DockStatus
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Image, LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

ROOM = 3.0  # walls at x, y = +-3 m in odom


def quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class FakeCreate3(Node):
    def __init__(self, obstacle, time_scale):
        super().__init__("fake_create3")
        self.lock = threading.Lock()
        self.x = self.y = self.yaw = 0.0
        self.docked = True
        self.obstacle = obstacle  # (x, y) centre of a 0.3 m box, or None
        self.time_scale = time_scale
        cb = ReentrantCallbackGroup()
        self.tf = TransformBroadcaster(self)
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id, st.child_frame_id = "base_link", "rplidar_link"
        st.transform.translation.x = -0.04
        st.transform.translation.z = 0.2
        (
            st.transform.rotation.x,
            st.transform.rotation.y,
            st.transform.rotation.z,
            st.transform.rotation.w,
        ) = quat(math.pi / 2)
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(st)
        self.scan_pub = self.create_publisher(
            LaserScan, "/scan", qos_profile_sensor_data
        )
        self.batt_pub = self.create_publisher(
            BatteryState, "/battery_state", qos_profile_sensor_data
        )
        self.dock_pub = self.create_publisher(
            DockStatus, "/dock_status", qos_profile_sensor_data
        )
        self.rgb_pub = self.create_publisher(
            Image, "/oakd_rgbd/rgb/image_raw", qos_profile_sensor_data
        )
        self.depth_pub = self.create_publisher(
            Image, "/oakd_rgbd/stereo/image_raw", qos_profile_sensor_data
        )
        self.create_timer(1 / 30, self.pub_tf, callback_group=cb)
        self.create_timer(1 / 8, self.pub_scan, callback_group=cb)
        self.create_timer(1.0, self.pub_status, callback_group=cb)
        self.create_timer(1 / 5, self.pub_camera, callback_group=cb)
        kw = dict(
            callback_group=cb,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda g: CancelResponse.ACCEPT,
        )
        self.servers = [
            ActionServer(self, DriveDistance, "/drive_distance", self.do_drive, **kw),
            ActionServer(self, RotateAngle, "/rotate_angle", self.do_rotate, **kw),
            ActionServer(self, Dock, "/dock", self.do_dock, **kw),
            ActionServer(self, Undock, "/undock", self.do_undock, **kw),
        ]

    # ---- publishers

    def pub_tf(self):
        with self.lock:
            x, y, yaw = self.x, self.y, self.yaw
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = "odom", "base_link"
        t.transform.translation.x, t.transform.translation.y = x, y
        (
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        ) = quat(yaw)
        self.tf.sendTransform(t)

    def pub_scan(self):
        with self.lock:
            if self.docked:
                return  # lidar motor stops on the dock
            x, y, yaw = self.x, self.y, self.yaw
        n = 720
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "rplidar_link"
        msg.angle_min, msg.angle_increment = -math.pi, 2 * math.pi / n
        msg.angle_max = msg.angle_min + (n - 1) * msg.angle_increment
        msg.range_min, msg.range_max = 0.15, 12.0
        # lidar pose in odom: base_link pose * (-0.04, 0, yaw + 90 deg)
        lx, ly, lyaw = (
            x - 0.04 * math.cos(yaw),
            y - 0.04 * math.sin(yaw),
            yaw + math.pi / 2,
        )
        ang = lyaw + msg.angle_min + np.arange(n) * msg.angle_increment
        dx, dy = np.cos(ang), np.sin(ang)
        with np.errstate(divide="ignore", invalid="ignore"):
            tx = np.where(
                dx > 0, (ROOM - lx) / dx, np.where(dx < 0, (-ROOM - lx) / dx, np.inf)
            )
            ty = np.where(
                dy > 0, (ROOM - ly) / dy, np.where(dy < 0, (-ROOM - ly) / dy, np.inf)
            )
        r = np.minimum(tx, ty)
        if self.obstacle is not None:  # 0.3 m box: slab intersection
            ox, oy = self.obstacle
            with np.errstate(divide="ignore", invalid="ignore"):
                t1x, t2x = (ox - 0.15 - lx) / dx, (ox + 0.15 - lx) / dx
                t1y, t2y = (oy - 0.15 - ly) / dy, (oy + 0.15 - ly) / dy
            tmin = np.maximum(np.minimum(t1x, t2x), np.minimum(t1y, t2y))
            tmax = np.minimum(np.maximum(t1x, t2x), np.maximum(t1y, t2y))
            hit = np.isfinite(tmin) & (tmax >= tmin) & (tmin > 0)
            r = np.where(hit, np.minimum(r, tmin), r)
        msg.ranges = [float(v) for v in r]
        self.scan_pub.publish(msg)

    def pub_status(self):
        b = BatteryState()
        b.header.stamp = self.get_clock().now().to_msg()
        b.percentage = 0.87
        self.batt_pub.publish(b)
        d = DockStatus()
        d.header.stamp = b.header.stamp
        with self.lock:
            d.is_docked = self.docked
            d.dock_visible = self.sees_dock()
        self.dock_pub.publish(d)

    def pub_camera(self):
        stamp = self.get_clock().now().to_msg()
        with self.lock:
            yaw = self.yaw
        w, h = 640, 360
        img = np.zeros((h, w, 3), np.uint8)
        img[..., 0] = np.linspace(40, 200, w, dtype=np.uint8)[None, :]  # B
        img[..., 1] = int((math.degrees(yaw) % 360) / 360 * 255)  # G encodes heading
        img[..., 2] = np.linspace(200, 40, h, dtype=np.uint8)[:, None]  # R
        m = Image(
            height=h,
            width=w,
            encoding="bgr8",
            is_bigendian=0,
            step=w * 3,
            data=img.tobytes(),
        )
        m.header.stamp, m.header.frame_id = stamp, "oakd_rgbd_rgb_camera_optical_frame"
        self.rgb_pub.publish(m)
        depth = np.full((h, w), 2000, np.uint16)
        depth[: h // 3] = 0
        dm = Image(
            height=h,
            width=w,
            encoding="16UC1",
            is_bigendian=0,
            step=w * 2,
            data=depth.tobytes(),
        )
        dm.header = m.header
        self.depth_pub.publish(dm)

    # ---- helpers

    def sees_dock(self):
        dist = math.hypot(self.x, self.y)
        if self.docked:
            return True
        bearing = math.atan2(-self.y, -self.x)
        return dist < 1.0 and abs(wrap(bearing - self.yaw)) < math.radians(30)

    def sleep(self, s):
        time.sleep(s * self.time_scale)

    def animate(self, goal_handle, steps, step_fn):
        for _ in range(steps):
            if goal_handle.is_cancel_requested:
                return False
            with self.lock:
                step_fn()
            self.sleep(0.05)
        return True

    # ---- actions

    def do_drive(self, gh):
        dist, speed = gh.request.distance, max(gh.request.max_translation_speed, 0.01)
        steps = max(1, int(abs(dist) / speed / 0.05))
        step = dist / steps

        def fn():
            self.docked = False
            self.x += step * math.cos(self.yaw)
            self.y += step * math.sin(self.yaw)

        done = self.animate(gh, steps, fn)
        return self.finish(gh, done, DriveDistance.Result())

    def do_rotate(self, gh):
        ang, speed = gh.request.angle, max(gh.request.max_rotation_speed, 0.01)
        steps = max(1, int(abs(ang) / speed / 0.05))
        step = ang / steps

        def fn():
            self.yaw = wrap(self.yaw + step)

        done = self.animate(gh, steps, fn)
        return self.finish(gh, done, RotateAngle.Result())

    def do_undock(self, gh):
        res = Undock.Result()
        with self.lock:
            if not self.docked:
                res.is_docked = False
                gh.abort()
                return res
            self.docked = False
        for _ in range(10):
            with self.lock:
                self.x -= 0.035
            self.sleep(0.05)
        for _ in range(10):
            with self.lock:
                self.yaw = wrap(self.yaw + math.pi / 10)
            self.sleep(0.05)
        res.is_docked = False
        gh.succeed()
        return res

    def do_dock(self, gh):
        res = Dock.Result()
        with self.lock:
            ok = self.sees_dock()
        if not ok:
            res.is_docked = False
            gh.abort()
            return res
        self.sleep(1.0)
        with self.lock:
            self.x = self.y = self.yaw = 0.0
            self.docked = True
        res.is_docked = True
        gh.succeed()
        return res

    def finish(self, gh, done, result):
        if done:
            gh.succeed()
        else:
            gh.canceled()
        return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--obstacle", type=float, nargs=2, metavar=("X", "Y"))
    ap.add_argument(
        "--time-scale",
        type=float,
        default=0.2,
        help="1.0 = real time; smaller = faster tests",
    )
    args = ap.parse_args()
    rclpy.init()
    node = FakeCreate3(tuple(args.obstacle) if args.obstacle else None, args.time_scale)
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    print("fake create3 running", flush=True)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
