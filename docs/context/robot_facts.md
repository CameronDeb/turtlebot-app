# TurtleBot 4 facts the app depends on

Source: `turtlebot-gs/notes/robot_config.md` on the lab PC (verified Oct 2026).

- Create 3 base, Raspberry Pi 4 (3.7 GB RAM), ROS 2 **Jazzy**, OAK-D Pro camera, RPLIDAR, no namespace, domain 0.
- Network: Windows Mobile Hotspot on the lab PC; the robot gets **192.168.137.140**. SSH `ubuntu@192.168.137.140`
  with key auth from **Windows** OpenSSH. **WSL (NAT mode) cannot reach the robot**, so the web app runs on Windows.
- Motion: Create 3 actions `/drive_distance` (DriveDistance: distance, max_translation_speed),
  `/rotate_angle` (RotateAngle: angle rad, + = CCW/left), `/dock`, `/undock` (irobot_create_msgs).
  `/cmd_vel` is TwistStamped (create3_repub).
- Pose: there is **no /odom topic** on the Pi; use TF `odom -> base_link`.
- Lidar `/scan`: the motor **stops while docked**, so there is no fresh scan on the dock.
- Camera:
  - Stock bringup: `/oakd/rgb/preview/image_raw`, 250x250, RGB only.
  - Our RGB-D node (`turtlebot-gs/tools/oakd_rgbd_start.sh`, IR projector OFF, never turn it on: it browned out
    the USB hub): `/oakd_rgbd/rgb/image_raw` bgr8 640x360 and `/oakd_rgbd/stereo/image_raw` 16UC1 mm, aligned.
  - OAK-D raw depth reads ~11-14% long; range model raw = (1090.1 + 14.82 Z) Z (mm, Z in m).
  - The OAK-D can drop off USB (X_LINK_ERROR); restart the node.
- Battery: ~25%/h idle with the camera on and off the dock. Dock between sessions.
- Rules: ask before robot motion; a person confirms the floor is clear; never drive unattended. Robot-side installs
  are allowed since 2026-11-04 but log them in `turtlebot-gs/notes/session_log.md`, keep the stock bringup working,
  watch SD space.
