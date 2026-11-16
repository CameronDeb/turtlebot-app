# App design and why

## Pieces
- **Django app** (`control/`), on the lab PC, Windows. This is the brain: commands, job runner, trip log, photo archive,
  and later image processing and LLM planning.
- **Robot agent** (`robot_agent/agent.py`), on the TurtleBot's Pi.
  - A stdlib HTTP server with a rclpy node that turns requests into Create 3 actions.
  - Token auth (`X-Agent-Token`).
  - Nothing to pip-install on the robot.
- **Simulator** (`control/robot/sim.py`): the same interface as the HTTP client, used for demos and tests.
- **Fake Create 3** (`robot_agent/dev/fake_create3.py`): a ROS 2 node with the real action, TF and topic names, so the
  agent can be tested without the robot. It is used in WSL with `irobot_create_msgs` built from source in
  `~/create_msgs_ws` (cmake, no colcon).

## Decisions
- **One command at a time.** `Executor` runs a Job in a background thread and rejects a new one (409) until it's done.
  `Stop` sets a cancel flag and calls the agent's `/stop`, which cancels the action goal and publishes a zero
  `cmd_vel`.
- **Pose from TF `odom -> base_link`.** The Pi has no `/odom` topic. Every motion reply carries the pose before and
  after, so partial moves (stopped, bumped) are recorded as they really happened.
- **Return = retrace waypoints in reverse, always driving forward.** We don't play back negated commands, because
  driving backward is blind and the Create 3 limits backups.
  - At each waypoint it re-aims from the measured pose, so errors don't add up.
  - **Commit-before-drive**: waypoints after the current target are marked `retraced` *before* driving, so a stopped
    return resumes toward the right point, and moves made after a stop are retraced too.
- **Docking.** A trip that starts on the dock records the dock position. Return stops at the post-undock waypoint,
  turns to face the dock position, and calls the `/dock` action. This works however the real undock leaves the robot.
- **A trip starts only when the robot actually moves**, so refused commands don't leave empty trips.
- **Obstacle check.** The lidar corridor check runs in `base_link` via TF from the scan frame:
  - Width ±0.21 m, from the bumper (0.17 m) to the move length + 0.15 m.
  - Points inside ±0.19 m (the robot's own posts) are ignored.
  - With no fresh scan (docked), it refuses with a clear message. The UI checkbox turns the check off.
  - Cameron found the earlier depth-camera check too conservative, which is why the check uses the lidar and can be
    turned off.
- **Commands are a fixed vocabulary** (`commands.validate`): forward, turn, photo, scan, undock, dock, return_home,
  sequence. Buttons, typed text and a future LLM all go through it.
- **Photos** are stored as PNG plus a JPEG thumbnail plus an optional 16-bit depth PNG (mm), in
  `archive/{photos,thumbs,depth}/YYYY/MM/DD/`. File names use Arizona local time. `Photo.detections` (JSON) is reserved
  for object search.
- **Times**: `TIME_ZONE = America/Phoenix`. The lab PC's Windows clock is set to Pacific time, so the UI formats times
  in the server's zone.
- **Media** is served by a Django view (local single-user tool). Static files use mtime cache-busting (`vstatic` tag).

## Not verified on the real robot yet
Everything was tested only with the simulator and the fake Create 3 in WSL. The robot was unreachable on 2026-11-16.
Check on the real robot:
1. The real undock motion, and that dock-from-retrace succeeds.
2. That `/cmd_vel` really is TwistStamped. Stop works without it anyway, by cancelling the action goal.
3. Lidar frame TF and the robot's own posts being ignored.
4. Battery and dock-status QoS.
5. Camera frames from the stock preview and from oakd_rgbd.
