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
- **Obstacle check (agent 1.1, 2026-11-18).** The lidar corridor check runs in `base_link` via TF from the scan frame:
  - Width ±0.21 m in front of the bumper (0.17 m); points inside ±0.19 m (the robot's own posts) are ignored.
  - A move is **shortened** to stop 0.10 m before the nearest return in the corridor; it is refused only with less
    than 2 cm of room. While driving, the lidar is polled every 0.1 s and the goal is canceled (outcome `obstacle`)
    if anything comes within 6 cm of the bumper.
  - Before this it refused the whole move if anything was within move + 0.15 m, which Cameron found far too
    restrictive in the cluttered lab.
  - With no fresh scan (docked: the TurtleBot's power saver stops the lidar and camera on the dock), it refuses with
    a clear message. The UI checkbox turns the check off.
- **E-stop.** The robot's config maps a short press of the Create 3 power button and controller B (○) to `EStop`, which
  disables the drive motors (`/wheel_status.wheels_enabled = false`); motion goals are then accepted but never move.
  The agent reports `wheels_enabled`, refuses motion while it is false, and `POST /motors` calls `/e_stop` with
  `e_stop_on: false`. The UI shows a banner with a **Motors on** button. (Found 2026-11-18 when nothing would move.)
- **Guards (agent 1.2, 2026-11-20)**, on every drive and turn: **no progress** for 1.5 s (< 2 cm or < 3°) cancels
  the motion (outcome `stalled`); Create 3 hazards (bump, cliff, wheel drop, stall) cancel it too, but
  `/hazard_detection` is **not republished to the Pi** on our robot (commented out in turtlebot4_bringup
  `republisher.yaml`; enabling it is a robot config change, ask Cameron). Forward is refused while the bumper is
  pressed. Added after Find drove into a chair's low sled base (below the 19 cm lidar plane) and pushed for 25 s.
- **Agent 1.3/1.4 (2026-11-20)**: camera topics are subscribed only while pictures are taken (released after 30 s
  idle; `GET /status?camera=1` warms them up), and a single-threaded rclpy executor: agent CPU 65% → 30% of a Pi core,
  Pi load 3.7 → 1.8. Stall guard tightened to 0.8 s. Real-time obstacles: while driving, anything that appears in the
  rest of the path (the move would now end closer than the margin) stops the robot 0.25 m before it (outcome
  `obstacle`), and Find re-plans around it (up to 5 times); anything within 6 cm stops it at once.
- **Route planning for Find (`pathplan.py`, 2026-11-20)**: the object's sighting becomes an odom position; each leg
  plans an A* route on a 5 cm grid from a fresh lidar scan (`GET /scan`), keep-out 0.30 m (gaps between legs
  narrower than ~0.6 m are closed, which keeps it from going under tables/chairs), extra cost inside 0.50 m; drive
  ≤1 m, re-look when the object should be in view (re-anchors the position, or keeps the remembered one if it is
  hidden), re-plan. No route → it reports instead of driving. Unseen space counts as free. Demo on the Oct 28 lidar
  map: `turtlebot-gs/tools/route_demo.py` → `turtlebot-gs/results/lidar_map/route_demo_run_06_L.png`.
- **Distance off-axis fix (2026-11-20)**: depth and floor geometry give distance along the camera axis; along the
  sight line it is longer by 1/cos(angle off axis). Found by the route tests (12% short at 28°).
- **Object search (`find`)**: see `executor.do_find` and `vision.py`.
  - Detector: Grounding DINO tiny in WSL (`detector/serve.py`, env `detect`, RTX 5080, ~0.2 s per picture),
    auto-started by `control/detector.py`. With the simulator, `SimDetector` uses the simulator's ground truth.
  - Search: overlapping pictures (step = 80% of the camera's horizontal FOV; stock preview ~42°, so 11 pictures for a
    full turn) until the object is seen.
  - Locate: bearing from the box centre and the camera intrinsics (agent sends them; the robot has no camera_info).
    Distance: depth if present, else the floor-contact geometry of the box bottom (camera 0.244 m up, level),
    tightened by the lidar only when the two agree within 0.3 m (a nearer lidar return is something in front of the
    object). When the box is cut off by the bottom of the picture, dead reckoning from the last estimate is used.
  - Approach: turn to it, drive min(gap - standoff, 1 m), look again; arrive when within 5 cm of the standoff. If a leg
    is shortened by an obstacle that isn't the object, it stops and reports. Lost close by (below the camera's view) →
    finish on dead reckoning.
  - All moves are recorded in the trip, so Return brings the robot back.
- **Commands are a fixed vocabulary** (`commands.validate`): forward, turn, photo, scan, undock, dock, return_home,
  find, sequence. Buttons, typed text and a future LLM all go through it.
- **Photos** are stored as PNG plus a JPEG thumbnail plus an optional 16-bit depth PNG (mm), in
  `archive/{photos,thumbs,depth}/YYYY/MM/DD/`. File names use Arizona local time. `Photo.detections` (JSON) is reserved
  for object search.
- **Times**: `TIME_ZONE = America/Phoenix`. The lab PC's Windows clock is set to Pacific time, so the UI formats times
  in the server's zone.
- **Media** is served by a Django view (local single-user tool). Static files use mtime cache-busting (`vstatic` tag).

## Not verified on the real robot yet
Verified on 2026-11-18: agent runs on the robot, status, photo (stock preview), wheel status / e-stop reporting,
detector on a real robot picture (chairs, table, whiteboard found; "red mug" correctly not found).
Still to check on the real robot (agent 1.1 changes were tested only with the simulator, not the fake Create 3):
1. The real undock motion, and that dock-from-retrace succeeds.
2. That `/cmd_vel` really is TwistStamped. Stop works without it anyway, by cancelling the action goal.
3. Lidar frame TF and the robot's own posts being ignored.
4. Battery and dock-status QoS.
5. Camera frames from oakd_rgbd (stock preview works).
6. Shortened moves, the in-motion lidar stop, `/range`, the camera geometry from TF, and a full Find run.
7. The stock-preview intrinsics (fx 323.5 at 250x250, from the 69° HFOV) — check the bearing error on a real Find.
