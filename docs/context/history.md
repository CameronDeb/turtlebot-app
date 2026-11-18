# History

## 2026-11-16: web app v1 built (Claude, lab PC)
- Request (Cameron): a Django web app with buttons for forward, turn and take picture; a local photo archive; and a
  return-to-dock that undoes the moves in reverse. It should be ready for an LLM layer later, with the end goal of
  scanning a room for a named object, processing images in Django and sending commands back. Push to
  github.com/CameronDeb/turtlebot-app.
- Built:
  - Django 6.0 app `control`.
  - Robot agent (`robot_agent/agent.py`).
  - Simulator.
  - Fake Create 3 for ROS-level tests.
  - 38 Django tests plus 1 end-to-end test (Windows Django to WSL agent to fake Create 3).
- Found and fixed along the way:
  - rclpy swallowed SIGTERM: the agent now installs its own signal handlers.
  - Media serving ignored the test settings: it now reads MEDIA_ROOT per request.
  - Refused moves created empty trips: trips now start only after real motion.
  - Browser CSS caching: added mtime cache-busting.
  - The PC clock is set to Pacific time, so the UI uses the server's time zone (Arizona).
- The robot was unreachable (hotspot off or robot away), so nothing has been run on the real TurtleBot yet.
- Also noted: Cameron's SO-101 arm setup on his laptop (see so101_arm.md). The arm will be mounted on the TurtleBot
  after the LLM object-finding works.
- Cameron asked for this context folder to be kept up to date every session.

## 2026-11-17: weekly report, paper direction, arm mount (Claude, lab PC)
- Weekly report draft for Nov 16–22 at `turtlebot-gs/notes/weekly_report_2026-11-22.md`, with a 6-slide deck at
  `turtlebot-gs/reports/weekly_2026-11-22.pptx`.
- Paper direction "find, fetch, remember": rough LaTeX draft at `turtlebot-gs/paper/vsi_3dgs_draft.tex`; novelty notes
  in paper_ideas.md.
- Arm: Cameron is working on a 3D-printed mount for the SO-101 follower on the TurtleBot.
- Next lab visit: first real-robot run of the web app (Cameron agreed).

## 2026-11-18: first time on the real robot, e-stop, object search, mount research (Claude + Cameron, lab)
- Pushed the 2026-11-17 context notes (b8d23fc).
- Robot online: installed the agent and token (logged in turtlebot-gs/notes/session_log.md). Status and photos work
  through the app.
- A 0.25 m forward was accepted but never moved. Cause: `/wheel_status.wheels_enabled = false`, i.e. the Create 3
  e-stop was on (power button short press or controller B toggles it). Cleared with `/e_stop {e_stop_on: false}`.
  The robot's power saver also stops the camera and lidar while docked.
- Cameron asked for: GUI driving that works, autonomous "go to a named object", no over-restrictive safety stop, and
  research for an SO-101 mount hanging in front for floor pickups.
- Built: agent 1.1 (shortened moves, lidar watch while driving, motors/e-stop, /range, camera geometry), Find in the
  app (Grounding DINO server in WSL, bearing/distance from the picture, approach in ≤1 m legs), UI Find card, e-stop
  banner, detection boxes, 13 new tests (51 total, all pass on the simulator). Detector checked on a real robot photo.
- Mount research written up in arm_mount.md (TurtleBot 4 plate/standoff CAD, SO-101 base and reach from its URDF).
- Bug caught by the new tests: a box in front of the object was taken as the object's distance (lidar), so the
  robot "arrived" at the box. Fixed: lidar only tightens the camera estimate when they agree within 0.3 m.
