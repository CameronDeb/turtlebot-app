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
