# Status (updated 2026-11-18)

## Done
- Web app v1, robot agent, simulator, fake Create 3, tests. Pushed to github.com/CameronDeb/turtlebot-app (`main`).
- 2026-11-18 (lab):
  - Agent installed on the robot (`~/robot_agent/`, token in `~/.turtlebot_agent_env`, started by hand with nohup,
    log `~/robot_agent/agent.log`; not a boot service). PC `.env` set to `ROBOT_BACKEND=agent`.
  - Found why nothing moved: the Create 3 **e-stop** was on (short press of the power button, or controller B/○).
  - Agent 1.1: moves shortened instead of refused, lidar watch while driving, e-stop status + Motors on, `/range`,
    camera intrinsics/geometry. App: **Find an object** (Grounding DINO in WSL), 5 m forward limit, detection boxes.
    51 tests pass (simulator).
  - Mount research for the SO-101 hanging in front: `arm_mount.md`.

## Next
1. **First real Find run** (Cameron watching, floor clear): undock → Find "<something visible>" with standoff 0.3 →
   check it stops in front → Return. Then check the items in app_design.md "Not verified on the real robot yet".
2. Start our RGB-D camera node for depth (better distances, 640x360 pictures) and compare with the floor estimate.
3. **SLAM + Nav2** (both installed on the robot) so Find can plan around obstacles and remember object locations.
4. **LLM interpreter**: `COMMAND_INTERPRETER` class whose output is validated by `commands.validate` (it can now use
   `find`).
5. **SO-101 mount**: measure the unknowns in arm_mount.md, mock up, print in PETG/ASA; then arm mask + new front edge
   in the agent. Leader arm servo-ID fix still pending (so101_arm.md).
6. **Paper**: "find, fetch, remember" draft; pick the main novelty (paper_ideas.md).

## Open questions
- What the real undock does: back up only, or back up and turn?
- Should the agent run as a boot service on the robot?
- Remap controller B away from EStop? (It's easy to hit by accident; changing it is a robot config change, ask first.)
- The `/e_stop` service answered `success=False` but the wheels came back on; watch whether Motors on works reliably.
