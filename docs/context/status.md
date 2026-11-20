# Status (updated 2026-11-20)

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
1. **Real-robot test of agent 1.2 + route planning**: Find the plant behind the table again (should go round),
   check the stall guard by driving gently into a chair base, measure real stop gaps with a tape.
   Real Finds already worked on 2026-11-20 (chair, plant, febreeze bottle) with the straight-line approach.
2. **Use the lidar map**: build a fresh slam_toolbox map and plan on map + live scan (Nav2 global/local costmaps
   do exactly this). The Oct 28 map is likely stale where furniture moved (ties into the revisit paper).
3. Start our RGB-D camera node for depth (better distances, 640x360 pictures) and compare with the floor estimate.
4. **SLAM + Nav2** (both installed on the robot) so Find can plan around obstacles and remember object locations.
5. **LLM interpreter**: `COMMAND_INTERPRETER` class whose output is validated by `commands.validate` (it can now use
   `find`).
6. **SO-101 mount** (Prof. Lee's rules, arm_mount.md): measure the unknowns, test print, then PETG/ASA; then the
   arm/base interlock, footprint and lidar mask in the agent. **Arm decision: end of January 2027.** Leader arm
   servo-ID fix still pending (so101_arm.md).
7. **Paper** (decided 2026-11-18): one paper, main contribution = **revisit planning** (when and where to look again
   when the map may be out of date), fetching = the experiment. Draft reframed; next: staleness terms in
   `revisit_planner.py`, object memory, exact references for GS-LTS / DynaMem / DREAM / MIF (paper_ideas.md).

## Open questions
- What the real undock does: back up only, or back up and turn?
- Should the agent run as a boot service on the robot?
- Remap controller B away from EStop? (It's easy to hit by accident; changing it is a robot config change, ask first.)
- Enable `/hazard_detection` in the robot's republisher.yaml so the bumper reaches the agent? (robot config change)
- The `/e_stop` service answered `success=False` but the wheels came back on; watch whether Motors on works reliably.
