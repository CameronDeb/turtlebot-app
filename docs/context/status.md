# Status (updated 2026-11-17)

## Done
- Web app v1, robot agent, simulator, fake Create 3, tests. Pushed to github.com/CameronDeb/turtlebot-app (`main`).

## Next
1. **First run on the real robot** (planned for the next lab visit) (Cameron in the lab, floor clear, someone watching):
   1. Copy `robot_agent/` to the Pi, create `~/.turtlebot_agent_env`, start `run_agent.sh`.
   2. Set `.env` on the PC: `ROBOT_BACKEND=agent` and the token.
   3. Order: status page → photo → undock → short forward (0.25 m) → turn → return (dock).
   4. Check the items in app_design.md under "Not verified on the real robot yet".
   5. Log the robot-side install in `turtlebot-gs/notes/session_log.md`.
2. **LLM interpreter**: `COMMAND_INTERPRETER` class whose output is validated by `commands.validate`.
3. **Object search**:
   - Grounding DINO on scan photos. A detector already exists in `turtlebot-gs/tools/detect_objects.py`, using the
     WSL env `detect`.
   - Fill `Photo.detections`, locate the object with depth and pose, then drive to it.
4. **SO-101 on the TurtleBot**: 3D-printed mount in progress (Cameron). Leader arm servo-ID fix still pending (so101_arm.md).
5. **Paper**: "find, fetch, remember" draft; pick the main novelty (paper_ideas.md) and ask Prof. Lee whether to merge it with the revisit study.

## Open questions
- What the real undock does: back up only, or back up and turn?
- Should the agent run as a boot service on the robot?
