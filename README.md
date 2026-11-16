# turtlebot-app

A local web app for driving a TurtleBot 4 from a browser. It offers buttons for **forward**, **turn**, **take picture**,
**360° scan**, **undock / dock**, and **return to start / dock**. It also has a **photo archive** stored on the PC and
a text box for short typed commands, which is the hook for a future LLM layer.

```
 Browser ──► Django app (lab PC, Windows) ──HTTP + token──► robot_agent/agent.py (TurtleBot's Pi) ──ROS 2──► Create 3
             - commands, job runner, trip log                - drive_distance / rotate_angle / dock / undock actions
             - photo archive (archive/ + SQLite)             - lidar clear-ahead check, camera frames, pose from TF
```

The Django app is the brain: it decides what to do, keeps the history and pictures, and will later run image
processing and an LLM planner. The robot only runs a small agent that executes one primitive at a time and reports
what really happened.

## What it does

- **Moves**:
  - Forward 0.1 to 2 m, at 0.15 m/s.
  - Turn left or right by 15 to 180°.
  - Before a forward move, the robot checks the corridor ahead with the lidar and refuses if something is in the way.
    Untick *Obstacle check* to skip it. The lidar stops while the robot is docked, so undock first.
- **STOP** cancels the running motion immediately, even part-way through.
- **Take picture** saves a full-size PNG, a thumbnail, and an aligned 16-bit depth PNG when the RGB-D camera node is
  running. Pictures go into `archive/photos/YYYY/MM/DD/`, with the robot's position and heading at that moment.
- **Scan 360°** takes 8 pictures, 45° apart, and ends facing the way it started.
- **Return to start / dock**: every move records the robot's measured pose (a *trip*). Return retraces those
  waypoints in reverse order.
  - At each step it turns to face the previous waypoint and drives to it, so it always drives forward, where the
    sensors point.
  - If the trip began with **Undock**, it ends by facing the dock and docking. Otherwise it stops at the start, facing
    the original direction.
  - It can be stopped and resumed at any point, and moves made in between are retraced too.
- **Photo archive** (`/archive/`): browse by date, trip, or 360° scans only. Each photo can have a note, and the
  original image and depth file can be downloaded.
- **Typed commands**, for example `undock, forward 1 m, turn left 90 and take a picture`. *Check* shows how the text
  was understood; *Run* runs the whole sequence.
- **Admin** (`/admin/`): raw trips, waypoints, jobs and photos. Create a login with
  `python manage.py createsuperuser`.

## Set up the PC (Windows)

```powershell
git clone https://github.com/CameronDeb/turtlebot-app
cd turtlebot-app
python -m venv .venv
.\.venv\Scripts\pip install -r requirements.txt
copy .env.example .env          # then edit .env
.\.venv\Scripts\python manage.py migrate
.\.venv\Scripts\python manage.py runserver
```

Open http://127.0.0.1:8000/. With `ROBOT_BACKEND=sim` (the default), a built-in simulator stands in for the robot, so
everything can be tried without hardware.

Run the app on **Windows**, not WSL, because WSL's NAT network can't reach the robot on the PC hotspot.

## Set up the robot (once)

The agent needs only what the stock TurtleBot 4 image already has (rclpy, tf2_ros, irobot_create_msgs, numpy).

```bash
# from the PC: copy the agent to the robot
scp -r robot_agent ubuntu@192.168.137.140:~/turtlebot-app/
# on the robot: choose a secret token (same value goes into ROBOT_AGENT_TOKEN in the PC's .env)
echo "AGENT_TOKEN=$(python3 -c 'import secrets; print(secrets.token_urlsafe(24))')" > ~/.turtlebot_agent_env
chmod 600 ~/.turtlebot_agent_env && cat ~/.turtlebot_agent_env
chmod +x ~/turtlebot-app/robot_agent/run_agent.sh
~/turtlebot-app/robot_agent/run_agent.sh          # listens on port 8765; Ctrl+C stops it
```

Then set `ROBOT_BACKEND=agent`, `ROBOT_AGENT_URL=http://192.168.137.140:8765` and `ROBOT_AGENT_TOKEN=...` in `.env`,
and restart the Django server. To start the agent at boot, see `robot_agent/turtlebot-agent.service`.

**Camera:**
- The stock bringup's 250×250 preview works.
- For 640×360 pictures with depth, start the RGB-D node from the research repo (`tools/oakd_rgbd_start.sh`, IR
  projector off).

**Safety:**
- A person watches the robot and keeps the floor clear.
- Keep STOP in reach.
- The token stops other hotspot users from driving the robot. Keep it secret.

## Tests

```powershell
.\.venv\Scripts\python manage.py test control
```

The tests cover command checks, the text parser, return-path geometry, the job runner (including stop part-way and
resuming a return), the archive and the API. They run against the simulator.

`control/tests/test_agent_e2e.py` drives the real agent code against `robot_agent/dev/fake_create3.py`, a fake Create 3
with the same ROS actions, TF, lidar and camera topics. You need ROS 2 Jazzy plus `irobot_create_msgs` (Linux or WSL).
The fake robot's docstring has the details. Never point that test at the real robot.

## Layout

| Path | What |
|---|---|
| `control/commands.py` | The command vocabulary and limits: the one gate every command passes |
| `control/executor.py` | Runs commands one at a time in a background thread; records trips; stop |
| `control/planner.py` | Return-path geometry |
| `control/interpreter.py` | Typed text to commands (rule-based now; swap in an LLM via `COMMAND_INTERPRETER`) |
| `control/robot/` | Robot clients: `http.py` (real agent), `sim.py` (simulator) |
| `control/archive.py` | Saving pictures into the archive |
| `robot_agent/agent.py` | The HTTP to ROS 2 bridge that runs on the robot |
| `docs/context/` | Running project notes: status, history, design decisions |

## Next steps (planned)

1. **LLM layer**: a class with `interpret(text) -> [{"kind", "params"}]`, selected by `COMMAND_INTERPRETER`. Its output
   goes through the same `commands.validate()`, so it can't do anything the buttons can't.
2. **Object search**:
   - Scan the room (360° scan), run an open-vocabulary detector (e.g. Grounding DINO) on the archived pictures on
     the PC, and store the results in `Photo.detections`.
   - Use depth plus the robot's pose to locate the object, then send `turn` and `forward` commands back to the robot.
3. **Arm**: mount the SO-101 follower on the TurtleBot and add a `pick` command (see `docs/context/so101_arm.md`).
