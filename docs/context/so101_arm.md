# SO-101 arms

Set up by Cameron on his **laptop** (not the lab PC). The source of truth is `C:\Users\camer\projects\so101_setup.md` on
that laptop, with the helper scripts `so101_follower_gui.py`, `so101_poses.json`, `so101_calibrate.py`,
`so101_ping.py` and `so101_wiggle_test.py`. Status as reported on 2026-11-16:

- Laptop env: Miniconda `C:\Users\camer\miniconda3`, conda env `lerobot` (Python 3.10, lerobot 0.4.4 + feetech).
- **Follower** (COM4, Waveshare adapter serial 5A7A058581, 12 V brick, **at least 5 A**): working. All 6 servos at
  ~12.5 V, calibrated (`follower_arm`), driven from a slider GUI, and a "pickup" pose is saved.
- **Leader** (COM3, adapter 5A7C118817, **5 V brick**): not done. IDs 1, 4, 5, 6 respond. ID 2 never replies, and ID 3
  gives garbled replies (probably motors 2 and 3 both set to ID 3).
  - Next: `lerobot-setup-motors --teleop.type=so101_leader --teleop.port=COM3`.
  - Then re-ping and calibrate with `so101_calibrate.py --teleop.id=leader_arm`.
  - Then test `lerobot-teleoperate`.
- Lessons:
  - lerobot gives up on one dropped serial reply, so use the retrying calibrate wrapper.
  - `max_relative_target` stalls joints under gravity; the GUI ramps its own target instead.
  - A 2 A brick made follower motors drop out when torque turned on.
  - Calibration middle pose: each joint halfway between its end stops.
- Safety:
  - Confirm the power before powering motors: follower on 12 V, leader on 5 V. 12 V damages the leader.
  - Ask before anything that moves an arm.
  - Run interactive commands in a terminal Cameron can see.

## Plan
- 2026-11-17: Cameron started a 3D-printed mount for the follower on the TurtleBot's top plate.
- 2026-11-18: mount research in arm_mount.md. The arm can't reach the floor from the top plate; it has to hang low
  in front (base bottom 0.10–0.15 m up), clamped to the two front Ø10 mm standoffs.
Once the web app's LLM layer can find objects and drive to them, **the follower arm gets mounted on the TurtleBot** for
pickup.
- Power: give the arm its own battery, not the TurtleBot's, because the robot's USB hub already browned out once.
- Mounting and payload need checking.
- Control: LeKiwi-style split. A small host runs on the robot's Pi (or a separate SBC) and the policy runs on the lab
  PC (see `turtlebot-gs/notes/ideas_dynamic_and_arm.md`).

The web app keeps the arm in mind: robot commands are generic "skills" run by one executor, so a future `pick` skill
can call an arm service the same way `photo` calls the camera.
