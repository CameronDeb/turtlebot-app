# Plan: travel router + real-robot search and verification runs (written Nov 22, 2026)

Prof. Lee's direction (Nov 22):
- Start real-robot search and verification now; fetching is an optional extension.
- IROS claim: the revisit policy improves success and reduces travel vs **age-only** and **reconstruction-quality**
  baselines.

The two parts below don't depend on each other. Part B runs on the agent and works on the hotspot today.

## Part A. Travel router (GL.iNet Opal GL-SFT1200)

Robot ROS settings, checked Nov 22 from `/etc/turtlebot4/setup.bash`:
- Jazzy, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=0`;
- no discovery server, so it uses ordinary multicast discovery;
- the Create 3 is reached through the Pi.

So the PC only needs to be on the same LAN with the same three settings. That is exactly what the Windows hotspot
blocked.
(Side note: `setup.bash` contains ~20 copies of the `ROS_SUPER_CLIENT` line. They are harmless and were probably
appended by repeated setup runs. Leave them.)

| Step | Who | What | Done when |
|---|---|---|---|
| A1 | Cameron | Pick up at ECE 256F (Mon Nov 23 afternoon, or from Tue). | Router in the lab |
| A2 | Cameron (passwords) | Open the router page at 192.168.8.1 and set an admin password. SSID e.g. `vsi-tb4`, WPA2; 5 GHz on. Check that AP/client isolation is off. | Phone joins it |
| A3 | Cameron + me | Choose how the PC connects (see question 1). DHCP reservations: robot `192.168.8.140`, PC `192.168.8.10`. | Both show fixed leases |
| A4 | Cameron (sudo) + me | Add the router to the robot's Wi-Fi **alongside** the hotspot in `/etc/netplan/50-wifis.yaml`, so it falls back to the hotspot. Apply with `sudo netplan try`, which reverts by itself after 120 s unless confirmed, so a mistake can't lock us out. Log it in session_log.md. | `ssh ubuntu@192.168.8.140` works |
| A5 | me | Change `ROBOT_AGENT_URL` in `.env` to `http://192.168.8.140:8765`. | App shows the robot; one Find runs |
| A6 | Cameron (admin) | Set WSL mirrored mode (`.wslconfig` `networkingMode=mirrored`). Run the Hyper-V firewall allow command for WSL. Mirrored mode should work now, because the router link is a real network adapter, not the hotspot. | `ping 192.168.8.140` from WSL |
| A7 | Cameron (sudo) + me | Install ROS 2 Jazzy (`ros-jazzy-ros-base` + `turtlebot4_msgs`) in WSL Ubuntu 24.04 with the same RMW and domain. | `ros2 topic list` shows `/scan`, `/odom`; `ros2 topic echo /odom --once` works |
| A8 | me | If multicast doesn't get through, run a Fast DDS discovery server on the PC and point the robot at it. This is the documented TB4 "discovery server" mode. | Same check as A7 |

After A7, Nav2 (with the depth camera in its costmap) runs on the PC, and the plant-behind-the-table test is its first
check. Undo for A4: delete the added netplan entry and run `sudo netplan apply`.

## Part B. Search and verification runs

**Task.** The robot keeps a memory of where objects were. Between sessions some objects are moved. Given a set of
queries, the robot must check (verify) or re-find the objects. The **policy** decides which remembered places to
check, and in what order, with the least driving.

| Step | What | Done when |
|---|---|---|
| B1 | **Object memory.** New `ObjectMemory` model: label, map x/y, last seen, number of views, viewing angles, detection score, and status (confirmed/missing). It is filled from Find and 360° scans. Positions are in the **slam map frame**, so they survive sessions; needs a `map_pose` from the agent (map→odom tf). | Memory survives an agent restart and the positions line up |
| B2 | **Verify command.** Drive to a viewpoint for a remembered object, look, and mark it confirmed, moved or missing. If moved, run Find from there. | Works in sim and on 2 real objects |
| B3 | **Policies, pluggable.** Age-only: oldest first. Quality-only: fewest views / lowest score first. Ours: reliability + staleness, where staleness is time since seen, changes found nearby, and the robot's own actions there. Each is weighed against travel. On the real robot, reliability v1 is a detection-based proxy (views, angles, score). The 3DGS reliability stays in the offline pipeline. | All three run on the same memory |
| B4 | **Logging.** One JSON per trial: policy, change script, queries, places checked in order, odometry travel, time, outcome. | Script makes a success/travel table |
| B5 | **Pilot in sim first.** Same scripts in `sim.py`, then tests. | Tests pass; sim shows the expected ordering |

**Protocol (real lab).**
- About 6 objects on taped spots, with about 8 possible spots.
- **Session 0:** the robot scans everything and builds the memory.
- **Each trial:** Cameron applies a pre-generated *change script* (move k = 0–2 objects). The robot starts at the
  dock and answers 3 queries.
- Every policy runs on the same change script, and objects are reset between policies.
- Size: 3 policies × 5 scripts × 3 queries = 45 queries, about 3 lab sessions.

**Design point to be honest about.** If every spot is equally likely to change, nothing can beat age-only. Ours
should win when change rates differ by area, as in a real lab: a desk area where people work changes often, a shelf
rarely. The change scripts will have that structure, and the paper must say so.

**Metrics.**
- success rate;
- travel distance;
- time;
- wasted checks: places visited where nothing had changed and nothing was asked.

## Rough schedule
- **Mon Nov 23:** router pickup (A1); B1 in sim.
- **Tue Nov 24:** A2–A5 with the robot docked; B1 on the robot.
- **Wed Nov 25:** A6–A7; B2.
- **Thu–Fri:** Thanksgiving.
- **Weekend / next week:** B3–B5, then the pilot real runs; full runs the week of Nov 30.

## Need from Cameron
1. Does the lab PC have an Ethernet port, or a USB-Ethernet adapter?
   - **If yes:** the PC uses Ethernet to the router and keeps university Wi-Fi for internet. This is simplest.
   - **If not:** the PC joins the router's Wi-Fi, and the router needs internet from a wall jack or a repeater.
2. Which objects and spots to use, and which area of the lab counts as "busy".
