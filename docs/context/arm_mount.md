# SO-101 mount on the TurtleBot 4 (low, front corner, removable, for floor pickups)

Research notes 2026-11-18, revised the same day after Prof. Lee's requirements. Numbers marked **(CAD)** come from the
official CAD (TurtleBot 4: github.com/turtlebot/turtlebot4-hardware; SO-101: github.com/TheRobotStudio/SO-ARM100).
**(robot)** = read from our robot's TF. **(measure)** = check with calipers/tape before printing.

## Requirements (Prof. Lee, 2026-11-18)
1. Read the TurtleBot 4 payload guide first: https://turtlebot.github.io/turtlebot4-user-manual/mechanical/payloads.html
2. Mount the arm **low**.
3. Use the robot's **existing M4 screw holes** (no drilling, no clamps).
4. **Don't block the camera.**
5. The arm gets **its own 12 V battery, at least 5 A**.
6. **Only move the arm while the robot is stopped.**
7. **Removable with a few bolts, and goes back in exactly the same spot.**

Also: the arm is only in the paper if it's reliable by the end of January 2027 (paper_ideas.md).

## What the payload guide says (applies here)
- Mount the payload so its **centre of gravity is supported by the wheelbase**; a COG further forward is fine if the
  front caster carries it (their example: an 8.9 kg NED2 arm placed forward over the front caster).
- **Lower is more stable**, especially when the robot rocks onto three wheels.
- The robot's threaded inserts are **M4x0.7**. Take the hole pattern from the CAD or from the plate itself.
- Torque fasteners to suit the thread and **use threadlocker** when reassembling (top plate: Loctite 222, 25 N·cm).
- Heavy payloads may need lower speed/acceleration (`wheel_accel_limit` on `motion_control`, 1–900). We drive at
  0.15 m/s, well under the 0.31 m/s limit.
- Their example powered the arm from a separate external 12 V battery.
- If the top plate is lowered on shorter standoffs, keep at least **108 mm** clearance for the OAK-D Pro.

## TurtleBot 4 Standard: the existing M4 points
| Item | Value |
|---|---|
| Overall | 342 x 339 x 351 mm, 3.95 kg, 9 kg rated payload, ground clearance 4.5 mm |
| Top plate (CAD 025394) | Ø270 mm circle with one flat edge (250 mm across the flat), 4 mm acrylic. 4x Ø4.5 holes: a pair **229 mm** apart near the flat edge and a pair **181 mm** apart, **107 mm** between the pairs |
| Standoffs (CAD 025481) | 4x Ø10 mm round aluminium, **200 mm**, M4 **male** bottom (8 mm thread) into the robot's M4 inserts at the PCBA level (~0.15 m up), M4 **female** top (12 mm deep) for the top-plate screws |
| Lidar | scan plane 0.193 m up **(robot)** |
| Camera (OAK-D Pro) | 0.244 m up, level **(robot)**. Horizontal FOV: ±32° for our 640x360 RGB-D node, ±21° for the stock preview; vertical ±19–21° |
| Front bumper | 0.17 m ahead of base_link |
| Robot power for accessories | 12 V @ 300 mA only: not usable for the arm |

So each standoff gives two existing M4 joints: **bottom** (standoff into the insert, ~0.15 m up: low) and **top**
(top-plate screw into the standoff, ~0.35 m up).

## SO-101 follower
| Item | Value |
|---|---|
| Mass | ~0.63 kg (URDF) / 0.7–0.8 kg as built |
| Base (CAD Base_SO101) | 111 mm wide x 87 mm deep x 72 mm tall; designed to be clamped by its two side tabs |
| Shoulder | pan axis 62 mm, shoulder-lift axis ~117 mm above the base bottom |
| Links (URDF) | upper arm 116 mm, forearm 135 mm, wrist to grasp point ~160 mm |
| Power | 12 V, at least 5 A peak (a 2 A brick made servos drop out) |

**Why low:** to pick from the floor with the gripper pointing down, the shoulder-lift axis has to be at most
~0.23–0.25 m up. On the top plate it would be at ~0.47 m (can't reach the floor). With the base bottom at
**0.08–0.12 m**, the shoulder is at 0.20–0.24 m and floor picks within ~0.10–0.20 m of the shoulder work.

## Design

### Two parts
1. **Robot bracket (stays on the robot).** A printed U-frame that spans the two front standoffs, held only by the
   existing M4 joints:
   - **Bottom tabs** (4 mm thick, Ø4.5 hole) at each front standoff's bottom joint. Stack, from the robot up:
     robot M4 insert ← **M4 male-female aluminium spacer, 10 mm body** (male thread ≥ 6 mm into the insert) ←
     bracket tab ← stock standoff (its 8 mm male thread goes through the 4 mm tab and 4 mm into the spacer, at least
     one screw diameter). Put the same spacer plus a 4 mm printed washer under the two **rear** standoffs too, so the
     top plate rises ~14 mm evenly instead of tilting. Threadlocker on every joint.
   - **Top tabs** sit under the top plate at the two front screws; use screws 4–5 mm longer than the stock ones (check
     the stock length), threadlocker, 25 N·cm.
   - Bottom + top joints on both posts make a stiff box with almost no bending load on the plate or the inserts.
   - From the bottom tabs a short **shelf** reaches forward and down to the front corner (see below), ending in a flat
     **docking face** with 2 dowel holes and 2–4 M4 heat-set inserts.
2. **Arm plate (comes off with the arm).** The SO-101 base is held on this plate permanently (printed copy of the
   table-clamp geometry, or a modified `Base_SO101.step` with a bolt flange; LeKiwi uses 4x M3x20). The plate
   carries **2 steel dowel pins** (Ø4 mm, pressed in) and **2–4 M4 bolts**.
   - **Repeatable location:** one dowel goes into a round hole and the other into a short slot along the line between
     them, so the plate can't rock or bind and goes back to the same spot every time (≈0.1 mm). The bolts only clamp.
   - Removing the arm: unplug the battery (XT60) and USB, undo the 2–4 bolts, lift off. The arm's calibration and the
     arm-to-robot transform stay valid, so measure the transform once.

### Where the arm sits: front corner, outside the camera's view
A centred arm in front would sit in the camera's lower field of view (the parked shoulder is ~0.2 m up, right where
the camera looks at the floor 0.2–0.4 m ahead). So the arm goes at a **front corner**:
- Base centre about **0.19 m to the side** of the centre line and **~0.15 m ahead** of base_link (roughly at the
  bumper line), base bottom at **0.08–0.12 m**.
- Its inner edge (~0.135 m from the centre line) is then just outside the RGB-D camera's ±32° cone at that distance
  (half-width ~0.125 m). The stock preview's ±21° cone is narrower still.
- Park pose for driving: folded back and outward, staying outside the camera cone.
- Pickups happen to that front corner. Find's approach can aim to stop with the object on the arm's side (a bearing
  offset) instead of dead ahead.
- **Check before printing:** where the OAK-D really sits (our TF puts it 0.06 m *behind* base_link, which looks
  odd), and confirm the cone with the live camera image while holding the arm in place.

### Material and printing
- PETG or ASA (PLA creeps under constant load), 4–5 walls, 40%+ gyroid infill, ribs on the shelf.
- Heat-set M4 inserts in the docking face; holes for the dowels reamed to size after printing.

### Battery (separate, 12 V, at least 5 A)
- A 12 V pack rated at least 5 A continuous, with an inline fuse (7.5 A) and a switch, XT60 to the Waveshare servo
  board's barrel jack. Don't exceed the servos' maximum input voltage: check the STS3215 12 V spec before choosing a
  3S pack (12.6 V full).
- Mount it **low, on the opposite rear side** from the arm, to balance it (Velcro + a printed cradle on the bracket or
  the Create 3 plate area). The servo board's USB goes to the Pi (data only).

### Weight check
Robot 3.95 kg at the centre; arm + plate + bracket ~1.1 kg at about (0.15 m ahead, 0.19 m to the side); battery
~0.5 kg at about (0.10 m behind, 0.10 m to the other side). Combined COG ≈ 2 cm ahead and 3 cm to the arm's side: well
inside the wheels (half-track ~0.12 m) and behind the front caster. Fine at 0.15 m/s; lower the acceleration if it nods
when stopping.

## Software rules that come with the arm
- **The arm only moves while the robot is stopped** (Prof. Lee). Planned interlock:
  - Arm motions become commands in the same executor, which already runs one command at a time, so an arm motion
    can never overlap a drive or turn.
  - The robot agent refuses drive/rotate unless the arm reports it is parked, and the arm side refuses to move while
    a base motion is active or the wheels are turning.
  - STOP stops both.
- **New footprint:** the arm corner sticks out past the round body, so the lidar corridor check needs a wider half
  width on that side and the front edge (`BODY_FRONT`) moves out. The bumper won't catch hits on the arm, so the lidar
  check matters more.
- **Lidar mask:** the arm crosses the 0.193 m lidar plane in a narrow sector at that corner; add a box in base_link that
  the obstacle check ignores, next to the existing body filter.
- **Docking:** the Create 3 drives forward onto the dock. Check that the arm corner clears the dock (height and width);
  if not, remove the arm (it's quick now) or park it so it clears.
- **Final grasp:** the robot's camera can't see the floor closer than ~0.6 m, so use the SO-101 wrist camera
  (TheRobotStudio's 32x32 UVC wrist mount) for the last part.

## Next steps
1. Measure: the flat-edge direction of the top plate (front or back), the stock top-plate screw length, the real
   OAK-D position, the bumper and Create 3 heights, the dock size, the standoff positions relative to the bumper.
2. Choose the arm side (left or right) so the cables and battery fit on the opposite side.
3. Quick PLA test print of the bracket and arm plate: check the bumper moves freely, the camera image is clear, the
   lidar's view, the dock, the floor reach, and that the arm goes back to the same spot after removing it.
4. Final print in PETG/ASA; then the interlock, footprint and lidar mask in the software.
