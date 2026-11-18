# SO-101 mount on the TurtleBot 4 (front, hanging low, for floor pickups)

Research notes, 2026-11-18. Numbers marked **(CAD)** come from the official CAD (TurtleBot 4:
github.com/turtlebot/turtlebot4-hardware, SO-101: github.com/TheRobotStudio/SO-ARM100). Numbers marked **(robot)**
were read from our robot's TF. **(measure)** = check with calipers/tape before printing.

## TurtleBot 4 Standard: what we can bolt to

| Item | Value |
|---|---|
| Overall size | 342 x 339 x 351 mm (L x W x H), 3.95 kg, ground clearance 4.5 mm, wheels Ø72 |
| Payload | 9 kg rated (15 kg custom). Keep the combined centre of gravity inside the wheelbase (front caster takes forward load) |
| Top plate (CAD 025394) | Ø270 mm circle with one flat edge (250 mm across the flat), **4 mm acrylic** (manual says HDPE on some units). Fragile: take it off to drill, start at Ø3 and step up |
| Top plate holes (CAD) | 4x Ø4.5 thru for M4. One pair **229 mm apart** near the flat edge, the other pair **181 mm apart**, the two pairs **107 mm** apart (front-to-back). Plate centre to the 229 pair: 30.6 mm; to the 181 pair: 76.1 mm |
| Screws | Round-head M4x0.7, 2.5 mm hex; refit with Loctite 222, 25 N·cm |
| Standoffs (CAD 025481) | 4x **Ø10 mm round aluminium, 200 mm long**, M4 male bottom / M4 female top (12 deep). They run from the PCBA level (~150 mm up) to the top plate (~350 mm) |
| Lidar (RPLIDAR A1) | scan plane **0.193 m** up, centre 0.04 m behind base_link **(robot)** |
| Camera (OAK-D Pro) | **0.244 m** up, level, 0.06 m behind base_link in our TF **(robot)**. Stock preview: ~42° horizontal FOV, sees the floor only beyond ~0.6 m |
| Front bumper | 0.17 m ahead of base_link (used by the app) |
| User power | Not enough for the arm: 12 V @ 300 mA, VBAT @ 300 mA, 5 V @ 500 mA; whole robot accessory budget ~14 W. USB-C ports share 3 A |

Unknowns (measure): which side the flat edge faces on our robot (front or back); heights of the Create 3 top
surface and the bumper top; the dock's height and how far it sticks out; where the two front standoffs sit relative to
the bumper.

## SO-101 follower: what has to be held

| Item | Value |
|---|---|
| Arm mass | ~0.63 kg (URDF links) / 0.7–0.8 kg printed with motors and cables |
| Base footprint (CAD Base_SO101) | 111 mm wide x 87 mm deep, 72 mm tall. It is designed to be clamped to a table by its two side tabs, not bolted |
| Shoulder | pan axis 62 mm above the base bottom; shoulder-lift axis ~117 mm above the base bottom |
| Links (URDF) | upper arm 116 mm, forearm 135 mm, wrist to grasp point ~160 mm. Max reach from the shoulder-lift axis ~0.41 m (straight out) |
| Payload | a few hundred grams near the body (servo torque 30 kg·cm at 12 V), much less fully stretched |
| Power | 12 V, **at least 5 A** peak (our 2 A brick made servos drop out) |

## Why it has to hang low in front

To pick from the floor with the gripper pointing down, the wrist has to get down to ~0.16 m above the floor. The
upper arm plus forearm reach is 0.25 m, so the shoulder-lift axis can be at most about 0.23–0.25 m above the floor
for a pick 0.10–0.20 m in front of it.

- **On the top plate** the base would sit at ~0.355 m, putting the shoulder at ~0.47 m: it **cannot reach the floor**.
- **Base bottom at 0.10–0.15 m** puts the shoulder at 0.22–0.27 m: floor picks from ~0.10 to ~0.22 m in front of the
  shoulder work, and it can still lift things up to the top plate.

So the base needs to sit in front of the bumper, at roughly bumper height or a little above.

## Recommended design: clamp bracket on the two front standoffs

```
   side view (front to the right)        top plate (~0.35 m)
   ==================================== ■━━━━━━━━━━━━┓  ← top clamp / bolt through the front M4 holes
                                        ║            ┃
             robot body                 ║ Ø10 standoff ┃ printed upright bracket (PETG/ASA, ribs)
                                        ║            ┃
   lidar (0.193 m) ◉ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─║─ ─ ─ ─ ─ ─ ┃─ ─ ─  ← arm crosses the lidar plane here
                                        ║  ┏━━━━━━━━━┛
   PCBA (~0.15 m) ──────────────────────■━━┫ shelf, base bottom at 0.10–0.15 m
   Create 3 + bumper                        ┗━ SO-101 base (bolted or clamped like on a table)
   ═══════════════════════════════════════════════════════════════ floor
```

1. **Two anchor points**, not a long cantilever: bolt the top of the bracket through the top plate's front hole pair
   (or replace those two M4 screws with longer ones), and clamp a split collar around each of the two front Ø10 mm
   standoffs near their bottom. This makes a stiff frame from 0.15 m to 0.35 m with almost no bending load on
   the plate.
2. **Shelf** sticking forward from the bottom collar, just above the bumper but not touching it. The bumper must still
   move freely, or the robot will think it is always in a collision.
3. **Hold the arm base** with the same two side tabs the table clamps use (print a copy of the clamp geometry into the
   shelf), or print a modified `Base_SO101.step` with a bolt flange added. LeKiwi bolts its SO-101 base with 4x M3x20,
   so a flanged base with M3 heat-set inserts is a proven approach.
4. **Material**: PETG or ASA (PLA creeps under constant load and gets soft in a warm room/car), 4–5 walls, 40%+
   gyroid infill, ribs on the vertical part. Or use a short 2020 aluminium extrusion as the upright with printed end
   brackets, which is much stiffer.
5. **Battery for the arm**: separate 3S Li-ion/LiPo (11.1 V nominal, 12.6 V full) or 12 V LiFePO4, rated at least 5 A,
   with an inline fuse (7.5 A) and a switch, into the Waveshare servo board's barrel jack. Mount it low and toward the
   back so it balances the arm. The servo board's USB goes to the Pi (data only).
6. **Weight check**: robot 3.95 kg at the centre plus arm + bracket + battery (~1.3 kg, arm ~0.25 m ahead). The combined
   centre of gravity moves ~6 cm forward, still behind the front caster. Fine at our speeds (0.15 m/s), but turn
   down acceleration if it nods when stopping.

## Things the mount changes (software and docking)

- **Docking**: the Create 3 drives forward onto the dock. Anything hanging in front may hit the dock. Measure the
  dock's height. If the shelf's lowest point can't clear it, park the arm raised and give the dock a clear lane, or
  only dock without the arm mounted.
- **Bumper no longer the front**: the shelf and arm stick out further than the bumper. In the agent, `BODY_FRONT`
  (0.17 m) has to become the new front edge, and the bumper won't catch collisions with the arm, so the lidar
  check matters more.
- **Lidar**: the arm's shoulder crosses the 0.193 m lidar plane, so the lidar will see the arm in a narrow sector in
  front (~15–20°). The agent's obstacle check would see that as a permanent obstacle. Add an arm mask (a box in
  base_link that is ignored) next to the existing body filter.
- **Camera**: the OAK-D only sees the floor beyond ~0.6 m with the stock preview, so the last part of a pickup is
  blind. Plan on the SO-101 wrist camera (TheRobotStudio has a 32x32 UVC wrist mount) for the final grasp.

## Next steps

1. Measure the unknowns above (flat-edge direction, bumper/Create 3 heights, dock height, standoff positions).
2. Mock up the bracket in cardboard or a quick PLA test print and check: the bumper moves freely, the lidar's view,
   the dock, and that the arm reaches the floor 10–20 cm in front of the shelf.
3. Print the real bracket in PETG/ASA, then add the arm mask and new front-edge settings to the agent.
