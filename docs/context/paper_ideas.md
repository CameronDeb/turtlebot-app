# Paper direction (decided by Prof. Lee, 2026-11-18)

Rough draft: `turtlebot-gs/paper/vsi_3dgs_draft.tex` on the lab PC (Cameron pastes it into Overleaf, under the title
"VSI Publication 3DGS"). Reframed on 2026-11-18 to match the decision below.

## Decision
- **One paper, not two.** Two papers by this deadline (IROS 2027, March 1, 2027) isn't realistic.
- Find + fetch + a 3D Gaussian memory is **not new on its own**: GS-LTS, DynaMem, DREAM and MIF already combine them
  (Prof. Lee's list; get the exact references).
- **Main contribution: revisit planning**, i.e. how the robot decides *when and where to look again* when its map
  might be out of date.
- **Fetching is the experiment** that shows it works.
- **Arm deadline: end of January 2027.** If the arm isn't reliable by then, submit without it (fetch becomes
  "reach": arrive at the pre-grasp pose and confirm the object is there).

## Working title
"When to Look Again: Revisit Planning for a Gaussian Splatting Map Memory That Goes Out of Date"

## Method sketch
- Existing baseline (`turtlebot-gs/tools/revisit_planner.py`): per floor cell, a reliability score from view count,
  viewing-direction spread and photometric residual, plus proposed revisit poses.
- New: **staleness** terms: time since the cell/object was last seen, change evidence nearby (render vs new image on a
  revisit), and the robot's own manipulation there.
- Decision rule: before acting on a remembered object, go straight there or re-observe first, by comparing the
  expected cost of a wasted trip with the cost of the revisit. Revisit poses chosen greedily by expected freshness
  gain per metre.

## Experiments
| | Question | Measure |
|---|---|---|
| E1 | Does the score find the parts of the map that are wrong? | Correlation with true map error (Isaac Sim, real room) |
| E2 | Does revisit planning make fetching better in a changing room? | Success, time, distance, wasted trips |
| E3 | How fresh does the map stay? | Stale objects corrected per metre; change precision/recall |
| E4 | (only if the arm is ready) Full fetch | Pick success |

Baselines: trust memory (never revisit), always search, reliability-only revisits (the existing planner).
Changes: move k objects between requests.

## Already have
- Depth correction cuts trajectory error from 16.5 to 2.4 cm (5 runs each, p ≈ 0.008).
- Detector distances match the lidar map to about 0.1 m.
- Reliability-only revisit planner (baseline) on MonoGS maps.
- Web app "find" on the robot (approach a named object), which the fetch experiment can build on.

## Earlier options (2026-11-17), now settled
The earlier list (A self-supervised change labels, B staleness-aware revisits, C room-specific detector, D
reliability-aware localisation) recommended B with A for evaluation. Prof. Lee's decision matches B: revisit planning is
the main claim, and the robot's own picks (A) can still serve as free ground truth for change detection.

## Open
- Get the exact GS-LTS, DynaMem, DREAM and MIF references and say precisely what each does when the scene changes.
- Revisit-score weights: hand-set, fitted, or learned?
