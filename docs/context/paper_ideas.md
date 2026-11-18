# Paper direction: find, fetch, and remember (2026-11-17)

Rough draft: `turtlebot-gs/paper/vsi_3dgs_draft.tex` on the lab PC (Cameron pastes it into Overleaf, under the title
"VSI Publication 3DGS").

## The loop
1. **Find**: the user names an object (later free text through an LLM). An open-vocabulary detector runs on the robot's
   images, and depth plus pose place the object in the map.
2. **Fetch**: drive to a pre-grasp spot about 0.8 m in front of the object, then the SO-101 does the task.
3. **Remember**: the 3DGS map keeps labelled objects, so a repeat request skips the search.
4. **Spot changes**: re-observe after the task or on the next visit, to see what moved, appeared or disappeared.

Planned experiments:

| | Question | Measure |
|---|---|---|
| E1 | How accurately are objects located? | Position error |
| E2 | Does map memory help repeat requests? | Time/distance with vs without memory |
| E3 | Does the task succeed? | Pick success rate |
| E4 | Does change detection work? | Precision/recall for moved, removed and added objects |

Already have:
- Depth correction cuts trajectory error from 16.5 to 2.4 cm (5 runs each, p ≈ 0.008).
- Detector distances match the lidar map to about 0.1 m.

## Novelty: it has to be unique
"Find, fetch, remember" as a system alone is close to existing work. Known neighbours (from memory, verify in the lit
review):
- OK-Robot: open-vocabulary pick and drop in homes.
- GOAT: go to any thing.
- DynaMem: a dynamic spatio-semantic memory for mobile manipulation in changing scenes.
- ConceptGraphs and HOV-SG: open-vocabulary 3D scene graphs.

Candidate angles that would make it ours. Pick one as the main claim:
- **A. The robot's own actions label the changes (self-supervised change detection).** The robot knows what it moved,
  so every pick gives a free ground-truth "this changed" label in its Gaussian map. Use these to train or calibrate
  change detection, and report how accuracy grows with experience.
- **B. When to trust memory vs look again (staleness-aware revisits).** Each remembered object has a confidence that
  decays, or drops when nearby changes are seen. The robot decides whether to go straight to the remembered spot or
  re-scan first. This links directly to Prof. Lee's revisit-planning paper.
- **C. Getting better at its own room.** Archived multi-view scans and 3DGS renderings fine-tune the detector for that
  specific room and its objects, measured as fewer false detections and faster finds over repeated sessions.
- **D. Reliability-aware object localisation on low-cost RGB-D.** This covers range-bias correction, glossy and
  low-depth surfaces, and choosing the view that makes the object's position trustworthy before approaching. It is
  more of a supporting contribution.

Recommended pairing: **B as the main claim, with A as the evaluation tool.** B ties to the revisit paper (a pickup makes
the map stale, and that is where a revisit pays off), and A gives labelled data cheaply.

## Open
- Ask Prof. Lee: merge with the revisit study, or write a second paper?
- Do a related-work pass before claiming novelty.
