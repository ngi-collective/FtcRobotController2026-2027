# The camera looks forward and 45° up, and "no tags from this side" means tipped

## Context

Verity's webcam used to point straight up from 12.5 in off the tiles. From there it saw a raised
CELL's tag cluster only from about 0.5 m short of it and closer. That was inside the 75° lob's
minimum range, so the robot could aim but not shoot from the same spot, and from neither AUTO start
square (both about 1.1 m from the nearest tag plate) could it see a tag at all.

The team wanted AUTOs that check which CELL of their HIVE is raised and shoot at that one, since a
TIP swaps them and a lowered CELL scores nothing.

All four tags of a HIVE face one way: both plates share a normal, which points toward the side the
raised CELL opens to and 60° below horizontal, and turns with the HIVE when it tips. From the other
side the plates are edge-on or turned away, at about 92° to the line of sight.

## Decision

- **The camera looks forward along the robot's nose, tilted 45° up** (`verity.json`: yaw 0, pitch
  45; `LaunchGeometry.camera()`). From either start square it sees its alliance's raised cluster,
  all four tags, at about 1.45 m and 20° off the plate's normal, whenever the HIVE faces that
  square. It frames the raised cluster from about 0.46 m to about 2 m short of the opening, which
  covers the whole shooting range, so the robot can now aim and shoot from the same spot. 45° is
  the design figure; the bracket's measured angle replaces it in both files.
- **The camera can only ever confirm "facing"; "behind" is the absence of tags.** No pitch sees a
  HIVE's tags from both sides, and the opponent's HIVE tips independently, so it says nothing about
  ours. `RaisedCellWatch` reads FACING when the alliance's raised cluster (by height, above
  `RAISED_CELL_HEIGHT_METRES`) was seen in the last 0.5 s, BEHIND after 0.5 s of frames without
  it, and NO_CAMERA when frames stop arriving. NO_CAMERA never turns the robot round: it keeps to
  the match-start arrangement, or to its last belief.
- **The tip-aware AUTOs** (`TipAwareAuto`, `… Tip-Aware`) read through INIT and commit at START;
  look again for up to 1 s before every shot; and if the raised CELL has turned away, drive round
  to the other side's shooting spot when at least 10 s of the AUTO is left, or park without
  shooting otherwise. One drive round per shot.

## Consequences

- Anything that blocks the camera's view of its own HIVE reads as a TIP. The robot drives round,
  sees nothing there either, and parks without shooting. Telemetry says which reading it acted on.
- The camera tests that stood the robot right under a CELL for the upward camera now stand it
  about 0.9 m short of the opening.
- The AUTO's re-look before a shot costs up to a second each time, usually one frame.
