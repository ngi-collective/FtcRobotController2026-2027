# NGI Collective FTC Robot

Team robot code for the FIRST Tech Challenge, plus a test framework that runs the same OpModes
against a simulated robot instead of hardware. This glossary fixes the words we use for the parts
of that framework and for the season's game pieces.

## Language

### Running OpModes

**Execution target**:
A runtime an OpMode can be run in. There are two: plain JVM and the Android emulator.
_Avoid_: environment, platform, backend

**OpMode**:
A unit of robot behavior the driver selects and runs, discovered by annotation.
_Avoid_: program, routine, mode

**Harness**:
The thing that drives an OpMode's lifecycle in place of the Driver Station.
_Avoid_: runner, driver, host

### Simulation

**Simulated robot**:
The whole stand-in robot: its device list, its physical dimensions, and the model that moves it.
_Avoid_: virtual robot, mock robot, sim bot

**Fake device**:
An implementation of a real vendor hardware interface whose readings come from a scripted
behavior rather than hardware.
_Avoid_: mock device, stub, virtual device

**Behavior**:
The rule that advances one fake device's state over elapsed time.
_Avoid_: strategy, script, profile

**Field frame**:
The coordinate frame all poses are expressed in: origin at field centre, metres, z up, heading in
radians CCW-positive, heading zero facing +X. The audience sits at -Y. The season CAD uses a
different frame (inches, +Y up, audience at +Z) and is converted on the way in.
_Avoid_: world frame, global coordinates, arena frame

**Pose**:
A position and heading in the field frame.
_Avoid_: location, transform, placement

**Simulated camera**:
A camera whose frames are rendered from the simulated world rather than captured from a lens.
Unlike a fake device, its output derives from the shared world model, not from a scripted
per-device behavior.
_Avoid_: virtual camera, fake camera, synthetic camera

**Camera mount**:
Where a camera sits relative to the robot's origin: a translation and a rotation, six degrees of
freedom, independent of the robot's own pose.
_Avoid_: camera offset, camera rig, extrinsics

### BioBuzz season

**Season inventory**:
The catalogue of what exists in a season: the kinds of scoring element, their sizes and colours,
and the AprilTags in play with their sizes and IDs.
_Avoid_: game config, season manifest

**Scenario**:
A named, versioned arrangement of a season's pieces on the field: where each AprilTag and
scoring element sits, and at what angle. What a simulated camera is pointed at.
_Avoid_: scene, setup, configuration

**POLLEN**:
A BioBuzz scoring element: a yellow ball, approximately 2.8 in (7.1 cm) across. Forty per match.

**NECTAR**:
A BioBuzz scoring element: a red or blue ball, approximately 3.6 in (9.1 cm) across. Eight of
each colour per match.

**HIVE**:
The structure at field centre carrying two CELLS. It is bi-stable: it tips, swapping which CELL
faces up, and carries both of its AprilTag Clusters with it.
_Avoid_: goal, tower, centre structure

**CELL**:
An opening on a HIVE that NECTAR is scored into, with an AprilTag Cluster on its underside.
_Avoid_: goal, basket, target

**FLOWER**:
One of the four POLLEN dispensers mounted on the perimeter wall.
_Avoid_: dispenser, feeder, chute

**AprilTag Cluster**:
A co-planar group of AprilTags whose members' positions are each defined relative to one shared
origin, mounted on the field as a single unit. BioBuzz uses four members per cluster, four
clusters in play.
_Avoid_: tag group, tag array, tag panel

**Cluster origin**:
The point a cluster's member positions are measured from, and the point a robot aims at. In
BioBuzz it sits at the centre of the Cell opening rather than on any tag face.
_Avoid_: tag origin, cluster centre
