# NGI Collective FTC Robot

Team robot code for the FIRST Tech Challenge, plus a test framework that runs the same OpModes
against a simulated robot instead of hardware. This glossary fixes the words we use for the parts
of that framework and for the season's game pieces.

## Language

### Software components

**Robot Controller app**:
The Android app that runs OpModes on a robot. Built two ways from this repo: the competition
build, which talks to real hardware, and the Simulated Robot Controller.
_Avoid_: the app, RC, robot controller (unqualified)

**Simulated Robot Controller**:
The build of the Robot Controller app whose hardware is a simulated robot and whose camera is a
simulated camera. Runs OpModes through the SDK's own event loop, on a device or emulator.
_Avoid_: the simulated app, sim build, emulator app

**Driver Station**:
The FTC app a driver uses to choose an OpMode and drive, connected to a Robot Controller app over
a network.
_Avoid_: DS (unqualified), driver app

**Driver Hub**:
REV's handheld hardware that runs the Driver Station. Hardware only: never a name for anything in
this repo.
_Avoid_: using this for the Driver Hub Dashboard

**Driver Hub Dashboard**:
The browser application a developer drives the simulation from. Also *Hub Dashboard* or *the
Dashboard*. Its own OpModes run through a harness, not through the SDK's event loop.
_Avoid_: Driver Hub, driver station replacement, web UI

**Dashboard Server**:
The process that serves the Driver Hub Dashboard and owns the simulation it shows.
_Avoid_: backend, dashboard host

**Dashboard Field View**:
The panel in the Driver Hub Dashboard that draws the field and the robot on it, from above or in
3D.
_Avoid_: the 3D view, scene, field panel

**Dashboard Camera View**:
The panel in the Driver Hub Dashboard that shows a camera view, with or without a camera overlay.
_Avoid_: video panel, preview pane

**Test framework**:
The library of fake devices, harnesses, the drive model and the camera renderer, shared by the
Dashboard Server, the Simulated Robot Controller and the tests.
_Avoid_: the framework, sim library

### Running OpModes

**Execution target**:
A runtime an OpMode can be run in. There are two: plain JVM and the Android emulator.
_Avoid_: environment, platform, backend

**OpMode**:
A unit of robot behavior the driver selects and runs, discovered by annotation.
_Avoid_: program, routine, mode

**Harness**:
The thing that drives an OpMode's lifecycle in place of the event loop, pressing init, start and
stop as a Driver Station would.
_Avoid_: runner, driver, host

**Event loop**:
The SDK's own driver of the active OpMode: it selects one, calls it once per control cycle, and
stops it. What a harness stands in for.
_Avoid_: run loop, scheduler, main loop

**Robot start**:
The SDK's bring-up of a robot: building the device list, starting the event loop, and registering
the OpModes a driver can select. It needs a network connection, so it cannot complete on an
emulator, and until it completes no OpMode can be selected.
_Avoid_: robot setup, boot, initialization

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

**Camera view**:
What a simulated camera sees, presented to a person: the frames it renders, unannotated.
_Avoid_: livestream, camera feed, preview

**Camera overlay**:
A camera view with detection annotations drawn over it &mdash; tag outlines, axes, contours.
Distinct from the SDK's Driver Station camera stream, which is the same idea on real hardware.
_Avoid_: annotated stream, debug view

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
