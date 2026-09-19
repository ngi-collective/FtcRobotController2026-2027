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
The whole stand-in robot: its device list, its physical dimensions, and the chassis that moves it.
_Avoid_: virtual robot, mock robot, sim bot

**Field world**:
The one rigid-body simulation everything on the field lives in: the balls, the perimeter, and the
robot. It owns the clock they are all stepped against, and it owns the robot's pose.
_Avoid_: physics engine, solver (unqualified), simulation

**Chassis**:
The thing that turns wheel speeds into a pose. Two of them exist: the rigid-body chassis, a body
in the field world with mass that can be shoved, and the kinematic chassis, which goes exactly
where its wheels say and is what remains when ode4j is not on the classpath.
_Avoid_: drive model, robot body, plant

**Slip**:
How fast a wheel's contact patch is moving relative to the speed that wheel is turning at. What
the force a wheel applies is proportional to, and what a robot pinned against a wall has a great
deal of while going nowhere.
_Avoid_: wheel error, traction loss, scrub

**Scrub**:
The sideways travel a mecanum roller loses, expressed as the drivetrain's strafe efficiency.
Distinct from slip: a wheel can be scrubbing while gripping perfectly well.
_Avoid_: slip, strafe loss

**Sweep**:
The space an intake's rollers reach into, and the drag they apply to a ball inside it. A region
with a surface speed, not a body: nothing is captured and nothing is held.
_Avoid_: intake zone, grabber, capture volume

**Launcher**:
A flywheel that throws a ball, named in the robot's configuration by the motor that spins it. The
only mechanism driven by a motor rather than a servo, because what matters about it is how fast it
is actually turning.
_Avoid_: shooter, cannon, turret

**Mouth**:
The space a launcher's wheel occupies. A ball in it while the wheel is spinning leaves; there is
no trigger, no magazine and nothing that counts shots.
_Avoid_: barrel, chamber, feeder, hopper

**Transfer efficiency**:
The fraction of a launcher wheel's surface speed a ball leaves with. About a half for one wheel
against a fixed hood, near one for two counter-rotating wheels.
_Avoid_: launch ratio, throw coefficient

**Spin-up**:
The time a flywheel takes to reach a commanded speed. The reason a launcher's motor is simulated
with inertia while every other motor on the robot is ideal.
_Avoid_: rev up, warm up, charge

**Fake device**:
An implementation of a real vendor hardware interface whose readings come from a scripted
behavior rather than hardware.
_Avoid_: mock device, stub, virtual device

**Behavior**:
The rule that advances one fake device's state over elapsed time.
_Avoid_: strategy, script, profile

**Field frame**:
The coordinate frame all poses are expressed in: origin at field centre, metres, z up, heading in
radians CCW-positive, heading zero facing +X. The alliance stations are on the X axis, red at -X,
and the audience sits at -Y. The season CAD uses a different frame (inches, +Y up, audience at +Z)
and is converted on the way in.
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
scoring element sits, at what angle, and which way each HIVE is tipped. Chosen while a session
runs, not only when one starts, and the absence of one is itself a choice &mdash; the robot's own
field, which is the official arrangement with nothing staged on it.
_Avoid_: scene, setup, configuration

**Structure**:
A piece of the field a robot can run into: the HIVE's frame, a HIVE, a FLOWER. Where it stands
comes from the season's CAD rather than from any choice. Almost all of it is bolted down; a HIVE
is the exception, and it is the only thing on a field that moves without being a ball or a robot.
Distinct from a scoring element, which is loose and gets carried around, and from a field surface,
which cannot be collided with.
_Avoid_: obstacle, prop, field element, furniture

**Pivot**:
The axis a structure turns on, and what it takes to turn it: where the axis runs, the two stable
states at the ends of its travel, the torque holding it against one of them, and the damper that
stops the swing slamming. A HIVE has one and nothing else does.
_Avoid_: hinge, joint, axle

**Tip angle**:
How far round a HIVE has swung, measured from the state the season's CAD captured. Continuous, so
a HIVE caught mid-swing has an angle and no state at all, which is the honest description of a
field a robot has just shot into.
_Avoid_: tip state, rotation, orientation

**HIVE TIP**:
The achievement: a HIVE moving from one stable state to the other, worth 20 points every time it
happens. Counted where the motion is, by the pivot arriving at the far stop, because the manual
dates one from the damper reaching the frame and that is what a stop is.
_Avoid_: flip, tipped (as a noun), tip event

**Attached**:
Declared to ride on a structure's pivot. A CELL's scoring volume and the AprilTag cluster under it
are attached to the HIVE they are cut into, so one rotation moves them and the panels together.
_Avoid_: parented, bound, linked

**Field surface**:
The flat parts of the field that exist to be looked at: the floor tiles and the faces of the
perimeter. Both renderers draw them from the field's dimensions; a robot collides with the
perimeter itself, never with anything drawn.
_Avoid_: floor, walls (unqualified), field geometry

**POLLEN**:
A BioBuzz scoring element: a yellow ball, approximately 2.8 in (7.1 cm) across. Forty per match.

**NECTAR**:
A BioBuzz scoring element: a red or blue ball, approximately 3.6 in (9.1 cm) across. Eight of
each colour per match.

**HIVE Structure**:
The whole assembly at field centre: a frame, and the two HIVEs it carries on pivots. The frame
never moves and is the only part of it a robot can reach.
_Avoid_: the HIVE (for the whole thing), goal structure, centre tower

**HIVE**:
One alliance's tipping assembly, carrying two CELLS on one pivot. It is bi-stable: it tips,
swapping which CELL faces up, and carries both of its AprilTag Clusters with it.
_Avoid_: goal, tower, centre structure, HIVE Structure

**Tip**:
Which of a HIVE's two CELLS is the raised one, as a whole state rather than a number: one of the
two, and a property of the HIVE rather than of either CELL, because both move together. What a
scenario file says and where a HIVE starts. Mid-swing there is no tip, only a tip angle.
_Avoid_: state, position, orientation, flip

**CELL**:
An opening on a HIVE that POLLEN and NECTAR are scored into, with an AprilTag Cluster on its
underside.
_Avoid_: goal, basket, target

**FLOWER**:
One of the four POLLEN dispensers mounted on the perimeter wall.
_Avoid_: dispenser, feeder, chute

**Retrieval Opening**:
The gap at the bottom of a FLOWER that a robot takes POLLEN out through. The rest of a FLOWER is
caged; this part is deliberately not.
_Avoid_: bottom opening, hatch, outlet

**Scoring Volume**:
A region of the field where a SCORING ELEMENT is worth points. Invisible, and the manual's own
term. A CELL's is its opening, and it swings with the basket; all four exist at every moment, and
which of them score is read off the way each mouth faces.
_Avoid_: scoring zone, goal volume, trigger volume

**AprilTag Cluster**:
A co-planar group of AprilTags whose members' positions are each defined relative to one shared
origin, mounted on the field as a single unit. BioBuzz uses four members per cluster, four
clusters in play.
_Avoid_: tag group, tag array, tag panel

**Cluster origin**:
The point a cluster's member positions are measured from, which is what a detection reports the
pose of. In BioBuzz it lands within an inch and a half of the centre of the CELL opening its tags
hang under, so it is also the point a launcher aims at.
_Avoid_: tag origin, cluster centre

**Aim point**:
Where a shot is meant to arrive: the middle of a raised CELL's opening. Distinct from a cluster
origin, which is where the camera says something is; that the two coincide is a measured fact
about this season rather than a definition.
_Avoid_: target, goal point

**Sighting**:
Something a camera reported, placed in the robot's frame with its height above the tiles. What is
left of a detection once the camera's own tilt has been taken out of it, and the only form a
ballistic solution can use.
_Avoid_: detection (for this), observation, measurement

**Shot**:
What it would take to score in a CELL from where the robot is standing: how far to turn, how far
the ball must fly, and the one wheel speed that flies it. Or the reason there is none.
_Avoid_: solution, trajectory, aim

**Reach**:
Whether a shot exists at all, and if not which way the robot is wrong: too close for the launch
angle, or past what the wheel can throw. Too close is a thing a driver fixes by reversing, never
by spinning faster.
_Avoid_: in range, valid, feasible
