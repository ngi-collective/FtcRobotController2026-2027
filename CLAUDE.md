# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A team fork of the FIRST Tech Challenge SDK (`FtcRobotController` v12.0, BIOBUZZ 2026-2027 season). It builds an Android APK that runs on the REV Control Hub; the REV Driver Hub runs the Driver Station app. Team-authored robot code lives in `TeamCode`; everything under `FtcRobotController/` is vendor SDK code and samples, left alone so upstream SDK updates merge cleanly. The simulator and test framework TeamCode is tested against is [ftc-sim](https://github.com/ngi-collective/ftc-sim), consumed from Maven Central; it began in this repository and moved out (#24). Building from Android Studio requires Narwhal 3 Feature Drop or later.

Two remotes, and the distinction matters:

- `origin` → `ngi-collective/FtcRobotController2026-2027` — the team fork. All team work goes here.
- `upstream` → `FIRST-Tech-Challenge/FtcRobotController` — the official SDK, pull-only.

`.github/CONTRIBUTING.md` is upstream's, and its main point applies: team code is never meant to go back to `upstream`. Never push or open a PR against it.

## Commands

**Every Gradle invocation must run under mise** — it supplies both the JDK and the Android SDK, and the build fails without it (see Toolchain). Prefer the tasks in `mise.toml`, which run inside that environment already:

```bash
mise run check       # compile TeamCode - fastest correctness check while editing OpModes
mise run test        # TeamCode's unit tests, headless simulated ones (vision included)
mise run build       # competition debug APK
mise run install     # build + adb install to a connected Robot Controller device
mise run simulator   # install + launch the simulated app on a running emulator
mise run test-acceptance  # the SDK's real robot start and event loop, on a running emulator
mise run dashboard   # Driver Hub in a browser against TeamCode's OpModes (http://localhost:8765)
mise run dashboard --args="--scenario tipping-hive"  # ...with a field arrangement loaded
mise run lint
mise run clean
mise run setup-sdk         # (re)install the SDK packages the build needs
mise run verify-toolchain  # assert CLI and IDE toolchains can both still build (CI gate)

mise tasks           # list the above with descriptions
```

The dashboard's flags go inside `--args=`, and an unrecognised flag is dropped in silence: the
symptom is a session on the robot's own field wondering where the scenario went.

Supervised processes: `hub ps` before starting anything, one name per service, and reuse a live one with `hub restart <name>`. Stop a process in the turn its work finishes. The one worth keeping between turns is `mise run dashboard`, which idles at 0% CPU once no browser is attached.

## Workflow

- Avoid making changes in `FtcRobotController` - that is managed upstream and will make conflicts more painful as the official SDK gets updated.
- Run `mise run test` for tests and `mise run lint` before considering any work done.

## Simulator

`TeamCode/build.gradle` applies `org.ngi-collective.ftc-sim` and sets `ftcSim { version; season;
seasonVersion }`. The plugin adds the `robot`/`simulated` flavours, the simulator's libraries, the
simulated app (compiled from source against this repo's `FtcRobotController`), unit-test setup
with the vision natives, the staged `robot-config/` and `scenarios/`, and the `dashboard` task. It
raises TeamCode's compile SDK to 34 if lower and declares `org.openpnp:opencv` before the SDK;
`OpenCvClasspathOrderTest` guards that order. To take a new simulator or season release, bump the
versions there. How the simulator itself works, and why, is ftc-sim's `CLAUDE.md` and `docs/adr/`.

What this repository supplies to it:

- **`TeamCode/robot-config/*.json` is the physics source of truth** for Verity: wheel radius, gear
  ratio, track width, strafe efficiency, grip, chassis dimensions and mass, encoder resolution,
  per-motor mirror, launcher wheel and aim, and the six-DOF camera mount. Never hardcode these
  numbers anywhere else. Read again on every INIT, so editing a file takes effect at the next
  INIT without restarting the dashboard.
- **`VerityRobot`** (`TeamCode/src/simulated/`) holds the device names, which are code so that a
  rename breaks the build, and returns the season. It is registered in
  `src/simulated/resources/META-INF/services/org.ngicollective.ftcsim.hardware.SimulatedRobot`;
  exactly one robot is allowed.
- **`TeamCode/robot-layouts/*.json` is cosmetic**: mesh, scale, label offsets.
- **`TeamCode/scenarios/*.json`** stage the field: balls on the tiles or in a named CELL, HIVE tip
  states. `match-staging` is the manual's setup (6-6 before anyone drives); `tipping-hive` is two
  good shots short of a TIP; `auto-sweep` and `aimed-shot` back the autonomous and aiming tests.

Facts about the simulation that decide how to write a test here:

- Assert drive behaviour on **pose**, not wheel powers (`MecanumDrivePoseTest`), and against
  physical bounds: grip bounds acceleration, so Verity reaches its 1.57 m/s free speed in about
  three tenths of a second, not instantly.
- Poses are in the **FTC field frame**: origin at field centre, metres, heading radians CCW from +X.
- A launcher is a **flywheel whose shaft speed sets the ball's speed**. A shot fired before spin-up
  falls short; spin up first, then present the ball. This robot cannot shoot on the move.
- Vision tests run on a plain JVM under Robolectric: call `PlainJvmVision.pump()` once per control
  cycle, or an asynchronously opened camera never finishes opening.
- Wall contact is a real collision and leaves the encoders counting, so dead-reckoning drift is
  reproducible.

## OpMode model

OpModes are discovered by annotation (`@TeleOp` / `@Autonomous`, optionally `@Disabled`).

## Aiming, and shooting what you aimed at

`AimedLauncherTeleOp` closes the loop: detect the cluster, range off it, set the flywheel from the
range, shoot. `LensMount` → `ShotSolver` → `LaunchGeometry` in `TeamCode/src/main`, all three
plain-JVM testable. See `docs/adr/0006-a-cluster-detection-is-an-aim-point.md`.

**To watch the whole game work at once**, run `mise run dashboard --scenario auto-sweep`, pick
**Auto: sweep and shoot**, drag the robot onto the start square the INIT telemetry names
(`x -0.55, y -1.43, heading 90`), and press START. Nine seconds later the score reads
`RED 20 (1 TIP)`. `SweepAndShootAuto` is the routine to read first: five timed steps, no vision,
no odometry, and `SweepAndShootAutoTest` runs it headlessly on every commit, as
`AimedLauncherAcceptanceTest` does the vision loop's equivalent.

- **An AUTO aims off the field drawing, not off a camera.** It starts on a known square facing a
  known direction, so the range is arithmetic before the robot has moved and with no tag in view.
  Same `ShotSolver`, different source of range — and the weakness is the obvious one: set the robot
  down half a metre out and every ball lands on the tiles, cheerfully.
- **It slides sideways rather than driving at the balls**, for the reason below: forward motion
  ruins a shot and sideways motion costs a quarter of an opening. Two rows, because the mouth
  reaches 24 cm and the bumper is at 20, so the band a ball can sit in without being shoved along
  by the chassis is one ball wide.
- **A routine that ends undoes its own work.** A dashboard session rebuilds the field when an
  OpMode stops, so the last step of an AUTO worth watching is `while (opModeIsActive())` holding
  still — which is also what a real one does while it waits for the buzzer.

- **A cluster detection is the aim point.** The SDK's cluster origin lands 1.42 in from the centre
  of the CELL opening its tags hang under — an eighth of the opening's height — identically for all
  four CELLs in both tip states, so there is no offset table. Measured, not chosen: the SDK's
  member offsets, FIRST's CAD and the manual's CELL dimensions agree there and none of them
  mentions the others.
- **Never use `ftcPose.range`, `bearing` or `elevation` on this robot.** They are measured in the
  camera's own frame — `range` is `hypot(x, y)` with the vertical dropped — and the camera aims up
  35°, so all three are wrong about the field by a plausible-looking amount. Use `x`, `y`, `z` and
  level them through the mount.
- **`robotPose` is useless this season.** The SDK declares all four BioBuzz clusters at
  `fieldPosition = (0,0,0)` with an identity orientation, so anything absolute must be built from
  relative measurements.
- **The two CELLs of a HIVE are one part mounted two ways**, the second turned 180° about the
  CELL's own rise axis. Their tag plates therefore differ by 180° of roll, and getting that wrong
  is nearly invisible: the tags still land on the measured plate and still detect, only the ids run
  the other way and the cluster origin lands two feet away, behind the closed end of the basket.
  `BioBuzzFieldTest.everyClusterOriginSitsAtItsCellsOpening` is the guard.
- **The detector reads about 3 % near** — a cluster 0.92 m away reports 0.89 m. Uniform, harmless
  for a steep lob, and the reason the acceptance test works in centimetres.
- **Which CELL to shoot at is a question about height.** Both plates of a HIVE share a normal, so a
  camera sees all four of its tags at once; the raised opening is at 1.51 m and the lowered one at
  0.97 m, and `LaunchGeometry.RAISED_CELL_HEIGHT_METRES` sits in the gap.
- **A 75° lob has a minimum range** (`rise / tan p`, about 40 cm) and needs its *least* speed at
  twice that. Closer shots are harder than mid-range ones, which is the opposite of the intuition a
  flat shooter gives a driver.
- **This robot cannot shoot on the move.** The intake's sweep and the launcher's mouth are the same
  6 cm of space, so a ball comes within reach and goes; at a third of power the chassis adds
  0.55 m/s to a 1.45 m/s horizontal component and the ball clears the far lip, which is 9 cm past
  the near one. Spin up, then roll the last couple of centimetres. Firing while driving needs a
  feeder, which this robot has not got.

## Conventions

Sample naming in `FtcRobotController/.../external/samples/` follows `Basic` / `Sensor` / `Robot` / `Concept` prefixes (plus some `Utility` classes); the scheme is documented in `sample_conventions.md` alongside them. Copy a sample into `TeamCode` rather than editing it in place.

## Agent skills

### Issue tracker

Issues live in this repo's GitHub Issues (ngi-collective/FtcRobotController2026-2027), via the `gh` CLI. See `docs/agents/issue-tracker.md`.

### Triage labels

Default label vocabulary: `needs-triage`, `needs-info`, `ready-for-agent`, `ready-for-human`, `wontfix`. See `docs/agents/triage-labels.md`.

### Domain docs

Single-context layout: `CONTEXT.md` + `docs/adr/` at the repo root (not yet created; created lazily by `/domain-modeling`). See `docs/agents/domain.md`.
