# Spike: can a real LinearOpMode run its lifecycle on plain JVM?

- Ticket: [issue #12](https://github.com/ngi-collective/FtcRobotController2026-2027/issues/12) (part of the FTC Test Framework wayfinder map, [issue #11](https://github.com/ngi-collective/FtcRobotController2026-2027/issues/11))
- Branch: `research/opmode-plain-jvm-spike`
- SDK under test: `org.firstinspires.ftc:RobotCore:11.2.0` (as pinned in `build.dependencies.gradle`)
- Method: primary-source reading of the RobotCore 11.2.0 sources jar (Gradle artifact `org.firstinspires.ftc:RobotCore:11.2.0:sources`, resolved locally at
  `~/.gradle/caches/modules-2/files-2.1/org.firstinspires.ftc/RobotCore/11.2.0/b3e561633e4f0dd3d95047aaf283037a17baef6d/RobotCore-11.2.0-sources.jar`)
  plus an empirical throwaway JUnit 5 spike run via `mise run test --tests 'org.firstinspires.ftc.teamcode.OpModePlainJvmLifecycleSpikeTest'`
  (`./gradlew :TeamCode:testDebugUnitTest --tests '...'` under the hood). The spike file was removed before this commit; the code and output below are reproduced verbatim from the run.

## Bottom line

**Yes, with a caveat.** A real, unmodified `LinearOpMode` subclass can be constructed and have `waitForStart()` → `opModeIsActive()` loop → `runOpMode()` completion driven end-to-end from a plain JUnit test on the JVM, with no Robolectric and no Android emulator. Construction, `waitForStart()`, `opModeIsActive()`, `idle()`, and hooking up a Mockito-mocked `HardwareMap` all work with zero Android-stub failures.

The one real gap: **`Telemetry.update()` throws a `NullPointerException`** the first time it actually attempts to transmit, because the SDK's own telemetry-transmission path (`OpModeManagerImpl.updateTelemetryNow` → `OpMode.internalUpdateTelemetryNow`) reaches into a package-private field (`internalOpModeServices`) that is normally wired by `OpModeManagerImpl` during real robot boot and is simply never set when an `OpMode` is hand-constructed outside that machinery. This is **not** an Android-stub failure (no `Stub!`/`UnsatisfiedLinkError`/`Context` involved) — it's a plain NPE from a missing internal collaborator, and it is trivially shimmable (see "Can a shim paper over it?" below).

Separately, other RobotCore surfaces genuinely do reach into real `android.*` classes and *do* fail hard under plain JVM without Robolectric — but the `LinearOpMode` lifecycle path itself (`waitForStart`/`opModeIsActive`/`runOpMode`) never touches them. They only matter if OpMode/library code calls them directly (e.g. `RobotLog.d(...)`, `Gamepad.refreshTimestamp()`).

## Primary-source evidence

### 1. Construction is clean — no Android calls in the field-init/constructor chain

A `LinearOpMode` subclass's construction chain is `ConcreteOpMode()` → `LinearOpMode()` → `OpMode()` → `OpModeInternal()` (implicit). Relevant field initializers:

- `OpModeInternal.java:81` — `public Telemetry telemetry = new TelemetryImpl((OpMode) this);` runs unconditionally at construction.
- `TelemetryImpl.java:651-656` — the constructor stores the `OpMode` reference, builds a `LogImpl` (`TelemetryImpl.java:466-626`, pure `ArrayList`/`String` bookkeeping, no Android import), and calls `resetTelemetryForOpMode()` (`TelemetryImpl.java:659-672`), which constructs a `com.qualcomm.robotcore.util.ElapsedTime` (`ElapsedTime.java:90-92`, backed by `System.nanoTime()`, no `SystemClock`).
- `OpMode.java:73-75` — `OpMode()`'s own body only calls `System.nanoTime()`.
- `OpModeInternal.java:70,75` — `gamepad1`/`gamepad2` default to `null` (not constructed eagerly); `hardwareMap` (`OpModeInternal.java:86`) also defaults to `null` and is a **public** field, confirmed settable directly from a test: `opMode.hardwareMap = mock(HardwareMap.class);` (matches the existing `DriveHardwareTest.java:32-41` Mockito-`HardwareMap` convention).

Empirical confirmation (`constructionIsCleanOnPlainJvm` test): construction printed
```
[spike] construction OK, telemetry=class org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl
```
with no exception.

### 2. `waitForStart()` / `opModeIsActive()` — real blocking semantics, driven via reflection

- `LinearOpMode.java:57-68` — `waitForStart()` loops `while (!isStarted())`, blocking on `runningNotifier.wait()` (a `private final Object`, `LinearOpMode.java:24`).
- `LinearOpMode.java:149-158` — `isStarted()` reads the package-private `boolean isStarted` field declared on `OpModeInternal.java:100`.
- `LinearOpMode.java:117-123` — `opModeIsActive()` = `!isStopRequested() && isStarted()`, calling `idle()` (`Thread.yield()`, `LinearOpMode.java:80-84`).

**There is no public API to simulate the driver station's START button from outside the `com.qualcomm.robotcore.eventloop.opmode` package.** The real event loop does it via the package-private `OpModeInternal.internalStart()` (`OpModeInternal.java:221-229`, sets `isStarted = true` then calls `internalOnStart()`, which for `LinearOpMode` does `runningNotifier.notifyAll()` — `LinearOpMode.java:206-211`). A JUnit test in `org.firstinspires.ftc.teamcode` has to reach across the package boundary with reflection: `Field.setAccessible(true)` on `isStarted` and on `runningNotifier`, set `isStarted = true`, then `synchronized(runningNotifier) { runningNotifier.notifyAll(); }`. This worked exactly as the SDK source predicts.

Empirical confirmation (`runOpModeLifecycleDrivenViaReflectionOnIsStartedFlag` test): the spike ran a real `runOpMode()` (calling `waitForStart()`, then looping on `opModeIsActive()`) on a background thread; the main thread let it genuinely block for 100 ms, then reflectively flipped `isStarted` and notified `runningNotifier`. The op-mode thread's own log confirmed it actually blocked and then resumed:
```
[spike] loopLog before start signalled: [before-waitForStart]
...
[spike] loopLog final: [before-waitForStart, after-waitForStart]
```
(The loop then hit the telemetry NPE below on its first iteration, so `loop-0`/`after-loop` never got appended — see next section.)

### 3. `telemetry.update()` — the one lifecycle-path failure, and it's a plain NPE, not an Android stub

- `TelemetryImpl.java:722-762` (`tryUpdate`) — on `USER` update reason (i.e. `Telemetry.update()`, `TelemetryImpl.java:710-713`), once the 250 ms transmission interval has elapsed (`msTransmissionInterval = 250`, `TelemetryImpl.java:669`) it calls `OpModeManagerImpl.updateTelemetryNow(this.opMode, transmitter)` (`TelemetryImpl.java:752`).
- `OpModeManagerImpl.java:829-831` — `updateTelemetryNow` just forwards to `opMode.internalUpdateTelemetryNow(telemetry)`.
- `OpMode.java:250-252` — `internalUpdateTelemetryNow` calls `this.internalOpModeServices.refreshUserTelemetry(telemetry, 0)`. `internalOpModeServices` is a package-private field (`OpModeInternal.java:99`, `volatile OpModeServices internalOpModeServices = null;`) that is **only ever assigned by the real `OpModeManagerImpl`** during genuine robot boot — nothing in `OpMode`/`LinearOpMode`/`TelemetryImpl` construction sets it, and there is no public setter.

Because our spike constructed the `OpMode` directly (never going through `OpModeManagerImpl.internalInit()`), `internalOpModeServices` stayed `null`, and the very first `telemetry.update()` call inside the loop threw:

```
java.lang.NullPointerException: Cannot invoke "org.firstinspires.ftc.robotcore.internal.opmode.OpModeServices.refreshUserTelemetry(com.qualcomm.robotcore.robocol.TelemetryMessage, double)" because "this.internalOpModeServices" is null
	at com.qualcomm.robotcore.eventloop.opmode.OpMode.internalUpdateTelemetryNow(OpMode.java:251)
	at com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl.updateTelemetryNow(OpModeManagerImpl.java:830)
	at org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl.tryUpdate(TelemetryImpl.java:752)
	at org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl.update(TelemetryImpl.java:712)
	at org.firstinspires.ftc.teamcode.OpModePlainJvmLifecycleSpikeTest$SpikeOpMode.runOpMode(OpModePlainJvmLifecycleSpikeTest.java:33)
	at org.firstinspires.ftc.teamcode.OpModePlainJvmLifecycleSpikeTest.lambda$runOpModeLifecycleDrivenViaReflectionOnIsStartedFlag$0(OpModePlainJvmLifecycleSpikeTest.java:95)
	at java.base/java.lang.Thread.run(Thread.java:1583)
```

This is a **wiring gap** (a null internal collaborator), not an Android-runtime stub failure — no `android.*` class appears anywhere in that stack trace.

#### Can a shim paper over it?

Yes, trivially, and without Robolectric. `OpModeServices` (`org/firstinspires/ftc/robotcore/internal/opmode/OpModeServices.java:43-63`) is a **plain Java interface** with zero Android imports:
```java
public interface OpModeServices {
    void refreshUserTelemetry(TelemetryMessage telemetry, double sInterval);
    void requestOpModeStop(OpMode opModeToStopIfActive);
}
```
A test harness can implement a no-op (or recording) `OpModeServices` and reflectively assign it to the same package-private `internalOpModeServices` field used above for `isStarted`/`runningNotifier`, exactly the same reflection pattern already demonstrated in this spike. No android.jar involvement is required for the lifecycle path itself.

### 4. The real Android-stub boundary does exist — just not on this path

The ticket specifically asked about `android.os.SystemClock`, `android.util.Log`, and `Context`. Those genuinely do throw under this Gradle setup (no Robolectric, no `unitTests.returnDefaultValues = true` in `TeamCode/build.gradle`) — but only when code calls them directly, and the `LinearOpMode` lifecycle path (`waitForStart`/`opModeIsActive`/`runOpMode`/`telemetry.update`) as exercised above never does:

- `RobotLog.java:33-36` imports `android.util.Log` (`import android.content.Context; import android.util.Log;`); `RobotLog.d(String)` (`RobotLog.java:199-202`) calls `internalLog` → `android.util.Log.println(...)` (`RobotLog.java:251-253`).
- `Gamepad.java:33` imports `android.os.SystemClock`; `Gamepad.refreshTimestamp()` (`Gamepad.java:359-361`) calls `SystemClock.uptimeMillis()`.

Empirical confirmation (`androidStubBoundary_robotLogAndGamepadTimestamp` test), calling these directly and in isolation:
```
[spike] RobotLog.d() threw: java.lang.RuntimeException: Method println in android.util.Log not mocked. See https://developer.android.com/r/studio-ui/build/not-mocked for details.
[spike] Gamepad.refreshTimestamp() threw: java.lang.RuntimeException: Method uptimeMillis in android.os.SystemClock not mocked. See https://developer.android.com/r/studio-ui/build/not-mocked for details.
```
This is the modern Android Gradle Plugin equivalent of the old `Stub!` `RuntimeException` — same root cause (the real `android.jar` used for compilation/unit-test classpath has method bodies that throw instead of running), different message text since AGP replaced the literal `"Stub!"` message a few versions back.

Where this actually bites in practice: `LinearOpMode.internalRunOpMode()` (`LinearOpMode.java:191-203`, **package-private**, only ever invoked by `OpModeInternal.internalInit()`, which our test cannot reach without also using reflection to call a package-private method) calls `RobotLog.d("User runOpModeMethod exited")` (`LinearOpMode.java:201`) after the user's `runOpMode()` returns, and `OpModeInternal.internalOnStopRequested()`/warnings paths use `RobotLog.addGlobalWarningMessage` (`LinearOpMode.java:238-240`, itself Android-free — only `RobotLog.d`/`.w`/`.e`/etc. touch `Log`). Because a JVM test calls the **public** `runOpMode()` directly rather than the package-private `internalRunOpMode()` wrapper, it naturally never reaches `RobotLog.d`. Any OpMode or library helper that itself calls `RobotLog.*` or a `Gamepad` timestamp/effects method, though, will hit this exact stub wall.

## Working / non-working boundary, precisely

| Surface | Works on plain JVM? | Evidence |
|---|---|---|
| Constructing a `LinearOpMode` subclass | ✅ Yes | `constructionIsCleanOnPlainJvm`, no exception |
| Assigning `HardwareMap` via the public `hardwareMap` field | ✅ Yes | `hardwareMapIsAPublicSettableField`, `OpModeInternal.java:86` |
| `waitForStart()` blocking + release (via reflection on `isStarted`/`runningNotifier`) | ✅ Yes | `runOpModeLifecycleDrivenViaReflectionOnIsStartedFlag` |
| `opModeIsActive()` / `idle()` | ✅ Yes | same test, loop entered after start signalled |
| `telemetry.addData(...)` (no transmission attempted yet) | ✅ Yes | no exception before first `update()` past the interval |
| `telemetry.update()` once transmission is actually attempted | ❌ NPE (`internalOpModeServices == null`) | stack trace above, `OpMode.java:251` |
| `RobotLog.d(...)` / any `RobotLog` method that calls `android.util.Log` | ❌ `RuntimeException` ("not mocked") | `androidStubBoundary_...` test |
| `Gamepad.refreshTimestamp()` (`android.os.SystemClock`) | ❌ `RuntimeException` ("not mocked") | `androidStubBoundary_...` test |
| `Gamepad()` construction / reading `gamepad.left_stick_x` etc. | ✅ Yes | `Gamepad()` ctor only calls `type()`, no Android (`Gamepad.java:369-371,549-551`) |

## Implication for the test-framework MVP

A plain-JVM execution target **can** run real, unmodified `LinearOpMode`s through a genuine `waitForStart()`/loop/stop lifecycle, provided the harness:

1. Sets `hardwareMap` directly (already the established convention, per `DriveHardwareTest.java`).
2. Reflectively drives `isStarted`/`stopRequested` and notifies `runningNotifier` to simulate driver-station start/stop (no public API exists for this — that's an unavoidable, but small and stable, reflection shim).
3. Reflectively installs a no-op/fake `OpModeServices` into `internalOpModeServices` before calling `runOpMode()`, so `telemetry.update()` doesn't NPE.
4. Avoids (or itself fakes/mocks) direct calls to `RobotLog.*` and `Gamepad` timestamp/rumble-effect methods, OR runs under Robolectric / `unitTests.returnDefaultValues = true` if such calls must be exercised for real.

None of this requires Robolectric or an Android emulator for the core `LinearOpMode` lifecycle itself — Robolectric only becomes necessary if OpMode/library code under test calls `android.*` APIs directly (logging via `RobotLog`, gamepad timestamps/rumble, hardware driver code that touches `Context`, etc.), which is a narrower and separately-decidable question from "can the lifecycle run at all."
