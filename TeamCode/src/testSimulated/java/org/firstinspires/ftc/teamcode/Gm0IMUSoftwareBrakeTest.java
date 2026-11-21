package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.simulated.VerityRobot;
import org.junit.After;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.ngicollective.ftcsim.hardware.FakeHardwareMap;
import org.ngicollective.ftcsim.hardware.FakePinpoint;
import org.ngicollective.ftcsim.harness.LinearOpModeHarness;
import org.ngicollective.ftcsim.harness.OpModeHarness;
import org.ngicollective.ftcsim.sim.Pose2d;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Runs {@link Gm0IMU} headlessly and checks its software brake: X pins the robot where it is, a
 * shove is pushed back out, and X again hands the drive back to the sticks.
 *
 * <p>Asserted on the simulated robot's true pose, in metres and radians, not on the follower's
 * estimate: the brake is only as good as where the robot actually ends up.</p>
 */
@RunWith(RobolectricTestRunner.class)
@Config(sdk = 28)
public class Gm0IMUSoftwareBrakeTest {

    private static final double TICK = 0.02;
    private static final double INCH = 0.0254;

    private FakeHardwareMap hardware;
    private LinearOpModeHarness harness;

    @After
    public void tearDown() {
        if (harness != null) {
            harness.pressStop();
            harness.awaitCompletion(2000);
        }
    }

    @Test
    public void aShoveIsPushedBackToTheBrakedPoseAndHeading() {
        launchAndStart();
        Pose2d braked = engageBrake();

        // Six inches sideways and fifteen degrees round, as a robot being rammed would be.
        shove(new Pose2d(braked.x() + 6 * INCH, braked.y() + 4 * INCH,
                braked.heading() + Math.toRadians(15.0)));
        assertTrue(offBy(braked) > 5 * INCH, "the shove should have moved the robot");

        steps(3.0);

        Pose2d now = hardware.drive().pose();
        assertTrue(offBy(braked) < 1.0 * INCH,
                "the brake should return the robot to " + braked + ", it is at " + now);
        assertEquals(0.0, headingError(braked), Math.toRadians(2.0),
                "the brake should restore the heading of " + braked + ", it is at " + now);
    }

    @Test
    public void theSticksDoNotMoveABrakedRobot() {
        launchAndStart();
        Pose2d braked = engageBrake();

        harness.gamepad1().left_stick_y = -1.0f;
        harness.gamepad1().right_stick_x = 1.0f;
        steps(1.0);

        assertTrue(offBy(braked) < 0.5 * INCH,
                "sticks moved a braked robot to " + hardware.drive().pose());
        assertEquals(0.0, headingError(braked), Math.toRadians(1.0),
                "sticks turned a braked robot to " + hardware.drive().pose());
    }

    @Test
    public void pressingXAgainReleasesTheBrake() {
        launchAndStart();
        Pose2d braked = engageBrake();

        pressX();
        await(() -> telemetryHas("Software Brake : off (X to engage)"),
                () -> "X never released the brake; telemetry was " + harness.lastTelemetry());

        harness.gamepad1().left_stick_y = -1.0f;
        steps(0.5);
        harness.gamepad1().left_stick_y = 0.0f;

        // Facing +X, forward on the stick drives along +X.
        Pose2d now = hardware.drive().pose();
        assertTrue(now.x() - braked.x() > 0.2,
                "a released robot should drive forward, it went from " + braked + " to " + now);
    }

    /** A fresh robot at the origin facing +X, past INIT and in its main loop. */
    private void launchAndStart() {
        hardware = new VerityRobot().create();
        hardware.drive().setPose(Pose2d.ORIGIN);
        harness = OpModeHarness.forLinear(new Gm0IMU(), hardware);
        harness.launch();
        await(() -> telemetryHas("Status : Initialized - waiting for start"),
                () -> "the OpMode never reached its init loop; telemetry was " + harness.lastTelemetry());
        // The Pinpoint recalibrates its gyro when the follower is built; a driver waits that out.
        steps(0.5);
        harness.pressStart();
        await(() -> telemetryHas("Software Brake : off (X to engage)"),
                () -> "the OpMode never reached its main loop; telemetry was " + harness.lastTelemetry());
    }

    /** Presses X, waits for the brake to report on, and returns where it is holding the robot. */
    private Pose2d engageBrake() {
        pressX();
        await(() -> telemetryHas("Software Brake : ON (X to release)"),
                () -> "X never engaged the brake; telemetry was " + harness.lastTelemetry());
        return hardware.drive().pose();
    }

    /** Press and let go: the brake toggles on the press, so holding X must not flicker it. */
    private void pressX() {
        harness.gamepad1().x = true;
        steps(0.1);
        harness.gamepad1().x = false;
        steps(0.1);
    }

    /**
     * Moves the robot the way a push does: the odometry pods roll with it and count the motion.
     *
     * <p>{@code setPose} alone is a placement, which the simulated Pinpoint ignores because the pods
     * did not turn. Telling it the chassis was last seen where it stood makes the next tick count the
     * whole move.</p>
     */
    private void shove(Pose2d to) {
        Pose2d from = hardware.drive().pose();
        hardware.drive().setPose(to);
        hardware.get(FakePinpoint.class, "pinpoint").state().chassisPlaced(from);
    }

    private double offBy(Pose2d target) {
        Pose2d now = hardware.drive().pose();
        return Math.hypot(now.x() - target.x(), now.y() - target.y());
    }

    private double headingError(Pose2d target) {
        return AngleUnit.normalizeRadians(hardware.drive().pose().heading() - target.heading());
    }

    private boolean telemetryHas(String line) {
        return harness.lastTelemetry().contains(line);
    }

    /**
     * Simulated time a tick at a time, each followed by a tick of wall time so the OpMode, on its own
     * thread, sees every step rather than being outrun by the world.
     */
    private void steps(double seconds) {
        for (int tick = 0; tick < Math.round(seconds / TICK); tick++) {
            harness.advance(TICK);
            harness.eventLoopIteration();
            try {
                Thread.sleep((long) (TICK * 1000.0));
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                throw new AssertionError("interrupted while stepping", e);
            }
        }
    }

    private void await(BooleanSupplier condition, Supplier<String> failure) {
        long deadline = System.nanoTime() + 3_000_000_000L;
        while (System.nanoTime() < deadline) {
            if (harness.failure() != null) {
                throw new AssertionError("the OpMode died", harness.failure());
            }
            if (condition.getAsBoolean()) {
                return;
            }
            steps(TICK);
        }
        throw new AssertionError(failure.get());
    }
}
