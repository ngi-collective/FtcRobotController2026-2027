package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.firstinspires.ftc.teamcode.simulated.VerityRobot;
import org.ngicollective.ftcsim.hardware.FakeHardwareMap;
import org.ngicollective.ftcsim.harness.LinearOpModeHarness;
import org.ngicollective.ftcsim.harness.OpModeHarness;
import org.ngicollective.ftcsim.sim.Pose2d;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Runs the real {@link iamyou} TeleOp end to end with no robot, no Driver Hub and no emulator.
 *
 * <p>The OpMode is unmodified: it looks its hardware up through {@code hardwareMap}, reads
 * {@code gamepad1}, and pushes telemetry exactly as it does on the field. Everything it touches is
 * simulated by the test framework.</p>
 *
 * <p>Driving is asserted on pose, not wheel powers. {@link IamYouLogic} shapes, caps and
 * friction-boosts every command, so the powers are tuning; where the robot goes is the
 * behaviour. {@link IamYouLogicTest} owns the numbers.</p>
 */
class IamYouHeadlessTest {

    private static final String[] WHEELS = {"FL", "FR", "BL", "BR"};
    private static final double TICK = 0.02;

    private FakeHardwareMap hardware;
    private LinearOpModeHarness harness;

    @AfterEach
    void tearDown() {
        if (harness != null) {
            harness.pressStop();
            harness.awaitCompletion(2000);
        }
    }

    @Test
    void initializesTheImuTheDriveMotorsAndTheIntakeBeforeStart() {
        launch();

        assertTrue(hardware.imu("imu").isInitialized(), "the OpMode must initialize the IMU");
        for (String wheel : WHEELS) {
            // Plain power on purpose: iamyou reads the encoders for its stop assist but drives
            // without the hub's velocity PID.
            assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, hardware.motor(wheel).getMode(), wheel);
            assertEquals(DcMotor.ZeroPowerBehavior.BRAKE,
                    hardware.motor(wheel).getZeroPowerBehavior(), wheel);
        }
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                hardware.motor(IntakeHardware.NAME).getMode());
        assertEquals(DcMotor.ZeroPowerBehavior.BRAKE,
                hardware.motor(IntakeHardware.NAME).getZeroPowerBehavior());
        // A line, not the whole frame: the position is followed by the D-pad legend.
        await(() -> harness.lastTelemetry().stream()
                        .anyMatch(line -> line.startsWith("Driver Position : SOUTH")),
                () -> "telemetry never reported the driver position; last was "
                        + harness.lastTelemetry());
    }

    @Test
    void forwardStickDrivesTheWayTheRobotFacedAtStart() {
        launchAndStart();
        Pose2d pose = drive(0.0, -1.0, 0.0f, 0.5);

        assertTrue(pose.x() > 0.2, "expected the robot to drive along +X, got " + pose);
        assertTrue(Math.abs(pose.y()) < 0.1 * pose.x(), "expected no lateral drift, got " + pose);
        assertTrue(Math.abs(pose.headingDegrees()) < 2.0, "expected no rotation, got " + pose);
    }

    @Test
    void rightStickStrafesTowardTheRobotsRight() {
        launchAndStart();
        Pose2d pose = drive(1.0, 0.0, 0.0f, 0.5);

        // Facing +X, the robot's right is -Y.
        assertTrue(pose.y() < -0.15, "expected the robot to strafe toward -Y, got " + pose);
        assertTrue(Math.abs(pose.x()) < 0.1 * -pose.y(), "expected no forward drift, got " + pose);
        assertTrue(Math.abs(pose.headingDegrees()) < 2.0, "expected no rotation, got " + pose);
    }

    @Test
    void slowModeDrivesWellUnderHalfSpeed() {
        launchAndStart();
        double full = drive(0.0, -1.0, 0.0f, 0.5).x();
        harness.pressStop();
        harness.awaitCompletion(2000);

        launchAndStart();
        double slow = drive(0.0, -1.0, 1.0f, 0.5).x();

        assertTrue(slow > 0.0, "slow mode should still drive, got " + slow);
        assertTrue(slow < 0.5 * full, "slow mode covered " + slow + " m against " + full + " m");
    }

    @Test
    void forwardStickKeepsDrivingDownFieldAfterTheChassisTurns() {
        launchAndStart();
        // A quarter turn left after START: the robot now faces +Y, but NORTH is still +X.
        hardware.drive().setPose(new Pose2d(0.0, 0.0, Math.PI / 2));
        harness.advance(TICK);
        assertEquals(90.0, yaw(), 0.5, "the IMU should report the turn");

        Pose2d pose = drive(0.0, -1.0, 0.0f, 0.5);

        assertTrue(pose.x() > 0.15, "expected field-centric drive along +X, got " + pose);
        assertTrue(Math.abs(pose.y()) < 0.1 * pose.x(), "expected no drift along Y, got " + pose);
    }

    @Test
    void reportsWhichWayTheRobotFacesInTelemetry() {
        launchAndStart();
        hardware.drive().setPose(new Pose2d(0.0, 0.0, -Math.PI / 2));
        harness.advance(TICK);

        await(() -> harness.lastTelemetry().contains("Facing (vs start) : EAST"),
                () -> "telemetry never reported the new heading; last was " + harness.lastTelemetry());
        assertEquals(-90.0, yaw(), 0.5);
    }

    @Test
    void triggerRunsTheIntakeInAndBumperEjects() {
        launchAndStart();

        harness.gamepad1().right_trigger = 1.0f;
        awaitIntakePower(IntakeLogic.INTAKE_POWER);

        // Eject wins over a held trigger.
        harness.gamepad1().right_bumper = true;
        awaitIntakePower(-IntakeLogic.EJECT_POWER);

        harness.gamepad1().right_bumper = false;
        harness.gamepad1().right_trigger = 0.0f;
        awaitIntakePower(0.0);
    }

    @Test
    void stoppingCutsPowerToEveryWheelAndTheIntake() {
        launchAndStart();
        harness.gamepad1().left_stick_y = -1.0f;
        harness.gamepad1().right_trigger = 1.0f;
        awaitSteadyWheelPowers();
        awaitIntakePower(IntakeLogic.INTAKE_POWER);

        harness.pressStop();
        harness.awaitCompletion(2000);

        for (String wheel : WHEELS) {
            assertEquals(0.0, hardware.motor(wheel).getPower(), 1e-9, wheel);
        }
        assertEquals(0.0, hardware.motor(IntakeHardware.NAME).getPower(), 1e-9);
    }

    /** A fresh robot at the origin facing +X, its OpMode spinning in the init loop. */
    private void launch() {
        hardware = new VerityRobot().create();
        hardware.drive().setPose(Pose2d.ORIGIN);
        harness = OpModeHarness.forLinear(new iamyou(), hardware);
        harness.launch();
        await(() -> harness.lastTelemetry().contains("Status : Initialized - waiting for start"),
                () -> "the OpMode never reached its init loop; last telemetry was "
                        + harness.lastTelemetry());
    }

    private void launchAndStart() {
        launch();
        harness.pressStart();
    }

    /**
     * Holds the sticks, waits for the OpMode's command to settle, then drives for {@code seconds}
     * of simulated time from where the robot stands.
     *
     * <p>The wait matters: iamyou ramps its command, and simulated time advancing mid-ramp would
     * measure the ramp rather than the drive.</p>
     */
    private Pose2d drive(double leftX, double leftY, float leftTrigger, double seconds) {
        harness.gamepad1().left_stick_x = (float) leftX;
        harness.gamepad1().left_stick_y = (float) leftY;
        harness.gamepad1().left_trigger = leftTrigger;
        awaitSteadyWheelPowers();

        Pose2d start = hardware.drive().pose();
        for (int tick = 0; tick < Math.round(seconds / TICK); tick++) {
            harness.advance(TICK);
            harness.eventLoopIteration();
        }
        Pose2d end = hardware.drive().pose();
        return new Pose2d(end.x() - start.x(), end.y() - start.y(), end.heading() - start.heading());
    }

    /** Polls until every wheel is commanded and the commands have stopped changing. */
    private void awaitSteadyWheelPowers() {
        double[] last = new double[WHEELS.length];
        long steadySince = System.nanoTime();
        long deadline = steadySince + 2_000_000_000L;
        while (System.nanoTime() < deadline) {
            failIfTheOpModeDied();
            boolean changed = false;
            boolean commanded = true;
            for (int i = 0; i < WHEELS.length; i++) {
                double power = hardware.motor(WHEELS[i]).getPower();
                changed |= Math.abs(power - last[i]) > 1e-9;
                commanded &= power != 0.0;
                last[i] = power;
            }
            if (changed || !commanded) {
                steadySince = System.nanoTime();
            } else if (System.nanoTime() - steadySince > 50_000_000L) {
                return;
            }
            Thread.yield();
        }
        throw new AssertionError("the wheel powers never settled; they were " + currentWheelPowers());
    }

    private void awaitIntakePower(double expected) {
        await(() -> Math.abs(hardware.motor(IntakeHardware.NAME).getPower() - expected) < 1e-9,
                () -> "the intake never reached " + expected + "; it was "
                        + hardware.motor(IntakeHardware.NAME).getPower());
    }

    private double yaw() {
        return hardware.imu("imu").getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private String currentWheelPowers() {
        StringBuilder powers = new StringBuilder();
        for (String wheel : WHEELS) {
            powers.append(wheel).append('=').append(hardware.motor(wheel).getPower()).append(' ');
        }
        return powers.toString().trim();
    }

    /**
     * The OpMode runs freely on its own thread, so the test waits for the condition it cares about
     * rather than assuming a particular interleaving.
     */
    private void await(java.util.function.BooleanSupplier condition,
                       java.util.function.Supplier<String> message) {
        long deadline = System.nanoTime() + 2_000_000_000L;
        while (System.nanoTime() < deadline) {
            if (condition.getAsBoolean()) {
                return;
            }
            failIfTheOpModeDied();
            Thread.yield();
        }
        throw new AssertionError(message.get());
    }

    private void failIfTheOpModeDied() {
        if (harness.failure() != null) {
            throw new AssertionError("the OpMode thread died", harness.failure());
        }
    }
}
