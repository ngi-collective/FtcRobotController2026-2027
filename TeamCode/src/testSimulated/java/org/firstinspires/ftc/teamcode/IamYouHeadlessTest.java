package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.junit.After;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.firstinspires.ftc.teamcode.simulated.VerityRobot;
import org.ngicollective.ftcsim.app.PlainJvmVision;
import org.ngicollective.ftcsim.hardware.FakeHardwareMap;
import org.ngicollective.ftcsim.harness.LinearOpModeHarness;
import org.ngicollective.ftcsim.harness.OpModeHarness;
import org.ngicollective.ftcsim.sim.Pose2d;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Runs the real {@link iamyou} TeleOp end to end with no robot, no Driver Hub and no emulator.
 *
 * <p>The OpMode is unmodified: it looks its hardware up through {@code hardwareMap}, reads
 * {@code gamepad1}, and pushes telemetry exactly as it does on the field. Everything it touches is
 * simulated by the test framework.</p>
 *
 * <p>Driving is asserted on pose, not wheel powers. The drive in {@link iamyou} deadzones, scales and
 * friction-boosts every command, so the powers are tuning; where the robot goes is the
 * behaviour. {@link IamYouTest} owns the numbers.</p>
 */
@RunWith(RobolectricTestRunner.class)
@Config(sdk = 28)
public class IamYouHeadlessTest {

    private static final String[] WHEELS = {"FL", "FR", "BL", "BR"};
    private static final double TICK = 0.02;

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
    public void initializesTheImuTheDriveMotorsAndTheIntakeBeforeStart() {
        launch();

        // Heading comes from the Pinpoint, which launch() has already watched finish calibrating.
        assertTrue(harness.lastTelemetry().stream()
                        .anyMatch(line -> line.startsWith("Heading Source : Pinpoint")),
                "telemetry was " + harness.lastTelemetry());
        for (String wheel : WHEELS) {
            // Plain power on purpose: iamyou reads the encoders for its stop assist but drives
            // without the hub's velocity PID.
            assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, hardware.motor(wheel).getMode(), wheel);
            assertEquals(DcMotor.ZeroPowerBehavior.BRAKE,
                    hardware.motor(wheel).getZeroPowerBehavior(), wheel);
        }
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                hardware.motor(iamyou.INTAKE_NAME).getMode());
        assertEquals(DcMotor.ZeroPowerBehavior.BRAKE,
                hardware.motor(iamyou.INTAKE_NAME).getZeroPowerBehavior());
        // A line, not the whole frame: the position is followed by the D-pad legend.
        await(() -> harness.lastTelemetry().stream()
                        .anyMatch(line -> line.startsWith("Driver Position : SOUTH")),
                () -> "telemetry never reported the driver position; last was "
                        + harness.lastTelemetry());
    }

    @Test
    public void forwardStickDrivesTheWayTheRobotFacedAtStart() {
        launchAndStart();
        Pose2d pose = drive(0.0, -1.0, 0.0f, 0.5);

        assertTrue(pose.x() > 0.2, "expected the robot to drive along +X, got " + pose);
        assertTrue(Math.abs(pose.y()) < 0.1 * pose.x(), "expected no lateral drift, got " + pose);
        assertTrue(Math.abs(pose.headingDegrees()) < 2.0, "expected no rotation, got " + pose);
    }

    @Test
    public void rightStickStrafesTowardTheRobotsRight() {
        launchAndStart();
        Pose2d pose = drive(1.0, 0.0, 0.0f, 0.5);

        // Facing +X, the robot's right is -Y.
        assertTrue(pose.y() < -0.15, "expected the robot to strafe toward -Y, got " + pose);
        assertTrue(Math.abs(pose.x()) < 0.1 * -pose.y(), "expected no forward drift, got " + pose);
        assertTrue(Math.abs(pose.headingDegrees()) < 2.0, "expected no rotation, got " + pose);
    }

    @Test
    public void slowModeDrivesWellUnderHalfSpeed() {
        launchAndStart();
        double full = drive(0.0, -1.0, 0.0f, 0.5).x();
        harness.pressStop();
        harness.awaitCompletion(2000);

        launchAndStart();
        double slow = drive(0.0, -1.0, 1.0f, 0.5).x();

        assertTrue(slow > 0.0, "slow mode should still drive, got " + slow);
        assertTrue(slow < 0.5 * full, "slow mode covered " + slow + " m against " + full + " m");
    }

    /**
     * The whole point of field-centric: wherever the robot has turned to, forward on the stick drives the way it
     * faced at START (+X), and right on the stick drives toward that start direction's right (-Y).
     *
     * <p>The robot is turned by driving it, not by placing it: a Pinpoint measures turns its gyro feels, and a robot
     * picked up and put down turned (setPose) has not turned as far as the Pinpoint knows.</p>
     */
    @Test
    public void sticksDriveTheSameFieldDirectionsAtEveryHeading() {
        launchAndStart();
        for (double target : new double[]{90.0, 180.0, -90.0, 0.0, 45.0}) {
            turnTo(target);
            double heading = hardware.drive().pose().heading();
            String at = " at heading " + Math.round(Math.toDegrees(heading));

            // Back to the middle of the field (placed, so the Pinpoint's heading is untouched), clear of the walls.
            hardware.drive().setPose(new Pose2d(0.0, 0.0, heading));
            Pose2d forward = drive(0.0, -1.0, 0.0f, 0.4);
            release();
            hardware.drive().setPose(new Pose2d(0.0, 0.0, heading));
            assertTrue(forward.x() > 0.15, "forward should drive along +X" + at + ", got " + forward);
            assertTrue(Math.abs(forward.y()) < 0.15 * forward.x(), "forward drifted along Y" + at + ": " + forward);
            assertTrue(Math.abs(forward.headingDegrees()) < 5.0, "forward turned the robot" + at + ": " + forward);

            Pose2d right = drive(1.0, 0.0, 0.0f, 0.4);
            release();
            assertTrue(right.y() < -0.15, "right should drive along -Y" + at + ", got " + right);
            assertTrue(Math.abs(right.x()) < 0.15 * -right.y(), "right drifted along X" + at + ": " + right);
        }
    }

    @Test
    public void withTheFlywheelOnItTurnsOntoThePickedAlliancesRaisedCell() throws InterruptedException {
        launch();
        harness.gamepad1().b = true;
        await(() -> harness.lastTelemetry().stream().anyMatch(line -> line.startsWith("Alliance : RED")),
                () -> "B before START should pick RED; last telemetry was " + harness.lastTelemetry());
        harness.gamepad1().b = false;
        harness.pressStart();
        await(() -> harness.lastTelemetry().stream().anyMatch(line -> line.startsWith("Aim : ")),
                () -> "the OpMode never reached its main loop; last telemetry was " + harness.lastTelemetry());

        // About 0.9 m short of red's raised CELL and fifteen degrees clockwise of it, well inside the 0.46-2 m band
        // the forward, 45-degree camera frames the cluster in.
        hardware.drive().setPose(new Pose2d(-0.324, -1.30, Math.toRadians(75.0)));
        // Flywheel on (gamepad 2, the operator's), and X on gamepad 1 to turn the aimer on: both toggles, so press and let go.
        harness.gamepad2().left_bumper = true;
        setX(true);
        for (int tick = 0; tick < 500 && !aimSettled(); tick++) {
            if (tick == 5) {
                harness.gamepad2().left_bumper = false;
                setX(false);
            }
            harness.advance(TICK);
            PlainJvmVision.pump();
            Thread.sleep(20);
        }

        assertTrue(aimSettled(), "the aimer never settled; last telemetry was " + harness.lastTelemetry());
        // Red's raised CELL is at x = -0.324 on the HIVE's centre line, so on target is facing straight down +Y.
        assertEquals(90.0, hardware.drive().pose().headingDegrees(), CellAimer.READY_DEGREES,
                "the nose should point at the raised CELL");
    }

    private boolean aimSettled() {
        return harness.lastTelemetry().stream().anyMatch(line -> line.startsWith("Aim : ON TARGET"));
    }

    /**
     * Sets X the way the Driver Station does. {@code xWasPressed()} only sees edges delivered through
     * {@link Gamepad#fromByteArray}; writing the field directly never registers a press.
     */
    private void setX(boolean pressed) {
        Gamepad state = new Gamepad();
        state.copy(harness.gamepad1());
        state.x = pressed;
        harness.gamepad1().fromByteArray(state.toByteArray());
    }

    @Test
    public void reportsWhichWayTheRobotFacesInTelemetry() {
        launchAndStart();
        turnTo(-90.0);

        await(() -> harness.lastTelemetry().contains("Facing (vs start) : EAST"),
                () -> "telemetry never reported the new heading; last was " + harness.lastTelemetry());
    }

    @Test
    public void optionsMakesTheCurrentDirectionNorth() {
        launchAndStart();
        turnTo(90.0);

        harness.gamepad1().options = true;
        await(() -> harness.lastTelemetry().contains("Facing (vs start) : NORTH"),
                () -> "Options never re-zeroed the heading; last was " + harness.lastTelemetry());
        harness.gamepad1().options = false;

        // Forward now drives the way the robot faces (+Y), not the original start direction.
        Pose2d forward = drive(0.0, -1.0, 0.0f, 0.4);
        assertTrue(forward.y() > 0.15, "forward should now drive along +Y, got " + forward);
        assertTrue(Math.abs(forward.x()) < 0.15 * forward.y(), "forward drifted along X: " + forward);
    }

    @Test
    public void operatorTriggerRunsTheIntakeInAndBumperEjects() {
        launchAndStart();

        harness.gamepad2().right_trigger = 1.0f;
        awaitIntakePower(iamyou.INTAKE_POWER);

        // Eject wins over a held trigger.
        harness.gamepad2().right_bumper = true;
        awaitIntakePower(-iamyou.EJECT_POWER);

        harness.gamepad2().right_bumper = false;
        harness.gamepad2().right_trigger = 0.0f;
        awaitIntakePower(0.0);
    }

    @Test
    public void stoppingCutsPowerToEveryWheelAndTheIntake() {
        launchAndStart();
        harness.gamepad1().left_stick_y = -1.0f;
        harness.gamepad2().right_trigger = 1.0f;
        awaitSteadyWheelPowers();
        awaitIntakePower(iamyou.INTAKE_POWER);

        harness.pressStop();
        harness.awaitCompletion(2000);

        for (String wheel : WHEELS) {
            assertEquals(0.0, hardware.motor(wheel).getPower(), 1e-9, wheel);
        }
        assertEquals(0.0, hardware.motor(iamyou.INTAKE_NAME).getPower(), 1e-9);
    }

    /** A fresh robot at the origin facing +X, its OpMode spinning in the init loop. */
    private void launch() {
        // iamyou opens the webcam to aim with, so the vision natives and the simulated camera have to be in place:
        // under Robolectric, as every test that builds a VisionPortal is. iamyou closes it again on STOP.
        try {
            PlainJvmVision.start();
        } catch (Exception e) {
            throw new IllegalStateException("the plain-JVM vision stack would not start", e);
        }
        hardware = new VerityRobot().create();
        // iamyou multiplies every Pinpoint turn by PINPOINT_TURN_SCALE. The simulated gyro is perfect, so it is made to
        // misread by the inverse, through the device's own yaw-scale register, for the correction to have something
        // to correct whatever the constant is set to.
        hardware.get(GoBildaPinpointDriver.class, "pinpoint").setYawScalar(1.0 / iamyou.PINPOINT_TURN_SCALE);
        hardware.drive().setPose(Pose2d.ORIGIN);
        harness = OpModeHarness.forLinear(new iamyou(), hardware);
        harness.launch();
        await(() -> harness.lastTelemetry().contains("Status : Initialized - waiting for start"),
                () -> "the OpMode never reached its init loop; last telemetry was "
                        + harness.lastTelemetry());
        // The Pinpoint recalibrates its gyro at INIT, in simulated time; a driver waits for READY, so the tests do.
        for (int tick = 0; tick < 50 && !pinpointReady(); tick++) {
            harness.advance(TICK);
            Thread.yield();
        }
        await(this::pinpointReady,
                () -> "the Pinpoint never reported READY; last telemetry was " + harness.lastTelemetry());
    }

    private boolean pinpointReady() {
        return harness.lastTelemetry().stream().anyMatch(line -> line.startsWith("Pinpoint : READY"));
    }

    /**
     * Turns the robot by driving it (right stick) until it faces within a few degrees of {@code degrees}, then lets
     * go and lets it stop. Where it ends up exactly does not matter: field-centric must work at whatever heading the
     * Pinpoint reports.
     */
    private void turnTo(double degrees) {
        for (int tick = 0; tick < 400; tick++) {
            double error = AngleUnit.normalizeDegrees(degrees - hardware.drive().pose().headingDegrees());
            if (Math.abs(error) < 4.0) {
                break;
            }
            // Positive right stick turns clockwise, which is a falling heading.
            harness.gamepad1().right_stick_x = (float) (error > 0 ? -0.4 : 0.4);
            step();
        }
        release();
    }

    /** Lets go of every stick and lets the robot come to rest. */
    private void release() {
        harness.gamepad1().left_stick_x = 0.0f;
        harness.gamepad1().left_stick_y = 0.0f;
        harness.gamepad1().right_stick_x = 0.0f;
        harness.gamepad1().left_trigger = 0.0f;
        for (int tick = 0; tick < 25; tick++) {
            step();
        }
    }

    /**
     * One tick of world time, then a tick of wall time for the OpMode to see it.
     *
     * <p>The OpMode reads its sensors and steers on its own thread. Stepping the world without waiting let it run
     * ahead of the OpMode on a loaded machine: the robot kept turning after the OpMode's last heading read, or drove
     * on commands mixed for a heading it had already left, and field-centric drifted ten degrees off line.</p>
     */
    private void step() {
        harness.advance(TICK);
        harness.eventLoopIteration();
        try {
            Thread.sleep((long) (TICK * 1000.0));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new AssertionError("interrupted while stepping", e);
        }
    }

    /**
     * Presses START and waits for the main loop.
     *
     * <p>iamyou zeroes the heading after START, so a test that turns the chassis before that reset
     * lands has its turn become NORTH. The loop's own telemetry is the first thing that comes
     * after the reset.</p>
     */
    private void launchAndStart() {
        launch();
        harness.pressStart();
        await(() -> harness.lastTelemetry().stream()
                        .anyMatch(line -> line.startsWith("Facing (vs start) : ")),
                () -> "the OpMode never reached its main loop; last telemetry was "
                        + harness.lastTelemetry());
    }

    /**
     * Holds the sticks, waits for the OpMode's command to settle, then drives for {@code seconds}
     * of simulated time from where the robot stands.
     *
     * <p>The wait matters: the OpMode reads the gamepad on its own thread, and simulated time advancing before it
     * has would measure a robot still obeying the previous sticks.</p>
     */
    private Pose2d drive(double leftX, double leftY, float leftTrigger, double seconds) {
        harness.gamepad1().left_stick_x = (float) leftX;
        harness.gamepad1().left_stick_y = (float) leftY;
        harness.gamepad1().left_trigger = leftTrigger;
        awaitSteadyWheelPowers();

        Pose2d start = hardware.drive().pose();
        for (int tick = 0; tick < Math.round(seconds / TICK); tick++) {
            step();
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
        await(() -> Math.abs(hardware.motor(iamyou.INTAKE_NAME).getPower() - expected) < 1e-9,
                () -> "the intake never reached " + expected + "; it was "
                        + hardware.motor(iamyou.INTAKE_NAME).getPower());
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
