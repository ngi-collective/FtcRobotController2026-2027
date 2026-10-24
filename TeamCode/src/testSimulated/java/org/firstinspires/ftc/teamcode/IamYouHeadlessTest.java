package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
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
 * Runs the real {@link iamyou} TeleOps end to end with no robot, no Driver Hub and no emulator.
 *
 * <p>The OpModes are unmodified: they look their hardware up through {@code hardwareMap}, read
 * the gamepads, and push telemetry exactly as they do on the field. Everything they touch is
 * simulated by the test framework.</p>
 *
 * <p>Driving is asserted on pose, not wheel powers. {@link IamYouLogic} shapes, caps and
 * friction-boosts every command, so the powers are tuning; where the robot goes is the
 * behaviour. {@link IamYouLogicTest} owns the numbers.</p>
 */
class IamYouHeadlessTest {

    private static final String[] WHEELS = {"FL", "FR", "BL", "BR"};
    private static final double TICK = 0.02;

    /** Steady wheel power for a full stick: the speed cap plus the static-friction push. */
    private static final double FULL = iamyou.TRANSLATION_SPEED_CAP + iamyou.STATIC_FRICTION_POWER;

    private FakeHardwareMap hardware;
    private LinearOpModeHarness harness;

    private void launch(iamyou opMode) {
        // The same configuration the dashboard drives, so a device rename breaks one place.
        hardware = new VerityRobot().create();
        harness = OpModeHarness.forLinear(opMode, hardware);
        harness.launch();
        awaitInitLoop();
    }

    private void start(iamyou opMode) {
        launch(opMode);
        harness.pressStart();
    }

    @AfterEach
    void tearDown() {
        if (harness != null) {
            harness.pressStop();
            harness.awaitCompletion(2000);
        }
    }

    @Test
    void initializesTheImuAndTheDriveMotorsBeforeStart() {
        launch(new iamyou());

        assertTrue(hardware.imu("imu").isInitialized(), "the OpMode must initialize the IMU");
        for (String wheel : WHEELS) {
            assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, hardware.motor(wheel).getMode(), wheel);
            assertEquals(DcMotor.ZeroPowerBehavior.BRAKE, hardware.motor(wheel).getZeroPowerBehavior(), wheel);
        }
        assertTrue(harness.lastTelemetry().contains("Controllers : 1"),
                "telemetry was " + harness.lastTelemetry());
    }

    @Test
    void wiresTheDiagonalsSoAForwardStickTurnsEveryWheelForward() {
        start(new iamyou());

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
        awaitWheelPowers(FULL, FULL, FULL, FULL);

        // Left and right are mounted mirrored, so the same commanded power has to spin the physical
        // shafts in opposite senses for the robot to actually go straight.
        assertTrue(hardware.motor("FL").state().getPhysicalPower() < 0);
        assertTrue(hardware.motor("BL").state().getPhysicalPower() < 0);
        assertTrue(hardware.motor("FR").state().getPhysicalPower() > 0);
        assertTrue(hardware.motor("BR").state().getPhysicalPower() > 0);
    }

    @Test
    void strafesWhenTheStickGoesSideways() {
        start(new iamyou());

        harness.gamepad1().left_stick_x = 1.0f;

        // A right strafe is the mecanum X pattern: the diagonals oppose each other. Strafe
        // compensation pushes it past full power, so the mixer scales every wheel back to 1.
        awaitWheelPowers(1.0, -1.0, -1.0, 1.0);
    }

    @Test
    void slowModeScalesEveryWheelDown() {
        start(new iamyou());

        harness.gamepad1().left_stick_y = -1.0f;
        harness.gamepad1().left_trigger = 1.0f;

        double slow = iamyou.TRANSLATION_SPEED_CAP * iamyou.SLOW_MODE_DRIVE_MULT + iamyou.STATIC_FRICTION_POWER;
        awaitWheelPowers(slow, slow, slow, slow);
    }

    @Test
    void heldHeadingKeepsTheRobotDrivingDownFieldWhenTheChassisRotates() {
        start(new iamyou());

        harness.gamepad1().left_stick_y = -1.0f;
        awaitWheelPowers(FULL, FULL, FULL, FULL);

        // Spin the chassis a quarter turn left without touching the stick: field-centric drive has
        // to turn a forward request into a pure right strafe to keep going the same way.
        hardware.imu("imu").state().setYaw(90.0);

        awaitWheelPowers(1.0, -1.0, -1.0, 1.0);
    }

    @Test
    void stoppingCutsPowerToEveryWheel() {
        start(new iamyou());
        harness.gamepad1().left_stick_y = -1.0f;
        awaitWheelPowers(FULL, FULL, FULL, FULL);

        harness.pressStop();
        harness.awaitCompletion(2000);

        for (String wheel : WHEELS) {
            assertEquals(0.0, hardware.motor(wheel).getPower(), 1e-9, wheel);
        }
        assertEquals(0.0, hardware.motor(IntakeHardware.NAME).getPower(), 1e-9);
    }

    @Test
    void reportsLiveHeadingInTelemetry() {
        start(new iamyou());
        hardware.imu("imu").state().setYaw(-90.0);

        await(() -> harness.lastTelemetry().contains("Facing (vs start) : EAST"),
                () -> "telemetry never reported the new heading; last was " + harness.lastTelemetry());

        assertEquals(-90.0,
                hardware.imu("imu").getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), 1e-6);
    }

    // ------------------------------------------------------------------ intake and outtake

    @Test
    void theIntakeServosAndStagingSpinWithTheIntakeMotor() {
        start(new iamyou());

        harness.gamepad1().right_trigger = 1.0f;
        awaitIntake(iamyou.INTAKE_POWER, "intake never reached full power together");

        // Pooping it out runs the motor and Staging backwards; each servo takes its outtake direction.
        harness.gamepad1().right_bumper = true;
        awaitIntake(-iamyou.EJECT_POWER, "intake never ejected together");
    }

    @Test
    void theLeftBumperTogglesTheFlywheel() {
        start(new iamyou());

        // One press turns it on, and it stays on after the bumper is let go.
        harness.gamepad1().left_bumper = true;
        awaitFlywheel(iamyou.OUTTAKE_POWER, "flywheel never reached full power");
        harness.gamepad1().left_bumper = false;
        awaitFlywheelPowerHeldFor(iamyou.OUTTAKE_POWER);

        // The next press turns it off.
        harness.gamepad1().left_bumper = true;
        awaitFlywheel(0.0, "flywheel never stopped");
    }

    @Test
    void withTwoControllersGamepadTwoRunsTheMechanismsAndGamepadOneOnlyDrives() {
        launch(new IamYouTwoController());
        assertTrue(harness.lastTelemetry().contains("Controllers : 2"),
                "telemetry was " + harness.lastTelemetry());
        harness.pressStart();

        // Gamepad 1's mechanism buttons do nothing in this version...
        harness.gamepad1().right_trigger = 1.0f;
        harness.gamepad1().left_bumper = true;
        harness.gamepad1().left_stick_y = -1.0f;
        awaitWheelPowers(FULL, FULL, FULL, FULL);
        assertEquals(0.0, hardware.motor("IntakeM").getPower(), 1e-9);
        assertEquals(0.0, hardware.motor("Outtake").getPower(), 1e-9);

        // ...while gamepad 2's run them.
        harness.gamepad2().right_trigger = 1.0f;
        harness.gamepad2().left_bumper = true;
        await(() -> near(hardware.motor("IntakeM").getPower(), iamyou.INTAKE_POWER)
                        && near(hardware.motor("Outtake").getPower(), iamyou.OUTTAKE_POWER),
                () -> "gamepad 2 never ran the mechanisms; intake " + hardware.motor("IntakeM").getPower()
                        + " flywheel " + hardware.motor("Outtake").getPower());
    }

    private void awaitIntake(double power, String failure) {
        boolean intaking = power >= 0.0;
        double speed = Math.abs(power);
        double left = speed * iamyou.LEFT_SERVO_TRIM
                * (intaking ? iamyou.LEFT_SERVO_INTAKE_DIR : iamyou.LEFT_SERVO_OUTTAKE_DIR);
        double right = speed * iamyou.RIGHT_SERVO_TRIM
                * (intaking ? iamyou.RIGHT_SERVO_INTAKE_DIR : iamyou.RIGHT_SERVO_OUTTAKE_DIR);
        await(() -> near(hardware.motor("IntakeM").getPower(), power)
                        && near(servoPower("IntakeSL"), left)
                        && near(servoPower("IntakeSR"), right)
                        && near(servoPower("Staging"), power),
                () -> failure + "; motor " + hardware.motor("IntakeM").getPower()
                        + " SL " + servoPower("IntakeSL") + " SR " + servoPower("IntakeSR")
                        + " Staging " + servoPower("Staging"));
    }

    private void awaitFlywheel(double power, String failure) {
        await(() -> near(hardware.motor("Outtake").getPower(), power),
                () -> failure + "; it was " + hardware.motor("Outtake").getPower());
    }

    /** The flywheel holds this power for a quarter second of real time: a toggle, not a hold-to-run. */
    private void awaitFlywheelPowerHeldFor(double power) {
        long end = System.nanoTime() + 250_000_000L;
        while (System.nanoTime() < end) {
            assertEquals(power, hardware.motor("Outtake").getPower(), 1e-9, "flywheel changed after release");
            Thread.yield();
        }
    }

    private static boolean near(double actual, double expected) {
        return Math.abs(actual - expected) < 1e-9;
    }

    private double servoPower(String name) {
        return hardware.get(CRServo.class, name).getPower();
    }

    /**
     * Waits until the OpMode is spinning in its init loop.
     *
     * <p>Anchored on the init loop's own line rather than a one-shot frame: polling the latest
     * frame for something the loop immediately overwrites is a race.</p>
     */
    private void awaitInitLoop() {
        await(() -> harness.lastTelemetry().contains("Status : Initialized - waiting for start"),
                () -> "the OpMode never reached its init loop; last telemetry was "
                        + harness.lastTelemetry());
    }

    /**
     * Presses START and waits for the main loop.
     *
     * <p>Waiting on the specific values rather than on "any power at all" is what makes this
     * deterministic: the OpMode reads the gamepad fields on its own schedule and ramps toward the
     * command, so an intermediate loop may well show only part of the way there.</p>
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
