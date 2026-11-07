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

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Runs the real {@link iamyou} TeleOps end to end with no robot, no Driver Hub and no emulator.
 *
 * <p>The OpModes are unmodified: they look their hardware up through {@code hardwareMap}, read
 * the gamepads, and push telemetry exactly as they do on the field. Everything they touch is
 * simulated by the test framework.</p>
 */
class IamYouHeadlessTest {

    private static final String[] WHEELS = {"FL", "FR", "BL", "BR"};

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

        // Push the left stick fully forward. The SDK reports forward as negative Y.
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
     * Polls until all four wheels carry exactly these commanded powers.
     *
     * <p>Waiting on the specific values rather than on "any power at all" is what makes this
     * deterministic: the OpMode reads the gamepad fields on its own schedule and ramps toward the
     * command, so an intermediate loop may well show only part of the way there.</p>
     */
    private void awaitWheelPowers(double fl, double fr, double bl, double br) {
        double[] expected = {fl, fr, bl, br};
        await(() -> {
            for (int i = 0; i < WHEELS.length; i++) {
                if (Math.abs(hardware.motor(WHEELS[i]).getPower() - expected[i]) > 1e-9) {
                    return false;
                }
            }
            return true;
        }, () -> "wheels never reached " + java.util.Arrays.toString(expected) + "; they were "
                + currentWheelPowers());
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
            if (harness.failure() != null) {
                throw new AssertionError("the OpMode thread died", harness.failure());
            }
            Thread.yield();
        }
        throw new AssertionError(message.get());
    }
}
