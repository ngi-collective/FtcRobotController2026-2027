package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.firstinspires.ftc.teamcode.simulated.VerityRobot;
import org.ngicollective.testframework.hardware.FakeHardwareMap;
import org.ngicollective.testframework.harness.LinearOpModeHarness;
import org.ngicollective.testframework.harness.OpModeHarness;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Runs the real {@link iamyou} TeleOp end to end with no robot, no Driver Hub and no emulator.
 *
 * <p>The OpMode is unmodified: it looks its hardware up through {@code hardwareMap}, reads
 * {@code gamepad1}, and pushes telemetry exactly as it does on the field. Everything it touches is
 * simulated by the test framework.</p>
 */
class IamYouHeadlessTest {

    private static final String[] WHEELS = {"FL", "FR", "BL", "BR"};

    private FakeHardwareMap hardware;
    private LinearOpModeHarness harness;

    @BeforeEach
    void setUp() {
        // The same configuration the dashboard drives, so a device rename breaks one place.
        hardware = new VerityRobot().create();
        for (String wheel : WHEELS) {
            // Round numbers keep the encoder assertions below readable.
            hardware.motor(wheel).state().setMaxTicksPerSecond(1000.0);
        }
        harness = OpModeHarness.forLinear(new iamyou(), hardware);
    }

    @AfterEach
    void tearDown() {
        harness.pressStop();
        harness.awaitCompletion(2000);
    }

    @Test
    void initializesTheImuAndTheDriveMotorsBeforeStart() {
        harness.launch();
        awaitInitLoop();

        assertTrue(hardware.imu("imu").isInitialized(), "the OpMode must initialize the IMU");
        for (String wheel : WHEELS) {
            assertEquals(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER,
                    hardware.motor(wheel).getMode(), wheel);
            assertEquals(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE,
                    hardware.motor(wheel).getZeroPowerBehavior(), wheel);
        }
        assertTrue(harness.lastTelemetry().contains("INIT Direction : NORTH"),
                "telemetry was " + harness.lastTelemetry());
    }

    @Test
    void wiresTheDiagonalsSoAForwardStickTurnsEveryWheelForward() {
        harness.launch();
        awaitInitLoop();
        harness.pressStart();

        // Push the left stick fully forward. The SDK reports forward as negative Y.
        harness.gamepad1().left_stick_y = -1.0f;
        awaitWheelPowers(1.0, 1.0, 1.0, 1.0);

        // Left and right are mounted mirrored, so the same commanded power has to spin the physical
        // shafts in opposite senses for the robot to actually go straight.
        assertEquals(-1.0, hardware.motor("FL").state().getPhysicalPower(), 1e-9);
        assertEquals(-1.0, hardware.motor("BL").state().getPhysicalPower(), 1e-9);
        assertEquals(1.0, hardware.motor("FR").state().getPhysicalPower(), 1e-9);
        assertEquals(1.0, hardware.motor("BR").state().getPhysicalPower(), 1e-9);

        hardware.advance(1.0);
        for (String wheel : WHEELS) {
            assertEquals(1000, hardware.motor(wheel).getCurrentPosition(), wheel);
        }
    }

    @Test
    void strafesWhenTheStickGoesSideways() {
        harness.launch();
        awaitInitLoop();
        harness.pressStart();

        harness.gamepad1().left_stick_x = 1.0f;

        // A right strafe is the mecanum X pattern: the diagonals oppose each other.
        awaitWheelPowers(-1.0, 1.0, 1.0, -1.0);
    }

    @Test
    void slowModeScalesEveryWheelDown() {
        harness.launch();
        awaitInitLoop();
        harness.pressStart();

        harness.gamepad1().left_stick_y = -1.0f;
        harness.gamepad1().left_trigger = 1.0f;

        double slow = IamYouLogic.SLOW_MODE_DRIVE_MULT;
        awaitWheelPowers(slow, slow, slow, slow);
    }

    @Test
    void heldHeadingKeepsTheRobotDrivingDownFieldWhenTheChassisRotates() {
        harness.launch();
        awaitInitLoop();
        harness.pressStart();

        harness.gamepad1().left_stick_y = -1.0f;
        awaitWheelPowers(1.0, 1.0, 1.0, 1.0);

        // Spin the chassis a quarter turn without touching the stick: field-centric drive has to
        // turn a forward request into a pure strafe to keep going the same way down the field.
        hardware.imu("imu").state().setYaw(90.0);

        awaitWheelPowers(-1.0, 1.0, 1.0, -1.0);
    }

    @Test
    void stoppingCutsPowerToEveryWheel() {
        harness.launch();
        awaitInitLoop();
        harness.pressStart();
        harness.gamepad1().left_stick_y = -1.0f;
        awaitWheelPowers(1.0, 1.0, 1.0, 1.0);

        harness.pressStop();
        harness.awaitCompletion(2000);

        for (String wheel : WHEELS) {
            assertEquals(0.0, hardware.motor(wheel).getPower(), 1e-9, wheel);
        }
    }

    /**
     * Waits until the OpMode is spinning in its init loop.
     *
     * <p>Anchored on the init loop's own line rather than the one-shot {@code "Status :
     * Initialized"} frame that precedes it: that first frame is immediately overwritten by the
     * loop, so polling the latest frame for it is a race.</p>
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
     * deterministic: the OpMode reads the gamepad fields on its own schedule, so an intermediate
     * loop iteration may well have seen only some of the inputs the test just wrote.</p>
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

    @Test
    void reportsLiveHeadingInTelemetry() {
        harness.launch();
        awaitInitLoop();
        harness.pressStart();
        hardware.imu("imu").state().setYaw(-90.0);

        await(() -> harness.lastTelemetry().contains("Cardinal Direction : EAST"),
                () -> "telemetry never reported the new heading; last was " + harness.lastTelemetry());

        assertEquals(-90.0,
                hardware.imu("imu").getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), 1e-6);
    }
}
