package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.simulated.VerityRobot;
import org.junit.jupiter.api.Test;
import org.ngicollective.testframework.hardware.FakeHardwareMap;
import org.ngicollective.testframework.harness.LinearOpModeHarness;
import org.ngicollective.testframework.harness.OpModeHarness;
import org.ngicollective.testframework.sim.Pose2d;

/**
 * Where the robot actually ends up for a given stick input, driven through the real
 * {@link BasicMecanumTeleOp} against the simulated drivetrain.
 *
 * <p>These assert displacement rather than wheel powers on purpose. A test that pins
 * {@code fl == +1 && fr == -1} passes happily when two motors are swapped or a mounting sign is
 * wrong; only the pose says whether the robot went where the driver asked. The first version of
 * the drive model passed every wheel-power test in this module and still turned a full-forward
 * stick into 106 degrees of rotation, because it read the physical shaft frame without accounting
 * for the mirrored left-side mounting.</p>
 */
class MecanumDrivePoseTest {

    private static final double TICK = 0.02;

    /** Free wheel speed: 312 RPM through 537.7 ticks/rev on a 48 mm wheel. */
    private static final double TOP_SPEED = 2796.0 / 537.7 * 2 * Math.PI * 0.048;

    private FakeHardwareMap hardware;
    private LinearOpModeHarness harness;

    private void stick(double leftX, double leftY, double rightX) {
        hardware = new VerityRobot().create();
        harness = OpModeHarness.forLinear(new BasicMecanumTeleOp(), hardware);
        harness.launch();
        harness.pressStart();
        harness.gamepad1().left_stick_x = (float) leftX;
        harness.gamepad1().left_stick_y = (float) leftY;
        harness.gamepad1().right_stick_x = (float) rightX;

        // The OpMode runs on its own thread; pump until it has reacted to the sticks, so what is
        // measured below is the drive model and not the scheduler.
        for (int spin = 0; spin < 100 && hardware.motor("FL").state().getVelocity() == 0.0; spin++) {
            harness.advance(TICK);
            harness.eventLoopIteration();
        }
    }

    private Pose2d after(double seconds) {
        for (int tick = 0; tick < Math.round(seconds / TICK); tick++) {
            harness.advance(TICK);
            harness.eventLoopIteration();
        }
        Pose2d pose = hardware.drive().pose();
        harness.pressStop();
        System.out.printf("x=%+.3f y=%+.3f heading=%+.1f deg  imu=%+.1f deg%n",
                pose.x(), pose.y(), pose.headingDegrees(),
                hardware.imu("imu").state().reportedYaw());
        return pose;
    }

    @Test
    void forwardStickDrivesAlongPlusX() {
        stick(0.0, -1.0, 0.0);              // stick Y is negative pushed away from the driver
        Pose2d pose = after(0.5);
        assertTrue(pose.x() > 0.9 * TOP_SPEED * 0.5, "expected full-speed travel along +X");
        assertTrue(Math.abs(pose.y()) < 0.005, "expected no lateral drift, got " + pose.y());
        assertTrue(Math.abs(pose.headingDegrees()) < 0.5, "expected no rotation");
    }

    @Test
    void strafeRightTravelsTowardMinusY() {
        stick(1.0, 0.0, 0.0);
        Pose2d pose = after(0.5);
        // 0.8 strafe efficiency: the rollers scrub, so sideways is slower than forward.
        assertTrue(pose.y() < -0.9 * 0.8 * TOP_SPEED * 0.5, "expected full-speed travel toward -Y");
        assertTrue(pose.y() > -TOP_SPEED * 0.5, "strafe must be slower than driving forward");
        assertTrue(Math.abs(pose.x()) < 0.005, "expected no forward drift, got " + pose.x());
    }

    @Test
    void twistRotatesClockwiseInPlace() {
        stick(0.0, 0.0, 1.0);
        Pose2d pose = after(0.5);
        assertTrue(pose.headingDegrees() < -30.0, "expected clockwise rotation");
        assertTrue(Math.hypot(pose.x(), pose.y()) < 0.005, "expected rotation in place");
    }

    @Test
    void wallStopsTheRobotButNotTheEncoders() {
        stick(0.0, -1.0, 0.0);
        Pose2d pose = after(4.0);
        double ticks = hardware.motor("FL").state().getPosition();
        System.out.printf("4 s at full forward -> x=%.3f contact=%s FL encoder=%.0f ticks%n",
                pose.x(), hardware.drive().inWallContact(), ticks);
        assertTrue(hardware.drive().inWallContact(), "expected wall contact");
        assertTrue(pose.x() < 1.6, "expected the footprint to stay inside the field");
        assertTrue(Math.abs(ticks) > 5000, "expected encoders to keep counting while pinned");
    }
}
