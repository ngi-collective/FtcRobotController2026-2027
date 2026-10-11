package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.simulated.VerityRobot;
import org.junit.jupiter.api.Test;
import org.ngicollective.testframework.hardware.FakeHardwareMap;
import org.ngicollective.testframework.sim.FieldConfig;
import org.ngicollective.testframework.sim.RobotConfig;
import org.ngicollective.testframework.sim.SimConfigFiles;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * That PedroPathing's follower builds against the simulated drivetrain and its drive-encoder
 * localizer tracks the pose the chassis actually reaches.
 *
 * <p>The localizer is pure dead-reckoning from the four drive encoders, so a correct multiplier
 * means it agrees with the physical pose during free driving &mdash; and this test is the guard
 * that keeps {@code Constants} honest against {@code pickup.json}.</p>
 */
class PedroPathingSimTest {

    private static final double TICK = 0.02;

    private static final double INCHES_PER_METRE = 39.3701;

    private Follower follower;
    private FakeHardwareMap hardware;

    private void drive(int millis, double forward, double strafe, double turn) {
        int steps = millis / 20;
        for (int i = 0; i < steps; i++) {
            follower.setTeleOpDrive(forward, strafe, turn, true);
            follower.update();
            hardware.advance(TICK);
        }
        follower.update();
    }

    private void setUp() {
        RobotConfig config = SimConfigFiles.robot("pickup");
        hardware = new VerityRobot(config, FieldConfig.standard()).create();
        follower = Constants.createFollower(hardware);
        follower.setStartingPose(new Pose(0.0, 0.0, 0.0));
        follower.startTeleopDrive();
        follower.update();
    }

    @Test
    void theFollowerDrivesForwardAndTheLocalizerTracksIt() {
        setUp();
        drive(1000, 1.0, 0.0, 0.0);

        double physicalMetres = hardware.drive().pose().x();
        double localizerMetres = follower.getPose().getX() / INCHES_PER_METRE;

        assertTrue(physicalMetres > 0.3, "the robot should drive forward, got " + physicalMetres);
        assertTrue(localizerMetres > 0.3, "the localizer should report forward, got " + localizerMetres);
        // The localizer is wheel dead-reckoning: during the ~0.3 s standing-start ramp the wheels
        // spin faster than the chassis can grip, so it over-reports by a bounded, fixed amount.
        assertEquals(physicalMetres, localizerMetres, 0.25,
                "the localizer should track the chassis during free driving");
    }

    @Test
    void theFollowerStrafesAndTheLocalizerTracksIt() {
        setUp();
        drive(1000, 0.0, 1.0, 0.0);

        double physicalMetres = hardware.drive().pose().y();
        double localizerMetres = follower.getPose().getY() / INCHES_PER_METRE;

        assertTrue(Math.abs(physicalMetres) > 0.2, "the robot should strafe, got " + physicalMetres);
        assertEquals(physicalMetres, localizerMetres, 0.25,
                "the localizer should track the strafe during free driving");
    }
}
