package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.simulated.VerityRobot;
import org.junit.jupiter.api.Test;
import org.ngicollective.ftcsim.hardware.FakeHardwareMap;
import org.ngicollective.ftcsim.sim.FieldConfig;
import org.ngicollective.ftcsim.sim.Pose2d;
import org.ngicollective.ftcsim.sim.RobotConfig;
import org.ngicollective.ftcsim.sim.SimConfigFiles;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * That Pedro Pathing 3.x's follower, built by {@link Constants}, drives the simulated robot and
 * that its Pinpoint localizer tracks the pose the chassis actually reaches.
 *
 * <p>The Pinpoint's pods roll on the floor, so unlike wheel dead-reckoning they are not fooled by
 * the standing-start slip: with {@code Constants} told the same pod mount as {@code pickup.json},
 * the localizer agrees with the physical pose to within millimetres. A disagreement here means
 * the two have drifted apart.</p>
 */
class PedroPathingSimTest {

    private static final double TICK = 0.02;

    private static final double INCHES_PER_METRE = 39.3701;

    private Follower follower;
    private FakeHardwareMap hardware;

    private void setUp() {
        RobotConfig config = SimConfigFiles.robot("pickup");
        hardware = new VerityRobot(config, FieldConfig.standard()).create();
        hardware.drive().setPose(Pose2d.ORIGIN);
        follower = Constants.createFollower(hardware);
        follower.setPose(new Pose(0.0, 0.0, 0.0));
        follower.update(TICK);
    }

    /** One control cycle: the follower commands the wheels, then simulated time passes. */
    private void cycle() {
        follower.update(TICK);
        hardware.advance(TICK);
    }

    private void manual(int millis, double forward, double strafe, double turn) {
        follower.manual(forward, strafe, turn);
        for (int i = 0; i < millis / 20; i++) {
            cycle();
        }
        follower.update(TICK);
    }

    @Test
    void manualDriveForwardIsTrackedByThePinpoint() {
        setUp();
        manual(1000, 1.0, 0.0, 0.0);

        double physicalMetres = hardware.drive().pose().x();
        double localizerMetres = follower.pose().x() / INCHES_PER_METRE;

        assertTrue(physicalMetres > 0.3, "the robot should drive forward, got " + physicalMetres);
        assertEquals(physicalMetres, localizerMetres, 0.005,
                "the Pinpoint should track the chassis, standing start included");
    }

    @Test
    void manualStrafeIsTrackedByThePinpoint() {
        setUp();
        manual(1000, 0.0, 1.0, 0.0);

        double physicalMetres = hardware.drive().pose().y();
        double localizerMetres = follower.pose().y() / INCHES_PER_METRE;

        assertTrue(physicalMetres > 0.2, "positive strafe is left in Pedro 3.x, got "
                + physicalMetres);
        assertEquals(physicalMetres, localizerMetres, 0.005,
                "the Pinpoint should track the strafe");
    }

    @Test
    void manualTurnIsTrackedByThePinpoint() {
        setUp();
        manual(500, 0.0, 0.0, 0.5);

        double physical = hardware.drive().pose().heading();

        assertTrue(physical > 0.3, "positive turn is counter-clockwise, got " + physical);
        assertEquals(physical, follower.pose().heading(), 0.01);
        assertEquals(0.0, follower.pose().x() / INCHES_PER_METRE, 0.005,
                "a turn in place is not travel when the pod offsets are right");
    }

    @Test
    void theFollowerDrivesALineToItsEnd() {
        setUp();
        Pose start = new Pose(0.0, 0.0, 0.0);
        Pose end = new Pose(24.0, 0.0, 0.0);
        follower.follow(Paths.line(start, end).linear(start, end));

        for (int i = 0; i < 250 && follower.isBusy(); i++) {
            cycle();
        }

        assertFalse(follower.isBusy(), "the follower should finish a 24 in line inside 5 s");
        Pose2d truth = hardware.drive().pose();
        assertEquals(24.0, truth.x() * INCHES_PER_METRE, 1.0);
        assertEquals(0.0, truth.y() * INCHES_PER_METRE, 1.0);
        assertEquals(0.0, truth.heading(), Math.toRadians(3.0));
    }
}
