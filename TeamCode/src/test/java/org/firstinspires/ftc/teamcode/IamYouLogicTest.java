package org.firstinspires.ftc.teamcode;

import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class IamYouLogicTest {

    private static final double DT = 0.02;          // 50 Hz loop
    private static final double EPS = 1e-6;
    private static final IamYouLogic.DriverPosition SOUTH = IamYouLogic.DriverPosition.SOUTH;

    /** Raw gamepad values: stick up is NEGATIVE y. Wheel velocities are all set to the same value. */
    private static IamYouLogic.DriveParameters input(double stickX, double stickY, double twist, double trigger,
                                                     double heading, IamYouLogic.DriverPosition pos,
                                                     double wheelVel) {
        return new IamYouLogic.DriveParameters(stickX, stickY, twist, trigger, heading, pos,
                wheelVel, wheelVel, wheelVel, wheelVel, DT);
    }

    /** Holds the inputs for 2 seconds so the acceleration limiter has fully settled. */
    private static IamYouLogic.DriveResult hold(IamYouLogic logic, double stickX, double stickY, double twist,
                                                double trigger, double heading, IamYouLogic.DriverPosition pos) {
        IamYouLogic.DriveResult result = null;
        for (int i = 0; i < 100; i++) {
            result = logic.calculate(input(stickX, stickY, twist, trigger, heading, pos, 0.0));
        }
        return result;
    }

    private static double angleDiff(double a, double b) {
        return Math.atan2(Math.sin(a - b), Math.cos(a - b));
    }

    // ------------------------------------------------------------------ cardinals

    @Test
    void testSnapToCardinal() {
        assertEquals(IamYouLogic.Cardinal.NORTH, IamYouLogic.snapToCardinal(0));
        assertEquals(IamYouLogic.Cardinal.NORTH, IamYouLogic.snapToCardinal(44));
        assertEquals(IamYouLogic.Cardinal.NORTH, IamYouLogic.snapToCardinal(-44));
        assertEquals(IamYouLogic.Cardinal.NORTH, IamYouLogic.snapToCardinal(400));   // wraps to 40

        // yaw is counter-clockwise positive, so +90 is a left turn = WEST
        assertEquals(IamYouLogic.Cardinal.WEST, IamYouLogic.snapToCardinal(90));
        assertEquals(IamYouLogic.Cardinal.WEST, IamYouLogic.snapToCardinal(46));
        assertEquals(IamYouLogic.Cardinal.WEST, IamYouLogic.snapToCardinal(134));

        assertEquals(IamYouLogic.Cardinal.EAST, IamYouLogic.snapToCardinal(-90));
        assertEquals(IamYouLogic.Cardinal.EAST, IamYouLogic.snapToCardinal(-46));
        assertEquals(IamYouLogic.Cardinal.EAST, IamYouLogic.snapToCardinal(-134));

        assertEquals(IamYouLogic.Cardinal.SOUTH, IamYouLogic.snapToCardinal(180));
        assertEquals(IamYouLogic.Cardinal.SOUTH, IamYouLogic.snapToCardinal(-180));
        assertEquals(IamYouLogic.Cardinal.SOUTH, IamYouLogic.snapToCardinal(-179));
        assertEquals(IamYouLogic.Cardinal.SOUTH, IamYouLogic.snapToCardinal(136));
        assertEquals(IamYouLogic.Cardinal.SOUTH, IamYouLogic.snapToCardinal(-136));
    }

    // ------------------------------------------------------------------ basic driving

    @Test
    void testForwardDrivesAllWheelsForward() {
        // Heading 0, driver SOUTH (baseline), stick pushed fully up.
        IamYouLogic.DriveResult r = hold(new IamYouLogic(), 0.0, -1.0, 0.0, 0.0, 0.0, SOUTH);

        double expected = IamYouLogic.TRANSLATION_SPEED_CAP + IamYouLogic.STATIC_FRICTION_POWER;
        assertEquals(expected, r.powers.getFL(), EPS);
        assertEquals(expected, r.powers.getFR(), EPS);
        assertEquals(expected, r.powers.getBL(), EPS);
        assertEquals(expected, r.powers.getBR(), EPS);
        assertFalse(r.braking);
    }

    @Test
    void testStrafeRight() {
        IamYouLogic.DriveResult r = hold(new IamYouLogic(), 1.0, 0.0, 0.0, 0.0, 0.0, SOUTH);

        assertTrue(r.powers.getFL() > 0);
        assertTrue(r.powers.getFR() < 0);
        assertTrue(r.powers.getBL() < 0);
        assertTrue(r.powers.getBR() > 0);
        assertEquals(r.powers.getFL(), r.powers.getBR(), EPS);
        assertEquals(r.powers.getFR(), r.powers.getBL(), EPS);
    }

    @Test
    void testTurnRightIsClockwise() {
        // Right stick pushed fully right: left wheels forward, right wheels backward.
        IamYouLogic.DriveResult r = hold(new IamYouLogic(), 0.0, 0.0, 1.0, 0.0, 0.0, SOUTH);

        assertEquals(1.0, r.powers.getFL(), EPS);
        assertEquals(-1.0, r.powers.getFR(), EPS);
        assertEquals(1.0, r.powers.getBL(), EPS);
        assertEquals(-1.0, r.powers.getBR(), EPS);
    }

    @Test
    void testSlowModeReducesSpeed() {
        IamYouLogic.DriveResult normal = hold(new IamYouLogic(), 0.0, -1.0, 0.0, 0.0, 0.0, SOUTH);
        IamYouLogic.DriveResult slow = hold(new IamYouLogic(), 0.0, -1.0, 0.0, 1.0, 0.0, SOUTH);

        assertTrue(slow.slowMode);
        assertFalse(normal.slowMode);
        assertTrue(slow.powers.getFL() > 0);
        assertTrue(slow.powers.getFL() < normal.powers.getFL());
    }

    // ------------------------------------------------------------------ field-centric

    @Test
    void testHeadingRotatesFieldForwardIntoRobotFrame() {
        // Robot has turned to face EAST (clockwise, so yaw = -90). Stick forward means "field north",
        // which is now the robot's LEFT, so it must strafe left.
        IamYouLogic.DriveResult r = hold(new IamYouLogic(), 0.0, -1.0, 0.0, 0.0, -90.0, SOUTH);

        assertTrue(r.powers.getFL() < 0);
        assertTrue(r.powers.getFR() > 0);
        assertTrue(r.powers.getBL() > 0);
        assertTrue(r.powers.getBR() < 0);
    }

    @Test
    void testRobotTurnedAroundDrivesBackwards() {
        // Robot faces SOUTH (heading 180). Stick forward = field north = robot's back.
        IamYouLogic.DriveResult r = hold(new IamYouLogic(), 0.0, -1.0, 0.0, 0.0, 180.0, SOUTH);

        assertTrue(r.powers.getFL() < 0);
        assertTrue(r.powers.getFR() < 0);
        assertTrue(r.powers.getBL() < 0);
        assertTrue(r.powers.getBR() < 0);
    }

    @Test
    void testDriverAtNorthReversesControls() {
        // Driver stands in front of the robot looking south, so "away from me" is field south.
        IamYouLogic.DriveResult r = hold(new IamYouLogic(), 0.0, -1.0, 0.0, 0.0, 0.0,
                IamYouLogic.DriverPosition.NORTH);

        assertTrue(r.powers.getFL() < 0);
        assertTrue(r.powers.getFR() < 0);
        assertTrue(r.powers.getBL() < 0);
        assertTrue(r.powers.getBR() < 0);
    }

    @Test
    void testDriverAtEastSendsForwardStickToTheLeft() {
        // Driver on the east side looking west: "away from me" is field west = robot's left at heading 0.
        IamYouLogic.DriveResult r = hold(new IamYouLogic(), 0.0, -1.0, 0.0, 0.0, 0.0,
                IamYouLogic.DriverPosition.EAST);

        assertTrue(r.powers.getFL() < 0);
        assertTrue(r.powers.getFR() > 0);
        assertTrue(r.powers.getBL() > 0);
        assertTrue(r.powers.getBR() < 0);
    }

    @Test
    void testRobotFollowsStickAngle() {
        // Guards against the "snaps to an axis" problem: at 60% stick, the direction the robot moves
        // must stay within 10 degrees of the direction the stick points, at every angle.
        for (double deg = 0; deg < 360; deg += 15) {
            double sx = Math.sin(Math.toRadians(deg)) * 0.6;
            double sy = Math.cos(Math.toRadians(deg)) * 0.6;
            IamYouLogic.DriveResult r = hold(new IamYouLogic(), sx, -sy, 0.0, 0.0, 0.0, SOUTH);

            double forward = (r.powers.getFL() + r.powers.getFR() + r.powers.getBL() + r.powers.getBR()) / 4.0;
            double right = (r.powers.getFL() - r.powers.getFR() - r.powers.getBL() + r.powers.getBR()) / 4.0
                    / IamYouLogic.STRAFE_COMPENSATION;
            double error = Math.abs(angleDiff(Math.atan2(right, forward), Math.atan2(sx, sy)));

            assertTrue(Math.toDegrees(error) < 10.0,
                    "stick angle " + deg + " drove " + Math.toDegrees(error) + " degrees off");
        }
    }

    // ------------------------------------------------------------------ smoothing and limits

    @Test
    void testAccelerationIsLimited() {
        IamYouLogic logic = new IamYouLogic();
        IamYouLogic.DriveResult first = logic.calculate(input(0.0, -1.0, 0.0, 0.0, 0.0, SOUTH, 0.0));

        double maxStep = IamYouLogic.MAX_TRANSLATION_CHANGE_PER_SEC * DT;
        assertTrue(first.stickForward > 0);
        assertTrue(first.stickForward <= maxStep + 1e-9);
    }

    @Test
    void testDeadzoneIgnoresTinyStickInput() {
        IamYouLogic.DriveResult r = hold(new IamYouLogic(), 0.03, -0.03, 0.03, 0.0, 0.0, SOUTH);

        assertEquals(0.0, r.powers.getFL(), 1e-9);
        assertEquals(0.0, r.powers.getFR(), 1e-9);
        assertEquals(0.0, r.powers.getBL(), 1e-9);
        assertEquals(0.0, r.powers.getBR(), 1e-9);
    }

    @Test
    void testWheelPowerNeverExceedsOne() {
        IamYouLogic logic = new IamYouLogic();
        IamYouLogic.DriverPosition[] positions = IamYouLogic.DriverPosition.values();
        Random random = new Random(1);

        for (int i = 0; i < 5000; i++) {
            IamYouLogic.DriveResult r = logic.calculate(input(
                    random.nextDouble() * 2 - 1, random.nextDouble() * 2 - 1, random.nextDouble() * 2 - 1,
                    random.nextDouble(), random.nextDouble() * 360 - 180,
                    positions[random.nextInt(positions.length)], 0.0));
            assertTrue(Math.abs(r.powers.getFL()) <= 1.0 + EPS);
            assertTrue(Math.abs(r.powers.getFR()) <= 1.0 + EPS);
            assertTrue(Math.abs(r.powers.getBL()) <= 1.0 + EPS);
            assertTrue(Math.abs(r.powers.getBR()) <= 1.0 + EPS);
        }
    }

    @Test
    void testIdleSendsExactlyZero() {
        IamYouLogic logic = new IamYouLogic();
        for (int i = 0; i < 50; i++) {
            IamYouLogic.DriveResult r = logic.calculate(input(0.0, 0.0, 0.0, 0.0, 0.0, SOUTH, 0.0));
            assertEquals(0.0, r.powers.getFL(), 1e-9);
            assertEquals(0.0, r.powers.getFR(), 1e-9);
            assertEquals(0.0, r.powers.getBL(), 1e-9);
            assertEquals(0.0, r.powers.getBR(), 1e-9);
        }
    }

    // ------------------------------------------------------------------ stop assist

    /** Drives, then releases the sticks with every wheel reading +2000 ticks/s until braking engages. */
    private static IamYouLogic releasedAndBraking() {
        IamYouLogic logic = new IamYouLogic();
        hold(logic, 0.0, -1.0, 0.0, 0.0, 0.0, SOUTH);
        boolean braking = false;
        for (int i = 0; i < 10 && !braking; i++) {
            braking = logic.calculate(input(0.0, 0.0, 0.0, 0.0, 0.0, SOUTH, 2000.0)).braking;
        }
        assertTrue(braking, "braking should engage shortly after the sticks are released");
        return logic;
    }

    @Test
    void testBrakeEngagesAfterBriefDelay() {
        IamYouLogic logic = new IamYouLogic();
        hold(logic, 0.0, -1.0, 0.0, 0.0, 0.0, SOUTH);

        int expectedLoops = (int) Math.ceil(IamYouLogic.BRAKE_ENGAGE_SEC / DT - 1e-9);
        for (int loop = 1; loop <= expectedLoops + 1; loop++) {
            IamYouLogic.DriveResult r = logic.calculate(input(0.0, 0.0, 0.0, 0.0, 0.0, SOUTH, 2000.0));
            assertEquals(loop >= expectedLoops, r.braking, "loop " + loop);
        }
    }

    @Test
    void testBrakeOpposesMotionWithinCap() {
        IamYouLogic logic = releasedAndBraking();

        IamYouLogic.DriveResult r = logic.calculate(new IamYouLogic.DriveParameters(
                0.0, 0.0, 0.0, 0.0, 0.0, SOUTH, 2000.0, -2000.0, 300.0, 40.0, DT));

        assertEquals(-IamYouLogic.BRAKE_MAX_POWER, r.powers.getFL(), EPS);   // fast wheel: full-strength brake
        assertEquals(0.0, r.powers.getFR(), 1e-9);                          // reversed vs when braking began: left alone
        assertTrue(r.powers.getBL() < 0 && r.powers.getBL() > -IamYouLogic.BRAKE_MAX_POWER);   // slow wheel: gentler
        assertEquals(0.0, r.powers.getBR(), 1e-9);                          // already about stopped: left alone
    }

    @Test
    void testBrakeNeverPushesAWheelBackwardsThroughZero() {
        IamYouLogic logic = releasedAndBraking();

        // Wheel overshoots to the other direction...
        IamYouLogic.DriveResult crossed = logic.calculate(new IamYouLogic.DriveParameters(
                0.0, 0.0, 0.0, 0.0, 0.0, SOUTH, -200.0, 0.0, 0.0, 0.0, DT));
        assertEquals(0.0, crossed.powers.getFL(), 1e-9);

        // ...and even if it reads fast forward again, braking stays off for that wheel until the next release.
        IamYouLogic.DriveResult after = logic.calculate(new IamYouLogic.DriveParameters(
                0.0, 0.0, 0.0, 0.0, 0.0, SOUTH, 1500.0, 0.0, 0.0, 0.0, DT));
        assertEquals(0.0, after.powers.getFL(), 1e-9);
    }

    @Test
    void testDrivingAfterBrakingRampsFromZero() {
        IamYouLogic logic = releasedAndBraking();

        IamYouLogic.DriveResult r = logic.calculate(input(0.0, -1.0, 0.0, 0.0, 0.0, SOUTH, 0.0));

        assertFalse(r.braking);
        assertTrue(r.stickForward > 0);
        assertTrue(r.stickForward <= IamYouLogic.MAX_TRANSLATION_CHANGE_PER_SEC * DT + 1e-9);
    }
}
