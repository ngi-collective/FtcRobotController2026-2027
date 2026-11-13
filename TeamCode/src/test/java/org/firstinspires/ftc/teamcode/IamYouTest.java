package org.firstinspires.ftc.teamcode;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/** The drive math inside {@link iamyou}: direct field-centric mecanum, slow mode and the stop assist. */
class IamYouTest {

    private static final double DT = 0.02;          // 50 Hz loop
    private static final double EPS = 1e-6;
    private static final iamyou.DriverPosition SOUTH = iamyou.DriverPosition.SOUTH;

    /** Raw gamepad values: stick up is NEGATIVE y. Wheel velocities are all set to the same value. */
    private static iamyou.DriveParameters input(double stickX, double stickY, double twist, double trigger,
                                                double heading, iamyou.DriverPosition pos, double wheelVel) {
        return new iamyou.DriveParameters(stickX, stickY, twist, trigger, heading, pos,
                wheelVel, wheelVel, wheelVel, wheelVel, DT);
    }

    private static iamyou.DriveResult once(double stickX, double stickY, double twist, double heading,
                                           iamyou.DriverPosition pos) {
        return new iamyou.Drive().calculate(input(stickX, stickY, twist, 0.0, heading, pos, 0.0));
    }

    private static double[] wheels(iamyou.DriveResult r) {
        return new double[]{r.powers.getFL(), r.powers.getFR(), r.powers.getBL(), r.powers.getBR()};
    }

    /** Where a set of wheel powers pushes the robot, in the robot's frame: {right, forward}. */
    private static double[] robotMotion(iamyou.DriveResult r) {
        double[] w = wheels(r);
        double forward = (w[0] + w[1] + w[2] + w[3]) / 4.0;
        double right = (w[0] - w[1] - w[2] + w[3]) / 4.0;
        return new double[]{right, forward};
    }

    // ------------------------------------------------------------------ cardinals

    @Test
    void testSnapToCardinal() {
        assertEquals(iamyou.Cardinal.NORTH, iamyou.snapToCardinal(0));
        assertEquals(iamyou.Cardinal.NORTH, iamyou.snapToCardinal(44));
        assertEquals(iamyou.Cardinal.NORTH, iamyou.snapToCardinal(-44));
        assertEquals(iamyou.Cardinal.NORTH, iamyou.snapToCardinal(400));   // wraps to 40
        assertEquals(iamyou.Cardinal.WEST, iamyou.snapToCardinal(90));
        assertEquals(iamyou.Cardinal.EAST, iamyou.snapToCardinal(-90));
        assertEquals(iamyou.Cardinal.SOUTH, iamyou.snapToCardinal(180));
        assertEquals(iamyou.Cardinal.SOUTH, iamyou.snapToCardinal(-180));
    }

    // ------------------------------------------------------------------ direct response

    @Test
    void testFullStickIsFullPowerOnTheFirstLoop() {
        // No ramp: the very first loop with the stick pushed commands full power.
        assertArrayEquals(new double[]{1, 1, 1, 1}, wheels(once(0.0, -1.0, 0.0, 0.0, SOUTH)), EPS);
    }

    @Test
    void testFullStrafeRightIsTheMecanumXPattern() {
        assertArrayEquals(new double[]{1, -1, -1, 1}, wheels(once(1.0, 0.0, 0.0, 0.0, SOUTH)), EPS);
    }

    @Test
    void testPositiveTwistTurnsClockwise() {
        double[] w = wheels(once(0.0, 0.0, 1.0, 0.0, SOUTH));
        assertTrue(w[0] > 0 && w[2] > 0, "left side forward");
        assertTrue(w[1] < 0 && w[3] < 0, "right side backward");
    }

    @Test
    void testHalfStickIsHalfPowerTimesTheDriveMultiplier() {
        // Linear response, scaled by DRIVE_MULT: no curve to make small movements feel dead.
        double fl = once(0.0, -0.5, 0.0, 0.0, SOUTH).powers.getFL();
        double stick = (0.5 - iamyou.JOYSTICK_DEADZONE) / (1.0 - iamyou.JOYSTICK_DEADZONE);
        assertEquals(stick * iamyou.DRIVE_MULT + iamyou.STATIC_FRICTION_POWER, fl, EPS);
    }

    @Test
    void testSlowModeSpeedIsUnchangedByTheDriveMultiplier() {
        double fl = new iamyou.Drive().calculate(input(0.0, -1.0, 0.0, 1.0, 0.0, SOUTH, 0.0)).powers.getFL();
        assertEquals(iamyou.SLOW_MODE_DRIVE_MULT + iamyou.STATIC_FRICTION_POWER, fl, EPS);
    }

    @Test
    void testReleasingTheStickCutsDrivePowerOnTheNextLoop() {
        iamyou.Drive logic = new iamyou.Drive();
        logic.calculate(input(0.0, -1.0, 0.0, 0.0, 0.0, SOUTH, 0.0));
        // Wheels already stopped, so the stop assist has nothing to do: every wheel goes straight to 0.
        assertArrayEquals(new double[]{0, 0, 0, 0},
                wheels(logic.calculate(input(0.0, 0.0, 0.0, 0.0, 0.0, SOUTH, 0.0))), EPS);
    }

    @Test
    void testAStickRestingJustOffCenterCountsAsReleased() {
        double off = iamyou.JOYSTICK_DEADZONE * 0.9;
        // hypot(off, 0.3 * off) is still inside the circular deadzone.
        assertArrayEquals(new double[]{0, 0, 0, 0}, wheels(once(off, -off * 0.3, off, 0.0, SOUTH)), EPS);
    }

    @Test
    void testSlowModeScalesDriving() {
        iamyou.DriveResult slow = new iamyou.Drive().calculate(input(0.0, -1.0, 0.0, 1.0, 0.0, SOUTH, 0.0));
        assertTrue(slow.slowMode);
        assertEquals(iamyou.SLOW_MODE_DRIVE_MULT + iamyou.STATIC_FRICTION_POWER, slow.powers.getFL(), EPS);
    }

    @Test
    void testDiagonalKeepsItsDirection() {
        // Scaling by the largest wheel must never bend the path toward an axis.
        for (double deg = 0; deg < 360; deg += 15) {
            double sx = Math.cos(Math.toRadians(deg));
            double sy = Math.sin(Math.toRadians(deg));
            double[] m = robotMotion(once(sx, -sy, 0.0, 0.0, SOUTH));
            // Undo the strafe compensation to compare against the stick direction.
            double actual = Math.atan2(m[1], m[0] / iamyou.STRAFE_COMPENSATION);
            double diff = Math.atan2(Math.sin(actual - Math.toRadians(deg)), Math.cos(actual - Math.toRadians(deg)));
            assertEquals(0.0, diff, 0.05, "stick at " + deg + " deg");
        }
    }

    // ------------------------------------------------------------------ field-centric

    /**
     * The whole circle, not just the cardinals: at every heading, "stick forward" must push the robot toward the
     * field direction it faced at zero. In the robot's frame that is the direction the robot turned AWAY from.
     */
    @Test
    void testForwardStickIsTheSameFieldDirectionAtEveryHeading() {
        for (double heading = -720; heading <= 720; heading += 15) {
            double[] m = robotMotion(once(0.0, -1.0, 0.0, heading, SOUTH));
            // Field "north" seen from a robot turned CCW by h is at angle (90 - h) in robot (x right, y fwd) terms.
            double expected = Math.toRadians(90.0 - heading);
            double actual = Math.atan2(m[1], m[0] / iamyou.STRAFE_COMPENSATION);
            double diff = Math.atan2(Math.sin(actual - expected), Math.cos(actual - expected));
            assertEquals(0.0, diff, 0.05, "heading " + heading);
        }
    }

    @Test
    void testTurnedNinetyLeftForwardStickStrafesRight() {
        assertArrayEquals(new double[]{1, -1, -1, 1}, wheels(once(0.0, -1.0, 0.0, 90.0, SOUTH)), EPS);
    }

    @Test
    void testTurnedNinetyRightForwardStickStrafesLeft() {
        assertArrayEquals(new double[]{-1, 1, 1, -1}, wheels(once(0.0, -1.0, 0.0, -90.0, SOUTH)), EPS);
    }

    @Test
    void testTurnedAroundForwardStickDrivesBackward() {
        assertArrayEquals(new double[]{-1, -1, -1, -1}, wheels(once(0.0, -1.0, 0.0, 180.0, SOUTH)), EPS);
    }

    @Test
    void testDriverStandingNorthPushesTheRobotSouth() {
        assertArrayEquals(new double[]{-1, -1, -1, -1},
                wheels(once(0.0, -1.0, 0.0, 0.0, iamyou.DriverPosition.NORTH)), EPS);
    }

    @Test
    void testDriverStandingEastPushesTheRobotWest() {
        // West = the robot's left at heading 0.
        assertArrayEquals(new double[]{-1, 1, 1, -1},
                wheels(once(0.0, -1.0, 0.0, 0.0, iamyou.DriverPosition.EAST)), EPS);
    }

    @Test
    void testFieldCentricHasNoMemory() {
        // Spinning the robot all the way round and back must leave the controls exactly as they started.
        iamyou.Drive logic = new iamyou.Drive();
        double[] before = wheels(logic.calculate(input(0.3, -0.7, 0.0, 0.0, 0.0, SOUTH, 0.0)));
        for (double h = 0; h <= 720; h += 7) {
            logic.calculate(input(0.3, -0.7, 0.4, 0.0, h, SOUTH, 0.0));
        }
        for (double h = 720; h >= 0; h -= 7) {
            logic.calculate(input(0.3, -0.7, -0.4, 0.0, h, SOUTH, 0.0));
        }
        double[] after = wheels(logic.calculate(input(0.3, -0.7, 0.0, 0.0, 0.0, SOUTH, 0.0)));
        assertArrayEquals(before, after, EPS);
    }

    // ------------------------------------------------------------------ stop assist

    /** Drove forward at full power, then released with every wheel still at the given speed. */
    private static iamyou.DriveResult releasedAt(iamyou.Drive logic, double wheelVel) {
        logic.calculate(input(0.0, -1.0, 0.0, 0.0, 0.0, SOUTH, wheelVel));
        return logic.calculate(input(0.0, 0.0, 0.0, 0.0, 0.0, SOUTH, wheelVel));
    }

    @Test
    void testStopAssistPushesBackTheMomentTheStickIsReleased() {
        iamyou.DriveResult r = releasedAt(new iamyou.Drive(), 2000.0);
        assertTrue(r.braking);
        for (double w : wheels(r)) {
            assertEquals(-iamyou.BRAKE_MAX_POWER, w, EPS);
        }
    }

    @Test
    void testStopAssistIsGentlerForASlowerWheel() {
        iamyou.Drive logic = new iamyou.Drive();
        logic.calculate(input(0.0, -1.0, 0.0, 0.0, 0.0, SOUTH, 0.0));
        iamyou.DriveResult r = logic.calculate(new iamyou.DriveParameters(
                0.0, 0.0, 0.0, 0.0, 0.0, SOUTH, 2000.0, 2000.0, 300.0, 2000.0, DT));
        assertEquals(-iamyou.BRAKE_MAX_POWER, r.powers.getFL(), EPS);
        assertTrue(r.powers.getBL() < 0 && r.powers.getBL() > -iamyou.BRAKE_MAX_POWER);
    }

    @Test
    void testStopAssistEndsWhenTheWheelStops() {
        iamyou.Drive logic = new iamyou.Drive();
        releasedAt(logic, 2000.0);
        iamyou.DriveResult r = logic.calculate(input(0.0, 0.0, 0.0, 0.0, 0.0, SOUTH, 0.0));
        assertFalse(r.braking);
        assertArrayEquals(new double[]{0, 0, 0, 0}, wheels(r), EPS);
        // And it stays off even if the encoder then reports movement again.
        r = logic.calculate(input(0.0, 0.0, 0.0, 0.0, 0.0, SOUTH, 2000.0));
        assertArrayEquals(new double[]{0, 0, 0, 0}, wheels(r), EPS);
    }

    @Test
    void testStopAssistNeverPushesTheWayTheWheelWasDriven() {
        // A wrongly-signed encoder (velocity opposite to power) must never turn the stop assist into a runaway.
        for (double vel : new double[]{-2700.0, -500.0, 0.0, 500.0, 2700.0}) {
            iamyou.Drive logic = new iamyou.Drive();
            logic.calculate(input(0.0, -1.0, 0.0, 0.0, 0.0, SOUTH, 0.0));   // every wheel driven forward
            for (int i = 0; i < 30; i++) {
                for (double w : wheels(logic.calculate(input(0.0, 0.0, 0.0, 0.0, 0.0, SOUTH, vel)))) {
                    assertTrue(w <= 1e-9, "vel " + vel + " loop " + i + " wheel " + w);
                }
            }
        }
    }

    @Test
    void testStopAssistGivesUpAfterTheWatchdogEvenIfWheelsNeverSlow() {
        iamyou.Drive logic = new iamyou.Drive();
        releasedAt(logic, 2700.0);
        iamyou.DriveResult r = null;
        int loops = (int) Math.ceil(iamyou.BRAKE_MAX_SEC / DT) + 2;
        for (int i = 0; i < loops; i++) {
            r = logic.calculate(input(0.0, 0.0, 0.0, 0.0, 0.0, SOUTH, 2700.0));
        }
        assertFalse(r.braking);
        assertArrayEquals(new double[]{0, 0, 0, 0}, wheels(r), EPS);
    }

    @Test
    void testPushingTheStickAgainCancelsTheBrakeImmediately() {
        iamyou.Drive logic = new iamyou.Drive();
        releasedAt(logic, 2000.0);
        iamyou.DriveResult r = logic.calculate(input(0.0, -1.0, 0.0, 0.0, 0.0, SOUTH, 2000.0));
        assertFalse(r.braking);
        assertArrayEquals(new double[]{1, 1, 1, 1}, wheels(r), EPS);
    }

    // ------------------------------------------------------------------ heading tracker

    /** Feeds both sensors the same true heading, turning at the given rate for the given number of loops. */
    private static double turnBoth(iamyou.HeadingTracker t, double startDeg, double degPerLoop, int loops) {
        double h = startDeg;
        for (int i = 0; i < loops; i++) {
            h += degPerLoop;
            t.update(wrap(h), true, wrap(h), true, DT);
        }
        return h;
    }

    private static double wrap(double deg) {
        return ((deg + 540.0) % 360.0) - 180.0;
    }

    @Test
    void testTrackerFollowsATurnThroughTheWrapAround() {
        iamyou.HeadingTracker t = new iamyou.HeadingTracker();
        t.update(170.0, true, 170.0, true, DT);
        t.zero();
        turnBoth(t, 170.0, 5.0, 40);   // 200 degrees CCW, crossing +180/-180
        assertEquals(-160.0, t.heading(), 1e-6, "200 degrees CCW reads as 160 degrees CW");
    }

    @Test
    void testAFullResetStartsOverFromTheSensorsNewZero() {
        iamyou.HeadingTracker t = new iamyou.HeadingTracker();
        t.update(0.0, true, 0.0, true, DT);
        double h = turnBoth(t, 0.0, 3.0, 40);                      // 120 degrees CCW
        assertEquals(120.0, t.heading(), 1e-6);
        // Share pressed: the tracker forgets everything and both sensors are reset to 0 (the Pinpoint calibrates
        // for a few loops first). The robot is standing still the whole time.
        t.fullReset();
        assertEquals(0.0, t.heading(), 0.0);
        for (int i = 0; i < 12; i++) {
            t.update(h, false, 0.0, true, DT, true);                // calibrating: the old reading, not fresh
        }
        assertEquals(0.0, t.heading(), 1e-6, "nothing moves while the Pinpoint calibrates");
        // READY again, now reading from 0; the robot turns 30 degrees clockwise.
        double fresh = 0.0;
        t.update(fresh, true, 0.0, true, DT);
        for (int i = 0; i < 10; i++) {
            fresh -= 3.0;
            t.update(fresh, true, fresh, true, DT);
        }
        assertEquals(-30.0, t.heading(), 1e-6);
        assertEquals(0, t.rejectedReadings());
    }

    @Test
    void testAPinpointRestartMidTurnDoesNotMoveTheHeading() {
        iamyou.HeadingTracker t = new iamyou.HeadingTracker();
        t.update(0.0, true, 0.0, true, DT);
        double h = turnBoth(t, 0.0, 3.0, 30);                      // 90 degrees
        // The Pinpoint browns out: it comes back reporting 0 and calibrating, while the robot keeps turning.
        for (int i = 0; i < 15; i++) {
            h += 3.0;
            t.update(0.0, false, wrap(h), true, DT);
        }
        // Back to READY, but now measuring from where it restarted.
        double pinpointOffset = h;
        for (int i = 0; i < 10; i++) {
            h += 3.0;
            t.update(wrap(h - pinpointOffset), true, wrap(h), true, DT);
        }
        assertEquals(h, t.heading(), 1e-6);
        assertTrue(t.rejectedReadings() > 0);
    }

    @Test
    void testASingleGlitchedReadingIsIgnored() {
        iamyou.HeadingTracker t = new iamyou.HeadingTracker();
        t.update(10.0, true, 10.0, true, DT);
        t.update(-150.0, true, 10.0, true, DT);   // garbage from the Pinpoint for one loop
        t.update(10.0, true, 10.0, true, DT);     // and back
        // The jump away and the jump back are both impossible turns; the backup says nothing moved.
        assertEquals(0.0, t.heading(), 1e-6);
        assertEquals(2, t.rejectedReadings());
    }

    @Test
    void testWithoutABackupAGlitchHoldsTheHeadingInsteadOfJumping() {
        iamyou.HeadingTracker t = new iamyou.HeadingTracker();
        t.update(20.0, true, Double.NaN, true, DT);
        t.update(25.0, true, Double.NaN, true, DT);
        t.update(-160.0, true, Double.NaN, true, DT);   // restart to a far-off value
        t.update(-157.0, true, Double.NaN, true, DT);   // turning again from the new baseline
        assertEquals(8.0, t.heading(), 1e-6);
    }

    @Test
    void testZeroMakesTheCurrentDirectionNorthWithoutTouchingTheSensors() {
        iamyou.HeadingTracker t = new iamyou.HeadingTracker();
        t.update(0.0, true, 0.0, true, DT);
        double h = turnBoth(t, 0.0, 3.0, 20);
        t.zero();
        assertEquals(0.0, t.heading(), 1e-9);
        turnBoth(t, h, -3.0, 10);
        assertEquals(-30.0, t.heading(), 1e-6);
    }

    @Test
    void testTheReportFlagsSensorsThatDisagree() {
        iamyou.HeadingTracker t = new iamyou.HeadingTracker();
        t.update(0.0, true, 0.0, true, DT);
        // The Pinpoint over-reads by 20 % (a tilted or badly scaled gyro): small enough per loop to be believed.
        for (int i = 1; i <= 30; i++) {
            t.update(wrap(3.6 * i), true, wrap(3.0 * i), true, DT);
        }
        assertTrue(t.sensorReport().contains("DISAGREE"), t.sensorReport());
    }

    @Test
    void testABackupThatTurnsTheOtherWayIsNeverUsed() {
        // The sensors disagree about direction. One of them is backwards, and the tracker cannot know which, so it
        // must not use the backup at all (a person settles it with the hand-turn check).
        iamyou.HeadingTracker t = new iamyou.HeadingTracker();
        t.update(0.0, true, 0.0, true, DT);
        for (int i = 1; i <= 30; i++) {
            t.update(wrap(3.0 * i), true, wrap(-3.0 * i), true, DT);
        }
        assertFalse(t.backupTrusted());
        for (int i = 1; i <= 10; i++) {
            t.update(90.0, false, wrap(-90.0 - 3.0 * i), true, DT);
        }
        assertEquals(90.0, t.heading(), 1e-6, "a backup turning the other way must not steer the heading");
        assertTrue(t.sensorReport().contains("OPPOSITE WAYS"), t.sensorReport());
    }

    @Test
    void testAFailedBackupReadIsNotATurn() {
        iamyou.HeadingTracker t = new iamyou.HeadingTracker();
        t.update(0.0, true, 30.0, true, DT);
        double h = 0.0;
        for (int i = 1; i <= 20; i++) {                        // learn: both turn the same way
            h += 3.0;
            t.update(wrap(h), true, wrap(30.0 + h), true, DT);
        }
        // The Pinpoint freezes and, on the same bus, the hub IMU's reads fail (reported unhealthy).
        for (int i = 0; i < 5; i++) {
            t.update(wrap(h), false, 0.0, false, DT);
        }
        assertEquals(h, t.heading(), 1e-6, "failed reads on both sensors must hold the heading");
        // The IMU recovers first, while the Pinpoint is still frozen: its first good read is a new baseline (not a
        // turn from the fake 0), and turns after that carry the heading.
        t.update(wrap(h), false, wrap(30.0 + h), true, DT);
        assertEquals(h, t.heading(), 1e-6);
        t.update(wrap(h), false, wrap(30.0 + h + 3.0), true, DT);
        assertEquals(h + 3.0, t.heading(), 1e-6);
    }

    @Test
    void testABackupOnTheWrongAxisIsNeverUsed() {
        // A backup that barely sees the turn (measuring about the wrong axis).
        iamyou.HeadingTracker t = new iamyou.HeadingTracker();
        t.update(0.0, true, 0.0, true, DT);
        for (int i = 1; i <= 30; i++) {
            t.update(wrap(3.0 * i), true, wrap(0.3 * i), true, DT);
        }
        assertFalse(t.backupTrusted());
        for (int i = 1; i <= 10; i++) {
            t.update(0.0, false, wrap(9.0 + 0.3 * i), true, DT);
        }
        assertEquals(90.0, t.heading(), 1e-6, "an untrusted backup must not steer the heading");
        assertTrue(t.sensorReport().contains("BACKUP WRONG"), t.sensorReport());
    }

    @Test
    void testRecoveringFromAFreezeDoesNotCountTheTurnTwice() {
        iamyou.HeadingTracker t = new iamyou.HeadingTracker();
        t.update(0.0, true, 0.0, true, DT);
        double h = turnBoth(t, 0.0, 3.0, 20);                       // 60 degrees, backup now trusted
        double frozenAt = h;
        for (int i = 0; i < 10; i++) {                              // Pinpoint frozen at 60 while turning 30 more
            h += 3.0;
            t.update(wrap(frozenAt), false, wrap(h), true, DT);
        }
        h += 3.0;
        t.update(wrap(h), true, wrap(h), true, DT);                 // back: its reading jumps the 33 it missed
        h += 3.0;
        t.update(wrap(h), true, wrap(h), true, DT);
        assertEquals(h, t.heading(), 1e-6);
    }

    @Test
    void testABackupIsNotTrustedBeforeTheRobotHasTurned() {
        iamyou.HeadingTracker t = new iamyou.HeadingTracker();
        t.update(0.0, true, 0.0, true, DT);
        t.update(0.0, false, 20.0, true, DT);   // Pinpoint restarting before anything was learned
        assertEquals(0.0, t.heading(), 1e-6);
    }

    @Test
    void testOnlyAMissingPodLeavesThePinpointHeadingFresh() {
        assertTrue(iamyou.HeadingTracker.freshHeading(
                com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.READY));
        assertTrue(iamyou.HeadingTracker.freshHeading(
                com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.FAULT_Y_POD_NOT_DETECTED));
        // A bad read is the driver repeating the previous value: frozen, not current.
        assertFalse(iamyou.HeadingTracker.freshHeading(
                com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ));
        assertFalse(iamyou.HeadingTracker.freshHeading(
                com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.CALIBRATING));
        assertFalse(iamyou.HeadingTracker.freshHeading(
                com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.NOT_READY));
    }

    @Test
    void testStandingStillTheHeadingIgnoresDrift() {
        iamyou.HeadingTracker t = new iamyou.HeadingTracker();
        t.update(0.0, true, Double.NaN, false, DT, true);
        // The gyro creeps 0.1 degree a second while the robot sits for 30 seconds.
        for (int i = 1; i <= 1500; i++) {
            t.update(0.1 * i * DT, true, Double.NaN, false, DT, true);
        }
        // Only the first half second (settling, which could be real skid) is followed.
        assertEquals(iamyou.STILL_SETTLE_SEC * 0.1, t.heading(), 0.01);
        assertEquals(0.1, t.driftDegPerSec(), 0.01, "the drift rate should have been measured");
    }

    @Test
    void testMeasuredDriftIsTakenOutWhileMoving() {
        iamyou.HeadingTracker t = new iamyou.HeadingTracker();
        double gyro = 0.0;
        t.update(gyro, true, Double.NaN, false, DT, true);
        for (int i = 0; i < 1500; i++) {                      // 30 s still: learn 0.1 deg/s of drift
            gyro += 0.1 * DT;
            t.update(wrap(gyro), true, Double.NaN, false, DT, true);
        }
        // Now turn 90 degrees over 1 s; the gyro reports the turn plus its drift.
        for (int i = 0; i < 50; i++) {
            gyro += 90.0 / 50 + 0.1 * DT;
            t.update(wrap(gyro), true, Double.NaN, false, DT, false);
        }
        assertEquals(90.0, t.heading(), 0.1);
    }

    @Test
    void testARobotTurnedWhileItsWheelsAreStoppedIsStillFollowed() {
        // Turned by hand in INIT, or shoved round on its rollers: the wheels say "still", the gyro says turning.
        iamyou.HeadingTracker t = new iamyou.HeadingTracker();
        double gyro = 0.0;
        t.update(gyro, true, Double.NaN, false, DT, true);
        for (int i = 0; i < 100; i++) {                       // sits for 2 s
            t.update(gyro, true, Double.NaN, false, DT, true);
        }
        for (int i = 0; i < 100; i++) {                       // then 60 degrees in 2 s, wheels never moving
            gyro += 0.6;
            t.update(wrap(gyro), true, Double.NaN, false, DT, true);
        }
        for (int i = 0; i < 300; i++) {                       // and sits again
            t.update(wrap(gyro), true, Double.NaN, false, DT, true);
        }
        assertEquals(60.0, t.heading(), 1.5, "the turn must not be thrown away as drift");
        assertEquals(0.0, t.driftDegPerSec(), 1e-9, "and must not be learned as drift");
    }

    @Test
    void testAPushedRobotIsNotStillAndItsTurnCounts() {
        // Wheels turning (a push) means not still: the turn is real and must be followed.
        iamyou.HeadingTracker t = new iamyou.HeadingTracker();
        t.update(0.0, true, Double.NaN, false, DT, false);
        for (int i = 1; i <= 10; i++) {
            t.update(3.0 * i, true, Double.NaN, false, DT, false);
        }
        assertEquals(30.0, t.heading(), 1e-6);
    }

    @Test
    void testContinuousTurningStaysWithinOneTurn() {
        iamyou.HeadingTracker t = new iamyou.HeadingTracker();
        t.update(0.0, true, 0.0, true, DT);
        turnBoth(t, 0.0, 6.0, 300);   // five full turns CCW
        assertTrue(Math.abs(t.heading()) <= 180.0, "heading " + t.heading());
        assertEquals(0.0, t.heading(), 1e-6);
    }

    @Test
    void testTheSensorTotalsKeepCountingForTheSpinTest() {
        iamyou.HeadingTracker t = new iamyou.HeadingTracker();
        t.update(0.0, true, 0.0, true, DT);
        turnBoth(t, 0.0, 10.0, 3600);   // a hundred full turns
        assertEquals(0.0, t.heading(), 1e-6);
        assertTrue(t.sensorReport().contains("main 36000.0"), t.sensorReport());
    }

    @Test
    void testTheTurnScaleCorrectsAGyroThatUnderReads() {
        // The gyro sees 98 % of every turn. Ten full turns read 3528; the spin test gives 3600 / 3528.
        iamyou.HeadingTracker t = new iamyou.HeadingTracker();
        t.setScales(3600.0 / 3528.0, 1.0);
        t.update(0.0, true, Double.NaN, false, DT);
        double gyro = 0.0;
        for (int i = 0; i < 1800; i++) {   // ten turns, 2 degrees of true turning a loop
            gyro += 2.0 * 0.98;
            t.update(wrap(gyro), true, Double.NaN, false, DT);
        }
        assertEquals(0.0, t.heading(), 1e-6, "after exactly ten turns the robot faces where it started");
    }

    // ------------------------------------------------------------------ drive check

    /** One second each of forward, strafe right and clockwise spin, on a robot with the given sign errors. */
    private static String driveCheckOn(double forwardSign, double strafeSign, double headingSign) {
        iamyou.DriveCheck check = new iamyou.DriveCheck();
        for (int i = 0; i < 50; i++) {
            check.update(1.0, 0.0, 0.0, forwardSign * 40.0, 0.0, 0.0, DT);
        }
        for (int i = 0; i < 50; i++) {
            check.update(0.0, 1.0, 0.0, 0.0, strafeSign * -30.0, 0.0, DT);   // right = negative leftward speed
        }
        for (int i = 0; i < 50; i++) {
            check.update(0.0, 0.0, 1.0, 0.0, 0.0, headingSign * -200.0, DT); // clockwise = heading falling
        }
        return check.report();
    }

    @Test
    void testDriveCheckPassesAConsistentRobot() {
        assertTrue(driveCheckOn(1, 1, 1).endsWith("all agree"), driveCheckOn(1, 1, 1));
    }

    @Test
    void testDriveCheckNamesAHeadingThatCountsBackwards() {
        String report = driveCheckOn(1, 1, -1);
        assertTrue(report.contains("turn BACKWARDS") && report.contains("PINPOINT_HEADING_REVERSED"), report);
    }

    @Test
    void testDriveCheckNamesMotorsThatAreAllBackwards() {
        assertTrue(driveCheckOn(-1, -1, -1).contains("ALL FOUR MOTORS BACKWARDS"), driveCheckOn(-1, -1, -1));
    }

    @Test
    void testDriveCheckNamesSwappedSides() {
        assertTrue(driveCheckOn(1, -1, -1).contains("LEFT/RIGHT MOTORS SWAPPED"), driveCheckOn(1, -1, -1));
        assertTrue(driveCheckOn(1, -1, 1).contains("FRONT/BACK MOTORS SWAPPED"), driveCheckOn(1, -1, 1));
    }

    @Test
    void testDriveCheckWaitsForEnoughCleanDriving() {
        iamyou.DriveCheck check = new iamyou.DriveCheck();
        // Diagonal driving, slow crawling and a stopped robot are not evidence of anything.
        for (int i = 0; i < 200; i++) {
            check.update(0.7, 0.7, 0.0, 30.0, -30.0, 0.0, DT);
            check.update(1.0, 0.0, 0.0, 1.0, 0.0, 0.0, DT);
            check.update(0.0, 0.0, 0.0, 40.0, 0.0, 300.0, DT);
        }
        assertTrue(check.report().startsWith("fwd ?  strafe ?  turn ?"), check.report());
    }

    // ------------------------------------------------------------------ driving while turning

    @Test
    void testFullDriveAndFullTwistShareThePowerEvenly() {
        double[] w = wheels(once(0.0, -1.0, 1.0, 0.0, SOUTH));
        double turn = (w[0] - w[1] + w[2] - w[3]) / 4.0;
        double forward = (w[0] + w[1] + w[2] + w[3]) / 4.0;
        assertEquals(forward, turn, 0.03);
        assertTrue(turn > 0.45 && forward > 0.45, "turn " + turn + " forward " + forward);
        for (double p : w) {
            assertTrue(Math.abs(p) <= 1.0 + 1e-12);
        }
    }

    @Test
    void testFullTwistWhileDrivingSlowlyStillTurnsHard() {
        double[] w = wheels(once(0.0, -0.5, 1.0, 0.0, SOUTH));
        double turn = (w[0] - w[1] + w[2] - w[3]) / 4.0;
        assertTrue(turn > 0.6, "turn " + turn);
    }

    @Test
    void testASmallTurnWhileDrivingSlowlyIsNotCut() {
        double[] w = wheels(once(0.0, -0.4, 0.3, 0.0, SOUTH));
        double turn = (w[0] - w[1] + w[2] - w[3]) / 4.0;
        double forward = (w[0] + w[1] + w[2] + w[3]) / 4.0;
        assertTrue(turn > 0.2 && forward > 0.3, "turn " + turn + " forward " + forward);
    }

    // ------------------------------------------------------------------ flywheel PIDF

    /**
     * A flywheel that settles, a third of a second later, at the speed its power and battery give it. It needs more
     * than the tuned feedforward guesses (friction 0.04, 3200 ticks/s per unit at 12 V, where kV = 0.0003 says 3333),
     * so P and I have work to do.
     */
    private static final class FlywheelModel {
        double tps = 0.0;

        void step(double power, double volts, double dt) {
            double steady = Math.max(0.0, power * volts / 12.0 - 0.04) * 3200.0;
            tps += (steady - tps) * dt / 0.3;
        }
    }

    private static double runFlywheel(iamyou.Outtake out, FlywheelModel wheel, double volts, double seconds) {
        double power = 0.0;
        for (int i = 0; i < Math.round(seconds / DT); i++) {
            power = out.update(false, wheel.tps, volts, DT);
            assertTrue(power >= 0.0 && power <= 1.0, "power " + power);
            wheel.step(power, volts, DT);
        }
        return power;
    }

    @Test
    void testFlywheelReachesTheTargetAndSaysReady() {
        iamyou.Outtake out = new iamyou.Outtake();
        FlywheelModel wheel = new FlywheelModel();
        out.update(true, 0.0, 12.5, DT);                        // press: ON
        assertTrue(out.isOn());
        assertFalse(out.isReady());
        runFlywheel(out, wheel, 12.5, 3.0);
        assertEquals(iamyou.FLYWHEEL_TARGET_TPS, wheel.tps, iamyou.FLYWHEEL_READY_TOLERANCE_TPS);
        assertTrue(out.isReady());
    }

    @Test
    void testFlywheelHoldsTheSameSpeedOnAFullAndATiredBattery() {
        for (double volts : new double[]{13.5, 12.0, 11.0}) {
            iamyou.Outtake out = new iamyou.Outtake();
            FlywheelModel wheel = new FlywheelModel();
            out.update(true, 0.0, volts, DT);
            runFlywheel(out, wheel, volts, 4.0);
            assertEquals(iamyou.FLYWHEEL_TARGET_TPS, wheel.tps, iamyou.FLYWHEEL_READY_TOLERANCE_TPS, volts + " V");
        }
    }

    @Test
    void testFlywheelRecoversFromAShotDipWithoutOvershootingMuch() {
        iamyou.Outtake out = new iamyou.Outtake();
        FlywheelModel wheel = new FlywheelModel();
        out.update(true, 0.0, 12.5, DT);
        runFlywheel(out, wheel, 12.5, 3.0);
        wheel.tps -= 400.0;                                     // a ball takes energy out of the wheel
        assertFalse(runFlywheel(out, wheel, 12.5, DT) > 1.0);
        assertFalse(out.isReady());
        double worst = 0.0;
        for (int i = 0; i < 50; i++) {                          // one second
            runFlywheel(out, wheel, 12.5, DT);
            worst = Math.max(worst, wheel.tps - iamyou.FLYWHEEL_TARGET_TPS);
        }
        assertTrue(worst < 100.0, "overshoot " + worst);
        assertTrue(out.isReady(), "back up to speed within a second");
    }

    @Test
    void testFlywheelOffMeansZeroPowerAndForgetsTheITerm() {
        iamyou.Outtake out = new iamyou.Outtake();
        FlywheelModel wheel = new FlywheelModel();
        out.update(true, 0.0, 12.5, DT);
        runFlywheel(out, wheel, 12.5, 3.0);
        out.update(false, wheel.tps, 12.5, DT);
        assertEquals(0.0, out.update(true, wheel.tps, 12.5, DT), EPS);   // press again: OFF
        assertFalse(out.isOn());
        assertFalse(out.isReady());
        assertEquals("F 0.00  P 0.00  I 0.00  D 0.00", out.pidf.terms());
    }

    @Test
    void testFlywheelIsNeverDrivenBackwardsOrHurtByBadNumbers() {
        iamyou.FlywheelPIDF pidf = new iamyou.FlywheelPIDF();
        assertEquals(0.0, pidf.update(2100.0, 4000.0, 12.0, DT), EPS);   // far too fast: coast, don't brake
        for (double b : new double[]{Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
            double p = pidf.update(2100.0, b, b, b);
            assertTrue(p >= 0.0 && p <= 1.0, "power " + p);
        }
    }

    // ------------------------------------------------------------------ NaN safety

    @Test
    void testBadNumbersNeverReachTheMotors() {
        double[] bad = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY};
        for (double b : bad) {
            iamyou.Drive logic = new iamyou.Drive();
            assertFinite(logic.calculate(new iamyou.DriveParameters(b, b, b, b, b, SOUTH, b, b, b, b, b)));
            assertFinite(logic.calculate(new iamyou.DriveParameters(
                    0.5, -0.5, 0.0, 0.0, 10.0, SOUTH, 0, 0, 0, 0, 0.0)));   // dt of exactly zero
            assertFinite(logic.calculate(new iamyou.DriveParameters(
                    0.0, 0.0, 0.0, 0.0, 10.0, SOUTH, b, b, b, b, DT)));     // bad encoder while braking
        }
    }

    private static void assertFinite(iamyou.DriveResult r) {
        for (double w : wheels(r)) {
            assertFalse(Double.isNaN(w) || Double.isInfinite(w));
            assertTrue(Math.abs(w) <= 1.0 + 1e-12);
        }
    }
}
