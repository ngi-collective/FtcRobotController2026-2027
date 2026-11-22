package org.firstinspires.ftc.teamcode;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/** The software brake's push-back: when it starts, how hard it pushes, which way, and when it stops. */
class PushBackTest {

    private static final double DT = 0.02;

    @Test
    void holdsQuietlyUntilTheRobotHasBeenMovedAnInch() {
        PushBack push = new PushBack(0.0, 0.0, 0.0);
        assertNull(push.update(0.0, 0.0, 0.0, DT));
        assertNull(push.update(0.9, 0.0, 0.0, DT));
        assertEquals(PushBack.Mode.HOLDING, push.mode());
    }

    @Test
    void aShovePastAnInchIsMetWithFullPowerTheOtherWay() {
        PushBack push = new PushBack(0.0, 0.0, 0.0);
        double[] wheels = push.update(1.5, 0.0, 0.0, DT);   // shoved forward, along +X

        assertNotNull(wheels);
        assertEquals(PushBack.Mode.PUSHING_BACK, push.mode());
        assertEquals(1.0, maxAbs(wheels), 1e-9, "the push-back should be at full power");
        for (double w : wheels) {
            assertEquals(-1.0, w, 1e-9, "shoved forward, every wheel should drive straight back");
        }
    }

    @Test
    void pushesBackAlongTheFieldWhateverWayTheRobotFaces() {
        // Facing +Y and shoved toward +Y: straight back again, in the robot's frame.
        PushBack push = new PushBack(10.0, 10.0, Math.PI / 2);
        double[] wheels = push.update(10.0, 12.0, Math.PI / 2, DT);
        for (double w : wheels) {
            assertEquals(-1.0, w, 1e-9);
        }

        // Facing +X and shoved toward +Y (its left): a strafe right, FL and BR forward, FR and BL back.
        push = new PushBack(0.0, 0.0, 0.0);
        wheels = push.update(0.0, 2.0, 0.0, DT);
        assertArrayEquals(new double[]{1.0, -1.0, -1.0, 1.0}, wheels, 1e-9);
    }

    @Test
    void drivesThroughTheSpotAndPastItBeforeHandingBackToTheHold() {
        PushBack push = new PushBack(0.0, 0.0, 0.0);
        push.update(2.0, 0.0, 0.0, DT);

        // Back at the spot it keeps going: the point is to move the pusher, not to stop where it was.
        assertNotNull(push.update(0.0, 0.0, 0.0, DT));
        assertNotNull(push.update(-PushBack.OVERSHOOT_INCHES + 0.5, 0.0, 0.0, DT));
        assertNull(push.update(-PushBack.OVERSHOOT_INCHES, 0.0, 0.0, DT));
        assertEquals(PushBack.Mode.HOLDING, push.mode());
    }

    @Test
    void theRollOutAfterAPushBackIsNotANewPush() {
        PushBack push = new PushBack(0.0, 0.0, 0.0);
        push.update(2.0, 0.0, 0.0, DT);
        push.update(-PushBack.OVERSHOOT_INCHES, 0.0, 0.0, DT);

        // Still rolling outward for a moment, then the hold brings it home.
        double x = -PushBack.OVERSHOOT_INCHES;
        for (double t = 0.0; t < PushBack.SETTLE_SECONDS; t += DT) {
            x -= 0.4;
            assertNull(push.update(x, 0.0, 0.0, DT));
        }
        for (; x < 0.0; x += 0.3) {
            assertNull(push.update(x, 0.0, 0.0, DT), "coming home at " + x + " is not a push");
        }
        assertEquals(1, push.pushes());
    }

    @Test
    void aSecondShoveOnTheWayHomeIsPushedBackToo() {
        PushBack push = new PushBack(0.0, 0.0, 0.0);
        push.update(2.0, 0.0, 0.0, DT);
        push.update(-PushBack.OVERSHOOT_INCHES, 0.0, 0.0, DT);
        for (double t = 0.0; t < PushBack.SETTLE_SECONDS + DT; t += DT) {
            push.update(-PushBack.OVERSHOOT_INCHES, 0.0, 0.0, DT);
        }
        push.update(-3.0, 0.0, 0.0, DT);   // on its way home

        // Shoved away from the spot again, along -X: push back along +X.
        double[] wheels = push.update(-4.5, 0.0, 0.0, DT);
        assertNotNull(wheels);
        for (double w : wheels) {
            assertEquals(1.0, w, 1e-9);
        }
        assertEquals(2, push.pushes());
    }

    @Test
    void givesUpAgainstSomethingThatWillNotMoveAndPushesAgainIfStillShoved() {
        PushBack push = new PushBack(0.0, 0.0, 0.0);
        push.update(2.0, 0.0, 0.0, DT);
        int steps = 0;
        while (steps < 200 && push.update(2.0, 0.0, 0.0, DT) != null) {
            steps++;
        }
        assertEquals(PushBack.MAX_PUSH_SECONDS / DT, steps, 1.5, "it should push for MAX_PUSH_SECONDS, then stop");
        assertEquals(PushBack.Mode.HOLDING, push.mode());

        for (double s = 0.0; s < PushBack.SETTLE_SECONDS + DT; s += DT) {
            push.update(2.0, 0.0, 0.0, DT);
        }
        assertNotNull(push.update(3.5, 0.0, 0.0, DT), "still being shoved, it should push again");
    }

    @Test
    void correctsItsHeadingWhilePushingBack() {
        PushBack push = new PushBack(0.0, 0.0, 0.0);
        // Shoved forward and turned 20 degrees counter-clockwise: turn back clockwise while driving back.
        double[] wheels = push.update(2.0, 0.0, Math.toRadians(20.0), DT);
        double turnCcw = (-wheels[0] + wheels[1] - wheels[2] + wheels[3]) / 4.0;
        assertTrue(turnCcw < 0.0, "expected a clockwise correction, wheels were " + java.util.Arrays.toString(wheels));
        assertEquals(1.0, maxAbs(wheels), 1e-9);
    }

    private static double maxAbs(double[] values) {
        double max = 0.0;
        for (double v : values) {
            max = Math.max(max, Math.abs(v));
        }
        return max;
    }
}
