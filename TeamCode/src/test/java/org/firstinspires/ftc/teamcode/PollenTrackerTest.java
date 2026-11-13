package org.firstinspires.ftc.teamcode;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class PollenTrackerTest {

    private static final double DT = 0.02;

    /** A pollen the camera sees at the given bearing (degrees, right-positive) and range. */
    private static BallDetector.Detection pollen(double bearingDeg, double range) {
        return new BallDetector.Detection(BallDetector.Kind.POLLEN, 160, 120, 6.0, range,
                Math.toRadians(bearingDeg), 0.0);
    }

    private static BallDetector.Detection nectar(double bearingDeg, double range) {
        return new BallDetector.Detection(BallDetector.Kind.RED_NECTAR, 160, 120, 8.0, range,
                Math.toRadians(bearingDeg), 0.0);
    }

    /** Runs a stationary robot for a while, delivering one frame, then returns the last output. */
    private static PollenTracker.Output sightOnce(PollenTracker t, double heading, List<BallDetector.Detection> frame) {
        PollenTracker.Output out = t.update(0.0, heading, true, frame);
        for (int i = 1; i <= 15; i++) {
            out = t.update(i * DT, heading, false, frame);
        }
        return out;
    }

    @Test
    void testNoDetectionsMeansNoTarget() {
        PollenTracker t = new PollenTracker();
        assertFalse(sightOnce(t, 0.0, Collections.emptyList()).hasTarget);
    }

    @Test
    void testPollenOnTheRightTurnsClockwise() {
        PollenTracker t = new PollenTracker();
        PollenTracker.Output out = sightOnce(t, 0.0, Arrays.asList(pollen(20.0, 1.0)));
        assertTrue(out.hasTarget);
        assertTrue(out.twist > 0, "pollen on the right needs a clockwise (positive) turn");
        assertTrue(out.errorDegrees < 0);
    }

    @Test
    void testPollenOnTheLeftTurnsCounterClockwise() {
        PollenTracker t = new PollenTracker();
        PollenTracker.Output out = sightOnce(t, 0.0, Arrays.asList(pollen(-20.0, 1.0)));
        assertTrue(out.twist < 0);
    }

    @Test
    void testPollenStraightAheadNeedsNoTurn() {
        PollenTracker t = new PollenTracker();
        PollenTracker.Output out = sightOnce(t, 0.0, Arrays.asList(pollen(0.0, 1.0)));
        assertTrue(out.hasTarget);
        assertEquals(0.0, out.twist, 1e-9);
    }

    @Test
    void testAimsAtTheMiddleOfAGroup() {
        PollenTracker t = new PollenTracker();
        // Three pollen 10 cm apart at the same range: -0.1, 0, +0.1 m. The middle one is straight ahead.
        PollenTracker.Output out = sightOnce(t, 0.0, Arrays.asList(
                pollen(Math.toDegrees(Math.atan2(-0.1, 1.0)), 1.0),
                pollen(0.0, 1.0),
                pollen(Math.toDegrees(Math.atan2(0.1, 1.0)), 1.0)));
        assertEquals(3, out.groupSize);
        assertEquals(0.0, out.errorDegrees, 0.5);
    }

    @Test
    void testAimsAtTheMiddleOfALopsidedGroup() {
        PollenTracker t = new PollenTracker();
        // Two pollen at +0.0 m and +0.2 m: the middle is +0.1 m right of straight ahead -> small clockwise turn.
        PollenTracker.Output out = sightOnce(t, 0.0, Arrays.asList(
                pollen(0.0, 1.0),
                pollen(Math.toDegrees(Math.atan2(0.2, 1.0)), 1.0)));
        assertEquals(2, out.groupSize);
        assertTrue(out.twist > 0);
        assertTrue(out.errorDegrees > -8.0 && out.errorDegrees < -3.0, "error " + out.errorDegrees);
    }

    @Test
    void testPrefersTheBiggerGroup() {
        PollenTracker t = new PollenTracker();
        PollenTracker.Output out = sightOnce(t, 0.0, Arrays.asList(
                pollen(-30.0, 1.0),                                   // lone pollen on the left
                pollen(25.0, 1.5), pollen(26.5, 1.5), pollen(27.5, 1.5)));   // group of three on the right
        assertEquals(3, out.groupSize);
        assertTrue(out.twist > 0);
    }

    @Test
    void testIgnoresNectarAndTinyBlobs() {
        PollenTracker t = new PollenTracker();
        BallDetector.Detection speck = new BallDetector.Detection(BallDetector.Kind.POLLEN, 5, 5, 1.0, 1.0, 0.3, 0.0);
        assertFalse(sightOnce(t, 0.0, Arrays.asList(nectar(10.0, 1.0), speck)).hasTarget);
    }

    @Test
    void testLosesTargetAfterTimeout() {
        PollenTracker t = new PollenTracker();
        t.update(0.0, 0.0, true, Arrays.asList(pollen(15.0, 1.0)));
        assertTrue(t.update(0.2, 0.0, false, null).hasTarget);
        PollenTracker.Output out = t.update(PollenTracker.LOST_TIMEOUT_SEC + 0.1, 0.0, false, null);
        assertFalse(out.hasTarget);
        assertEquals(0.0, out.twist, 1e-12);
    }

    @Test
    void testTwistNeverExceedsTheLimit() {
        PollenTracker t = new PollenTracker();
        PollenTracker.Output out = sightOnce(t, 0.0, Arrays.asList(pollen(35.0, 1.0)));
        assertTrue(Math.abs(out.twist) <= PollenTracker.MAX_TWIST + 1e-12);
    }

    @Test
    void testNaNHeadingIsIgnored() {
        PollenTracker t = new PollenTracker();
        assertFalse(t.update(0.0, Double.NaN, true, Arrays.asList(pollen(10.0, 1.0))).hasTarget);
    }

    /** Closed loop: a simulated robot with a 100 ms camera delay should settle on the pollen without overshooting much. */
    @Test
    void testClosedLoopSettlesOnPollenDespiteCameraLatency() {
        PollenTracker t = new PollenTracker();
        double pollenWorld = -30.0;            // pollen sits 30 degrees to the right of the start heading (CCW positive)
        double heading = 0.0;
        double spin = 0.0;                     // deg/s
        double maxSpinDegPerSec = 250.0;       // robot turn rate at full twist
        List<double[]> pastHeadings = new ArrayList<>();
        double worstOvershoot = 0.0;
        double now = 0.0;
        for (int step = 0; step < 300; step++) {
            now = step * DT;
            pastHeadings.add(new double[]{now, heading});
            // Camera frame every 33 ms, showing the world as it was 100 ms ago.
            boolean newFrame = step % 2 == 0;
            List<BallDetector.Detection> frame = null;
            if (newFrame) {
                double seenAt = heading;
                for (double[] h : pastHeadings) {
                    if (h[0] <= now - PollenTracker.CAMERA_LATENCY_SEC) seenAt = h[1];
                }
                double bearingRight = -(pollenWorld - seenAt);    // right-positive bearing
                frame = Arrays.asList(pollen(bearingRight, 1.2));
            }
            PollenTracker.Output out = t.update(now, heading, newFrame, frame);
            double twist = out.hasTarget ? out.twist : 0.0;
            // First-order motor response: spin chases (-twist * max) with a 60 ms time constant.
            double targetSpin = -twist * maxSpinDegPerSec;
            spin += (targetSpin - spin) * (DT / 0.06);
            heading += spin * DT;
            worstOvershoot = Math.max(worstOvershoot, pollenWorld - heading);
        }
        assertEquals(pollenWorld, heading, 3.0, "should end up facing the pollen");
        assertTrue(worstOvershoot < 8.0, "overshoot past the pollen: " + worstOvershoot);
    }
}