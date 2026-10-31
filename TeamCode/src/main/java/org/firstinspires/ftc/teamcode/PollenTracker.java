package org.firstinspires.ftc.teamcode;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

/**
 * Turns camera sightings of yellow POLLEN into a turn command that points the robot's front at the pollen.
 * No FTC SDK dependencies, so it runs (and is tested) on a laptop. It consumes the {@link BallDetector}
 * detections the repo already produces from "Webcam 1".
 *
 * <p>How it works:
 * <ol>
 *   <li>Every new camera frame, keep only POLLEN that is a believable size and distance.</li>
 *   <li>Group pollen that sits within {@link #GROUP_RADIUS_METRES} of its neighbours. If there are several
 *       groups, pick the biggest one (staying on the one already being tracked if it is still there).</li>
 *       The aim point is the MIDDLE of that group.</li>
 *   <li>Camera frames are about {@link #CAMERA_LATENCY_SEC} old, so the bearing is added to the heading the robot
 *       had when the picture was taken (kept in a short history), giving a fixed target heading. Between frames the
 *       robot simply turns toward that heading using the IMU, which updates far faster than the camera.</li>
 *   <li>A PD controller turns the heading error into a clockwise-positive twist command.</li>
 * </ol>
 * If no pollen has been seen for {@link #LOST_TIMEOUT_SEC}, the tracker reports no target and the driver keeps
 * manual control of turning.</p>
 *
 * <p>Conventions: heading is degrees, counter-clockwise positive (IMU yaw). A detection's bearing is
 * right-positive, so a pollen to the right has a target heading LESS than the current heading.</p>
 */
public final class PollenTracker {

    /** Camera distance in front of the robot's center, metres (robot-config/pickup.json). */
    public static final double CAMERA_FORWARD_METRES = 0.16;
    /** Estimated age of a camera frame by the time the OpMode sees it. Tune if the aim lags or overshoots. */
    public static final double CAMERA_LATENCY_SEC = 0.10;
    /** Pollen closer than this to a neighbour is the same group. */
    public static final double GROUP_RADIUS_METRES = 0.30;
    /** Ignore blobs smaller than this many pixels in radius (noise). */
    public static final double MIN_RADIUS_PIXELS = 2.5;
    public static final double MIN_RANGE_METRES = 0.10;
    public static final double MAX_RANGE_METRES = 3.50;
    /** Stop aiming this long after the last sighting. */
    public static final double LOST_TIMEOUT_SEC = 0.50;
    /** A group within this many degrees of the current target counts as "the same group". */
    public static final double STICKY_DEGREES = 12.0;

    /** Turn command per radian of heading error. */
    public static final double KP = 2.2;
    /** Damping: turn command removed per rad/s of spin, so it eases in instead of overshooting. */
    public static final double KD = 0.10;
    public static final double MAX_TWIST = 0.70;
    /** Close enough: no turn command inside this many degrees, which stops hunting back and forth. */
    public static final double DEADBAND_DEGREES = 1.5;
    /** Fraction of the way a new sighting pulls the target (1 = jump straight to it, lower = smoother). */
    public static final double TARGET_BLEND = 0.6;

    /** What the tracker wants this loop. */
    public static final class Output {
        public final boolean hasTarget;
        /** Clockwise-positive turn command in -1..1, or 0 when there is no target. */
        public final double twist;
        /** Heading error in degrees, CCW positive (target minus heading); 0 when there is no target. */
        public final double errorDegrees;
        /** How many pollen are in the group being aimed at. */
        public final int groupSize;

        Output(boolean hasTarget, double twist, double errorDegrees, int groupSize) {
            this.hasTarget = hasTarget;
            this.twist = twist;
            this.errorDegrees = errorDegrees;
            this.groupSize = groupSize;
        }
    }

    private static final Output NONE = new Output(false, 0.0, 0.0, 0);

    // Heading history as {timeSeconds, unwrappedHeadingDegrees}.
    private final ArrayDeque<double[]> history = new ArrayDeque<>();
    private boolean haveHeading = false;
    private double lastRawHeading = 0.0;
    private double unwrapped = 0.0;
    private double spinRadPerSec = 0.0;

    private boolean haveTarget = false;
    private double targetHeading = 0.0;   // unwrapped degrees
    private double lastSeenSeconds = Double.NEGATIVE_INFINITY;
    private int groupSize = 0;

    /** Forget the current target (call when tracking is switched off). */
    public void reset() {
        haveTarget = false;
        lastSeenSeconds = Double.NEGATIVE_INFINITY;
        groupSize = 0;
        spinRadPerSec = 0.0;
    }

    /**
     * @param nowSeconds     a steadily increasing clock
     * @param headingDegrees IMU yaw, CCW positive
     * @param newFrame       true only on the loop where the camera delivered a frame not seen before
     * @param detections     that frame's detections (ignored unless {@code newFrame}); may be null
     */
    public Output update(double nowSeconds, double headingDegrees, boolean newFrame,
                         List<BallDetector.Detection> detections) {
        if (Double.isNaN(nowSeconds) || Double.isNaN(headingDegrees)
                || Double.isInfinite(nowSeconds) || Double.isInfinite(headingDegrees)) {
            return NONE;
        }

        // Track heading without the +-180 wrap so subtraction and interpolation are always safe.
        if (!haveHeading) {
            unwrapped = headingDegrees;
            haveHeading = true;
        } else {
            unwrapped += wrap180(headingDegrees - lastRawHeading);
        }
        lastRawHeading = headingDegrees;

        double[] previous = history.peekLast();
        if (previous != null) {
            double dt = nowSeconds - previous[0];
            if (dt > 1e-3) {
                double raw = Math.toRadians(unwrapped - previous[1]) / dt;
                spinRadPerSec = 0.6 * spinRadPerSec + 0.4 * raw;
            }
        }
        history.addLast(new double[]{nowSeconds, unwrapped});
        while (history.size() > 2 && history.peekFirst()[0] < nowSeconds - 1.0) {
            history.removeFirst();
        }

        if (newFrame && detections != null) {
            acceptFrame(nowSeconds, detections);
        }

        if (!haveTarget || nowSeconds - lastSeenSeconds > LOST_TIMEOUT_SEC) {
            haveTarget = false;
            return NONE;
        }

        double errorDeg = targetHeading - unwrapped;
        double twist = 0.0;
        if (Math.abs(errorDeg) > DEADBAND_DEGREES) {
            // Error is CCW-positive and twist is CW-positive, hence the minus on P and the plus on the damping.
            twist = -KP * Math.toRadians(errorDeg) + KD * spinRadPerSec;
            twist = Math.max(-MAX_TWIST, Math.min(MAX_TWIST, twist));
        }
        return new Output(true, twist, errorDeg, groupSize);
    }

    private void acceptFrame(double nowSeconds, List<BallDetector.Detection> detections) {
        // Pollen only, in the robot's ground plane: x = right, y = forward from the robot's center.
        List<double[]> points = new ArrayList<>();
        for (BallDetector.Detection d : detections) {
            if (d.kind() != BallDetector.Kind.POLLEN) continue;
            if (d.apparentRadiusPixels() < MIN_RADIUS_PIXELS) continue;
            double range = d.rangeMetres();
            if (!(range >= MIN_RANGE_METRES && range <= MAX_RANGE_METRES)) continue;
            double bearing = d.bearingRadians();
            if (Double.isNaN(bearing)) continue;
            points.add(new double[]{range * Math.tan(bearing), range + CAMERA_FORWARD_METRES});
        }
        if (points.isEmpty()) return;

        double captureHeading = headingAt(nowSeconds - CAMERA_LATENCY_SEC);

        // Single-link grouping: anything within GROUP_RADIUS of a group member joins the group.
        int n = points.size();
        int[] group = new int[n];
        java.util.Arrays.fill(group, -1);
        int groups = 0;
        for (int i = 0; i < n; i++) {
            if (group[i] != -1) continue;
            ArrayDeque<Integer> open = new ArrayDeque<>();
            open.add(i);
            group[i] = groups;
            while (!open.isEmpty()) {
                int a = open.poll();
                for (int b = 0; b < n; b++) {
                    if (group[b] == -1 && Math.hypot(points.get(a)[0] - points.get(b)[0],
                            points.get(a)[1] - points.get(b)[1]) <= GROUP_RADIUS_METRES) {
                        group[b] = groups;
                        open.add(b);
                    }
                }
            }
            groups++;
        }

        // Score each group: more pollen is better, closer is better, and staying on the current target is better.
        double bestScore = Double.NEGATIVE_INFINITY;
        double bestHeading = 0.0;
        int bestSize = 0;
        for (int g = 0; g < groups; g++) {
            double sx = 0.0;
            double sy = 0.0;
            int count = 0;
            for (int i = 0; i < n; i++) {
                if (group[i] == g) {
                    sx += points.get(i)[0];
                    sy += points.get(i)[1];
                    count++;
                }
            }
            double cx = sx / count;   // count >= 1: every group was created by at least one point
            double cy = sy / count;
            double bearingDeg = Math.toDegrees(Math.atan2(cx, cy));
            double headingForGroup = captureHeading - bearingDeg;
            double score = count - 0.3 * Math.hypot(cx, cy);
            if (haveTarget && Math.abs(headingForGroup - targetHeading) <= STICKY_DEGREES) {
                score += 2.0;
            }
            if (score > bestScore) {
                bestScore = score;
                bestHeading = headingForGroup;
                bestSize = count;
            }
        }

        if (haveTarget && Math.abs(bestHeading - targetHeading) <= 30.0) {
            targetHeading += TARGET_BLEND * (bestHeading - targetHeading);
        } else {
            targetHeading = bestHeading;
        }
        haveTarget = true;
        lastSeenSeconds = nowSeconds;
        groupSize = bestSize;
    }

    /** The robot's (unwrapped) heading at an earlier time, interpolated from the history. */
    private double headingAt(double timeSeconds) {
        double[] before = null;
        for (double[] sample : history) {
            if (sample[0] >= timeSeconds) {
                if (before == null) return sample[1];
                double span = sample[0] - before[0];
                if (span < 1e-6) return sample[1];
                double f = (timeSeconds - before[0]) / span;
                return before[1] + f * (sample[1] - before[1]);
            }
            before = sample;
        }
        return before == null ? unwrapped : before[1];
    }

    static double wrap180(double degrees) {
        double h = degrees % 360.0;
        if (h > 180.0) h -= 360.0;
        if (h <= -180.0) h += 360.0;
        return h;
    }
}