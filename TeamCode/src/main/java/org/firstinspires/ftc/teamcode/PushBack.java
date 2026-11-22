package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;

/**
 * The software brake's answer to a shove: push back at full power, drive the pusher off, then come home.
 *
 * <p>Without it the brake only ever returns: shoved six inches, it gives the six inches and walks
 * back. With it, the moment the robot has been moved {@link #PUSH_DETECT_INCHES} it stops giving
 * ground and drives the other way with every wheel at full power, back through the spot it was
 * braked at and {@link #OVERSHOOT_INCHES} beyond, taking whatever was pushing it along. Then it
 * hands back to Pedro's hold, which returns it to the spot:</p>
 *
 * <pre>
 *   &lt;----------          shoved
 *   ----------------->  pushes back, past the spot, at full power
 *          &lt;-------    returns to the spot
 * </pre>
 *
 * <p>Full power means the largest wheel command is exactly 1: as hard as the motors can push in
 * that direction, however small the shove. Held there against a robot that will not move, it gives
 * up after {@link #MAX_PUSH_SECONDS} so a wall cannot pin the motors at stall, lets the hold take
 * over and, if it is still being moved, pushes again.</p>
 *
 * <p>No hardware here: poses in, wheel powers out, so it is tested on a laptop. Inches and radians,
 * Pedro's field frame (heading counter-clockwise, 0 facing +X).</p>
 */
final class PushBack {

    /** Moved this far from the spot (or from the closest it got on the way back) counts as being pushed. */
    static final double PUSH_DETECT_INCHES = 1.0;
    /** How far past the spot it drives the pusher before turning round and coming home. */
    static final double OVERSHOOT_INCHES = 6.0;
    /** Longest a single push-back lasts, for a robot that will not budge or a wall in the way. */
    static final double MAX_PUSH_SECONDS = 1.0;
    /**
     * After a push-back the robot is still rolling outward, which is not a new push. Detection waits
     * this long, then measures afresh from wherever the robot is.
     */
    static final double SETTLE_SECONDS = 0.3;
    /** Heading correction while pushing back, in turn power per radian off the braked heading. */
    static final double HEADING_GAIN = 2.0;
    /** Most of the push the turn may take; the rest is spent driving the pusher off. */
    static final double MAX_TURN = 0.5;

    enum Mode { HOLDING, PUSHING_BACK }

    private final double spotX;
    private final double spotY;
    private final double spotHeading;

    private Mode mode = Mode.HOLDING;
    /** Closest the robot has come to the spot since detection last (re)started. */
    private double closest;
    private double settleLeft = 0.0;
    /** Unit vector the push-back drives along: from where the robot was shoved to, toward the spot. */
    private double dirX;
    private double dirY;
    private double pushSeconds;
    private int pushes = 0;

    PushBack(double spotX, double spotY, double spotHeading) {
        this.spotX = spotX;
        this.spotY = spotY;
        this.spotHeading = spotHeading;
        this.closest = 0.0;
    }

    Mode mode() {
        return mode;
    }

    /** How many times it has pushed back since the brake engaged. */
    int pushes() {
        return pushes;
    }

    /**
     * One control step. Returns FL, FR, BL, BR at full power while pushing back, or null while the
     * hold should drive (no push, or on the way home).
     */
    double[] update(double x, double y, double heading, double dt) {
        double distance = Math.hypot(x - spotX, y - spotY);
        if (mode == Mode.PUSHING_BACK) {
            pushSeconds += dt;
            double past = (x - spotX) * dirX + (y - spotY) * dirY;
            if (past >= OVERSHOOT_INCHES || pushSeconds >= MAX_PUSH_SECONDS) {
                mode = Mode.HOLDING;
                settleLeft = SETTLE_SECONDS;
                closest = distance;
                return null;
            }
            return fullPowerToward(spotX + dirX * OVERSHOOT_INCHES, spotY + dirY * OVERSHOOT_INCHES, x, y, heading);
        }

        if (settleLeft > 0.0) {
            settleLeft -= dt;
            closest = distance;
            return null;
        }
        closest = Math.min(closest, distance);
        if (distance > closest + PUSH_DETECT_INCHES) {
            mode = Mode.PUSHING_BACK;
            dirX = (spotX - x) / distance;
            dirY = (spotY - y) / distance;
            pushSeconds = 0.0;
            pushes++;
            return fullPowerToward(spotX + dirX * OVERSHOOT_INCHES, spotY + dirY * OVERSHOOT_INCHES, x, y, heading);
        }
        return null;
    }

    /** Wheel powers driving from (x, y) toward the target, scaled up so the largest wheel is exactly 1. */
    private double[] fullPowerToward(double targetX, double targetY, double x, double y, double heading) {
        double dx = targetX - x;
        double dy = targetY - y;
        double length = Math.hypot(dx, dy);
        double fieldX = length > 1e-9 ? dx / length : dirX;
        double fieldY = length > 1e-9 ? dy / length : dirY;
        // Field direction into the robot's frame: forward along its heading, left 90 degrees round from it.
        double forward = Math.cos(heading) * fieldX + Math.sin(heading) * fieldY;
        double left = -Math.sin(heading) * fieldX + Math.cos(heading) * fieldY;
        double headingError = Math.atan2(Math.sin(spotHeading - heading), Math.cos(spotHeading - heading));
        double turn = Math.max(-MAX_TURN, Math.min(MAX_TURN, HEADING_GAIN * headingError));

        double[] wheels = SoftwareBrake.WheelPowers.unnormalized(new DrivePowers(forward, left, turn));
        double max = 0.0;
        for (double w : wheels) {
            max = Math.max(max, Math.abs(w));
        }
        for (int i = 0; i < wheels.length; i++) {
            wheels[i] /= max;
        }
        return wheels;
    }
}
