package org.firstinspires.ftc.teamcode;

/**
 * Field-centric mecanum drive math. No FTC SDK dependencies, so it can be
 * compiled and tested on a laptop.
 *
 * Conventions used everywhere in this file:
 *  - Vectors are (x = right, y = forward).
 *  - Headings are degrees, counter-clockwise positive (the FTC IMU convention).
 *  - Heading 0 is whatever direction the robot faced when the IMU was last
 *    zeroed (start of the OpMode, or the Options button). That direction is "NORTH".
 *  - Positive twist turns the robot clockwise (right).
 *  - Positive wheel power drives that wheel forward (DriveHardware handles motor direction).
 *  - The drive motors run in RUN_WITHOUT_ENCODER (plain power) mode, so power is a motor voltage fraction.
 */
public class IamYouLogic {

    // ---- Driving constants ----
    public static final double JOYSTICK_DEADZONE        = 0.05;
    public static final double SLOW_MODE_DRIVE_MULT     = 0.35;
    public static final double SLOW_MODE_TWIST_MULT     = 0.50;
    public static final double SLOW_MODE_TRIGGER_THRESH = 0.10;

    /**
     * STRAFE CALIBRATION. Mecanum robots strafe slower than they drive, and a weak strafe makes diagonals bend
     * toward forward/back (feels like snapping to an axis). This multiplies the sideways command to cancel that.
     * To tune: drive forward at the same stick amount for 1 s, then strafe for 1 s, and measure both distances.
     * STRAFE_COMPENSATION = forward distance / strafe distance. 1.0 = no compensation.
     */
    public static final double STRAFE_COMPENSATION      = 1.30;

    // ---- Smoothness constants (tune these on the robot) ----
    /**
     * Translation speed at full stick, as a fraction of max wheel power. A mecanum drive can only reach about
     * 0.6-0.7 of its axis speed on the diagonals, so asking for more than that in every direction clips the
     * diagonals and turns a circle of stick motion into a diamond. Raise toward 1.0 for more top speed at the
     * cost of a more diamond-shaped feel.
     */
    public static final double TRANSLATION_SPEED_CAP = 0.62;
    /** Stick response curve: 0 = linear, 1 = fully cubic (gentle near center, still reaches full at the edge). */
    public static final double TRANSLATION_EXPO      = 0.35;
    public static final double TWIST_EXPO            = 0.35;
    /** Acceleration limit on the translation command, in power per second. 6.0 means 0 to full in about 0.12 s. Lower = softer. */
    public static final double MAX_TRANSLATION_CHANGE_PER_SEC = 6.0;
    public static final double MAX_TWIST_CHANGE_PER_SEC       = 8.0;
    public static final double MIN_DT = 0.001;
    public static final double MAX_DT = 0.100;

    /**
     * Static-friction compensation. A motor driving a loaded wheel does nothing below roughly this power, so a
     * wheel that should creep at 0.05 just sits there and the robot "snaps" to the direction of the other wheels.
     * Every nonzero wheel command gets this much extra push in its own direction (blended smoothly through zero).
     */
    public static final double STATIC_FRICTION_POWER = 0.06;
    /** Wheel commands smaller than this get a proportionally smaller friction boost, so the sign flip at zero is smooth. */
    public static final double STATIC_FRICTION_BLEND = 0.15;

    // ---- Stop assist constants (tune these on the robot) ----
    /** Wheel speed in ticks/sec at full power. goBILDA 312/435/1150 RPM motors are all about 2700-2800. */
    public static final double MAX_WHEEL_TICKS_PER_SEC = 2700.0;
    /** How hard to brake relative to wheel speed. Higher = stops sooner. */
    public static final double BRAKE_GAIN              = 4.0;
    /** Hard ceiling on braking power. Lower this if the stop feels like whiplash. */
    public static final double BRAKE_MAX_POWER         = 0.6;
    /** Below this fraction of max speed a wheel counts as stopped. */
    public static final double BRAKE_MIN_SPEED         = 0.05;
    /** Seconds of zero stick input before braking engages. Stops a quick stick flick through center from triggering a brake. */
    public static final double BRAKE_ENGAGE_SEC        = 0.04;

    public enum DriverPosition {
        SOUTH(0.0),     // behind the robot's starting orientation (default)
        EAST(90.0),     // to the east side, looking west
        NORTH(180.0),   // in front of the robot, looking south
        WEST(-90.0);    // to the west side, looking east

        /** Counter-clockwise rotation applied to the stick vector so "stick forward" means "away from the driver". */
        public final double offsetDegrees;

        DriverPosition(double offsetDegrees) {
            this.offsetDegrees = offsetDegrees;
        }
    }

    public enum Cardinal { NORTH, WEST, SOUTH, EAST }

    public static class DriveParameters {
        public final double leftStickX;
        public final double leftStickY;      // raw FTC value: up is negative
        public final double rightStickX;
        public final double leftTrigger;
        public final double headingDegrees;  // IMU yaw, zeroed at start/reset
        public final DriverPosition driverPosition;
        public final double flVelocity;      // ticks/sec, positive = wheel driving robot forward
        public final double frVelocity;
        public final double blVelocity;
        public final double brVelocity;
        public final double dtSeconds;       // time since the previous loop, for acceleration smoothing

        public DriveParameters(double leftStickX, double leftStickY, double rightStickX,
                               double leftTrigger, double headingDegrees,
                               DriverPosition driverPosition,
                               double flVelocity, double frVelocity,
                               double blVelocity, double brVelocity,
                               double dtSeconds) {
            this.leftStickX = leftStickX;
            this.leftStickY = leftStickY;
            this.rightStickX = rightStickX;
            this.leftTrigger = leftTrigger;
            this.headingDegrees = headingDegrees;
            this.driverPosition = driverPosition;
            this.flVelocity = flVelocity;
            this.frVelocity = frVelocity;
            this.blVelocity = blVelocity;
            this.brVelocity = brVelocity;
            this.dtSeconds = dtSeconds;
        }
    }

    public static class DriveResult {
        public final ChassisMotors powers;
        public final double headingRadians;
        public final double stickForward;   // smoothed command, after driver-position rotation
        public final double stickRight;
        public final double robotForward;   // after field-centric rotation
        public final double robotRight;
        public final double twist;
        public final boolean slowMode;
        public final boolean braking;

        public DriveResult(ChassisMotors powers, double headingRadians,
                           double stickForward, double stickRight,
                           double robotForward, double robotRight,
                           double twist, boolean slowMode, boolean braking) {
            this.powers = powers;
            this.headingRadians = headingRadians;
            this.stickForward = stickForward;
            this.stickRight = stickRight;
            this.robotForward = robotForward;
            this.robotRight = robotRight;
            this.twist = twist;
            this.slowMode = slowMode;
            this.braking = braking;
        }
    }

    // Seconds of continuous zero stick input (for the stop assist).
    private double noInputSeconds = 0.0;
    private boolean wasBraking = false;
    // Per-wheel brake state, order: FL, FR, BL, BR.
    private final int[] brakeDirection = new int[4];
    private final boolean[] brakeDone = new boolean[4];

    // Smoothed commands (stick space), carried between loops for the acceleration limiter.
    private double cmdRight = 0.0;
    private double cmdForward = 0.0;
    private double cmdTwist = 0.0;

    public DriveResult calculate(DriveParameters params) {
        double dt = Math.max(MIN_DT, Math.min(MAX_DT, params.dtSeconds));

        // ================= STICKS =================
        // Convert to (x = right, y = forward). The deadzone is circular and rescaled, so direction is never distorted.
        double[] stick = radialDeadzone(params.leftStickX, -params.leftStickY, JOYSTICK_DEADZONE);
        double twistIn = axisDeadzone(params.rightStickX, JOYSTICK_DEADZONE);  // positive = clockwise

        boolean noInput = stick[0] == 0.0 && stick[1] == 0.0 && twistIn == 0.0;
        noInputSeconds = noInput ? noInputSeconds + dt : 0.0;
        boolean braking = noInputSeconds >= BRAKE_ENGAGE_SEC;

        // ================= RESPONSE CURVE =================
        // Shape the stick's magnitude only (direction untouched), then scale so full stick = TRANSLATION_SPEED_CAP.
        double mag = Math.hypot(stick[0], stick[1]);
        double targetRight = 0.0;
        double targetForward = 0.0;
        if (mag > 0.0) {
            double shaped = ((1.0 - TRANSLATION_EXPO) * mag + TRANSLATION_EXPO * mag * mag * mag) * TRANSLATION_SPEED_CAP;
            targetRight   = stick[0] / mag * shaped;
            targetForward = stick[1] / mag * shaped;
        }
        double tMag = Math.abs(twistIn);
        double targetTwist = Math.copySign(
                (1.0 - TWIST_EXPO) * tMag + TWIST_EXPO * tMag * tMag * tMag, twistIn);

        // ================= ACCELERATION SMOOTHING =================
        // Limit how fast the command can change. The limit acts on the 2D vector's length, not per axis,
        // so a curved stick path stays a curved path instead of getting squared off.
        if (braking) {
            cmdRight = 0.0;
            cmdForward = 0.0;
            cmdTwist = 0.0;
        } else {
            double dx = targetRight - cmdRight;
            double dy = targetForward - cmdForward;
            double dist = Math.hypot(dx, dy);
            double maxStep = MAX_TRANSLATION_CHANGE_PER_SEC * dt;
            if (dist <= maxStep) {
                cmdRight = targetRight;
                cmdForward = targetForward;
            } else {
                cmdRight   += dx / dist * maxStep;
                cmdForward += dy / dist * maxStep;
            }
            double twistStep = MAX_TWIST_CHANGE_PER_SEC * dt;
            cmdTwist += Math.max(-twistStep, Math.min(twistStep, targetTwist - cmdTwist));
        }

        // ================= FIELD-CENTRIC ROTATION =================
        // Step 1: driver position rotates the stick into field space (CCW by offset).
        // Step 2: robot heading rotates field space into robot space (CW by heading).
        // Both are rotations, so they combine into one angle.
        double headingRad = Math.toRadians(params.headingDegrees);
        double angle = Math.toRadians(params.driverPosition.offsetDegrees) - headingRad;
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);

        double robotRight   = cmdRight * cos - cmdForward * sin;
        double robotForward = cmdRight * sin + cmdForward * cos;

        // ================= SLOW MODE =================
        boolean slow = params.leftTrigger > SLOW_MODE_TRIGGER_THRESH;
        double driveMult = slow ? SLOW_MODE_DRIVE_MULT : 1.0;
        double twistMult = slow ? SLOW_MODE_TWIST_MULT : 1.0;

        double d = robotForward * driveMult;
        double s = robotRight * STRAFE_COMPENSATION * driveMult;
        double t = cmdTwist * twistMult;

        ChassisMotors powers;
        if (braking) {
            double[] vel = {params.flVelocity, params.frVelocity, params.blVelocity, params.brVelocity};
            if (!wasBraking) {
                // Remember which way each wheel was moving when braking began.
                for (int k = 0; k < 4; k++) {
                    double speed = vel[k] / MAX_WHEEL_TICKS_PER_SEC;
                    brakeDone[k] = Math.abs(speed) < BRAKE_MIN_SPEED;
                    brakeDirection[k] = brakeDone[k] ? 0 : (speed > 0 ? 1 : -1);
                }
            }
            double[] p = new double[4];
            for (int k = 0; k < 4; k++) {
                if (brakeDone[k]) continue;
                double speed = vel[k] / MAX_WHEEL_TICKS_PER_SEC;
                // Brake only while the wheel is still moving the way it was. Once it slows to a stop or crosses
                // zero, stop braking for good: pushing a wheel backwards through zero is what causes shaking.
                if (Math.abs(speed) < BRAKE_MIN_SPEED || (speed > 0 ? 1 : -1) != brakeDirection[k]) {
                    brakeDone[k] = true;
                } else {
                    p[k] = -Math.max(-BRAKE_MAX_POWER, Math.min(BRAKE_MAX_POWER, BRAKE_GAIN * speed));
                }
            }
            powers = new ChassisMotors(p[0], p[1], p[2], p[3]);
        } else {
            // ================= MECANUM MIXER (positive t = clockwise) =================
            double fl = frictionBoost(d + s + t);
            double fr = frictionBoost(d - s - t);
            double bl = frictionBoost(d - s + t);
            double br = frictionBoost(d + s - t);

            // Scale by the largest wheel so direction is preserved at any heading.
            double max = Math.max(1.0,
                    Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                            Math.max(Math.abs(bl), Math.abs(br))));
            powers = new ChassisMotors(fl / max, fr / max, bl / max, br / max);
        }
        wasBraking = braking;

        return new DriveResult(powers, headingRad, cmdForward, cmdRight,
                robotForward, robotRight, cmdTwist, slow, braking);
    }

    /** Adds the static-friction push in the command's direction; continuous through zero. */
    private static double frictionBoost(double p) {
        return p + STATIC_FRICTION_POWER * Math.max(-1.0, Math.min(1.0, p / STATIC_FRICTION_BLEND));
    }

    /** Which cardinal the robot is facing, relative to its zeroed start direction. Yaw is CCW positive, so +90 = left = WEST. */
    public static Cardinal snapToCardinal(double headingDeg) {
        double h = headingDeg % 360.0;
        if (h >= 180.0)  h -= 360.0;
        if (h < -180.0)  h += 360.0;

        if      (h >= -45.0  && h <  45.0)  return Cardinal.NORTH;
        else if (h >=  45.0  && h < 135.0)  return Cardinal.WEST;
        else if (h >= -135.0 && h < -45.0)  return Cardinal.EAST;
        else                                return Cardinal.SOUTH;
    }

    /** Circular deadzone that rescales so output ramps smoothly from 0 at the edge to 1 at full deflection. */
    private static double[] radialDeadzone(double x, double y, double deadzone) {
        double mag = Math.hypot(x, y);
        if (mag < deadzone) return new double[]{0.0, 0.0};
        double clamped = Math.min(mag, 1.0);
        double scaled = (clamped - deadzone) / (1.0 - deadzone);
        return new double[]{x / mag * scaled, y / mag * scaled};
    }

    private static double axisDeadzone(double value, double deadzone) {
        double mag = Math.abs(value);
        if (mag < deadzone) return 0.0;
        double scaled = (Math.min(mag, 1.0) - deadzone) / (1.0 - deadzone);
        return Math.copySign(scaled, value);
    }
}
