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
    public static final double STRAFE_COMPENSATION      = 1.25;

    // ---- Smoothness constants (tune these on the robot) ----
    /**
     * Translation speed at full stick, as a fraction of max wheel power. Direction is ALWAYS preserved (the mixer
     * scales every wheel by the same factor), so a bigger number never bends the path or snaps to an axis. It only
     * makes the diagonals a little slower than straight-ahead once a wheel would exceed 100%. 0.85 is a good
     * fast-but-smooth default; 1.0 is the fastest and 0.62 is perfectly uniform speed in every direction.
     */
    public static final double TRANSLATION_SPEED_CAP = 0.85;
    /** Stick response curve: 0 = linear, 1 = fully cubic (gentle near center, still reaches full at the edge). */
    public static final double TRANSLATION_EXPO      = 0.30;
    public static final double TWIST_EXPO            = 0.35;
    /** How fast the command may GROW, in power per second. 7.0 = 0 to full in about 0.14 s (soft launch, no wheelspin). */
    public static final double MAX_TRANSLATION_CHANGE_PER_SEC = 7.0;
    /** How fast the command may SHRINK. Much higher than the launch rate so lifting off the stick feels instant. */
    public static final double MAX_TRANSLATION_DECEL_PER_SEC  = 24.0;
    public static final double MAX_TWIST_CHANGE_PER_SEC       = 9.0;
    public static final double MAX_TWIST_DECEL_PER_SEC        = 30.0;
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
    public static final double BRAKE_ENGAGE_SEC        = 0.03;
    /** Absolute limit on how long the stop assist may push, no matter what the encoders say. This is the runaway watchdog. */
    public static final double BRAKE_MAX_SEC           = 0.18;
    /** A wheel must have been commanded at least this hard for its direction to count as "the way it was driving". */
    public static final double BRAKE_DRIVE_THRESHOLD   = 0.08;

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
        /** Auto-aim turn command (clockwise positive). NaN = not auto-aiming. Replaces the right stick while set. */
        public final double autoTwist;
        /** True = the left stick is robot-relative (forward = where the robot faces), ignoring heading and driver position. */
        public final boolean robotCentric;

        public DriveParameters(double leftStickX, double leftStickY, double rightStickX,
                               double leftTrigger, double headingDegrees,
                               DriverPosition driverPosition,
                               double flVelocity, double frVelocity,
                               double blVelocity, double brVelocity,
                               double dtSeconds) {
            this(leftStickX, leftStickY, rightStickX, leftTrigger, headingDegrees, driverPosition,
                    flVelocity, frVelocity, blVelocity, brVelocity, dtSeconds, Double.NaN, false);
        }

        public DriveParameters(double leftStickX, double leftStickY, double rightStickX,
                               double leftTrigger, double headingDegrees,
                               DriverPosition driverPosition,
                               double flVelocity, double frVelocity,
                               double blVelocity, double brVelocity,
                               double dtSeconds, double autoTwist, boolean robotCentric) {
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
            this.autoTwist = autoTwist;
            this.robotCentric = robotCentric;
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
    private double brakeSeconds = 0.0;
    // Per-wheel stop-assist state, order: FL, FR, BL, BR.
    /** Direction (+1/-1/0) each wheel was last driven hard. Comes from OUR command, never from the encoder's sign. */
    private final int[] driveSign = new int[4];
    private final boolean[] brakeDone = new boolean[4];
    private final double[] lastOut = new double[4];

    // Smoothed commands (stick space), carried between loops for the acceleration limiter.
    private double cmdRight = 0.0;
    private double cmdForward = 0.0;
    private double cmdTwist = 0.0;

    public DriveResult calculate(DriveParameters params) {
        // Bad numbers must never reach a motor: one NaN or Infinity would otherwise spread through every wheel.
        double dt = Math.max(MIN_DT, Math.min(MAX_DT, finite(params.dtSeconds, 0.02)));
        double lsx = finite(params.leftStickX, 0.0);
        double lsy = finite(params.leftStickY, 0.0);
        double rsx = finite(params.rightStickX, 0.0);
        double headingDeg = finite(params.headingDegrees, 0.0);
        boolean autoAim = !Double.isNaN(params.autoTwist) && !Double.isInfinite(params.autoTwist);

        // ================= STICKS =================
        // Convert to (x = right, y = forward). The deadzone is circular and rescaled, so direction is never distorted.
        double[] stick = radialDeadzone(lsx, -lsy, JOYSTICK_DEADZONE);
        double twistIn = axisDeadzone(rsx, JOYSTICK_DEADZONE);  // positive = clockwise
        // The driver's right stick always wins; auto-aim only steers while it is centered.
        boolean aimActive = autoAim && twistIn == 0.0;
        if (aimActive) {
            twistIn = Math.max(-1.0, Math.min(1.0, params.autoTwist));
        }

        boolean noInput = stick[0] == 0.0 && stick[1] == 0.0 && twistIn == 0.0;
        noInputSeconds = noInput ? noInputSeconds + dt : 0.0;
        boolean braking = noInputSeconds >= BRAKE_ENGAGE_SEC;

        // ================= RESPONSE CURVE =================
        // Shape the stick's magnitude only (direction untouched), then scale so full stick = TRANSLATION_SPEED_CAP.
        double mag = Math.hypot(stick[0], stick[1]);
        double targetRight = 0.0;
        double targetForward = 0.0;
        if (mag > 1e-9) {
            double shaped = ((1.0 - TRANSLATION_EXPO) * mag + TRANSLATION_EXPO * mag * mag * mag) * TRANSLATION_SPEED_CAP;
            targetRight   = stick[0] / mag * shaped;
            targetForward = stick[1] / mag * shaped;
        }
        // Auto-aim output is already a finished command, so it skips the stick curve.
        double tMag = Math.abs(twistIn);
        double targetTwist = !aimActive
                ? Math.copySign((1.0 - TWIST_EXPO) * tMag + TWIST_EXPO * tMag * tMag * tMag, twistIn)
                : twistIn;

        // ================= ACCELERATION SMOOTHING =================
        // Limit how fast the command can change. The limit acts on the 2D vector's length, not per axis, so a
        // curved stick path stays a curved path instead of getting squared off. Growing is gentle; shrinking is
        // fast, so there is no input lag when you back off the stick.
        if (braking) {
            cmdRight = 0.0;
            cmdForward = 0.0;
            cmdTwist = 0.0;
        } else {
            double dx = targetRight - cmdRight;
            double dy = targetForward - cmdForward;
            double dist = Math.hypot(dx, dy);
            boolean shrinking = Math.hypot(targetRight, targetForward) < Math.hypot(cmdRight, cmdForward);
            double maxStep = (shrinking ? MAX_TRANSLATION_DECEL_PER_SEC : MAX_TRANSLATION_CHANGE_PER_SEC) * dt;
            if (dist <= maxStep) {
                cmdRight = targetRight;
                cmdForward = targetForward;
            } else {
                cmdRight   += dx / dist * maxStep;
                cmdForward += dy / dist * maxStep;
            }
            boolean twistShrinking = Math.abs(targetTwist) < Math.abs(cmdTwist);
            double twistStep = (twistShrinking ? MAX_TWIST_DECEL_PER_SEC : MAX_TWIST_CHANGE_PER_SEC) * dt;
            cmdTwist += Math.max(-twistStep, Math.min(twistStep, targetTwist - cmdTwist));
        }

        // ================= FIELD-CENTRIC ROTATION =================
        // Step 1: driver position rotates the stick into field space (CCW by offset).
        // Step 2: robot heading rotates field space into robot space (CW by heading).
        // Both are rotations, so they combine into one angle. Robot-centric mode skips both.
        double headingRad = Math.toRadians(headingDeg);
        double angle = params.robotCentric ? 0.0
                : Math.toRadians(params.driverPosition.offsetDegrees) - headingRad;
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);

        double robotRight   = cmdRight * cos - cmdForward * sin;
        double robotForward = cmdRight * sin + cmdForward * cos;

        // ================= SLOW MODE =================
        boolean slow = finite(params.leftTrigger, 0.0) > SLOW_MODE_TRIGGER_THRESH;
        double driveMult = slow ? SLOW_MODE_DRIVE_MULT : 1.0;
        double twistMult = slow ? SLOW_MODE_TWIST_MULT : 1.0;

        double d = robotForward * driveMult;
        double s = robotRight * STRAFE_COMPENSATION * driveMult;
        double t = cmdTwist * twistMult;

        double[] out = new double[4];
        if (braking) {
            // ================= STOP ASSIST =================
            // Briefly pushes each wheel against the direction it was driven, to kill momentum fast.
            // Why this cannot run away: the push direction comes from OUR last command, not from the encoder's sign,
            // so a wrongly-signed encoder can only cancel the assist, never feed it. It also ends per wheel the
            // moment that wheel stops or reverses, and ends for everything after BRAKE_MAX_SEC no matter what.
            double[] vel = {finite(params.flVelocity, 0.0), finite(params.frVelocity, 0.0),
                    finite(params.blVelocity, 0.0), finite(params.brVelocity, 0.0)};
            if (!wasBraking) {
                brakeSeconds = 0.0;
                for (int k = 0; k < 4; k++) {
                    brakeDone[k] = driveSign[k] == 0;
                }
            }
            brakeSeconds += dt;
            if (brakeSeconds > BRAKE_MAX_SEC) {
                for (int k = 0; k < 4; k++) {
                    brakeDone[k] = true;
                    driveSign[k] = 0;
                }
            }
            for (int k = 0; k < 4; k++) {
                if (brakeDone[k]) continue;
                double along = vel[k] / MAX_WHEEL_TICKS_PER_SEC * driveSign[k];   // + = still moving the way we drove it
                if (along <= BRAKE_MIN_SPEED) {
                    brakeDone[k] = true;
                    driveSign[k] = 0;
                } else {
                    out[k] = -driveSign[k] * Math.min(BRAKE_MAX_POWER, BRAKE_GAIN * along);
                }
            }
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
            out[0] = fl / max;
            out[1] = fr / max;
            out[2] = bl / max;
            out[3] = br / max;
            for (int k = 0; k < 4; k++) {
                if (Math.abs(out[k]) >= BRAKE_DRIVE_THRESHOLD) {
                    driveSign[k] = out[k] > 0 ? 1 : -1;
                }
            }
        }
        wasBraking = braking;

        for (int k = 0; k < 4; k++) {
            out[k] = Math.max(-1.0, Math.min(1.0, finite(out[k], 0.0)));
            lastOut[k] = out[k];
        }
        ChassisMotors powers = new ChassisMotors(out[0], out[1], out[2], out[3]);

        return new DriveResult(powers, headingRad, cmdForward, cmdRight,
                robotForward, robotRight, cmdTwist, slow, braking);
    }

    /** Returns the value, or the fallback if it is NaN or infinite. */
    private static double finite(double v, double fallback) {
        return (Double.isNaN(v) || Double.isInfinite(v)) ? fallback : v;
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