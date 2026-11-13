package org.firstinspires.ftc.teamcode;

/**
 * Mecanum traction control: compares each wheel's encoder speed with the ground speed the Pinpoint measures, and
 * scales all four wheel powers by one factor to hold the worst wheel's slip near {@link TractionConstants#TARGET_SLIP}.
 * One factor for all four keeps the robot driving in the commanded direction; cutting one wheel would curve it.
 *
 * No FTC SDK dependencies, so it is tested on a plain JVM.
 *
 * Conventions:
 *  - Wheel velocities are encoder ticks per second, positive = that wheel driving the robot forward
 *    ({@link DriveHardware} sets motor directions so this holds).
 *  - Pinpoint velocity is in the field frame, metres per second: X is the robot's forward at the Pinpoint's last
 *    reset, Y is its left. Heading is radians, counter-clockwise positive, as the Pinpoint reports it.
 *  - Slip = (measured − expected) / max(|expected|, floor). Positive = wheel faster than the ground (spin),
 *    negative = slower or reversed (lockup). Both count.
 *
 * Each loop: {@link #observe} the Pinpoint first (a field-centric OpMode needs the trusted heading before it can
 * mix), then {@link #update} with the mixed powers. Holds state between loops (the power scale and the last good
 * Pinpoint reading), so use one instance per OpMode.
 */
public class TractionControlLogic {

    /** What the Pinpoint said about its own reading this loop. */
    public enum PinpointHealth {
        READY,
        /** One corrupt I²C read. The previous good reading stands in for a single loop. */
        BAD_READ,
        /** Any other non-ready status: calibrating, not ready, a pod missing, IMU runaway. */
        FAULT
    }

    /** What the driver's LED shows. */
    public enum GripState { FULL_GRIP, LIMITED, HEAVY_SLIP, ODOMETRY_FAULT }

    /** Measured wheel speeds, ticks per second. */
    public static final class WheelVelocities {
        public final double fl, fr, bl, br;

        public WheelVelocities(double fl, double fr, double bl, double br) {
            this.fl = fl;
            this.fr = fr;
            this.bl = bl;
            this.br = br;
        }
    }

    /** One Pinpoint reading, in its own field frame. */
    public static final class Odometry {
        public final PinpointHealth health;
        public final double velXMetresPerSecond;
        public final double velYMetresPerSecond;
        public final double headingRadians;
        public final double headingRateRadiansPerSecond;

        public Odometry(PinpointHealth health, double velXMetresPerSecond, double velYMetresPerSecond,
                        double headingRadians, double headingRateRadiansPerSecond) {
            this.health = health;
            this.velXMetresPerSecond = velXMetresPerSecond;
            this.velYMetresPerSecond = velYMetresPerSecond;
            this.headingRadians = headingRadians;
            this.headingRateRadiansPerSecond = headingRateRadiansPerSecond;
        }
    }

    public static final class Result {
        /** The commanded powers times {@link #scale}. */
        public final ChassisMotors powers;
        public final double scale;
        /** Per-wheel slip, or NaN for all four while odometry is untrusted. */
        public final double flSlip, frSlip, blSlip, brSlip;
        /** Largest |slip| among wheels commanded above {@link TractionConstants#POWERED_WHEEL_THRESHOLD}; 0 if none. */
        public final double worstPoweredSlip;
        public final GripState state;
        /** False = fail open: powers pass through unscaled. */
        public final boolean odometryTrusted;

        Result(ChassisMotors powers, double scale, double flSlip, double frSlip, double blSlip, double brSlip,
               double worstPoweredSlip, GripState state, boolean odometryTrusted) {
            this.powers = powers;
            this.scale = scale;
            this.flSlip = flSlip;
            this.frSlip = frSlip;
            this.blSlip = blSlip;
            this.brSlip = brSlip;
            this.worstPoweredSlip = worstPoweredSlip;
            this.state = state;
            this.odometryTrusted = odometryTrusted;
        }
    }

    /** Longest loop gap the controller integrates over, so a stalled loop cannot slam the scale. */
    static final double MAX_DT = 0.1;

    private double scale = 1.0;
    private Odometry lastGood = null;
    private Odometry reading = null;
    private int consecutiveBadReads = 0;

    /**
     * Takes this loop's Pinpoint reading. A single {@link PinpointHealth#BAD_READ} is covered by the previous good
     * reading; a second in a row, or any {@link PinpointHealth#FAULT}, makes odometry untrusted until READY returns.
     *
     * @return whether odometry is trusted this loop; false = fail open, and field-centric drive must not use heading
     */
    public boolean observe(Odometry odometry) {
        switch (odometry.health) {
            case READY:
                consecutiveBadReads = 0;
                lastGood = odometry;
                reading = odometry;
                break;
            case BAD_READ:
                consecutiveBadReads++;
                reading = consecutiveBadReads == 1 ? lastGood : null;
                break;
            default:
                lastGood = null;
                reading = null;
        }
        return reading != null;
    }

    /** Heading of the reading in use (the last good one through a single bad read); NaN while untrusted. */
    public double headingRadians() {
        return reading == null ? Double.NaN : reading.headingRadians;
    }

    /**
     * @param commanded wheel powers after mixing and slow mode, before traction control
     * @param bypass    true while the driver holds the bypass button: powers pass through unscaled
     */
    public Result update(ChassisMotors commanded, WheelVelocities measured, boolean bypass, double dtSeconds) {
        if (reading == null) {
            scale = 1.0;
            return new Result(commanded, 1.0, Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                    0.0, GripState.ODOMETRY_FAULT, false);
        }

        // Pinpoint velocity is field-frame; the wheels see it in the robot's frame.
        double cos = Math.cos(reading.headingRadians);
        double sin = Math.sin(reading.headingRadians);
        double forward = reading.velXMetresPerSecond * cos + reading.velYMetresPerSecond * sin;
        double left = -reading.velXMetresPerSecond * sin + reading.velYMetresPerSecond * cos;

        // Mecanum inverse kinematics, in wheel surface speed. A strafe needs 1/efficiency more wheel speed than
        // the chassis gets, and counter-clockwise yaw drives the right side forward and the left side back.
        double strafe = left / TractionConstants.STRAFE_EFFICIENCY;
        double turn = reading.headingRateRadiansPerSecond
                * (TractionConstants.TRACK_WIDTH_METRES + TractionConstants.WHEEL_BASE_METRES) / 2.0;
        double flSlip = slip(measured.fl, forward - strafe - turn);
        double frSlip = slip(measured.fr, forward + strafe + turn);
        double blSlip = slip(measured.bl, forward + strafe - turn);
        double brSlip = slip(measured.br, forward - strafe + turn);

        double worst = 0.0;
        worst = worstIfPowered(worst, commanded.getFL(), flSlip);
        worst = worstIfPowered(worst, commanded.getFR(), frSlip);
        worst = worstIfPowered(worst, commanded.getBL(), blSlip);
        worst = worstIfPowered(worst, commanded.getBR(), brSlip);

        double dt = Math.max(0.0, Math.min(dtSeconds, MAX_DT));
        if (bypass) {
            scale = 1.0;
        } else {
            // With no wheel powered, slip comes from braking or being pushed, which scaling cannot change: recover.
            double excess = worst - TractionConstants.TARGET_SLIP;
            if (excess > 0.0) {
                scale -= TractionConstants.CUT_GAIN * excess * dt;
            } else {
                scale += TractionConstants.RECOVERY_PER_SECOND * dt;
            }
            scale = Math.max(TractionConstants.MIN_SCALE, Math.min(1.0, scale));
        }

        ChassisMotors powers = new ChassisMotors(
                commanded.getFL() * scale, commanded.getFR() * scale,
                commanded.getBL() * scale, commanded.getBR() * scale);
        GripState state = worst > TractionConstants.HEAVY_SLIP ? GripState.HEAVY_SLIP
                : scale < TractionConstants.LIMITED_SCALE_THRESHOLD ? GripState.LIMITED
                : GripState.FULL_GRIP;
        return new Result(powers, scale, flSlip, frSlip, blSlip, brSlip, worst, state, true);
    }

    private static double slip(double ticksPerSecond, double expectedMetresPerSecond) {
        double measured = ticksPerSecond / TractionConstants.TICKS_PER_REV
                * TractionConstants.GEAR_RATIO * 2.0 * Math.PI * TractionConstants.WHEEL_RADIUS_METRES;
        double floor = Math.max(Math.abs(expectedMetresPerSecond),
                TractionConstants.SLIP_SPEED_FLOOR_METRES_PER_SECOND);
        return (measured - expectedMetresPerSecond) / floor;
    }

    private static double worstIfPowered(double worst, double power, double slip) {
        return Math.abs(power) > TractionConstants.POWERED_WHEEL_THRESHOLD ? Math.max(worst, Math.abs(slip)) : worst;
    }
}
