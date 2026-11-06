package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Tuning for {@link TractionControlLogic} and the Traction Mecanum OpModes.
 *
 * Every field is live-editable from Panels while an OpMode runs. Settle a value on the robot, then copy it back
 * here, since Panels edits are lost when the robot restarts.
 */
@Configurable
public class TractionConstants {

    // ---- Slip controller ----
    /** Slip the controller holds the worst powered wheel at. 0.15 = wheel surface 15% faster (or slower) than the ground. */
    public static double TARGET_SLIP = 0.15;
    /** How fast the power scale climbs back toward 1 once slip is under target, in scale per second. */
    public static double RECOVERY_PER_SECOND = 2.0;
    /** How fast the scale drops, per second, per unit of slip above target. 20 cuts 0.5 of excess slip in 0.1 s. */
    public static double CUT_GAIN = 20.0;
    /** The scale never goes below this, so a robot pinned in a pushing match still pushes. */
    public static double MIN_SCALE = 0.2;
    /**
     * Slip divides by max(|expected wheel speed|, this). Near a standing start the expected speed is about zero, and
     * this turns the ratio into an absolute speed error over the floor, so a spinning wheel is still caught.
     */
    public static double SLIP_SPEED_FLOOR_METRES_PER_SECOND = 0.15;
    /** A wheel commanded below this power cannot be helped by scaling, so its slip is reported but not acted on. */
    public static double POWERED_WHEEL_THRESHOLD = 0.02;

    // ---- Driver feedback ----
    /** Below this scale the LED shows amber: traction control is limiting power. */
    public static double LIMITED_SCALE_THRESHOLD = 0.95;
    /** Above this worst-wheel slip the LED shows red. */
    public static double HEAVY_SLIP = 0.5;

    // ---- Drivetrain geometry (measured on the robot; Verity's figures from robot-config/verity.json) ----
    public static double WHEEL_RADIUS_METRES = 0.048;
    /** Wheel revolutions per motor revolution. */
    public static double GEAR_RATIO = 1.0;
    /** Encoder ticks per motor revolution. goBILDA 5203 312 rpm = 537.7. */
    public static double TICKS_PER_REV = 537.7;
    /** Left-right distance between wheel contact patches. */
    public static double TRACK_WIDTH_METRES = 0.32;
    /** Front-back distance between wheel contact patches. */
    public static double WHEEL_BASE_METRES = 0.29;
    /**
     * Chassis sideways speed per unit of wheel surface speed when strafing. Rollers lose some of a strafe by design,
     * so the wheels turn 1/STRAFE_EFFICIENCY faster than the chassis slides, and that must not read as slip.
     */
    public static double STRAFE_EFFICIENCY = 0.8;

    // ---- Driver controls ----
    public static double STICK_DEADBAND = 0.05;
    /** Wheel power multiplier while the left bumper is held. */
    public static double SLOW_MODE_SCALE = 0.4;
}
