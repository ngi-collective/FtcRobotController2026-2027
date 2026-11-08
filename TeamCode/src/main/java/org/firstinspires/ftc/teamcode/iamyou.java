package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import android.util.Size;

import java.util.ArrayList;
import java.util.List;

/**
 * The whole I AM YOU TeleOp in one file: hardware setup, field-centric drive math, intake and flywheel control.
 * This class is the ONE-controller version; {@link IamYouTwoController} is the same program with the intake and
 * outtake moved to gamepad 2.
 *
 * Wiring (robot configuration names):
 *   Control Hub motors ...... 0 FL, 1 FR, 2 BL, 3 BR
 *   Expansion Hub motors .... 0 IntakeM, 1 Outtake (the flywheel)
 *   Servos .................. IntakeSL, IntakeSR, Staging (continuous rotation; they always spin with IntakeM)
 *   Control Hub I2C bus 0 ... imu (the hub's built-in IMU; the only heading source, there is no odometry)
 *
 * Driver controls (gamepad 1):
 *   Left stick .......... drive (field-centric)
 *   Right stick X ....... turn
 *   Left trigger ........ slow mode
 *   Right trigger ....... intake (hold; harder pull = faster)
 *   Right bumper ........ eject (hold; wins over the trigger)
 *   Left bumper ......... pollen lock (press to turn ON/OFF). While ON the robot turns to face the yellow pollen
 *                         (the middle of the group if there are several) and the left stick drives ROBOT-relative,
 *                         so pushing forward drives straight into it. Moving the right stick overrides the aim.
 *   D-pad ............... where the driver is standing (Up=N Down=S Left=W Right=E)
 *   Options ............. re-zero heading (robot's current front becomes NORTH)
 *
 * Mechanism controls (gamepad 1 in this version, gamepad 2 in the two-controller version):
 *   Right trigger ....... intake (hold). Fixed speed, however far the trigger is pulled.
 *   Right bumper ........ outtake / poop out (hold; wins over the trigger). Intake motor and Staging run backwards.
 *   Left bumper ......... flywheel ON/OFF (press to toggle). Runs at OUTTAKE_POWER until pressed again.
 *   Each intake servo's direction for intake and for outtake is set separately (LEFT_SERVO_* / RIGHT_SERVO_*).
 */
@TeleOp(name = "I AM YOU (1 controller)")
public class iamyou extends LinearOpMode {

    // ---- Hardware names ----
    static final String FL_NAME = "FL";
    static final String FR_NAME = "FR";
    static final String BL_NAME = "BL";
    static final String BR_NAME = "BR";
    static final String INTAKE_NAME = "IntakeM";
    static final String INTAKE_SERVO_LEFT_NAME = "IntakeSL";
    static final String INTAKE_SERVO_RIGHT_NAME = "IntakeSR";
    static final String STAGING_NAME = "Staging";
    static final String OUTTAKE_NAME = "Outtake";
    /** The Control Hub's built-in IMU. There is no odometry, so this is the only heading source. */
    static final String IMU_NAME = "imu";

    /**
     * Flip this if positive power (right trigger) spits pieces out instead of pulling them in.
     * With a big-gear-drives-small-gear mesh the roller turns opposite to the motor, so
     * the first run may need this set to true.
     */
    static final boolean INTAKE_REVERSED = true;

    /**
     * Which way each intake servo spins (+1 or -1) while intaking (right trigger) and while outtaking (right bumper),
     * set separately per button because the robot needs them that way. Flip one sign to flip that one case.
     * Both servos always get the same speed; only the sign differs.
     */
    static final double LEFT_SERVO_INTAKE_DIR   = -1.0;
    static final double LEFT_SERVO_OUTTAKE_DIR  = -1.0;
    static final double RIGHT_SERVO_INTAKE_DIR  = -1.0;
    static final double RIGHT_SERVO_OUTTAKE_DIR = -1.0;
    /**
     * Speed trim, 0..1, for servos that are physically faster than their partner at the same command. Both get the
     * same command at 1.0. If one still spins faster on the robot, lower ITS trim until they match.
     */
    static final double LEFT_SERVO_TRIM  = 1.0;
    static final double RIGHT_SERVO_TRIM = 1.0;
    /** Flip this if Staging moves pieces away from the flywheel while intaking. */
    static final boolean STAGING_REVERSED = true;

    /** Flip this if the flywheel spins backwards. */
    static final boolean OUTTAKE_REVERSED = true;

    /** The right stick turned the robot the wrong way on the real robot, so it is flipped here. */
    static final boolean TURN_REVERSED = true;

    /** How often to read the intake motor's current (each read is a separate hub command, so don't do it every loop). */
    private static final double INTAKE_CURRENT_READ_SEC = 0.10;

    private final Drive driveLogic = new Drive();
    private final Intake intakeLogic = new Intake();
    private final Outtake outtakeLogic = new Outtake();

    /** True = gamepad 2 runs the intake and outtake while gamepad 1 only drives. */
    protected boolean twoControllers() {
        return false;
    }

    @Override
    public void runOpMode() {

        // ================= FAST HUB READS =================
        // MANUAL caching = one bulk read per hub per loop, cleared at the top of the loop below.
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // ================= CONFIGURATION CHECK =================
        // Check every device up front, so a misnamed configuration reports ALL its problems in one message instead
        // of crashing on the first and hiding the rest.
        List<String> missing = new ArrayList<>();
        requireDevice(DcMotorEx.class, FL_NAME, missing);
        requireDevice(DcMotorEx.class, FR_NAME, missing);
        requireDevice(DcMotorEx.class, BL_NAME, missing);
        requireDevice(DcMotorEx.class, BR_NAME, missing);
        requireDevice(DcMotorEx.class, INTAKE_NAME, missing);
        requireDevice(DcMotorEx.class, OUTTAKE_NAME, missing);
        requireDevice(CRServo.class, INTAKE_SERVO_LEFT_NAME, missing);
        requireDevice(CRServo.class, INTAKE_SERVO_RIGHT_NAME, missing);
        requireDevice(CRServo.class, STAGING_NAME, missing);
        if (!missing.isEmpty()) {
            while (!isStopRequested()) {
                telemetry.addData("CONFIG ERROR", "The active robot configuration is missing:");
                for (String device : missing) {
                    telemetry.addLine("  " + device);
                }
                telemetry.addLine("Fix it on the Driver Hub: Configure Robot, then restart the robot.");
                telemetry.update();
                sleep(100);
            }
            return;
        }

        // ================= IMU SETUP =================
        // The Control Hub's built-in IMU. The hub stands on its edge with its logo facing the robot's RIGHT side
        // and its USB ports facing FORWARD (toward the intake). If it is missing from the configuration the robot
        // still drives, robot-relative, instead of refusing to move.
        IMU imu = hardwareMap.tryGet(IMU.class, IMU_NAME);
        if (imu != null) {
            imu.initialize(new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
            ));
        }
        String imuStatus = imu != null ? "OK"
                : "MISSING - driving ROBOT-relative. Add \"" + IMU_NAME + "\" to the Control Hub's I2C bus 0";

        // ================= DRIVE MOTORS =================
        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, FL_NAME);
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, FR_NAME);
        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, BL_NAME);
        DcMotorEx br = hardwareMap.get(DcMotorEx.class, BR_NAME);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        DcMotorEx[] wheels = {fl, fr, bl, br};
        // Plain power mode for driving: power is a direct motor-voltage fraction, with none of the hub's velocity-PID
        // lag or locking. Encoders are still read for the stop assist.
        for (DcMotorEx m : wheels) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // ================= INTAKE MOTOR =================
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, INTAKE_NAME);
        intake.setDirection(INTAKE_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        // Open-loop power: no encoder needed, and the ramping happens in Intake.
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ================= INTAKE SERVOS =================
        CRServo intakeServoLeft = hardwareMap.get(CRServo.class, INTAKE_SERVO_LEFT_NAME);
        CRServo intakeServoRight = hardwareMap.get(CRServo.class, INTAKE_SERVO_RIGHT_NAME);
        // Both FORWARD: their per-button directions are the *_DIR constants, so there is one place to change them.
        intakeServoLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeServoRight.setDirection(DcMotorSimple.Direction.FORWARD);
        CRServo staging = hardwareMap.get(CRServo.class, STAGING_NAME);
        staging.setDirection(STAGING_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        // ================= FLYWHEEL (OUTTAKE MOTOR) =================
        DcMotorEx outtake = hardwareMap.get(DcMotorEx.class, OUTTAKE_NAME);
        outtake.setDirection(OUTTAKE_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        // Open loop at OUTTAKE_POWER for now. The encoder is still read, so telemetry shows the real wheel speed, which
        // is the number velocity PID tuning will need later.
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Coast to a stop: braking a spinning flywheel hard is a shock load on the motor and gears.
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Whoever runs the intake and outtake: the driver, or a second operator.
        Gamepad mech = twoControllers() ? gamepad2 : gamepad1;

        DriverPosition driverPosition = DriverPosition.SOUTH;

        try {
            // ================= PRE-START =================
            while (!isStarted() && !isStopRequested()) {
                driverPosition = readDriverPosition(driverPosition);

                telemetry.addData("Status", "Initialized - waiting for start");
                telemetry.addData("Controllers", twoControllers()
                        ? "2 (gamepad 1 drives, gamepad 2 runs intake/outtake)" : "1 (gamepad 1 does everything)");
                telemetry.addData("IMU", imuStatus);
                telemetry.addData("Driver Position", driverPosition + "  (D-pad: Up=N Down=S Left=W Right=E)");
                telemetry.addData("Note", "Whatever direction the robot faces at START becomes NORTH");
                telemetry.update();
            }

            if (isStopRequested()) return;

            // Zero heading at the moment of start: the robot's current front is NORTH.
            if (imu != null) {
                imu.resetYaw();
            }

            ElapsedTime loopTimer = new ElapsedTime();
            ElapsedTime currentTimer = new ElapsedTime();
            telemetry.setMsTransmissionInterval(100);
            double lastIntakePower = 0.0;
            double intakeAmps = 0.0;

            // ================= MAIN LOOP =================
            while (opModeIsActive()) {

                for (LynxModule hub : hubs) {
                    hub.clearBulkCache();
                }

                double dt = loopTimer.seconds();
                loopTimer.reset();

                // Options button: re-zero so the robot's current front becomes NORTH.
                if (gamepad1.options && imu != null) {
                    imu.resetYaw();
                }

                // No IMU = heading stuck at 0 and driver position ignored, so the left stick is plain robot-relative.
                double heading = imu != null ? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) : 0.0;
                driverPosition = imu != null ? readDriverPosition(driverPosition) : DriverPosition.SOUTH;

                // ---- Drive ----
                DriveParameters params = new DriveParameters(
                        gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        TURN_REVERSED ? -gamepad1.right_stick_x : gamepad1.right_stick_x,
                        gamepad1.left_trigger,
                        heading,
                        driverPosition,
                        fl.getVelocity(),
                        fr.getVelocity(),
                        bl.getVelocity(),
                        br.getVelocity(),
                        dt
                );

                DriveResult result = driveLogic.calculate(params);
                fl.setPower(result.powers.getFL());
                fr.setPower(result.powers.getFR());
                bl.setPower(result.powers.getBL());
                br.setPower(result.powers.getBR());

                // ---- Intake ----
                if (Math.abs(lastIntakePower) > 0.05) {
                    if (currentTimer.seconds() >= INTAKE_CURRENT_READ_SEC) {
                        intakeAmps = intake.getCurrent(CurrentUnit.AMPS);
                        currentTimer.reset();
                    }
                } else {
                    intakeAmps = 0.0;
                    currentTimer.reset();
                }
                IntakeResult intakeResult = intakeLogic.update(
                        mech.right_trigger, mech.right_bumper, intakeAmps, dt);
                intake.setPower(intakeResult.power);
                // The servos run whenever the motor runs, at the same speed as each other. Positive motor power =
                // intaking, negative = outtaking (or clearing a jam); each servo has its own direction for each.
                double servoSpeed = Math.min(1.0, Math.abs(intakeResult.power));
                boolean intaking = intakeResult.power >= 0.0;
                double leftServo  = servoSpeed * LEFT_SERVO_TRIM  * (intaking ? LEFT_SERVO_INTAKE_DIR  : LEFT_SERVO_OUTTAKE_DIR);
                double rightServo = servoSpeed * RIGHT_SERVO_TRIM * (intaking ? RIGHT_SERVO_INTAKE_DIR : RIGHT_SERVO_OUTTAKE_DIR);
                intakeServoLeft.setPower(leftServo);
                intakeServoRight.setPower(rightServo);
                // Staging feeds forward on intake and backward on outtake, with the intake motor.
                double stagingPower = Math.max(-1.0, Math.min(1.0, intakeResult.power));
                staging.setPower(1);
                lastIntakePower = intakeResult.power;

                // ---- Flywheel ----
                double outtakePower = outtakeLogic.update(mech.left_bumper, dt);
                outtake.setPower(outtakePower);
                double flywheelTicksPerSec = outtake.getVelocity();

                // ================= TELEMETRY =================
                telemetry.addData("Loop", "%.1f ms  (%.0f Hz)", dt * 1000.0, dt > 0 ? 1.0 / dt : 0.0);
                if (imu == null) {
                    telemetry.addData("IMU", imuStatus);
                }
                telemetry.addData("Facing (vs start)",  snapToCardinal(heading));
                telemetry.addData("Driver Position",    driverPosition + "  (D-pad to change)");
                telemetry.addData("Heading (deg)",      "%.1f", heading);
                telemetry.addData("Slow Mode",          result.slowMode ? "ON" : "OFF");
                telemetry.addData("Stop Assist",        result.braking ? "BRAKING" : "off");
                telemetry.addData("Intake",             "%s  power %.2f  %.1f A",
                        intakeResult.state, intakeResult.power, intakeAmps);
                telemetry.addData("Intake Servos L | R", "%.2f | %.2f", leftServo, rightServo);
                telemetry.addData("Staging",            "%.2f", stagingPower);
                telemetry.addData("Flywheel",           "%s  power %.2f  %.0f ticks/s",
                        outtakeLogic.isOn() ? "ON" : "OFF", outtakePower, flywheelTicksPerSec, outtake.getVelocity());
                telemetry.addData("---", "---");
                telemetry.addData("Cmd Fwd | Right",    "%.2f | %.2f", result.stickForward, result.stickRight);
                telemetry.addData("Robot Fwd | Right",  "%.2f | %.2f", result.robotForward, result.robotRight);
                telemetry.addData("Twist",              "%.2f", result.twist);
                telemetry.addData("FL | FR",            "%.2f | %.2f", result.powers.getFL(), result.powers.getFR());
                telemetry.addData("BL | BR",            "%.2f | %.2f", result.powers.getBL(), result.powers.getBR());
                telemetry.addData("FL Vel | FR Vel",    "%.0f | %.0f", params.flVelocity, params.frVelocity);
                telemetry.addData("BL Vel | BR Vel",    "%.0f | %.0f", params.blVelocity, params.brVelocity);
                telemetry.update();
            }
        } finally {
            // Runs on every exit path, including an early stop during init.
            for (DcMotorEx m : wheels) {
                m.setPower(0.0);
            }
            intake.setPower(0.0);
            intakeServoLeft.setPower(0.0);
            intakeServoRight.setPower(0.0);
            staging.setPower(0.0);
            outtake.setPower(0.0);
        }
    }

    /** Adds "name (type)" to missing if the configuration has no device of that type under that name. */
    private void requireDevice(Class<?> type, String name, List<String> missing) {
        if (hardwareMap.tryGet(type, name) == null) {
            missing.add("\"" + name + "\" (" + type.getSimpleName() + ")");
        }
    }

    private DriverPosition readDriverPosition(DriverPosition current) {
        if (gamepad1.dpad_up)    return DriverPosition.NORTH;
        if (gamepad1.dpad_down)  return DriverPosition.SOUTH;
        if (gamepad1.dpad_left)  return DriverPosition.WEST;
        if (gamepad1.dpad_right) return DriverPosition.EAST;
        return current;
    }

    // =====================================================================================================
    // DRIVE MATH
    //
    // Field-centric mecanum drive math. No FTC SDK calls, so it can be tested on a laptop.
    //
    // Conventions used everywhere below:
    //  - Vectors are (x = right, y = forward).
    //  - Headings are degrees, counter-clockwise positive (the FTC IMU convention).
    //  - Heading 0 is whatever direction the robot faced when the IMU was last
    //    zeroed (start of the OpMode, or the Options button). That direction is "NORTH".
    //  - Positive twist turns the robot clockwise (right).
    //  - Positive wheel power drives that wheel forward (runOpMode sets the motor directions).
    //  - The drive motors run in RUN_WITHOUT_ENCODER (plain power) mode, so power is a motor voltage fraction.
    // =====================================================================================================

    // ---- Driving constants ----
    static final double JOYSTICK_DEADZONE        = 0.05;
    static final double SLOW_MODE_DRIVE_MULT     = 0.35;
    static final double SLOW_MODE_TWIST_MULT     = 0.50;
    static final double SLOW_MODE_TRIGGER_THRESH = 0.10;

    /**
     * STRAFE CALIBRATION. Mecanum robots strafe slower than they drive, and a weak strafe makes diagonals bend
     * toward forward/back (feels like snapping to an axis). This multiplies the sideways command to cancel that.
     * To tune: drive forward at the same stick amount for 1 s, then strafe for 1 s, and measure both distances.
     * STRAFE_COMPENSATION = forward distance / strafe distance. 1.0 = no compensation.
     */
    static final double STRAFE_COMPENSATION      = 1.25;

    // ---- Smoothness constants (tune these on the robot) ----
    /**
     * Translation speed at full stick, as a fraction of max wheel power. Direction is ALWAYS preserved (the mixer
     * scales every wheel by the same factor), so a bigger number never bends the path or snaps to an axis. It only
     * makes the diagonals a little slower than straight-ahead once a wheel would exceed 100%. 0.85 is a good
     * fast-but-smooth default; 1.0 is the fastest and 0.62 is perfectly uniform speed in every direction.
     */
    static final double TRANSLATION_SPEED_CAP = 0.85;
    /** Stick response curve: 0 = linear, 1 = fully cubic (gentle near center, still reaches full at the edge). */
    static final double TRANSLATION_EXPO      = 0.30;
    static final double TWIST_EXPO            = 0.35;
    /** How fast the command may GROW, in power per second. 7.0 = 0 to full in about 0.14 s (soft launch, no wheelspin). */
    static final double MAX_TRANSLATION_CHANGE_PER_SEC = 7.0;
    /** How fast the command may SHRINK. Much higher than the launch rate so lifting off the stick feels instant. */
    static final double MAX_TRANSLATION_DECEL_PER_SEC  = 24.0;
    static final double MAX_TWIST_CHANGE_PER_SEC       = 9.0;
    static final double MAX_TWIST_DECEL_PER_SEC        = 30.0;
    static final double MIN_DT = 0.001;
    static final double MAX_DT = 0.100;

    /**
     * Static-friction compensation. A motor driving a loaded wheel does nothing below roughly this power, so a
     * wheel that should creep at 0.05 just sits there and the robot "snaps" to the direction of the other wheels.
     * Every nonzero wheel command gets this much extra push in its own direction (blended smoothly through zero).
     */
    static final double STATIC_FRICTION_POWER = 0.06;
    /** Wheel commands smaller than this get a proportionally smaller friction boost, so the sign flip at zero is smooth. */
    static final double STATIC_FRICTION_BLEND = 0.15;

    // ---- Stop assist constants (tune these on the robot) ----
    /** Wheel speed in ticks/sec at full power. goBILDA 312/435/1150 RPM motors are all about 2700-2800. */
    static final double MAX_WHEEL_TICKS_PER_SEC = 2700.0;
    /** How hard to brake relative to wheel speed. Higher = stops sooner. */
    static final double BRAKE_GAIN              = 4.0;
    /** Hard ceiling on braking power. Lower this if the stop feels like whiplash. */
    static final double BRAKE_MAX_POWER         = 0.6;
    /** Below this fraction of max speed a wheel counts as stopped. */
    static final double BRAKE_MIN_SPEED         = 0.05;
    /** Seconds of zero stick input before braking engages. Stops a quick stick flick through center from triggering a brake. */
    static final double BRAKE_ENGAGE_SEC        = 0.03;
    /** Absolute limit on how long the stop assist may push, no matter what the encoders say. This is the runaway watchdog. */
    static final double BRAKE_MAX_SEC           = 0.18;
    /** A wheel must have been commanded at least this hard for its direction to count as "the way it was driving". */
    static final double BRAKE_DRIVE_THRESHOLD   = 0.08;

    enum DriverPosition {
        SOUTH(0.0),     // behind the robot's starting orientation (default)
        EAST(90.0),     // to the east side, looking west
        NORTH(180.0),   // in front of the robot, looking south
        WEST(-90.0);    // to the west side, looking east

        /** Counter-clockwise rotation applied to the stick vector so "stick forward" means "away from the driver". */
        final double offsetDegrees;

        DriverPosition(double offsetDegrees) {
            this.offsetDegrees = offsetDegrees;
        }
    }

    enum Cardinal { NORTH, WEST, SOUTH, EAST }

    /** The four wheel powers, in FL, FR, BL, BR order. */
    static class WheelPowers {
        private final double fl, fr, bl, br;

        WheelPowers(double fl, double fr, double bl, double br) {
            this.fl = fl;
            this.fr = fr;
            this.bl = bl;
            this.br = br;
        }

        double getFL() { return fl; }
        double getFR() { return fr; }
        double getBL() { return bl; }
        double getBR() { return br; }
    }

    static class DriveParameters {
        final double leftStickX;
        final double leftStickY;      // raw FTC value: up is negative
        final double rightStickX;
        final double leftTrigger;
        final double headingDegrees;  // IMU yaw, zeroed at start/reset
        final DriverPosition driverPosition;
        final double flVelocity;      // ticks/sec, positive = wheel driving robot forward
        final double frVelocity;
        final double blVelocity;
        final double brVelocity;
        final double dtSeconds;       // time since the previous loop, for acceleration smoothing

        DriveParameters(double leftStickX, double leftStickY, double rightStickX,
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

    static class DriveResult {
        final WheelPowers powers;
        final double headingRadians;
        final double stickForward;   // smoothed command, after driver-position rotation
        final double stickRight;
        final double robotForward;   // after field-centric rotation
        final double robotRight;
        final double twist;
        final boolean slowMode;
        final boolean braking;

        DriveResult(WheelPowers powers, double headingRadians,
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

    /** Drive state carried between loops: the acceleration limiter and the stop assist. */
    static class Drive {
        // Seconds of continuous zero stick input (for the stop assist).
        private double noInputSeconds = 0.0;
        private boolean wasBraking = false;
        private double brakeSeconds = 0.0;
        // Per-wheel stop-assist state, order: FL, FR, BL, BR.
        /** Direction (+1/-1/0) each wheel was last driven hard. Comes from OUR command, never from the encoder's sign. */
        private final int[] driveSign = new int[4];
        private final boolean[] brakeDone = new boolean[4];

        // Smoothed commands (stick space), carried between loops for the acceleration limiter.
        private double cmdRight = 0.0;
        private double cmdForward = 0.0;
        private double cmdTwist = 0.0;

        DriveResult calculate(DriveParameters params) {
            // Bad numbers must never reach a motor: one NaN or Infinity would otherwise spread through every wheel.
            double dt = Math.max(MIN_DT, Math.min(MAX_DT, finite(params.dtSeconds, 0.02)));
            double lsx = finite(params.leftStickX, 0.0);
            double lsy = finite(params.leftStickY, 0.0);
            double rsx = finite(params.rightStickX, 0.0);
            double headingDeg = finite(params.headingDegrees, 0.0);

            // ================= STICKS =================
            // Convert to (x = right, y = forward). The deadzone is circular and rescaled, so direction is never distorted.
            double[] stick = radialDeadzone(lsx, -lsy, JOYSTICK_DEADZONE);
            double twistIn = axisDeadzone(rsx, JOYSTICK_DEADZONE);  // positive = clockwise

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
            double tMag = Math.abs(twistIn);
            double targetTwist = Math.copySign((1.0 - TWIST_EXPO) * tMag + TWIST_EXPO * tMag * tMag * tMag, twistIn);

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
            // Both are rotations, so they combine into one angle.
            double headingRad = Math.toRadians(headingDeg);
            double angle = Math.toRadians(params.driverPosition.offsetDegrees) - headingRad;
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
            }
            WheelPowers powers = new WheelPowers(out[0], out[1], out[2], out[3]);

            return new DriveResult(powers, headingRad, cmdForward, cmdRight,
                    robotForward, robotRight, cmdTwist, slow, braking);
        }
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
    static Cardinal snapToCardinal(double headingDeg) {
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

    // =====================================================================================================
    // INTAKE MATH
    //
    // Power convention: positive = pulling game pieces IN. (INTAKE_REVERSED flips the motor direction
    // if the gear mesh makes positive spit them out, so this code never needs to change for that.)
    //
    // Built for the goBILDA starter-kit intake: a large gear on the motor driving a small gear on the roller.
    // That is a gear-UP, so the roller spins faster than the motor and the gears take shock loads. This logic therefore:
    //  - ramps power up and down instead of jumping, which protects the gear teeth and the battery,
    //  - caps power below 1.0 so the roller doesn't fling pieces,
    //  - watches motor current and briefly reverses to clear a jam instead of stalling the motor.
    // =====================================================================================================

    // ---- Tunable constants ----
    /** Power while intaking. A set speed: pulling the trigger harder does not change it. */
    static final double INTAKE_POWER        = 1.00;
    /** Power while the eject button is held. */
    static final double EJECT_POWER         = 0.80;
    /** Power change limit in power-per-second. 8.0 = 0 to full in about 0.12 s; a full direction flip takes about 0.2 s. */
    static final double RAMP_PER_SEC        = 8.0;
    /** How far the trigger must be pulled to count as pressed. */
    static final double TRIGGER_PRESSED     = 0.20;

    // ---- Jam protection ----
    static final boolean JAM_PROTECTION     = true;
    /** Motor current that counts as stalled. goBILDA 5203 motors stall around 9 A, so only a true stall should reach this. */
    static final double STALL_CURRENT_AMPS  = 8.0;
    /** Current must stay above the threshold this long before reversing. */
    static final double STALL_TIME_SEC      = 0.50;
    /** Ignore current for this long after spin-up begins (startup inrush is not a jam). */
    static final double SPINUP_GRACE_SEC    = 0.35;
    static final double UNJAM_TIME_SEC      = 0.35;
    static final double UNJAM_POWER         = 0.60;
    /** Ramp is this many times faster while clearing a jam. */
    static final double UNJAM_RAMP_MULT     = 3.0;

    enum IntakeState { IDLE, INTAKING, EJECTING, UNJAMMING }

    static class IntakeResult {
        final double power;
        final IntakeState state;

        IntakeResult(double power, IntakeState state) {
            this.power = power;
            this.state = state;
        }
    }

    /** Intake state carried between loops: the power ramp and the jam timers. */
    static class Intake {
        private double power = 0.0;        // actual (ramped) power being sent
        private double spinupTimer = 0.0;  // how long the roller has been spinning inward
        private double stallTimer = 0.0;   // how long current has been over the stall threshold
        private double unjamTimer = 0.0;   // time left in the current unjam reverse

        /**
         * @param intakeTrigger 0..1 analog trigger value (hold to intake; any pull past TRIGGER_PRESSED = full speed)
         * @param eject         true while the eject button is held (wins over the trigger)
         * @param currentAmps   motor current in amps; pass 0 if not measured this loop
         * @param dtSeconds     time since the previous loop
         */
        IntakeResult update(double intakeTrigger, boolean eject, double currentAmps, double dtSeconds) {
            double dt = Math.max(0.001, Math.min(0.1, dtSeconds));

            // ---- What the driver is asking for ----
            double requested = 0.0;
            if (eject) {
                requested = -EJECT_POWER;
            } else if (intakeTrigger > TRIGGER_PRESSED) {
                requested = INTAKE_POWER;
            }

            // Releasing the trigger, or ejecting, cancels any unjam and restarts the jam timers.
            if (requested <= 0.0) {
                unjamTimer = 0.0;
                stallTimer = 0.0;
            }

            // ---- Jam detection (intake direction only) ----
            boolean unjamming = unjamTimer > 0.0;
            if (JAM_PROTECTION && requested > 0.0 && !unjamming) {
                spinupTimer = power > 0.1 ? spinupTimer + dt : 0.0;
                if (spinupTimer > SPINUP_GRACE_SEC && currentAmps > STALL_CURRENT_AMPS) {
                    stallTimer += dt;
                } else {
                    stallTimer = Math.max(0.0, stallTimer - dt);
                }
                if (stallTimer >= STALL_TIME_SEC) {
                    unjamTimer = UNJAM_TIME_SEC;
                    stallTimer = 0.0;
                    spinupTimer = 0.0;
                    unjamming = true;
                }
            } else if (requested <= 0.0) {
                spinupTimer = 0.0;
            }

            // ---- Pick the target ----
            double target = requested;
            IntakeState state;
            double rate = RAMP_PER_SEC;
            if (unjamming) {
                target = -UNJAM_POWER;
                rate = RAMP_PER_SEC * UNJAM_RAMP_MULT;
                unjamTimer -= dt;
                state = IntakeState.UNJAMMING;
            } else if (requested < 0.0) {
                state = IntakeState.EJECTING;
            } else if (requested > 0.0) {
                state = IntakeState.INTAKING;
            } else {
                state = IntakeState.IDLE;
            }

            // ---- Ramp toward it ----
            double step = rate * dt;
            power += Math.max(-step, Math.min(step, target - power));

            return new IntakeResult(power, state);
        }
    }

    // =====================================================================================================
    // FLYWHEEL (OUTTAKE) MATH
    //
    // One press turns the flywheel on, the next turns it off. It runs open loop at OUTTAKE_POWER for now; velocity
    // PID comes later. Spin-up is ramped so the motor and gears never see a full-power step; turning it off cuts
    // power at once and the wheel coasts down (the motor is set to FLOAT).
    // =====================================================================================================

    /** Flywheel power while ON. Half power for now; velocity PID will replace this. */
    static final double OUTTAKE_POWER         = 0.51;
    /** Spin-up limit in power-per-second. 4.0 = 0 to full in a quarter second. */
    static final double OUTTAKE_RAMP_PER_SEC  = 4.0;

    /** Flywheel state carried between loops: on/off and the spin-up ramp. */
    static class Outtake {
        private boolean on = false;
        private boolean lastButton = false;
        private double power = 0.0;

        /**
         * @param toggleButton true while the flywheel button is held; each new press flips ON/OFF
         * @param dtSeconds    time since the previous loop
         * @return the power to send to the flywheel motor
         */
        double update(boolean toggleButton, double dtSeconds) {
            double dt = Math.max(0.001, Math.min(0.1, finite(dtSeconds, 0.02)));
            if (toggleButton && !lastButton) {
                on = !on;
            }
            lastButton = toggleButton;

            if (!on) {
                power = 0.0;
            } else {
                double step = OUTTAKE_RAMP_PER_SEC * dt;
                power += Math.max(-step, Math.min(step, OUTTAKE_POWER - power));
            }
            return power;
        }

        boolean isOn() {
            return on;
        }
    }
}
