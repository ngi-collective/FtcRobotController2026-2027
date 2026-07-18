package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;

@TeleOp(name = "I AM YOU V2")
public class iamyou extends LinearOpMode {

    // ---- Tunable constants ----
    private static final double JOYSTICK_DEADZONE    = 0.05;
    private static final double SLOW_MODE_DRIVE_MULT = 0.35;
    private static final double SLOW_MODE_TWIST_MULT = 0.50;

    // Cardinal direction heading offsets (degrees, FTC IMU convention: CCW = positive)
    // NORTH = bot facing IMU's native forward axis at startup (~0°)
    // EAST  = bot rotated 90° clockwise from NORTH at startup (~-90°)
    // SOUTH = bot facing opposite of NORTH at startup (~±180°)
    // WEST  = bot rotated 90° counter-clockwise from NORTH at startup (~+90°)
    private static final double HEADING_NORTH =   0.0;
    private static final double HEADING_EAST  = -90.0;
    private static final double HEADING_SOUTH = 180.0;
    private static final double HEADING_WEST  =  90.0;

    // ================= DRIVER POSITION OFFSETS =================
    // This is SEPARATE from the robot's INIT cardinal above.
    // It answers "which side of the field is the human driver standing on?"
    // SOUTH = 0° = the confirmed-working baseline. Everything else is
    // defined relative to it.
    private static double driverOffsetRadians(String pos) {
        switch (pos) {
            case "EAST":  return Math.toRadians(-90.0);
            case "NORTH": return Math.toRadians(180.0);
            case "WEST":  return Math.toRadians(90.0);
            case "SOUTH":
            default:      return 0.0; // baseline, unchanged from original working code
        }
    }

    @Override
    public void runOpMode() {

        // ================= IMU SETUP =================
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // If your Control Hub is NOT mounted "logo up, USB forward," change these two values:
        imu.initialize(new IMU.Parameters(
                new com.qualcomm.hardware.rev.RevHubOrientationOnRobot(
                        com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));
        // NOTE: We do NOT call imu.resetYaw() here. We read the raw heading first to
        // determine which cardinal direction the bot started in. That reading is only
        // meaningful against the IMU's own physical reference, not a zeroed reference.

        // ================= INIT DIRECTION SNAP =================
        // Read the raw heading at startup and snap it to the nearest cardinal direction.
        // This determines the heading-offset correction used for the entire match.
        double rawInitHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        String initCardinal   = snapToCardinal(rawInitHeading);

        // ================= DRIVER POSITION =================
        // Which side of the field YOU (the driver) are standing on, relative to the
        // robot. Default SOUTH = the confirmed-working baseline. D-pad acts like a
        // compass and can be changed any time, including mid-match, if you move.
        String driverPosition = "SOUTH";

        // ================= DRIVE MOTORS =================
        DcMotorEx FL = hardwareMap.get(DcMotorEx.class, "FL");
        DcMotorEx FR = hardwareMap.get(DcMotorEx.class, "FR");
        DcMotorEx BL = hardwareMap.get(DcMotorEx.class, "BL");
        DcMotorEx BR = hardwareMap.get(DcMotorEx.class, "BR");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : Arrays.asList(FL, FR, BL, BR)) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // --- Pre-start telemetry (shows on Driver Station during Init) ---
        telemetry.addData("Status", "Initialized");
        telemetry.addData("INIT Direction", initCardinal);
        telemetry.addData("Cardinal Direction", initCardinal);
        telemetry.addData("Driver Position", driverPosition + "  (D-pad: Up=N Down=S Left=W Right=E)");
        telemetry.addData(">", "Left Stick = Field-Centric Drive/Strafe");
        telemetry.addData(">", "Right Stick X = Rotate");
        telemetry.addData(">", "Left Trigger = Slow Mode");
        telemetry.addData(">", "OPTIONS/BACK = Re-zero heading");
        telemetry.update();

        // Allow selecting driver position before pressing start too
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_up)    driverPosition = "NORTH";
            if (gamepad1.dpad_down)  driverPosition = "SOUTH";
            if (gamepad1.dpad_left)  driverPosition = "WEST";
            if (gamepad1.dpad_right) driverPosition = "EAST";

            telemetry.addData("Status", "Initialized - waiting for start");
            telemetry.addData("INIT Direction", initCardinal);
            telemetry.addData("Driver Position", driverPosition + "  (D-pad: Up=N Down=S Left=W Right=E)");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {

            // ================= HEADING READ =================
            double rawHeading   = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            String liveCardinal = snapToCardinal(rawHeading);

            // Compute corrected heading based on which cardinal the bot started in.
            // Subtracting the init offset re-aligns the rotation matrix so that
            // "field forward" always corresponds to the driver's intended direction.
            double correctedHeading;
            if (initCardinal.equals("NORTH")) {
                correctedHeading = Math.toRadians(rawHeading - HEADING_NORTH);
            } else if (initCardinal.equals("EAST")) {
                correctedHeading = Math.toRadians(rawHeading - HEADING_EAST);
            } else if (initCardinal.equals("SOUTH")) {
                correctedHeading = Math.toRadians(rawHeading - HEADING_SOUTH);
            } else {
                // WEST
                correctedHeading = Math.toRadians(rawHeading - HEADING_WEST);
            }

            // Re-zero on demand (driver presses Options/Back mid-match)
            if (gamepad1.options) {
                imu.resetYaw();
                rawInitHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            }

            // ================= DRIVER POSITION UPDATE =================
            // Lets you re-declare your position mid-match if you physically move
            // to a different side of the field.
            if (gamepad1.dpad_up)    driverPosition = "NORTH";
            if (gamepad1.dpad_down)  driverPosition = "SOUTH";
            if (gamepad1.dpad_left)  driverPosition = "WEST";
            if (gamepad1.dpad_right) driverPosition = "EAST";

            // ================= GAMEPAD READ =================
            double rawDrive  = -gamepad1.left_stick_y;
            double rawStrafe =  gamepad1.left_stick_x;
            double twist     = applyDeadzone(-gamepad1.right_stick_x, JOYSTICK_DEADZONE);

            // Circular (radial) deadzone on the translation stick.
            // Deadzoning each axis independently distorts diagonal inputs;
            // deadzoning the magnitude keeps the drive/strafe ratio clean.
            double[] trans = applyDeadzoneVector(rawDrive, rawStrafe, JOYSTICK_DEADZONE);
            double drive   = trans[0];
            double strafe  = trans[1];

            // ================= DRIVER POSITION ROTATION =================
            // Rotates the raw stick vector by however far around the field YOU are
            // standing, relative to the SOUTH baseline. At SOUTH this is a 0°
            // rotation, so drive/strafe pass through completely unchanged —
            // identical to the original confirmed-working behavior.
            double doff  = driverOffsetRadians(driverPosition);
            double cosD  = Math.cos(doff);
            double sinD  = Math.sin(doff);
            double driveR  = drive * cosD - strafe * sinD;
            double strafeR = drive * sinD + strafe * cosD;
            drive  = driveR;
            strafe = strafeR;

            // ================= PER-SEGMENT INPUT CORRECTION =================
            // Each init cardinal has a different physical relationship between
            // joystick intent and motor output. Correcting the INPUT before the
            // rotation matrix ensures the fix holds at ALL runtime headings, not
            // just at heading=0.
            //
            //   NORTH: strafe L/R swapped          -> negate strafe
            //   SOUTH: fwd/back swapped             -> negate drive
            //   EAST:  axes 90° CW swapped          -> d=strafe,  s=drive
            //   WEST:  axes 90° CCW swapped         -> d=-strafe, s=-drive
            double d, s;
            if (initCardinal.equals("NORTH")) {
                d =  drive;   s = -strafe;
            } else if (initCardinal.equals("SOUTH")) {
                d = -drive;   s =  strafe;
            } else if (initCardinal.equals("EAST")) {
                d =  strafe;  s =  drive;
            } else {
                // WEST
                d = -strafe;  s = -drive;
            }

            // ================= FIELD-CENTRIC ROTATION MATRIX =================
            // Rotates the corrected input vector into the robot's local frame.
            // Twist bypasses this entirely so rotation always works normally
            // regardless of bot orientation.
            double cosH = Math.cos(correctedHeading);
            double sinH = Math.sin(correctedHeading);

            double fieldDrive  =  d * cosH + s * sinH;
            double fieldStrafe = -d * sinH + s * cosH;

            // ================= SLOW MODE =================
            double driveMult = 1.0;
            double twistMult = 1.0;
            if (gamepad1.left_trigger > 0.1) {
                driveMult = SLOW_MODE_DRIVE_MULT;
                twistMult = SLOW_MODE_TWIST_MULT;
            }

            // ================= MECANUM KINEMATICS =================
            // Normalize against raw (pre-multiplied) magnitudes so wheel ratios
            // are correct before slow mode scales everything down.
            double denominator = Math.max(
                    Math.abs(fieldDrive) + Math.abs(fieldStrafe) + Math.abs(twist), 1.0);

            double normDrive  = (fieldDrive  / denominator) * driveMult;
            double normStrafe = (fieldStrafe / denominator) * driveMult;
            double normTwist  = (twist       / denominator) * twistMult;

            double flPower = normDrive + normStrafe + normTwist;
            double blPower = normDrive - normStrafe + normTwist;
            double frPower = normDrive - normStrafe - normTwist;
            double brPower = normDrive + normStrafe - normTwist;

            // ================= SET MOTOR POWERS =================
            FL.setPower(flPower);
            BL.setPower(blPower);
            FR.setPower(frPower);
            BR.setPower(brPower);

            // ================= TELEMETRY =================
            telemetry.addData("Cardinal Direction", liveCardinal);
            telemetry.addData("INIT Direction",     initCardinal);
            telemetry.addData("Driver Position",    driverPosition + "  (D-pad to change)");
            telemetry.addData("---", "---");
            telemetry.addData("Heading Raw (deg)",       "%.2f", rawHeading);
            telemetry.addData("Heading Corrected (deg)", "%.2f", Math.toDegrees(correctedHeading));
            telemetry.addData("Slow Mode",               gamepad1.left_trigger > 0.1 ? "ON" : "OFF");
            telemetry.addData("Drive Input",             "%.2f", drive);
            telemetry.addData("Strafe Input",            "%.2f", strafe);
            telemetry.addData("Twist Input",             "%.2f", twist);
            telemetry.addData("Field Drive",             "%.2f", fieldDrive);
            telemetry.addData("Field Strafe",            "%.2f", fieldStrafe);
            telemetry.addData("FL | FR",                 "%.2f | %.2f", flPower, frPower);
            telemetry.addData("BL | BR",                 "%.2f | %.2f", blPower, brPower);
            telemetry.addData("FL Vel | FR Vel",         "%.0f | %.0f", FL.getVelocity(), FR.getVelocity());
            telemetry.addData("BL Vel | BR Vel",         "%.0f | %.0f", BL.getVelocity(), BR.getVelocity());
            telemetry.update();
        }

        // Stop all motors
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    // ---------------------------------------------------------------
    // Snap a raw IMU heading (degrees) to the nearest cardinal (±45°)
    // ---------------------------------------------------------------
    private String snapToCardinal(double headingDeg) {
        while (headingDeg >  180.0) headingDeg -= 360.0;
        while (headingDeg <= -180.0) headingDeg += 360.0;

        if      (headingDeg >= -45.0  && headingDeg <  45.0)  return "NORTH";
        else if (headingDeg >=  45.0  && headingDeg < 135.0)  return "WEST";
        else if (headingDeg >= -135.0 && headingDeg < -45.0)  return "EAST";
        else                                                    return "SOUTH";
    }

    // ---------------------------------------------------------------
    // Single-axis deadzone — prevents rotation stick creep
    // ---------------------------------------------------------------
    private double applyDeadzone(double value, double deadzone) {
        return Math.abs(value) < deadzone ? 0.0 : value;
    }

    // ---------------------------------------------------------------
    // Circular deadzone for 2D translation stick — keeps diagonal
    // inputs clean by deadzoning the magnitude, not each axis
    // ---------------------------------------------------------------
    private double[] applyDeadzoneVector(double x, double y, double deadzone) {
        return Math.hypot(x, y) < deadzone ? new double[]{0.0, 0.0} : new double[]{x, y};
    }
}