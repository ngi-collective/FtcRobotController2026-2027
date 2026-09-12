package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Field-relative mecanum drive, adapted from the vendor sample
 * FtcRobotController/.../external/samples/RobotTeleopMecanumFieldRelativeDrive.java.
 *
 * The math is left as close to that sample as possible so the two files can be
 * compared directly: convert the joystick vector to polar coordinates, subtract
 * the robot's current IMU heading, convert back to cartesian, then run the
 * standard mecanum kinematics. That's the entire field-relative trick - one
 * rotation, no cardinal-direction snapping, no per-segment sign corrections.
 *
 * Hold left bumper for robot-relative (RC-car style) driving, same as the sample.
 * Press A to re-zero yaw to the robot's current heading, same as the sample.
 *
 * Driver-position calibration: hold the left trigger and point the left stick
 * the same direction the robot is CURRENTLY facing, relative to where you're
 * standing (e.g. if the robot's front is pointed off to your right, push the
 * stick right). That single gesture is enough to derive the offset between
 * "your forward" and the robot's zero-heading frame - see driveFieldRelative
 * for the derivation. The left stick doesn't drive while the trigger is held.
 */
@TeleOp(name = "Field Relative Mecanum (Sample)", group = "Linear OpMode")
public class FieldRelativeMecanumTeleOp extends LinearOpMode {

    private static final double CALIBRATION_TRIGGER_THRESHOLD = 0.5;
    private static final double CALIBRATION_STICK_DEADZONE = 0.2;

    @Override
    public void runOpMode() {

        // ================= IMU SETUP =================
        // Orientation must match how the Control/Expansion Hub is actually
        // mounted on the robot - this matches iamyou.java's mounting.
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        // ================= DRIVE MOTORS =================
        DriveHardware drive = DriveHardware.initWithoutEncoders(hardwareMap);

        // Radians added to the joystick angle so "forward" means "away from
        // the driver". 0 is correct as long as the robot starts pointed away
        // from you; hold the left trigger and aim the stick at the robot's
        // current facing (relative to you) any time to recalibrate.
        double driverOffsetRadians = 0.0;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Calibrate", "Hold Left Trigger, point Left Stick where the robot's front is currently aimed (relative to you)");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            driverOffsetRadians = calibrateDriverOffset(driverOffsetRadians, imu);

            telemetry.addData("Status", "Initialized - waiting for start");
            telemetry.addData("Calibrate", "Hold Left Trigger, point Left Stick where the robot's front is currently aimed (relative to you)");
            telemetry.addData("Driver Offset (deg)", "%.1f", Math.toDegrees(driverOffsetRadians));
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                double headingBeforeReset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                imu.resetYaw();
                driverOffsetRadians = AngleUnit.normalizeRadians(driverOffsetRadians - headingBeforeReset);
            }

            driverOffsetRadians = calibrateDriverOffset(driverOffsetRadians, imu);
            boolean calibrating = gamepad1.left_trigger >= CALIBRATION_TRIGGER_THRESHOLD;

            // Real stick Y is negative when pushed forward; both forward and
            // right below use the sample's convention: positive = forward / right.
            // While calibrating, the left stick sets the offset instead of driving.
            double forward = calibrating ? 0.0 : -gamepad1.left_stick_y;
            double right = calibrating ? 0.0 : gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            ChassisMotors powers = gamepad1.left_bumper
                    ? driveRobotRelative(forward, right, rotate)
                    : driveFieldRelative(forward, right, rotate, imu, driverOffsetRadians);

            drive.setPowers(powers);

            telemetry.addData("Mode", calibrating ? "Calibrating"
                    : gamepad1.left_bumper ? "Robot Relative" : "Field Relative");
            telemetry.addData("Driver Offset (deg)", "%.1f", Math.toDegrees(driverOffsetRadians));
            telemetry.addData("Heading (deg)", "%.2f",
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("FL | FR", "%.2f | %.2f", powers.getFL(), powers.getFR());
            telemetry.addData("BL | BR", "%.2f | %.2f", powers.getBL(), powers.getBR());
            telemetry.update();
        }

        drive.stop();
    }

    /**
     * While the left trigger is held past the threshold and the left stick is
     * pushed out of its deadzone, reads the driver-position offset directly
     * off the stick and returns the new value. Otherwise returns
     * {@code currentOffsetRadians} unchanged.
     */
    private double calibrateDriverOffset(double currentOffsetRadians, IMU imu) {
        if (gamepad1.left_trigger < CALIBRATION_TRIGGER_THRESHOLD) {
            return currentOffsetRadians;
        }

        double stickForward = -gamepad1.left_stick_y;
        double stickRight = gamepad1.left_stick_x;
        if (Math.hypot(stickForward, stickRight) < CALIBRATION_STICK_DEADZONE) {
            return currentOffsetRadians;
        }

        // The stick angle IS the robot's front, expressed relative to the
        // driver: "up" is straight away from you, matching normal driving
        // input. See driveFieldRelative's javadoc for why this single reading
        // is enough to derive the full offset.
        double stickAngle = Math.atan2(stickForward, stickRight);
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return AngleUnit.normalizeRadians(Math.toRadians(90.0) + heading - stickAngle);
    }

    /**
     * Drives the robot field-relative: the direction you push the stick is the
     * direction the robot moves on the field, regardless of which way the
     * robot is currently facing.
     *
     * {@code driverOffsetRadians} folds in where the driver is standing.
     * Derivation: "pure forward" on the stick is angle 90 (see the atan2 call
     * below); the robot's own front, at any moment, sits at angle
     * {@code 90 + heading} in the robot's zero-heading reference frame (that's
     * the definition of heading). Calibration reads the stick angle the
     * driver used to point at the robot's front and solves
     * {@code stickAngle + offset = 90 + heading} for offset - which is
     * exactly the line in {@link #calibrateDriverOffset}. Applying that same
     * offset here, before subtracting the live heading, re-expresses every
     * future stick direction in the robot's reference frame the same way.
     */
    private ChassisMotors driveFieldRelative(double forward, double right, double rotate, IMU imu,
                                              double driverOffsetRadians) {
        // First, convert the requested direction to polar coordinates.
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, add the driver-position offset (see javadoc above).
        theta += driverOffsetRadians;

        // Third, rotate that angle by the angle the robot is currently pointing.
        theta = AngleUnit.normalizeRadians(theta
                - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Finally, convert back to cartesian - now expressed relative to the robot.
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        return driveRobotRelative(newForward, newRight, rotate);
    }

    /**
     * Drives the robot relative to itself, like an RC car: pushing the stick
     * forward always moves the robot toward wherever its front currently points.
     */
    private ChassisMotors driveRobotRelative(double forward, double right, double rotate) {
        double flPower = forward + right + rotate;
        double frPower = forward - right - rotate;
        double blPower = forward - right + rotate;
        double brPower = forward + right - rotate;

        // Only scale down if a wheel would exceed +/-1.0; never scale up.
        double max = 1.0;
        max = Math.max(max, Math.abs(flPower));
        max = Math.max(max, Math.abs(frPower));
        max = Math.max(max, Math.abs(blPower));
        max = Math.max(max, Math.abs(brPower));

        return new ChassisMotors(flPower / max, frPower / max, blPower / max, brPower / max);
    }
}
