package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * gm0's field-centric mecanum drive, with a software brake on X.
 *
 * <p>Driving is gm0's IMU math, unchanged, but the powers go through Pedro Pathing's
 * {@link Follower} rather than straight to the motors, because the follower owns the drive motors
 * and the Pinpoint odometry the brake needs, and is updated every loop so the odometry never
 * loses track of where the robot is.</p>
 *
 * <p><b>Software brake</b> (gamepad 1 X, press to engage, press again to release): records the
 * odometry pose at the press and hands it to {@link Follower#hold}, Pedro's hold-endpoint. Every
 * loop the follower drives the chassis back toward that x, y and heading, so a shove is met with
 * a push the other way that grows with how far the robot has been moved, and once the shove stops
 * the robot returns to the spot and the heading it was braked at. The sticks are ignored while the
 * brake is on.</p>
 *
 * <p>How stiffly it holds is Pedro's translational and heading gains in {@link Constants}; those
 * are untuned starting values until {@code ForesightTuner} has been run on the real robot.</p>
 */
@TeleOp
public class Gm0IMU extends LinearOpMode {

    /** Strafe gain gm0 uses to counteract imperfect strafing. */
    private static final double STRAFE_CORRECTION = 1.1;

    private Follower follower;
    private boolean braked = false;
    private Pose brakePose = null;
    private boolean brakeButtonWasDown = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // The follower maps FL/FR/BL/BR and the Pinpoint by the names in Constants.
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(Pose.zero());

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Keep the odometry tracking through INIT, so the robot can be set down and nudged.
        while (opModeInInit()) {
            follower.update();
            telemetry.addData("Status", "Initialized - waiting for start");
            telemetry.addData("Software Brake", "X to engage / release");
            telemetry.update();
        }

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // X toggles the brake on the press, not while held.
            boolean brakeButtonDown = gamepad1.x;
            if (brakeButtonDown && !brakeButtonWasDown) {
                if (braked) {
                    releaseBrake();
                } else {
                    engageBrake();
                }
            }
            brakeButtonWasDown = brakeButtonDown;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            if (!braked) {
                drive(imu);
            }

            // Reads the odometry, then either applies the manual powers or holds the brake pose.
            follower.update();

            Pose pose = follower.pose();
            telemetry.addData("Software Brake", braked ? "ON (X to release)" : "off (X to engage)");
            telemetry.addData("Pose (in, deg)", "x %.1f  y %.1f  heading %.1f",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            if (braked) {
                telemetry.addData("Holding (in, deg)", "x %.1f  y %.1f  heading %.1f",
                        brakePose.x(), brakePose.y(), Math.toDegrees(brakePose.heading()));
                telemetry.addData("Off by (in, deg)", "%.2f  %.1f",
                        Math.hypot(pose.x() - brakePose.x(), pose.y() - brakePose.y()),
                        Math.toDegrees(AngleUnit.normalizeRadians(pose.heading() - brakePose.heading())));
            }
            telemetry.update();
        }

        follower.stop();
        follower.update();
    }

    /** Pins the robot to where it is right now, heading included. */
    private void engageBrake() {
        brakePose = follower.pose();
        follower.hold(brakePose);
        braked = true;
    }

    /** Hands the drive back to the sticks. */
    private void releaseBrake() {
        braked = false;
        brakePose = null;
        follower.manual(0.0, 0.0, 0.0);
    }

    /** gm0's field-centric drive, sent to the follower as manual powers. */
    private void drive(IMU imu) {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * STRAFE_CORRECTION;  // Counteract imperfect strafing

        // Pedro's strafe is positive to the left and its turn positive counter-clockwise, the
        // opposite of the stick's right and clockwise. The follower's Mecanum scales the wheel
        // powers down together if any would exceed 1, which is what gm0's denominator did.
        follower.manual(rotY, -rotX, -rx);
    }
}
