package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;

@TeleOp(name = "I AM VERITY V2")
public class iamyou extends LinearOpMode {

    private final IamYouLogic logic = new IamYouLogic();

    @Override
    public void runOpMode() {

        // ================= IMU SETUP =================
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new com.qualcomm.hardware.rev.RevHubOrientationOnRobot(
                        com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        // ================= INIT DIRECTION SNAP =================
        double rawInitHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        String initCardinal   = logic.snapToCardinal(rawInitHeading);

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

        // --- Pre-start telemetry ---
        telemetry.addData("Status", "Initialized");
        telemetry.addData("INIT Direction", initCardinal);
        telemetry.addData("Cardinal Direction", initCardinal);
        telemetry.addData("Driver Position", driverPosition + "  (D-pad: Up=N Down=S Left=W Right=E)");
        telemetry.update();

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

            double rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            String liveCardinal = logic.snapToCardinal(rawHeading);

            if (gamepad1.options) {
                imu.resetYaw();
            }

            if (gamepad1.dpad_up)    driverPosition = "NORTH";
            if (gamepad1.dpad_down)  driverPosition = "SOUTH";
            if (gamepad1.dpad_left)  driverPosition = "WEST";
            if (gamepad1.dpad_right) driverPosition = "EAST";

            IamYouLogic.DriveParameters params = new IamYouLogic.DriveParameters(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                gamepad1.left_trigger,
                rawHeading,
                initCardinal,
                driverPosition
            );

            IamYouLogic.DriveResult result = logic.calculate(params);

            FL.setPower(result.powers.getFL());
            BL.setPower(result.powers.getBL());
            FR.setPower(result.powers.getFR());
            BR.setPower(result.powers.getBR());

            // ================= TELEMETRY =================
            telemetry.addData("Cardinal Direction", liveCardinal);
            telemetry.addData("INIT Direction",     initCardinal);
            telemetry.addData("Driver Position",    driverPosition + "  (D-pad to change)");
            telemetry.addData("---", "---");
            telemetry.addData("Heading Raw (deg)",       "%.2f", rawHeading);
            telemetry.addData("Heading Corrected (deg)", "%.2f", Math.toDegrees(result.correctedHeadingRadians));
            telemetry.addData("Slow Mode",               gamepad1.left_trigger > 0.1 ? "ON" : "OFF");
            telemetry.addData("Drive Input",             "%.2f", result.drive);
            telemetry.addData("Strafe Input",            "%.2f", result.strafe);
            telemetry.addData("Twist Input",             "%.2f", result.twist);
            telemetry.addData("Field Drive",             "%.2f", result.fieldDrive);
            telemetry.addData("Field Strafe",            "%.2f", result.fieldStrafe);
            telemetry.addData("FL | FR",                 "%.2f | %.2f", result.powers.getFL(), result.powers.getFR());
            telemetry.addData("BL | BR",                 "%.2f | %.2f", result.powers.getBL(), result.powers.getBR());
            telemetry.addData("FL Vel | FR Vel",         "%.0f | %.0f", FL.getVelocity(), FR.getVelocity());
            telemetry.addData("BL Vel | BR Vel",         "%.0f | %.0f", BL.getVelocity(), BR.getVelocity());
            telemetry.update();
        }

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }
}
