package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Two Controller TeleOp", group = "Linear OpMode")
public class TwoControllerBasicTeleOp extends LinearOpMode {

    private final MecanumMovement movement = new MecanumMovement();

    @Override
    public void runOpMode() {

        DriveHardware drive = DriveHardware.initWithEncoders(hardwareMap);

        JoystickInput leftStick = new JoystickInput();
        JoystickInput rightStick = new JoystickInput();

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "INTAKE");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotorEx outtake = hardwareMap.get(DcMotorEx.class, "OUTTAKE");
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Real stick Y is negative when pushed forward; MecanumMovement's
            // algorithm treats positive Y as forward, so negate it here.
            leftStick.update(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            rightStick.update(gamepad1.right_stick_x, gamepad1.right_stick_y);

            ChassisMotors powers = movement.calculate(leftStick, rightStick);

            //intake power
            if (gamepad2.right_bumper){
                intake.setPower(5.00);
            } else if (gamepad2.left_bumper) {
                intake.setPower(-5.00);
            } else{
                intake.setPower(0.00);
            }

            //outtake power
            if (gamepad2.right_trigger!=0){
                outtake.setVelocity(gamepad2.right_trigger*-100000);
            }else{
                outtake.setPower(0);
            }

            drive.setPowers(powers);

            telemetry.addData("FL | FR", "%.2f | %.2f", powers.getFL(), powers.getFR());
            telemetry.addData("BL | BR", "%.2f | %.2f", powers.getBL(), powers.getBR());
            telemetry.addData("Intake","#.2f | %.2f", intake.getPower() );
            telemetry.addData("Outtake power", outtake.getPower());
            telemetry.addData("Outtake velocity", outtake.getVelocity());
            telemetry.addData("Outtake mode", outtake.getMode());
            telemetry.addData("Trigger", gamepad2.right_trigger);
            telemetry.update();
        }

        drive.stop();
    }
}
