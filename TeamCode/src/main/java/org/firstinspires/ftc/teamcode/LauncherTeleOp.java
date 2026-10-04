package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Drive, collect, and launch POLLEN into a HIVE's raised CELL.
 *
 * <p>The driver's side of a flywheel, which is a different thing to fly from an intake. A roller
 * either runs or does not; a flywheel has to be <em>brought up to speed before the ball reaches
 * it</em>, because there is nothing holding a ball back from a spinning wheel. So the flywheel
 * latches here where the intake deliberately does not: a driver needs it running while both hands
 * are busy driving up to a ball.</p>
 *
 * <p>Speed is commanded and read back in revolutions per minute, and the telemetry shows both. That
 * gap is the mechanism: hold the bumper, watch the wheel climb, and only then collect. Firing into
 * a wheel that has not arrived throws the ball a foot.</p>
 *
 * <p>Range comes from the speed, and the speed is set by hand on the D-pad rather than solved for.
 * Solving it needs a distance to the target, which is what the AprilTag Cluster under each CELL is
 * for, and that is a job for an OpMode that looks up rather than one a driver aims.</p>
 */
@TeleOp(name = "Launcher Mecanum", group = "Linear OpMode")
public class LauncherTeleOp extends LinearOpMode {

    /** Below this the trigger is noise rather than intent. */
    private static final double TRIGGER_DEADBAND = 0.1;

    /**
     * Where the speed starts, in rev/min.
     *
     * <p>About what it takes to drop a POLLEN into a raised CELL from the middle of an alliance's
     * half of the field: roughly a third of this motor's free speed, which leaves room to trim in
     * both directions. It is a starting point for a driver, not a calibration.</p>
     */
    private static final double DEFAULT_TARGET_RPM = 2100.0;

    /** What one press of the D-pad is worth, in rev/min. */
    private static final double TRIM_RPM = 100.0;

    /** Within this of the target, in rev/min, the wheel is worth feeding. */
    private static final double READY_RPM = 60.0;

    private final MecanumMovement movement = new MecanumMovement();

    @Override
    public void runOpMode() {
        DriveHardware drive = DriveHardware.initWithoutEncoders(hardwareMap);

        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        // A shooter is commanded by speed, not by power: a tiring battery that drops a
        // power-commanded wheel by 200 rev/min drops every shot short with it, and the controller
        // can hold the speed instead if it is told to.
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        TouchSensor loaded = hardwareMap.get(TouchSensor.class, "intakeTouch");
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        JoystickInput leftStick = new JoystickInput();
        JoystickInput rightStick = new JoystickInput();

        double targetRpm = DEFAULT_TARGET_RPM;
        boolean trimmed = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            leftStick.update(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            rightStick.update(gamepad1.right_stick_x, gamepad1.right_stick_y);
            drive.setPowers(movement.calculate(leftStick, rightStick));

            // On the press rather than while held, or one nudge of the D-pad would run the speed
            // all the way to the end of its range in a fifth of a second.
            boolean trimming = gamepad1.dpad_up || gamepad1.dpad_down;
            if (trimming && !trimmed) {
                targetRpm += gamepad1.dpad_up ? TRIM_RPM : -TRIM_RPM;
                if (targetRpm < 0.0) {
                    targetRpm = 0.0;
                }
            }
            trimmed = trimming;

            boolean spinning = gamepad1.right_bumper;
            // Degrees per second, so that nothing here has to know this motor's encoder
            // resolution: that number lives in the robot's configuration, and a second copy in an
            // OpMode is one that can disagree with the gearbox actually bolted on.
            flywheel.setVelocity(spinning ? targetRpm * 360.0 / 60.0 : 0.0, AngleUnit.DEGREES);
            double actualRpm = flywheel.getVelocity(AngleUnit.DEGREES) * 60.0 / 360.0;
            boolean ready = spinning && Math.abs(actualRpm - targetRpm) < READY_RPM;

            intake.setPower(gamepad1.right_trigger > TRIGGER_DEADBAND ? 1.0
                    : gamepad1.left_trigger > TRIGGER_DEADBAND ? -1.0 : 0.0);

            telemetry.addData("flywheel", "%.0f / %.0f rpm%s",
                    actualRpm, targetRpm, ready ? "  READY" : spinning ? "  spinning up" : "");
            telemetry.addData("intake", loaded.isPressed() ? "LOADED" : "empty");
            telemetry.addData("battery", "%.2f V", battery.getVoltage());
            telemetry.update();
        }

        flywheel.setVelocity(0.0);
        intake.setPower(0.0);
        drive.stop();
    }
}
