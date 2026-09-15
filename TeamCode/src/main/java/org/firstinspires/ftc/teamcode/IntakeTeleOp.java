package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Drive, and run an intake that picks up game elements.
 *
 * <p>Written to be flown by a driver and to be the thing the simulator's mechanisms and sensors are
 * exercised through. Every reading on the telemetry comes out of the world rather than out of a
 * configuration file: the touch sensor is pressed because a ball is really in the mouth, the colour
 * is the colour of that ball, the range is the distance the beam actually travelled, and the
 * voltage sags while the wheels pull current.</p>
 *
 * <p>The intake runs while the right trigger is held and reverses on the left, rather than
 * latching. A latch is the first thing a team adds and the first thing that hides a mechanism
 * fault: a driver holding a trigger can feel whether the ball came in.</p>
 */
@TeleOp(name = "Intake Mecanum", group = "Linear OpMode")
public class IntakeTeleOp extends LinearOpMode {

    /** Below this the trigger is noise rather than intent. */
    private static final double TRIGGER_DEADBAND = 0.1;

    private final MecanumMovement movement = new MecanumMovement();

    @Override
    public void runOpMode() {
        DriveHardware drive = DriveHardware.initWithoutEncoders(hardwareMap);

        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        TouchSensor loaded = hardwareMap.get(TouchSensor.class, "intakeTouch");
        ColorSensor colour = hardwareMap.get(ColorSensor.class, "intakeColor");
        DistanceSensor ahead = hardwareMap.get(DistanceSensor.class, "frontRange");
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        JoystickInput leftStick = new JoystickInput();
        JoystickInput rightStick = new JoystickInput();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            leftStick.update(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            rightStick.update(gamepad1.right_stick_x, gamepad1.right_stick_y);
            drive.setPowers(movement.calculate(leftStick, rightStick));

            // Both triggers at once cancels, which is what a real pair of rollers would do.
            double sweep = 0.0;
            if (gamepad1.right_trigger > TRIGGER_DEADBAND) {
                sweep += gamepad1.right_trigger;
            }
            if (gamepad1.left_trigger > TRIGGER_DEADBAND) {
                sweep -= gamepad1.left_trigger;
            }
            intake.setPower(sweep);

            telemetry.addData("intake", "%+.2f%s", sweep, loaded.isPressed() ? "  LOADED" : "");
            telemetry.addData("in the mouth", "r%d g%d b%d",
                    colour.red(), colour.green(), colour.blue());
            double metres = ahead.getDistance(DistanceUnit.METER);
            // A distance sensor that sees nothing reports NaN, and %.2f of NaN is the word "NaN" --
            // which is exactly what a driver should see rather than a plausible-looking number.
            telemetry.addData("ahead", Double.isNaN(metres) ? "out of range"
                    : String.format("%.2f m", metres));
            telemetry.addData("battery", "%.2f V", battery.getVoltage());
            telemetry.update();
        }

        intake.setPower(0.0);
        drive.stop();
    }
}
