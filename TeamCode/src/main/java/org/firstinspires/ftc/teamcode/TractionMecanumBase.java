package org.firstinspires.ftc.teamcode;

import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

/**
 * The drive loop shared by the two Traction Mecanum TeleOps: mecanum mixing, then {@link TractionControlLogic}
 * scaling all four wheels against the Pinpoint's measured ground speed.
 *
 * Gamepad 1:
 *   Left stick .......... drive (robot- or field-centric, per subclass)
 *   Right stick X ....... turn
 *   Right bumper (hold) . bypass traction control (pushing matches)
 *   Left bumper (hold) .. slow mode
 *   Options ............. field-centric only: the robot's current front becomes "forward"
 *
 * LED: green = full grip, amber = power limited, red = heavy slip, flashing purple = Pinpoint fault (fail open:
 * plain mecanum, and field-centric drives robot-centric until the Pinpoint recovers).
 */
public abstract class TractionMecanumBase extends LinearOpMode {

    private static final double LED_FLASH_SECONDS = 0.25;

    private final boolean fieldCentric;
    private final MecanumMovement movement = new MecanumMovement();
    private final TractionControlLogic traction = new TractionControlLogic();

    private String ledShown = "";

    protected TractionMecanumBase(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    @Override
    public void runOpMode() {
        // One bulk read per hub per loop, so four getVelocity() calls are not four hub round trips.
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        DriveHardware drive = DriveHardware.initWithoutEncoders(hardwareMap);
        GoBildaPinpointDriver pinpoint = initPinpoint();
        // Zeroes position and heading, so field "forward" is wherever the robot faces now.
        pinpoint.resetPosAndIMU();

        while (!isStarted() && !isStopRequested()) {
            pinpoint.update();
            telemetry.addData("Status", "Initialized - waiting for start");
            telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
            telemetry.addData("Drive", fieldCentric ? "field-centric (forward = facing at INIT)" : "robot-centric");
            telemetry.update();
        }
        if (isStopRequested()) return;

        JoystickInput leftStick = new JoystickInput();
        JoystickInput rightStick = new JoystickInput();
        double headingZero = 0.0;
        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime clock = new ElapsedTime();
        telemetry.setMsTransmissionInterval(100);

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }
            double dt = loopTimer.seconds();
            loopTimer.reset();

            pinpoint.update();
            GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
            boolean trusted = traction.observe(new TractionControlLogic.Odometry(
                    health(status),
                    pinpoint.getVelX(DistanceUnit.METER),
                    pinpoint.getVelY(DistanceUnit.METER),
                    pinpoint.getHeading(AngleUnit.RADIANS),
                    pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)));

            if (fieldCentric && trusted && gamepad1.options) {
                headingZero = traction.headingRadians();
            }

            // Real stick Y is negative when pushed forward.
            double forward = deadband(-gamepad1.left_stick_y);
            double right = deadband(gamepad1.left_stick_x);
            double twist = deadband(gamepad1.right_stick_x);
            boolean driveFieldCentric = fieldCentric && trusted;
            if (driveFieldCentric) {
                // Stick is in the field frame; rotate it into the robot's frame (heading counter-clockwise positive).
                double h = traction.headingRadians() - headingZero;
                double robotForward = forward * Math.cos(h) - right * Math.sin(h);
                double robotRight = forward * Math.sin(h) + right * Math.cos(h);
                forward = robotForward;
                right = robotRight;
            }
            leftStick.update(right, forward);
            rightStick.update(twist, 0.0);
            ChassisMotors commanded = movement.calculate(leftStick, rightStick);
            if (gamepad1.left_bumper) {
                double s = TractionConstants.SLOW_MODE_SCALE;
                commanded = new ChassisMotors(commanded.getFL() * s, commanded.getFR() * s,
                        commanded.getBL() * s, commanded.getBR() * s);
            }

            TractionControlLogic.Result result = traction.update(commanded,
                    new TractionControlLogic.WheelVelocities(
                            drive.fl.getVelocity(), drive.fr.getVelocity(),
                            drive.bl.getVelocity(), drive.br.getVelocity()),
                    gamepad1.right_bumper, dt);
            drive.setPowers(result.powers);
            showLed(result.state, clock.seconds());

            telemetry.addData("Loop", "%.1f ms", dt * 1000.0);
            telemetry.addData("Drive", fieldCentric
                    ? (driveFieldCentric ? "field-centric" : "ROBOT-CENTRIC (Pinpoint untrusted)")
                    : "robot-centric");
            telemetry.addData("Traction", "%s  scale %.2f%s", result.state, result.scale,
                    gamepad1.right_bumper ? "  BYPASSED" : "");
            telemetry.addData("Slow Mode", gamepad1.left_bumper ? "ON" : "OFF");
            telemetry.addData("Pinpoint", status);
            telemetry.addData("Slip FL | FR", "%.2f | %.2f", result.flSlip, result.frSlip);
            telemetry.addData("Slip BL | BR", "%.2f | %.2f", result.blSlip, result.brSlip);
            telemetry.addData("Pose (m, deg)", "x %.2f  y %.2f  h %.1f",
                    pinpoint.getPosX(DistanceUnit.METER), pinpoint.getPosY(DistanceUnit.METER),
                    pinpoint.getHeading(AngleUnit.DEGREES));
            if (fieldCentric) {
                telemetry.addData("Field forward (deg)", "%.1f", Math.toDegrees(headingZero));
            }
            telemetry.addData("FL | FR", "%.2f | %.2f", result.powers.getFL(), result.powers.getFR());
            telemetry.addData("BL | BR", "%.2f | %.2f", result.powers.getBL(), result.powers.getBR());
            telemetry.update();
        }

        drive.stop();
    }

    /** Configured from Pedro's localizer constants, so the Pinpoint mount has one source of truth. */
    private GoBildaPinpointDriver initPinpoint() {
        PinpointConfig config = Constants.localizerConfig;
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, config.name.get());
        pinpoint.setOffsets(config.xPodOffset.get(), config.yPodOffset.get(), config.offsetUnits.get());
        if (config.ticksPerUnit.get().isPresent()) {
            pinpoint.setEncoderResolution(config.ticksPerUnit.get().getAsDouble(), config.encoderResolutionUnit.get());
        } else {
            pinpoint.setEncoderResolution(config.podType.get());
        }
        pinpoint.setEncoderDirections(config.xPodDirection.get(), config.yPodDirection.get());
        return pinpoint;
    }

    private static TractionControlLogic.PinpointHealth health(GoBildaPinpointDriver.DeviceStatus status) {
        switch (status) {
            case READY:
                return TractionControlLogic.PinpointHealth.READY;
            case FAULT_BAD_READ:
                return TractionControlLogic.PinpointHealth.BAD_READ;
            default:
                return TractionControlLogic.PinpointHealth.FAULT;
        }
    }

    private static double deadband(double value) {
        return Math.abs(value) < TractionConstants.STICK_DEADBAND ? 0.0 : value;
    }

    /** Sends the LED colour only when it changes; every call is a message to the Driver Hub. */
    private void showLed(TractionControlLogic.GripState state, double seconds) {
        String colour;
        switch (state) {
            case FULL_GRIP:
                colour = "green";
                break;
            case LIMITED:
                colour = "amber";
                break;
            case HEAVY_SLIP:
                colour = "red";
                break;
            default:
                colour = ((int) (seconds / LED_FLASH_SECONDS)) % 2 == 0 ? "purple" : "off";
        }
        if (colour.equals(ledShown)) return;
        ledShown = colour;
        switch (colour) {
            case "green":
                gamepad1.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
                break;
            case "amber":
                gamepad1.setLedColor(1, 0.5, 0, Gamepad.LED_DURATION_CONTINUOUS);
                break;
            case "red":
                gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
                break;
            case "purple":
                gamepad1.setLedColor(0.6, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
                break;
            default:
                gamepad1.setLedColor(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }
}
