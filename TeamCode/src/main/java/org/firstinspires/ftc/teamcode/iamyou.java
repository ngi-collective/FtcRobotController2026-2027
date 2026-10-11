package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

/**
 * Gamepad 1 controls:
 *   Left stick .......... drive (field-centric)
 *   Right stick X ....... turn
 *   Left trigger ........ slow mode
 *   Right trigger ....... intake (hold; harder pull = faster)
 *   Right bumper ........ eject (hold; wins over the trigger)
 *   D-pad ............... where the driver is standing (Up=N Down=S Left=W Right=E)
 *   Options ............. re-zero heading (robot's current front becomes NORTH)
 */
@TeleOp(name = "I AM YOU V2")
public class iamyou extends LinearOpMode {

    private final IamYouLogic logic = new IamYouLogic();
    private final IntakeLogic intakeLogic = new IntakeLogic();

    /** How often to read the intake motor's current (each read is a separate hub command, so don't do it every loop). */
    private static final double INTAKE_CURRENT_READ_SEC = 0.10;

    @Override
    public void runOpMode() {

        // ================= FAST HUB READS =================
        // Without this, every getVelocity() is its own round trip to the hub and the loop crawls, which feels choppy.
        // MANUAL caching = one bulk read per hub per loop, cleared at the top of the loop below.
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // ================= IMU SETUP =================
        // Control Hub lies flat with its logo facing UP and its USB ports facing the
        // robot's RIGHT side (photo orientation: top of image = robot front).
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        ));

        // ================= HARDWARE =================
        DriveHardware drive = DriveHardware.initWithEncoders(hardwareMap);
        // Plain power mode for driving: power is a direct motor-voltage fraction, with none of the hub's velocity-PID
        // lag or locking. Encoders are still read for the stop assist. (BRAKE zero-power behavior is already set.)
        for (DcMotorEx m : new DcMotorEx[]{drive.fl, drive.fr, drive.bl, drive.br}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        IntakeHardware intake = IntakeHardware.init(hardwareMap);

        IamYouLogic.DriverPosition driverPosition = IamYouLogic.DriverPosition.SOUTH;

        // ================= PRE-START =================
        while (!isStarted() && !isStopRequested()) {
            driverPosition = readDriverPosition(driverPosition);

            telemetry.addData("Status", "Initialized - waiting for start");
            telemetry.addData("Driver Position", driverPosition + "  (D-pad: Up=N Down=S Left=W Right=E)");
            telemetry.addData("Note", "Whatever direction the robot faces at START becomes NORTH");
            telemetry.update();
        }

        if (isStopRequested()) return;

        // Zero heading at the moment of start: the robot's current front is NORTH.
        imu.resetYaw();

        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime currentTimer = new ElapsedTime();
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
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            driverPosition = readDriverPosition(driverPosition);

            // ---- Drive ----
            IamYouLogic.DriveParameters params = new IamYouLogic.DriveParameters(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_trigger,
                    heading,
                    driverPosition,
                    drive.fl.getVelocity(),
                    drive.fr.getVelocity(),
                    drive.bl.getVelocity(),
                    drive.br.getVelocity(),
                    dt
            );

            IamYouLogic.DriveResult result = logic.calculate(params);
            drive.setPowers(result.powers);

            // ---- Intake ----
            if (Math.abs(lastIntakePower) > 0.05) {
                if (currentTimer.seconds() >= INTAKE_CURRENT_READ_SEC) {
                    intakeAmps = intake.getCurrentAmps();
                    currentTimer.reset();
                }
            } else {
                intakeAmps = 0.0;
                currentTimer.reset();
            }
            IntakeLogic.Result intakeResult = intakeLogic.update(
                    gamepad1.right_trigger, gamepad1.right_bumper, intakeAmps, dt);
            intake.setPower(intakeResult.power);
            lastIntakePower = intakeResult.power;

            // ================= TELEMETRY =================
            telemetry.addData("Loop", "%.1f ms  (%.0f Hz)", dt * 1000.0, dt > 0 ? 1.0 / dt : 0.0);
            telemetry.addData("Facing (vs start)",  IamYouLogic.snapToCardinal(heading));
            telemetry.addData("Driver Position",    driverPosition + "  (D-pad to change)");
            telemetry.addData("Heading (deg)",      "%.1f", heading);
            telemetry.addData("Slow Mode",          result.slowMode ? "ON" : "OFF");
            telemetry.addData("Stop Assist",        result.braking ? "BRAKING" : "off");
            telemetry.addData("Intake",             "%s  power %.2f  %.1f A",
                    intakeResult.state, intakeResult.power, intakeAmps);
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

        drive.stop();
        intake.stop();
    }

    private IamYouLogic.DriverPosition readDriverPosition(IamYouLogic.DriverPosition current) {
        if (gamepad1.dpad_up)    return IamYouLogic.DriverPosition.NORTH;
        if (gamepad1.dpad_down)  return IamYouLogic.DriverPosition.SOUTH;
        if (gamepad1.dpad_left)  return IamYouLogic.DriverPosition.WEST;
        if (gamepad1.dpad_right) return IamYouLogic.DriverPosition.EAST;
        return current;
    }
}