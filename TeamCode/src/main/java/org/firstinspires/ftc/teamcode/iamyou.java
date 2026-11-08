package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import android.util.Size;

import java.util.ArrayList;
import java.util.List;

/**
 * The whole I AM YOU TeleOp in one file: hardware setup, field-centric drive math, intake and flywheel control.
 * This class is the ONE-controller version; {@link IamYouTwoController} is the same program with the intake and
 * outtake moved to gamepad 2. {@link IamYouPinpoint} and {@link IamYouPinpointTwoController} are the same two programs
 * taking their heading from the goBILDA Pinpoint instead of the Control Hub's IMU.
 *
 * Wiring (robot configuration names):
 *   Control Hub motors ...... 0 FL, 1 FR, 2 BL, 3 BR
 *   Expansion Hub motors .... 0 IntakeM, 1 Outtake (the flywheel)
 *   Servos .................. IntakeSL, IntakeSR, Staging (continuous rotation; they always spin with IntakeM)
 *   Control Hub I2C bus 0 ... imu (the hub's built-in IMU; the only heading source, there is no odometry)
 *   USB ..................... Webcam 1 (points straight up at the CELLs' tags; optional, aiming is off without it)
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
 *   Options ............. zero heading: wherever the robot faces right now becomes NORTH
 *
 * Mechanism controls (gamepad 1 in this version, gamepad 2 in the two-controller version):
 *   Right trigger ....... intake (hold). Fixed speed, however far the trigger is pulled.
 *   Right bumper ........ outtake / poop out (hold; wins over the trigger). Intake motor and Staging run backwards.
 *   Left bumper ......... flywheel ON/OFF (press to toggle). Runs at OUTTAKE_POWER until pressed again.
 *                         While ON, if this alliance's raised CELL is in view, the robot turns onto it
 *                         (CellAimer) and the right stick's turn is ignored; the left stick still drives.
 *
 * Before START (gamepad 1):
 *   X / B ............... alliance: X = BLUE, B = RED. Aiming stays off until one is picked.
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
    static final String WEBCAM_NAME = "Webcam 1";

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

    /** A heading sensor tilted more than this (pitch or roll) cannot measure the robot's turn correctly. */
    private static final double LEVEL_TOLERANCE_DEG = 10.0;

    /**
     * Flips the right stick. Off: the turn was only reversed while the drive motors' directions were wrong (all four
     * backwards), and they now come from the Mecanum Tuner's measurement in Constants.
     */
    static final boolean TURN_REVERSED = false;

    /** How often to read the intake motor's current (each read is a separate hub command, so don't do it every loop). */
    private static final double INTAKE_CURRENT_READ_SEC = 0.10;

    private final Drive driveLogic = new Drive();
    private final Intake intakeLogic = new Intake();
    private final Outtake outtakeLogic = new Outtake();
    private final HeadingTracker headingTracker = new HeadingTracker();

    /** True = gamepad 2 runs the intake and outtake while gamepad 1 only drives. */
    protected boolean twoControllers() {
        return false;
    }
    boolean toggleAprilTagAdjust = false;

    /**
     * True = heading comes from the goBILDA Pinpoint instead of the Control Hub's IMU. The Pinpoint is set up from
     * Constants.localizerConfig (name, pod type, offsets, directions), the same config Pedro Pathing uses.
     */
    protected boolean usePinpoint() {
        return true;
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
        String pinpointName = Constants.localizerConfig.name.get();
        if (usePinpoint()) {
            requireDevice(GoBildaPinpointDriver.class, pinpointName, missing);
        }
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

        // ================= HEADING SOURCE =================
        // Pinpoint versions: the Pinpoint's own IMU. It must sit still while it resets, so this happens during INIT.
        GoBildaPinpointDriver pinpoint = null;
        if (usePinpoint()) {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pinpointName);
            configurePinpoint(pinpoint);
        }

        // The Control Hub's built-in IMU. The hub stands on its edge with its logo facing the robot's RIGHT side and
        // its USB ports facing FORWARD (toward the intake). Control Hub versions steer by it; Pinpoint versions read it
        // too, as a backup that covers the moments the Pinpoint resets or glitches (see HeadingTracker). If it is
        // missing from the configuration the robot still drives, robot-relative, instead of refusing to move.
        IMU imu = hardwareMap.tryGet(IMU.class, IMU_NAME);
        if (imu != null) {
            imu.initialize(new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
            ));
        }
        String imuStatus = pinpoint != null
                ? "Pinpoint \"" + pinpointName + "\"" + (imu != null ? ", Control Hub IMU as backup" : ", no backup")
                : imu != null ? "Control Hub IMU"
                : "MISSING - driving ROBOT-relative. Add \"" + IMU_NAME + "\" to the Control Hub's I2C bus 0";
        boolean hasHeading = pinpoint != null || imu != null;

        // ================= DRIVE MOTORS =================
        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, FL_NAME);
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, FR_NAME);
        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, BL_NAME);
        DcMotorEx br = hardwareMap.get(DcMotorEx.class, BR_NAME);
        // Directions come from Constants.drivetrainConfig: measured on the real robot with Pedro's Mecanum Tuner, and
        // the same ones autonomous drives with, so TeleOp and AUTO can never disagree about which way is forward.
        fl.setDirection(Constants.drivetrainConfig.frontLeftDirection.get());
        bl.setDirection(Constants.drivetrainConfig.backLeftDirection.get());
        fr.setDirection(Constants.drivetrainConfig.frontRightDirection.get());
        br.setDirection(Constants.drivetrainConfig.backRightDirection.get());
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

        // ================= CAMERA =================
        // Only for aiming at the raised CELL. Optional like the IMU: without it the robot drives and shoots, unaimed.
        // Metres out of the processor, because CellAimer levels the pose through the mount in metres.
        WebcamName webcam = hardwareMap.tryGet(WebcamName.class, WEBCAM_NAME);
        AprilTagProcessor tags = null;
        VisionPortal camera = null;
        if (webcam != null) {
            tags = new AprilTagProcessor.Builder()
                    .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                    .build();
            // No LiveView: the Control Hub has no screen to show it on, and drawing it costs loop time.
            camera = new VisionPortal.Builder()
                    .setCamera(webcam)
                    .addProcessor(tags)
                    .enableLiveView(false)
                    .build();
            // AprilTag detection is heavy work for the Control Hub and slows every loop, so it only runs while the
            // aimer can use it (flywheel on, aim toggle on). See the Aim section of the main loop.
            camera.setProcessorEnabled(tags, false);
        }
        boolean tagsEnabled = false;
        // The aimer turns the robot itself only from its blocking aim(); here it is asked for a turn each loop and
        // the turn is mixed into the drive, so this handle is never driven directly.
        DriveHardware driveHardware = DriveHardware.initWithoutEncoders(hardwareMap);

        // Whoever runs the intake and outtake: the driver, or a second operator.
        Gamepad mech = twoControllers() ? gamepad2 : gamepad1;

        DriverPosition driverPosition = DriverPosition.SOUTH;
        // null until the driver picks one; a ball in the other alliance's CELL scores for them, so there is no default.
        Boolean red = null;

        try {
            // ================= PRE-START =================
            while (!isStarted() && !isStopRequested()) {
                driverPosition = readDriverPosition(driverPosition);
                if (gamepad1.x) {
                    red = false;
                } else if (gamepad1.b) {
                    red = true;
                }

                telemetry.addData("Status", "Initialized - waiting for start");
                telemetry.addData("Controllers", twoControllers()
                        ? "2 (gamepad 1 drives, gamepad 2 runs intake/outtake)" : "1 (gamepad 1 does everything)");
                telemetry.addData("Heading Source", imuStatus);
                if (hasHeading) {
                    // Live, so the sensor can be checked before START: turn the robot LEFT by hand and this must
                    // count UP; put it back and it must return to where it was; left alone it must not move.
                    headingTracker.update(pinpoint, imu, 0.02);
                    telemetry.addData("Heading (deg)", "%.1f  (turn robot LEFT by hand: must go UP)",
                            headingTracker.heading());
                    telemetry.addData("Sensors", headingTracker.sensorReport());
                    telemetry.addData("Level Check", levelCheck(pinpoint, imu));
                }
                if (pinpoint != null) {
                    telemetry.addData("Pinpoint", pinpointStatus(pinpoint));   // the tracker above updated it
                }
                telemetry.addData("Zero Heading",  "Options button: wherever the robot faces becomes NORTH");
                telemetry.addData("Driver Position", driverPosition + "  (D-pad: Up=N Down=S Left=W Right=E)");
                telemetry.addData("Alliance", red == null ? "NOT SET - aiming off  (X = BLUE, B = RED)"
                        : (red ? "RED" : "BLUE") + "  (X = BLUE, B = RED)");
                telemetry.addData("Camera", camera != null ? camera.getCameraState().toString()
                        : "MISSING - aiming off. Add \"" + WEBCAM_NAME + "\" to the configuration");
                telemetry.addData("Note", "Whatever direction the robot faces at START becomes NORTH");
                telemetry.update();
            }

            if (isStopRequested()) return;

            // Built now the alliance is final: CellAimer fixes it at construction.
            CellAimer aimer = tags != null && red != null ? new CellAimer(tags, driveHardware, red) : null;

            // Zero heading at the moment of start: the robot's current front is NORTH.
            headingTracker.zero();

            ElapsedTime loopTimer = new ElapsedTime();
            ElapsedTime currentTimer = new ElapsedTime();
            telemetry.setMsTransmissionInterval(100);
            double lastIntakePower = 0.0;
            boolean lastOptions = false;
            double intakeAmps = 0.0;

            // ================= MAIN LOOP =================
            while (opModeIsActive()) {

                for (LynxModule hub : hubs) {
                    hub.clearBulkCache();
                }

                double dt = loopTimer.seconds();
                loopTimer.reset();

                // Options button: re-zero so the robot's current front becomes NORTH. Once per press.
                if (gamepad1.options && !lastOptions) {
                    headingTracker.zero();
                }
                lastOptions = gamepad1.options;

                // No heading source = heading stuck at 0 and driver position ignored: plain robot-relative driving.
                double heading = headingTracker.update(pinpoint, imu, dt);
                driverPosition = hasHeading ? readDriverPosition(driverPosition) : DriverPosition.SOUTH;

                // ---- Aim ----
                // While the flywheel is on, the aimer owns the turn whenever it can see the CELL. Asked every loop so
                // its "settled" count is of consecutive frames, and reset when the flywheel goes off so a later spin-up
                // starts counting afresh.
                CellAimer.Aim aim = null;
                boolean wantAim = aimer != null && outtakeLogic.isOn() && toggleAprilTagAdjust;
                if (camera != null && wantAim != tagsEnabled) {
                    camera.setProcessorEnabled(tags, wantAim);
                    tagsEnabled = wantAim;
                }
                if (wantAim) {
                    aim = aimer.update();
                } else if (aimer != null) {
                    aimer.reset();
                }
                boolean aiming = aim != null && aim.hasTarget();
                if (gamepad1.xWasPressed()){
                    toggleAprilTagAdjust = !toggleAprilTagAdjust;
                }
                // ---- Drive ----
                DriveParameters params = new DriveParameters(
                        gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        aiming ? 0.0 : TURN_REVERSED ? -gamepad1.right_stick_x : gamepad1.right_stick_x,
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
                double[] wheelPowers = {result.powers.getFL(), result.powers.getFR(),
                        result.powers.getBL(), result.powers.getBR()};
                if (aiming) {
                    // Added after the drive math rather than fed in as a stick: the stick path's deadzone, friction
                    // boost and slow mode would reshape the aimer's tuned turn. CCW = left wheels back, right forward.
                    wheelPowers = addTurn(wheelPowers, aim.turnPower());
                }
                fl.setPower(wheelPowers[0]);
                fr.setPower(wheelPowers[1]);
                bl.setPower(wheelPowers[2]);
                br.setPower(wheelPowers[3]);

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

                //seperate staging from intake (because why not????)
                if (mech.a){
                    staging.setPower(1.0);
                }else if (mech.b){
                    staging.setPower(-1.0);
                }else{
                    staging.setPower(0.0);
                }

                lastIntakePower = intakeResult.power;

                // ---- Flywheel ----
                double outtakePower = outtakeLogic.update(mech.left_bumper, dt);
                outtake.setPower(outtakePower);
                double flywheelTicksPerSec = outtake.getVelocity();

                // ================= TELEMETRY =================
                telemetry.addData("Loop", "%.1f ms  (%.0f Hz)", dt * 1000.0, dt > 0 ? 1.0 / dt : 0.0);
                if (!hasHeading) {
                    telemetry.addData("IMU", imuStatus);
                }
                if (pinpoint != null) {
                    telemetry.addData("Pinpoint", "%s  x %.1f in  y %.1f in", pinpoint.getDeviceStatus(),
                            pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH));
                }
                telemetry.addData("Facing (vs start)",  snapToCardinal(heading));
                telemetry.addData("Driver Position",    driverPosition + "  (D-pad to change)");
                telemetry.addData("Heading (deg)",      "%.1f", heading);
                telemetry.addData("Sensors",            headingTracker.sensorReport());
                telemetry.addData("Slow Mode",          result.slowMode ? "ON" : "OFF");
                telemetry.addData("Stop Assist",        result.braking ? "BRAKING" : "off");
                telemetry.addData("Intake",             "%s  power %.2f  %.1f A",
                        intakeResult.state, intakeResult.power, intakeAmps);
                telemetry.addData("Intake Servos L | R", "%.2f | %.2f", leftServo, rightServo);
                telemetry.addData("Staging",            "%.2f", staging.getPower());
                telemetry.addData("Flywheel",           "%s  power %.2f  %.0f ticks/s %s velocity",
                        outtakeLogic.isOn() ? "ON" : "OFF", outtakePower, flywheelTicksPerSec, outtake.getVelocity());
                telemetry.addData("Aim", aimStatus(aimer, aim, outtakeLogic.isOn()));
                telemetry.addData("---", "---");
                telemetry.addData("Cmd Fwd | Right",    "%.2f | %.2f", result.stickForward, result.stickRight);
                telemetry.addData("Robot Fwd | Right",  "%.2f | %.2f", result.robotForward, result.robotRight);
                telemetry.addData("Twist",              "%.2f", result.twist);
                telemetry.addData("FL | FR",            "%.2f | %.2f", wheelPowers[0], wheelPowers[1]);
                telemetry.addData("BL | BR",            "%.2f | %.2f", wheelPowers[2], wheelPowers[3]);
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
            if (camera != null) {
                camera.close();
            }
        }
    }

    /** Applies Constants.localizerConfig to the Pinpoint, then resets its position and IMU (robot must be still). */
    private void configurePinpoint(GoBildaPinpointDriver pinpoint) {
        pinpoint.setOffsets(Constants.localizerConfig.xPodOffset.get(), Constants.localizerConfig.yPodOffset.get(),
                Constants.localizerConfig.offsetUnits.get());
        if (Constants.localizerConfig.ticksPerUnit.get().isPresent()) {
            pinpoint.setEncoderResolution(Constants.localizerConfig.ticksPerUnit.get().getAsDouble(),
                    Constants.localizerConfig.encoderResolutionUnit.get());
        } else {
            pinpoint.setEncoderResolution(Constants.localizerConfig.podType.get());
        }
        pinpoint.setEncoderDirections(Constants.localizerConfig.xPodDirection.get(),
                Constants.localizerConfig.yPodDirection.get());
        // The gyro calibrates during the reset (about a quarter second). If the robot moves before it finishes, the
        // heading drifts for the whole run. The INIT loop shows the Pinpoint's status until it says READY.
        pinpoint.resetPosAndIMU();
    }

    /** One INIT telemetry line: whether the Pinpoint has finished calibrating its gyro. */
    private static String pinpointStatus(GoBildaPinpointDriver pinpoint) {
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        return status == GoBildaPinpointDriver.DeviceStatus.READY ? "READY"
                : status + " - DON'T TOUCH THE ROBOT until this says READY";
    }

    /**
     * Whether the heading sensor is level. Heading is the rotation about the sensor's own up axis, so a sensor that
     * is tilted (or a Control Hub whose mounting is described wrongly below) measures a different turn from the
     * robot's: right near the start, wrong after the robot has turned, and wrong in a way that does not undo when it
     * turns back.
     */
    private static String levelCheck(GoBildaPinpointDriver pinpoint, IMU imu) {
        double pitch;
        double roll;
        try {
            if (pinpoint != null) {
                pitch = pinpoint.getPitch(AngleUnit.DEGREES);
                roll = pinpoint.getRoll(AngleUnit.DEGREES);
            } else if (imu != null) {
                pitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
                roll = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
            } else {
                return "no heading sensor";
            }
        } catch (RuntimeException olderFirmware) {
            return "unavailable (Pinpoint firmware too old to report tilt)";
        }
        boolean level = Math.abs(pitch) < LEVEL_TOLERANCE_DEG && Math.abs(roll) < LEVEL_TOLERANCE_DEG;
        return String.format("%s  pitch %.0f  roll %.0f", level ? "OK"
                : pinpoint != null ? "NOT LEVEL - mount the Pinpoint flat, heading will be wrong"
                : "NOT LEVEL - hub orientation in code is wrong, heading will be wrong", pitch, roll);
    }

    /**
     * Adds a counter-clockwise turn to FL, FR, BL, BR wheel powers, scaled down together if any wheel would pass full
     * power so the drive direction is kept.
     */
    static double[] addTurn(double[] powers, double turn) {
        double[] out = {powers[0] - turn, powers[1] + turn, powers[2] - turn, powers[3] + turn};
        double max = 1.0;
        for (double p : out) {
            max = Math.max(max, Math.abs(p));
        }
        for (int k = 0; k < 4; k++) {
            out[k] /= max;
        }
        return out;
    }

    /** One telemetry line saying what the aimer is doing, or why it is not. */
    private static String aimStatus(CellAimer aimer, CellAimer.Aim aim, boolean flywheelOn) {
        if (aimer == null) {
            return "off (no camera or no alliance picked before START)";
        }
        if (!flywheelOn) {
            return "waiting for flywheel";
        }
        if (aim == null || !aim.hasTarget()) {
            return "no CELL in view";
        }
        return String.format("%s  %.1f deg off  %.2f m",
                aim.settled() ? "ON TARGET" : "turning", Math.toDegrees(aim.errorRadians()), aim.rangeMetres());
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
    // Field-centric mecanum drive. No FTC SDK calls, so it can be tested on a laptop.
    //
    // Direct: every loop turns the sticks straight into wheel powers. There is no acceleration ramp, no response
    // curve and no speed cap, so the robot answers the stick on the very next loop and stops the moment it is let go.
    //
    // Conventions used everywhere below:
    //  - Vectors are (x = right, y = forward).
    //  - Headings are degrees, counter-clockwise positive (the FTC IMU and Pinpoint convention).
    //  - Heading 0 is whatever direction the robot faced when the heading was last zeroed (START, or the Options
    //    button). That direction is "NORTH".
    //  - Positive twist turns the robot clockwise (right).
    //  - Positive wheel power drives that wheel forward (runOpMode sets the motor directions).
    // =====================================================================================================

    // ---- Driving constants ----
    /**
     * Stick travel ignored around center. Worn sticks rest a little off center; below this they count as released,
     * so the robot cannot creep along in the last direction it was pushed.
     */
    static final double JOYSTICK_DEADZONE        = 0.08;
    static final double SLOW_MODE_DRIVE_MULT     = 0.35;
    static final double SLOW_MODE_TWIST_MULT     = 0.50;
    static final double SLOW_MODE_TRIGGER_THRESH = 0.10;

    /**
     * STRAFE CALIBRATION. Mecanum robots strafe slower than they drive, and a weak strafe makes diagonals bend
     * toward forward/back. This multiplies the sideways command to cancel that.
     * To tune: drive forward at the same stick amount for 1 s, then strafe for 1 s, and measure both distances.
     * STRAFE_COMPENSATION = forward distance / strafe distance. 1.0 = no compensation.
     */
    static final double STRAFE_COMPENSATION      = 1.25;

    /**
     * Static-friction compensation. A loaded wheel does nothing below roughly this power, so a wheel that should
     * creep at 0.04 just sits there and the robot pulls toward the other wheels' direction. Every wheel that is
     * commanded at all gets this much extra push in its own direction (blended smoothly through zero).
     */
    static final double STATIC_FRICTION_POWER = 0.05;
    /** Wheel commands smaller than this get a proportionally smaller friction boost, so the sign flip at zero is smooth. */
    static final double STATIC_FRICTION_BLEND = 0.10;

    static final double MIN_DT = 0.001;
    static final double MAX_DT = 0.100;

    // ---- Stop assist constants (tune these on the robot) ----
    /** Wheel speed in ticks/sec at full power. goBILDA 312/435/1150 RPM motors are all about 2700-2800. */
    static final double MAX_WHEEL_TICKS_PER_SEC = 2700.0;
    /** How hard to brake relative to wheel speed. Higher = stops sooner. */
    static final double BRAKE_GAIN              = 4.0;
    /** Hard ceiling on braking power. Lower this if the stop feels like whiplash. */
    static final double BRAKE_MAX_POWER         = 0.6;
    /** Below this fraction of max speed a wheel counts as stopped, and its brake ends. */
    static final double BRAKE_MIN_SPEED         = 0.05;
    /** Absolute limit on how long the stop assist may push, no matter what the encoders say. The runaway watchdog. */
    static final double BRAKE_MAX_SEC           = 0.25;
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
        final double headingDegrees;  // zeroed at START / Options
        final DriverPosition driverPosition;
        final double flVelocity;      // ticks/sec, positive = wheel driving robot forward
        final double frVelocity;
        final double blVelocity;
        final double brVelocity;
        final double dtSeconds;       // time since the previous loop, for the stop assist's watchdog

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
        final double stickForward;   // stick after the deadzone, before any rotation
        final double stickRight;
        final double robotForward;   // after field-centric rotation, before slow mode
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

    /** Drive state carried between loops: only the stop assist's. The drive itself has no memory. */
    static class Drive {
        // Per-wheel stop-assist state, order: FL, FR, BL, BR.
        /** Direction (+1/-1/0) each wheel was last driven hard. Comes from OUR command, never from the encoder's sign. */
        private final int[] driveSign = new int[4];
        private final boolean[] brakeDone = new boolean[4];
        private boolean wasReleased = false;
        private double brakeSeconds = 0.0;

        DriveResult calculate(DriveParameters params) {
            // Bad numbers must never reach a motor: one NaN or Infinity would otherwise spread through every wheel.
            double dt = Math.max(MIN_DT, Math.min(MAX_DT, finite(params.dtSeconds, 0.02)));
            double headingRad = Math.toRadians(finite(params.headingDegrees, 0.0));

            // ================= STICKS =================
            // Convert to (x = right, y = forward). The deadzone is circular and rescaled, so direction is never
            // distorted and full stick is still full speed.
            double[] stick = radialDeadzone(finite(params.leftStickX, 0.0), -finite(params.leftStickY, 0.0),
                    JOYSTICK_DEADZONE);
            double twist = axisDeadzone(finite(params.rightStickX, 0.0), JOYSTICK_DEADZONE);  // + = clockwise

            // ================= FIELD-CENTRIC ROTATION =================
            // Driver position turns the stick into field directions (CCW by its offset); the heading turns field
            // directions into robot directions (CW by the heading). Both are rotations, so they combine into one.
            double angle = Math.toRadians(params.driverPosition.offsetDegrees) - headingRad;
            double cos = Math.cos(angle);
            double sin = Math.sin(angle);
            double robotRight   = stick[0] * cos - stick[1] * sin;
            double robotForward = stick[0] * sin + stick[1] * cos;

            // ================= SLOW MODE =================
            boolean slow = finite(params.leftTrigger, 0.0) > SLOW_MODE_TRIGGER_THRESH;
            double driveMult = slow ? SLOW_MODE_DRIVE_MULT : 1.0;
            double twistMult = slow ? SLOW_MODE_TWIST_MULT : 1.0;
            double d = robotForward * driveMult;
            double s = robotRight * STRAFE_COMPENSATION * driveMult;
            double t = twist * twistMult;

            boolean released = stick[0] == 0.0 && stick[1] == 0.0 && twist == 0.0;
            double[] out = new double[4];
            boolean braking = false;
            if (!released) {
                // ================= MECANUM MIXER (positive t = clockwise) =================
                double[] mix = {
                        frictionBoost(d + s + t),
                        frictionBoost(d - s - t),
                        frictionBoost(d - s + t),
                        frictionBoost(d + s - t)
                };
                // Scale by the largest wheel so the direction is kept when a wheel would pass full power.
                double max = 1.0;
                for (double w : mix) {
                    max = Math.max(max, Math.abs(w));
                }
                for (int k = 0; k < 4; k++) {
                    out[k] = mix[k] / max;
                    if (Math.abs(out[k]) >= BRAKE_DRIVE_THRESHOLD) {
                        driveSign[k] = out[k] > 0 ? 1 : -1;
                    }
                }
            } else {
                // ================= STOP ASSIST =================
                // Starts the loop the sticks are released: each wheel is pushed briefly against the way it was being
                // driven, to kill the robot's momentum, then the motors' BRAKE mode holds it.
                // Why this cannot run away: the push direction comes from OUR last command, not from the encoder's
                // sign, so a wrongly-signed encoder can only end the assist early, never feed it. It also ends per
                // wheel the moment that wheel stops or reverses, and for everything after BRAKE_MAX_SEC regardless.
                double[] vel = {finite(params.flVelocity, 0.0), finite(params.frVelocity, 0.0),
                        finite(params.blVelocity, 0.0), finite(params.brVelocity, 0.0)};
                if (!wasReleased) {
                    brakeSeconds = 0.0;
                    for (int k = 0; k < 4; k++) {
                        brakeDone[k] = driveSign[k] == 0;
                    }
                }
                brakeSeconds += dt;
                for (int k = 0; k < 4; k++) {
                    if (brakeSeconds > BRAKE_MAX_SEC) {
                        brakeDone[k] = true;
                    }
                    if (brakeDone[k]) {
                        driveSign[k] = 0;
                        continue;
                    }
                    double along = vel[k] / MAX_WHEEL_TICKS_PER_SEC * driveSign[k];   // + = still going the driven way
                    if (along <= BRAKE_MIN_SPEED) {
                        brakeDone[k] = true;
                        driveSign[k] = 0;
                    } else {
                        out[k] = -driveSign[k] * Math.min(BRAKE_MAX_POWER, BRAKE_GAIN * along);
                        braking = true;
                    }
                }
            }
            wasReleased = released;

            for (int k = 0; k < 4; k++) {
                out[k] = Math.max(-1.0, Math.min(1.0, finite(out[k], 0.0)));
            }
            return new DriveResult(new WheelPowers(out[0], out[1], out[2], out[3]), headingRad,
                    stick[1], stick[0], robotForward, robotRight, twist, slow, braking);
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
    // HEADING
    //
    // The heading iamyou steers by is built from how far the robot TURNED each loop, not from a sensor's raw value.
    // That is what lets it survive the sensor going wrong for a moment:
    //  - The Pinpoint is powered through its I2C cable. Hard driving can dip the voltage enough to restart it; it then
    //    reports heading 0 and recalibrates its gyro while the robot is moving, so a raw reading would be wrong from
    //    then on. Read as turns, the restart is one impossible jump, which is ignored.
    //  - A loop where the Pinpoint is restarting or calibrating, or turns faster than any robot can, uses the Control
    //    Hub IMU's turn for that loop instead - but only once the IMU has been seen to agree with the Pinpoint. A
    //    backup whose turns run the wrong way (a hub mounting described wrongly) would otherwise turn every glitch
    //    into a heading error, which is exactly how the controls came to invert. Untrusted, a glitch holds the heading.
    //  - The Pinpoint also reports "not READY" for things that leave its heading valid: a single bad read (the driver
    //    repeats the previous value) or a pod not detected. Those are not glitches.
    // Both sensors' turns since the last zero are also kept separately and shown, so a sensor that slowly drifts
    // (tilted mount, wrong scale) shows up as the two disagreeing.
    // =====================================================================================================

    /** Faster than this, a change of reading is a glitch or a sensor restart, not the robot turning. */
    static final double MAX_TURN_RATE_DEG_PER_SEC = 1000.0;
    /** Slack on top of the rate limit, for a slow loop or a reading taken late. */
    static final double TURN_JUMP_SLACK_DEG = 5.0;
    /** How much the robot must have turned, by both sensors, before the backup's agreement is judged. */
    static final double BACKUP_TRUST_AFTER_DEG = 45.0;
    /** The backup is trusted while its turns are this close to the main sensor's (1.0 = identical). */
    static final double BACKUP_MIN_AGREEMENT = 0.85;
    static final double BACKUP_MAX_AGREEMENT = 1.15;
    /** Turns smaller than this in one loop are too small to judge agreement by (sensor noise). */
    static final double BACKUP_JUDGE_MIN_TURN_DEG = 0.2;

    /** Turns sensor readings into the heading iamyou steers by. Plain math, so it is unit-testable. */
    static class HeadingTracker {
        private double heading = 0.0;
        private double primarySinceZero = 0.0;
        private double backupSinceZero = 0.0;
        private double lastPrimary = Double.NaN;
        private double lastBackup = Double.NaN;
        private int rejected = 0;
        private boolean usedBackup = false;
        private boolean hasPrimary = false;
        private boolean hasBackup = false;
        // Agreement between the sensors, from loops where both were good and the robot was turning.
        private double sumPrimarySquared = 0.0;
        private double sumPrimaryTimesBackup = 0.0;
        private double judgedTurn = 0.0;
        private String lastFault = null;

        /** Reads the sensors on the robot and updates the heading. The Pinpoint is primary when there is one. */
        double update(GoBildaPinpointDriver pinpoint, IMU imu, double dtSeconds) {
            double imuYaw = imu != null ? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) : Double.NaN;
            if (pinpoint != null) {
                pinpoint.update();
                GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
                if (status != GoBildaPinpointDriver.DeviceStatus.READY) {
                    lastFault = status.toString();
                }
                return update(pinpoint.getHeading(AngleUnit.DEGREES), headingValid(status), imuYaw, true, dtSeconds);
            }
            return update(imuYaw, true, Double.NaN, true, dtSeconds);
        }

        /**
         * @param primaryDeg     the main sensor's heading, degrees CCW; NaN if there is none
         * @param primaryHealthy false while the main sensor says its reading cannot be trusted (calibrating)
         * @param backupDeg      the backup sensor's heading, degrees CCW; NaN if there is none
         * @param backupHealthy  false while the backup cannot be trusted
         * @return the heading to steer by, degrees CCW, 0 = where the robot faced at the last zero
         */
        double update(double primaryDeg, boolean primaryHealthy, double backupDeg, boolean backupHealthy,
                      double dtSeconds) {
            double dt = Math.max(0.0, finite(dtSeconds, 0.02));
            double limit = MAX_TURN_RATE_DEG_PER_SEC * dt + TURN_JUMP_SLACK_DEG;
            double primaryTurn = turn(lastPrimary, primaryDeg);
            double backupTurn = turn(lastBackup, backupDeg);
            hasPrimary = !Double.isNaN(primaryDeg) && !Double.isInfinite(primaryDeg);
            hasBackup = !Double.isNaN(backupDeg) && !Double.isInfinite(backupDeg);
            boolean primaryOk = primaryHealthy && !Double.isNaN(primaryTurn) && Math.abs(primaryTurn) <= limit;
            boolean backupOk = backupHealthy && !Double.isNaN(backupTurn) && Math.abs(backupTurn) <= limit;

            if (primaryOk && backupOk && Math.abs(primaryTurn) >= BACKUP_JUDGE_MIN_TURN_DEG) {
                sumPrimarySquared += primaryTurn * primaryTurn;
                sumPrimaryTimesBackup += primaryTurn * backupTurn;
                judgedTurn += Math.abs(primaryTurn);
            }

            usedBackup = false;
            if (primaryOk) {
                heading += primaryTurn;
            } else if (backupOk && (!hasPrimary || backupTrusted())) {
                // No main sensor at all (Control Hub versions pass the IMU as primary, so this is the fallback for
                // an unplugged Pinpoint), or a backup proven to agree.
                heading += backupTurn;
                usedBackup = true;
            }
            if (!primaryOk && !Double.isNaN(primaryTurn)) {
                rejected++;
            }
            if (primaryOk) {
                primarySinceZero += primaryTurn;
            }
            if (backupOk) {
                backupSinceZero += backupTurn;
            }
            // Always move the baseline on, so after a restart the next turn is measured from the new reading.
            if (hasPrimary) {
                lastPrimary = primaryDeg;
            }
            if (hasBackup) {
                lastBackup = backupDeg;
            }
            return heading;
        }

        /** Whether the backup's turns have been seen to match the main sensor's, both in direction and in size. */
        boolean backupTrusted() {
            double agreement = backupAgreement();
            return judgedTurn >= BACKUP_TRUST_AFTER_DEG
                    && agreement >= BACKUP_MIN_AGREEMENT && agreement <= BACKUP_MAX_AGREEMENT;
        }

        /** Backup turn per degree of main-sensor turn: 1 = they agree, -1 = the backup runs backwards. */
        double backupAgreement() {
            return sumPrimarySquared > 0.0 ? sumPrimaryTimesBackup / sumPrimarySquared : Double.NaN;
        }

        /**
         * Whether a Pinpoint status leaves its heading usable. Restarting, calibrating or a runaway gyro do not. A bad
         * read (the driver repeats the previous value) and a missing pod (heading comes from the gyro, not the pods)
         * do.
         */
        static boolean headingValid(GoBildaPinpointDriver.DeviceStatus status) {
            return status != GoBildaPinpointDriver.DeviceStatus.NOT_READY
                    && status != GoBildaPinpointDriver.DeviceStatus.CALIBRATING
                    && status != GoBildaPinpointDriver.DeviceStatus.FAULT_IMU_RUNAWAY;
        }

        /** Makes the robot's current front heading 0 (NORTH). The sensors themselves are not touched. */
        void zero() {
            heading = 0.0;
            primarySinceZero = 0.0;
            backupSinceZero = 0.0;
        }

        double heading() {
            return heading;
        }

        /** How many readings were thrown away as glitches or restarts since the OpMode began. */
        int rejectedReadings() {
            return rejected;
        }

        /** One telemetry line: each sensor's turn since the last zero, and how many readings were thrown away. */
        String sensorReport() {
            if (!hasPrimary && !hasBackup) {
                return "none";
            }
            StringBuilder line = new StringBuilder();
            if (hasPrimary) {
                line.append(String.format("main %.1f", primarySinceZero));
            }
            if (hasBackup) {
                line.append(String.format("  backup %.1f", backupSinceZero));
                if (hasPrimary && Math.abs(AngleUnit.normalizeDegrees(primarySinceZero - backupSinceZero)) > 10.0) {
                    line.append("  DISAGREE");
                }
                if (hasPrimary) {
                    double agreement = backupAgreement();
                    line.append(judgedTurn < BACKUP_TRUST_AFTER_DEG ? "  backup unchecked"
                            : backupTrusted() ? "  backup OK"
                            : String.format("  BACKUP WRONG (x%.2f, not used)", agreement));
                }
            }
            line.append(String.format("  glitches %d", rejected));
            if (lastFault != null) {
                line.append("  last status ").append(lastFault);
            }
            if (usedBackup) {
                line.append("  USING BACKUP");
            }
            return line.toString();
        }

        /** The turn from one reading to the next, the short way round; NaN if either is missing. */
        private static double turn(double before, double after) {
            if (Double.isNaN(before) || Double.isNaN(after) || Double.isInfinite(before) || Double.isInfinite(after)) {
                return Double.NaN;
            }
            return AngleUnit.normalizeDegrees(after - before);
        }
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
    static final double OUTTAKE_POWER         = 0.505;
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
