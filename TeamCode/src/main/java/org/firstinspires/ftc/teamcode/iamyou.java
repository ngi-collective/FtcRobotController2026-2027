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
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
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
 *   USB ..................... Webcam 1 (looks forward, tilted 45 degrees up at the CELLs' tags; optional, aiming is off without it)
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
    public static final String INTAKE_NAME = "IntakeM";
    public static final String INTAKE_SERVO_LEFT_NAME = "IntakeSL";
    public static final String INTAKE_SERVO_RIGHT_NAME = "IntakeSR";
    public static final String STAGING_NAME = "Staging";
    public static final String OUTTAKE_NAME = "Outtake";
    /** The Control Hub's built-in IMU. There is no odometry, so this is the only heading source. */
    static final String IMU_NAME = "imu";
    public static final String WEBCAM_NAME = "Webcam 1";

    /**
     * Flip this if positive power (right trigger) spits pieces out instead of pulling them in.
     * With a big-gear-drives-small-gear mesh the roller turns opposite to the motor, so
     * the first run may need this set to true.
     */
    public static final boolean INTAKE_REVERSED = true;

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
    public static final boolean STAGING_REVERSED = true;

    /** Flip this if the flywheel spins backwards. */
    public static final boolean OUTTAKE_REVERSED = true;

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
    private final DriveCheck driveCheck = new DriveCheck();

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
            ElapsedTime initTimer = new ElapsedTime();
            while (!isStarted() && !isStopRequested()) {
                for (LynxModule hub : hubs) {
                    hub.clearBulkCache();
                }
                double initDt = initTimer.seconds();
                initTimer.reset();
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
                    // Sitting in INIT is the best time to measure gyro drift: the robot is still for seconds on end.
                    headingTracker.update(pinpoint, imu, initDt, wheelsStill(wheels));
                    telemetry.addData("Heading (deg)", "%.1f", headingTracker.heading());
                    telemetry.addData("Sensors", headingTracker.sensorReport());
                    telemetry.addData("Hand-turn check", "turn the robot LEFT: main AND backup must go UP");
                    telemetry.addData("Level Check", levelCheck(pinpoint, imu));
                }
                if (pinpoint != null) {
                    telemetry.addData("Pinpoint", pinpointStatus(pinpoint));   // the tracker above updated it
                    // goBILDA calibrates this on every device before shipping and asks that it not be changed; if it
                    // looks wrong (far from about 1.0), the guide says to contact tech@gobilda.com.
                    telemetry.addData("Pinpoint Yaw Scalar", "%.4f (factory value)", pinpoint.getYawScalar());
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
            double lastCheckedHeading = 0.0;
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
                // Still = nobody asking it to move and no wheel turning (a push turns the wheels). Any heading change
                // then is gyro drift, which the tracker ignores and measures.
                boolean still = wheelsStill(wheels) && !sticksTouched(gamepad1) && !aimerMayTurn(aimer, outtakeLogic);
                double heading = headingTracker.update(pinpoint, imu, dt, still);
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

                // ---- Drive check ----
                // Compares what the sticks asked the robot to do with what the Pinpoint's pods and the heading say it
                // actually did. Field-centric only works when all three agree, however good the heading is.
                double headingRate = dt > 0 ? AngleUnit.normalizeDegrees(heading - lastCheckedHeading) / dt : 0.0;
                lastCheckedHeading = heading;
                if (!aiming && !(gamepad1.options)) {
                    double measuredForward = Double.NaN;
                    double measuredLeft = Double.NaN;
                    if (pinpoint != null) {
                        // The Pinpoint reports velocity in its own field frame; turned back by its own heading, that
                        // is simply what its two pods are doing: forward and left, in inches per second.
                        double h = pinpoint.getHeading(AngleUnit.RADIANS);
                        double vx = pinpoint.getVelX(DistanceUnit.INCH);
                        double vy = pinpoint.getVelY(DistanceUnit.INCH);
                        measuredForward = Math.cos(h) * vx + Math.sin(h) * vy;
                        measuredLeft = -Math.sin(h) * vx + Math.cos(h) * vy;
                        // Its gyro's own turn rate, with the same sign setting as the heading iamyou steers by:
                        // steadier than differencing two headings a loop apart.
                        headingRate = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)
                                * (PINPOINT_HEADING_REVERSED ? -1.0 : 1.0);
                    }
                    driveCheck.update(result.robotForward, result.robotRight, result.twist,
                            measuredForward, measuredLeft, headingRate, dt);
                }
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
                telemetry.addData("Drive Check",        driveCheck.report());
                telemetry.addData("Still Check",        stillReport(wheels, gamepad1, still,
                        aimerMayTurn(aimer, outtakeLogic)));
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

    /** True when every drive wheel's encoder says it is stopped. */
    private static boolean wheelsStill(DcMotorEx[] wheels) {
        for (DcMotorEx wheel : wheels) {
            if (Math.abs(wheel.getVelocity()) > STILL_WHEEL_TICKS_PER_SEC) {
                return false;
            }
        }
        return true;
    }

    /** One telemetry line: whether the robot counts as standing still, and if not, what is stopping it. */
    private static String stillReport(DcMotorEx[] wheels, Gamepad pad, boolean still, boolean aimerBusy) {
        double fastest = 0.0;
        for (DcMotorEx wheel : wheels) {
            fastest = Math.max(fastest, Math.abs(wheel.getVelocity()));
        }
        return String.format("%s  wheels %.0f t/s (limit %.0f)  sticks %.2f %.2f %.2f%s", still ? "yes" : "NO",
                fastest, STILL_WHEEL_TICKS_PER_SEC, pad.left_stick_x, pad.left_stick_y, pad.right_stick_x,
                aimerBusy ? "  aimer on" : "");
    }

    /** True when the driver is pushing either drive stick past its deadzone. */
    private static boolean sticksTouched(Gamepad pad) {
        return Math.hypot(pad.left_stick_x, pad.left_stick_y) >= JOYSTICK_DEADZONE
                || Math.abs(pad.right_stick_x) >= JOYSTICK_DEADZONE;
    }

    /** True while the aimer may be turning the robot on its own. */
    private boolean aimerMayTurn(CellAimer aimer, Outtake outtake) {
        return aimer != null && outtake.isOn() && toggleAprilTagAdjust;
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
    // DRIVE CHECK
    //
    // Field-centric steering needs three things to agree: which way the robot moves for a "forward" command, which way
    // for a "right" command, and which way the heading counts when it turns. If any one is backwards, the robot drives
    // correctly facing its start direction and wrongly after turning, even with a perfect heading. This watches the
    // driver's own driving and reports which of the three is backwards. It only reports: nothing is flipped by itself.
    // =====================================================================================================

    /** A stick command below this is too small to judge; above it, the other two commands must be below IDLE. */
    static final double CHECK_COMMAND = 0.30;
    static final double CHECK_IDLE = 0.15;
    /** The robot must actually be moving this fast (inches/sec, degrees/sec) for its direction to count. */
    static final double CHECK_MIN_SPEED_IN_PER_SEC = 4.0;
    static final double CHECK_MIN_TURN_DEG_PER_SEC = 20.0;
    /** Seconds of one-sided evidence needed for a verdict. */
    static final double CHECK_VERDICT_SEC = 0.5;
    static final double CHECK_MAX_SEC = 3.0;

    /** Judges, from ordinary driving, whether forward, strafe and turn each go the way the code assumes. */
    static class DriveCheck {
        // Seconds of evidence: positive = goes the way commanded, negative = goes the opposite way.
        private double forward = 0.0;
        private double strafe = 0.0;
        private double turn = 0.0;

        /**
         * @param cmdForward       commanded robot-forward, -1..1
         * @param cmdRight         commanded robot-right, -1..1
         * @param cmdTwist         commanded turn, clockwise positive, -1..1
         * @param measuredForward  the pods' forward speed, inches/sec; NaN if there are no pods
         * @param measuredLeft     the pods' leftward speed, inches/sec; NaN if there are no pods
         * @param headingRate      how fast the heading iamyou steers by is changing, degrees/sec, CCW positive
         */
        void update(double cmdForward, double cmdRight, double cmdTwist,
                    double measuredForward, double measuredLeft, double headingRate, double dtSeconds) {
            double dt = Math.max(0.0, Math.min(0.1, finite(dtSeconds, 0.0)));
            boolean onlyForward = Math.abs(cmdForward) >= CHECK_COMMAND
                    && Math.abs(cmdRight) < CHECK_IDLE && Math.abs(cmdTwist) < CHECK_IDLE;
            boolean onlyRight = Math.abs(cmdRight) >= CHECK_COMMAND
                    && Math.abs(cmdForward) < CHECK_IDLE && Math.abs(cmdTwist) < CHECK_IDLE;
            boolean onlyTwist = Math.abs(cmdTwist) >= CHECK_COMMAND
                    && Math.abs(cmdForward) < CHECK_IDLE && Math.abs(cmdRight) < CHECK_IDLE;

            if (onlyForward && Math.abs(finite(measuredForward, 0.0)) >= CHECK_MIN_SPEED_IN_PER_SEC) {
                forward = tally(forward, cmdForward * measuredForward, dt);
            }
            if (onlyRight && Math.abs(finite(measuredLeft, 0.0)) >= CHECK_MIN_SPEED_IN_PER_SEC) {
                strafe = tally(strafe, cmdRight * -measuredLeft, dt);
            }
            // A clockwise command must make the (counter-clockwise positive) heading fall.
            if (onlyTwist && Math.abs(finite(headingRate, 0.0)) >= CHECK_MIN_TURN_DEG_PER_SEC) {
                turn = tally(turn, cmdTwist * -headingRate, dt);
            }
        }

        private static double tally(double evidence, double agreement, double dt) {
            return Math.max(-CHECK_MAX_SEC, Math.min(CHECK_MAX_SEC, evidence + Math.signum(agreement) * dt));
        }

        /** +1 = agrees, -1 = backwards, 0 = not enough driving yet. */
        private static int verdict(double evidence) {
            return evidence >= CHECK_VERDICT_SEC ? 1 : evidence <= -CHECK_VERDICT_SEC ? -1 : 0;
        }

        private static String word(int verdict) {
            return verdict > 0 ? "OK" : verdict < 0 ? "BACKWARDS" : "?";
        }

        /** One telemetry line: the three verdicts, then what they mean. */
        String report() {
            int f = verdict(forward);
            int s = verdict(strafe);
            int t = verdict(turn);
            String line = "fwd " + word(f) + "  strafe " + word(s) + "  turn " + word(t) + "  -> ";
            if (f == 0 || s == 0 || t == 0) {
                return line + "drive forward, then strafe, then spin (one at a time, 1 s each)";
            }
            if (f > 0 && s > 0 && t > 0) {
                return line + "all agree";
            }
            if (f > 0 && s > 0) {
                return line + "HEADING COUNTS BACKWARDS: flip PINPOINT_HEADING_REVERSED";
            }
            if (f < 0 && s < 0 && t < 0) {
                return line + "ALL FOUR MOTORS BACKWARDS: flip every direction in Constants.drivetrainConfig";
            }
            if (f > 0 && s < 0 && t < 0) {
                return line + "LEFT/RIGHT MOTORS SWAPPED: swap FL with FR and BL with BR in the configuration";
            }
            if (f > 0 && s < 0) {
                return line + "FRONT/BACK MOTORS SWAPPED: swap FL with BL and FR with BR in the configuration";
            }
            return line + "unexpected combination: send a photo of this line";
        }
    }

    // =====================================================================================================
    // HEADING
    //
    // The heading iamyou steers by is built from how far the robot TURNED each loop, not from a sensor's raw value.
    // That is what lets it survive the sensor going wrong for a moment:
    //  - The Pinpoint is powered through its I2C cable. Hard driving can dip the voltage enough to restart it; it then
    //    reports heading 0 and recalibrates its gyro while the robot is moving, so a raw reading would be wrong from
    //    then on. Read as turns, the restart is one impossible jump, which is ignored.
    //  - A loop where the Pinpoint is frozen (restarting, calibrating, or a bad read the driver covers by repeating
    //    the previous value) or turns faster than any robot can uses the Control Hub IMU's turn instead - once the
    //    IMU has been seen to turn the same way and by the same amount as the Pinpoint. A backup that turns the other
    //    way is never used: which of the two is backwards is settled by hand (PINPOINT_HEADING_REVERSED and
    //    HUB_IMU_HEADING_REVERSED), not guessed. Untrusted, a frozen stretch holds the heading.
    //  - A failed Control Hub IMU read (heading 0, acquisition time 0) is ignored, not taken as a turn to 0. A
    //    Pinpoint on I2C port 0 shares a bus with that IMU and can make its reads fail exactly when it glitches.
    //  - When the Pinpoint comes back, its first reading is a new starting point rather than a turn, so the motion
    //    the IMU already covered is not counted twice.
    //  - A pod not detected leaves the heading valid: it comes from the Pinpoint's gyro, not its pods.
    // Both sensors' turns since the last zero are also kept separately and shown, so a sensor that slowly drifts
    // (tilted mount, wrong scale) shows up as the two disagreeing. Those totals keep counting past 360, which is
    // what the spin test for the TURN_SCALE constants reads; the heading itself stays within -180..180.
    //  - A robot standing still (no stick, no wheel turning) cannot turn, so heading changes then are gyro drift: the
    //    heading holds, and the drift rate is measured and subtracted while the robot moves.
    // =====================================================================================================

    /**
     * Which way each heading sensor counts. Turning the robot LEFT (counter-clockwise from above) must make both the
     * "main" and the "backup" numbers on the INIT telemetry's Sensors line go UP; flip whichever one goes down.
     * A Pinpoint mounted upside down counts backwards; so does a Control Hub whose mounting is described wrongly.
     */
    static final boolean PINPOINT_HEADING_REVERSED = false;
    static final boolean HUB_IMU_HEADING_REVERSED = false;

    /**
     * TURN SCALE. A gyro that measures a full turn as 352 degrees is 8 degrees out after one spin and 80 after ten, so
     * a robot that spins a lot slowly loses track of which way is which. To measure it:
     *  1. After START, line the robot up square against a wall or a tile seam and press Options.
     *  2. Spin it ten full turns with the right stick (either way), slowly at the end, and line it up exactly again.
     *  3. Read the Sensors line: "main" should say 3600 (or -3600), "backup" the same.
     *  4. Set the scale to 3600 / |reading|, e.g. main 3528 -> PINPOINT_TURN_SCALE = 3600 / 3528.0 = 1.0204.
     */
    // Measured 2026-11-08: ten full turns read 3453 on the Pinpoint (it under-reads every turn by 4.1 %).
    static final double PINPOINT_TURN_SCALE = 3600.0 / 3453.0;
    static final double HUB_IMU_TURN_SCALE = 1.0;

    /** A wheel slower than this (encoder ticks/sec) counts as stopped when deciding the robot is standing still. */
    static final double STILL_WHEEL_TICKS_PER_SEC = 15.0;
    /**
     * How long the wheels must have been stopped before the robot counts as standing still. Right after stopping, the
     * chassis can still skid and turn on locked wheels, and that turn is real.
     */
    static final double STILL_SETTLE_SEC = 0.5;
    /**
     * The most a gyro can plausibly drift. Anything faster while the wheels are stopped is the robot really turning:
     * pushed, sliding round on its mecanum rollers, or turned by hand. A healthy Pinpoint measures about 0.01.
     */
    static final double MAX_DRIFT_DEG_PER_SEC = 0.15;
    /** Heading change allowed on top of that before a held robot is judged to be really turning (sensor noise). */
    static final double HOLD_SLACK_DEG = 1.0;
    /** Drift is measured over stretches of stillness this long, never from a single loop. */
    static final double DRIFT_SAMPLE_SEC = 3.0;

    /** Faster than this, a change of reading is a glitch or a sensor restart, not the robot turning. */
    static final double MAX_TURN_RATE_DEG_PER_SEC = 1000.0;
    /** Slack on top of the rate limit, for a slow loop or a reading taken late. */
    static final double TURN_JUMP_SLACK_DEG = 5.0;
    /** How much the robot must have turned, by both sensors, before the backup's agreement is judged. */
    static final double BACKUP_TRUST_AFTER_DEG = 45.0;
    /** The backup is trusted while its turns are this close to the main sensor's (1.0 = identical, same way round). */
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
        /** True after a loop the main sensor was frozen: its next reading starts a new baseline, it is not a turn. */
        private boolean primaryGap = false;
        /** The same for the backup, after a failed read. */
        private boolean backupGap = false;
        private int rejected = 0;
        private boolean usedBackup = false;
        private boolean hasPrimary = false;
        private boolean hasBackup = false;
        // Agreement between the sensors, from loops where both were good and the robot was turning.
        private double sumPrimarySquared = 0.0;
        private double sumPrimaryTimesBackup = 0.0;
        private double judgedTurn = 0.0;
        private String lastFault = null;
        // Gyro drift: measured while the robot stands still, subtracted while it moves.
        private double stillSeconds = 0.0;
        private double driftDegPerSec = 0.0;
        // While holding: everything the sensor has reported since the hold began (given back if it turns out to be a
        // real turn), and the current stretch being measured for drift.
        private double heldTurn = 0.0;
        private double heldSeconds = 0.0;
        private double sampleTurn = 0.0;
        private double sampleSeconds = 0.0;
        private boolean still = false;
        /** Multiplies each sensor's turns (the TURN_SCALE constants for whichever sensor is in that role). */
        private double primaryScale = 1.0;
        private double backupScale = 1.0;

        /** Reads the sensors on the robot and updates the heading. The Pinpoint is primary when there is one. */
        double update(GoBildaPinpointDriver pinpoint, IMU imu, double dtSeconds, boolean robotStill) {
            this.still = robotStill;
            return update(pinpoint, imu, dtSeconds);
        }

        double update(GoBildaPinpointDriver pinpoint, IMU imu, double dtSeconds) {
            double imuYaw = Double.NaN;
            boolean imuRead = false;
            if (imu != null) {
                YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
                // A failed read comes back as heading 0 with acquisition time 0 (the SDK returns the identity
                // orientation). Taken as a reading, it is a sudden turn to 0, so it is treated as no reading at all.
                imuRead = angles.getAcquisitionTime() != 0;
                imuYaw = signed(angles.getYaw(AngleUnit.DEGREES), HUB_IMU_HEADING_REVERSED);
            }
            if (pinpoint != null) {
                pinpoint.update();
                GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
                if (status != GoBildaPinpointDriver.DeviceStatus.READY) {
                    lastFault = status.toString();
                }
                primaryScale = PINPOINT_TURN_SCALE;
                backupScale = HUB_IMU_TURN_SCALE;
                return update(signed(pinpoint.getHeading(AngleUnit.DEGREES), PINPOINT_HEADING_REVERSED),
                        freshHeading(status), imuYaw, imuRead, dtSeconds);
            }
            primaryScale = HUB_IMU_TURN_SCALE;
            return update(imuYaw, imuRead, Double.NaN, false, dtSeconds);
        }

        private static double signed(double deg, boolean reversed) {
            return reversed ? -deg : deg;
        }

        /**
         * @param primaryDeg    the main sensor's heading, degrees CCW; NaN if there is none
         * @param primaryFresh  false while the main sensor's reading is frozen or meaningless (restarting,
         *                      calibrating, or a bad read the driver papered over with the previous value)
         * @param backupDeg     the backup sensor's heading, degrees CCW; NaN if there is none
         * @param backupHealthy false while the backup cannot be trusted
         * @return the heading to steer by, degrees CCW, 0 = where the robot faced at the last zero
         */
        /** As below, saying whether the robot is standing still (its heading changes are then drift). */
        double update(double primaryDeg, boolean primaryFresh, double backupDeg, boolean backupHealthy,
                      double dtSeconds, boolean robotStill) {
            this.still = robotStill;
            return update(primaryDeg, primaryFresh, backupDeg, backupHealthy, dtSeconds);
        }

        double update(double primaryDeg, boolean primaryFresh, double backupDeg, boolean backupHealthy,
                      double dtSeconds) {
            double dt = Math.max(0.0, finite(dtSeconds, 0.02));
            double limit = MAX_TURN_RATE_DEG_PER_SEC * dt + TURN_JUMP_SLACK_DEG;
            boolean firstPrimary = Double.isNaN(lastPrimary);
            hasPrimary = isReading(primaryDeg);
            hasBackup = isReading(backupDeg);

            double backupTurn = backupHealthy && !backupGap ? turn(lastBackup, backupDeg) * backupScale : Double.NaN;
            boolean backupOk = !Double.isNaN(backupTurn) && Math.abs(backupTurn) <= limit;
            boolean fresh = hasPrimary && primaryFresh;
            // After a frozen stretch the main sensor's first reading is a new baseline, not a turn: the backup (or
            // nothing) already stood in for the motion it missed, and adding its catch-up would count it twice.
            double primaryTurn = fresh && !primaryGap ? turn(lastPrimary, primaryDeg) * primaryScale : Double.NaN;
            boolean primaryOk = !Double.isNaN(primaryTurn) && Math.abs(primaryTurn) <= limit;

            if (primaryOk && backupOk && Math.abs(primaryTurn) >= BACKUP_JUDGE_MIN_TURN_DEG) {
                sumPrimarySquared += primaryTurn * primaryTurn;
                sumPrimaryTimesBackup += primaryTurn * backupTurn;
                judgedTurn += Math.abs(primaryTurn);
            }

            usedBackup = false;
            double turned = 0.0;
            boolean measured = false;
            if (primaryOk) {
                turned = primaryTurn;
                measured = true;
                primarySinceZero += primaryTurn;
            } else if (backupOk && !hasPrimary && Double.isNaN(lastPrimary)) {
                // No main sensor at all: the backup is all there is.
                turned = backupTurn;
                measured = true;
                usedBackup = true;
            } else if (backupOk && backupTrusted()) {
                turned = backupTurn;
                usedBackup = true;
            }

            stillSeconds = still ? stillSeconds + dt : 0.0;
            if (still && stillSeconds >= STILL_SETTLE_SEC) {
                // Wheels stopped and settled: small heading changes are gyro drift, so the heading is held. But stopped
                // wheels do not prove the robot is not turning (mecanum rollers let it be pushed round, and it can
                // be turned by hand), so what is held is kept, and given back the moment it adds up to more than
                // drift could explain.
                heldTurn += turned;
                heldSeconds += dt;
                if (measured) {
                    sampleTurn += turned;
                    sampleSeconds += dt;
                }
                if (Math.abs(heldTurn) > HOLD_SLACK_DEG + MAX_DRIFT_DEG_PER_SEC * heldSeconds) {
                    heading += heldTurn;
                    endHold();
                    stillSeconds = 0.0;
                } else if (sampleSeconds >= DRIFT_SAMPLE_SEC) {
                    double rate = sampleTurn / sampleSeconds;
                    if (Math.abs(rate) <= MAX_DRIFT_DEG_PER_SEC) {
                        driftDegPerSec += (rate - driftDegPerSec) * 0.5;
                    }
                    sampleTurn = 0.0;
                    sampleSeconds = 0.0;
                }
            } else {
                endHold();
                // Moving (or just stopped): the drift is still there under the real turning, so take it back out.
                heading += measured ? turned - driftDegPerSec * dt : turned;
            }
            if (backupOk) {
                backupSinceZero += backupTurn;
            }
            if (hasPrimary && !firstPrimary && !primaryOk) {
                rejected++;
            }

            if (fresh) {
                lastPrimary = primaryDeg;
                primaryGap = false;
            } else if (!firstPrimary) {
                primaryGap = true;
            }
            if (hasBackup && backupHealthy) {
                lastBackup = backupDeg;
                backupGap = false;
            } else if (!Double.isNaN(lastBackup)) {
                backupGap = true;
            }
            // Kept within -180..180 so it reads sensibly: two full turns are the same direction as none, and the drive
            // only uses the heading through cos and sin, which do not care.
            heading = AngleUnit.normalizeDegrees(heading);
            return heading;
        }

        /**
         * Whether the backup's turns have been seen to match the main sensor's: same size and the same way round.
         * Never flipped to fit: which sensor counts backwards is for a person to settle with the hand-turn check (see
         * PINPOINT_HEADING_REVERSED), because a backup flipped to match a main sensor that is itself backwards would
         * be wrong too, and a wrong guess turns every glitch into a heading error.
         */
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
         * Whether a Pinpoint status means its heading reading is current. Restarting, calibrating and a runaway gyro
         * do not; nor does a bad read, because the driver then repeats the previous value. A missing pod does: heading
         * comes from the gyro, not the pods.
         */
        static boolean freshHeading(GoBildaPinpointDriver.DeviceStatus status) {
            return status == GoBildaPinpointDriver.DeviceStatus.READY
                    || status == GoBildaPinpointDriver.DeviceStatus.FAULT_X_POD_NOT_DETECTED
                    || status == GoBildaPinpointDriver.DeviceStatus.FAULT_Y_POD_NOT_DETECTED
                    || status == GoBildaPinpointDriver.DeviceStatus.FAULT_NO_PODS_DETECTED;
        }

        /** Forgets the current hold. Drift is only ever learned from complete samples, so a part sample is dropped. */
        private void endHold() {
            heldTurn = 0.0;
            heldSeconds = 0.0;
            sampleTurn = 0.0;
            sampleSeconds = 0.0;
        }

        /** The measured gyro drift of the steering sensor, degrees per second, being subtracted while moving. */
        double driftDegPerSec() {
            return driftDegPerSec;
        }

        /** For tests: the turn scales update(pinpoint, imu) would take from the constants. */
        void setScales(double primary, double backup) {
            primaryScale = primary;
            backupScale = backup;
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

        /** How many loops the main sensor's reading was frozen or impossible, since the OpMode began. */
        int rejectedReadings() {
            return rejected;
        }

        /** One telemetry line: each sensor's turn since the last zero, the backup's health, and the glitch count. */
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
                    line.append(judgedTurn < BACKUP_TRUST_AFTER_DEG ? "  backup unchecked (turn the robot)"
                            : backupTrusted() ? "  backup OK"
                            : agreement < 0 ? "  OPPOSITE WAYS (backup not used: do the hand-turn check)"
                            : String.format("  BACKUP WRONG (x%.2f, not used)", agreement));
                }
            }
            line.append(String.format("  drift %.2f/s", driftDegPerSec));
            if (still && stillSeconds >= STILL_SETTLE_SEC) {
                line.append("  STILL");
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

        private static boolean isReading(double deg) {
            return !Double.isNaN(deg) && !Double.isInfinite(deg);
        }

        /** The turn from one reading to the next, the short way round; NaN if either is missing. */
        private static double turn(double before, double after) {
            if (!isReading(before) || !isReading(after)) {
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
