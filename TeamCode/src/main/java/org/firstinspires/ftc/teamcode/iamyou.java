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
 * One controller, field-centric, heading from the goBILDA Pinpoint (the Control Hub's IMU is only a backup).
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
 *   Start (Options) ..... zero heading: wherever the robot faces right now becomes NORTH
 *   Logitech logo ....... FULL heading reset: also resets the Pinpoint (position + gyro recalibration), the hub
 *                         IMU and all drift/glitch history. Waits until the robot is stopped, then the drive is
 *                         off for about a quarter second while the gyro recalibrates.
 *
 * Mechanism controls (gamepad 1 in this version, gamepad 2 in the two-controller version):
 *   Right trigger ....... intake (hold). Fixed speed, however far the trigger is pulled.
 *   Right bumper ........ outtake / poop out (hold; wins over the trigger). Intake motor and Staging run backwards.
 *   Left bumper ......... flywheel ON/OFF (press to toggle). Holds FLYWHEEL_TARGET_TPS with velocity PIDF; the
 *                         controller rumbles once and telemetry says READY when it is up to speed.
 *                         While ON, if this alliance's raised CELL is in view, the robot turns onto it
 *                         (CellAimer) and the right stick's turn is ignored; the left stick still drives.
 *
 * Before START (gamepad 1):
 *   X / B ............... alliance: X = BLUE, B = RED. Aiming stays off until one is picked.
 *   Each intake servo's direction for intake and for outtake is set separately (LEFT_SERVO_* / RIGHT_SERVO_*).
 */
@TeleOp(name = "I AM YOU (2 controllers)")
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

    /** The full reset (logo button) waits until the robot has been stopped this long. */
    static final double FULL_RESET_STILL_SEC = 0.2;
    /** The drive comes back after this even if the Pinpoint never reports READY. */
    static final double FULL_RESET_TIMEOUT_SEC = 1.0;

    /** True = add the sensor and drivetrain diagnostics back to the telemetry (for chasing heading problems). */
    static final boolean DEBUG_TELEMETRY = false;

    private final Drive driveLogic = new Drive();
    private final Intake intakeLogic = new Intake();
    private final Outtake outtakeLogic = new Outtake();
    private final HeadingTracker headingTracker = new HeadingTracker();
    private final DriveCheck driveCheck = new DriveCheck();

    /** True = gamepad 2 runs the intake and outtake while gamepad 1 only drives. */
    protected boolean twoControllers() {
        return true;
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
        // Velocity mode with the hub's own PIDF (shootCommand.VELOCITY_PIDF) holds the speed while ON. While OFF the
        // motor is put back in plain power mode, so the wheel coasts down instead of being braked to zero.
        shootCommand.configure(outtake);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean flywheelVelocityMode = false;
        // Coast to a stop: braking a spinning flywheel hard is a shock load on the motor and gears.
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        boolean wasReady = false;

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

                // Kept running in INIT so heading drift is measured while the robot sits still.
                if (hasHeading) {
                    headingTracker.update(pinpoint, imu, initDt, wheelsStill(wheels));
                }
                telemetry.addData("Status", "Initialized - waiting for start");
                telemetry.addData("Heading Source", imuStatus);
                if (pinpoint != null) {
                    telemetry.addData("Pinpoint", pinpointStatus(pinpoint));
                }
                telemetry.addData("Alliance", red == null ? "NOT SET  (X = BLUE, B = RED)" : (red ? "RED" : "BLUE"));
                telemetry.addData("Driver Position", driverPosition + "  (D-pad)");
                if (camera == null) {
                    telemetry.addData("Camera", "MISSING - aiming off");
                }
                if (DEBUG_TELEMETRY && hasHeading) {
                    telemetry.addData("Heading (deg)", "%.1f", headingTracker.heading());
                    telemetry.addData("Sensors", headingTracker.sensorReport());
                    telemetry.addData("Level Check", levelCheck(pinpoint, imu));
                }
                telemetry.update();
            }

            if (isStopRequested()) return;

            // Built now the alliance is final: CellAimer fixes it at construction.
            CellAimer aimer = tags != null && red != null ? new CellAimer(tags, driveHardware, red) : null;

            // Zero heading at the moment of start: the robot's current front is NORTH.
            headingTracker.zero();
            // The Pinpoint's own heading count at the last zero, for the "pinpoint own" telemetry total.
            double pinpointRawAtZero = pinpoint != null ? pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES) : 0.0;

            ElapsedTime loopTimer = new ElapsedTime();
            ElapsedTime currentTimer = new ElapsedTime();
            telemetry.setMsTransmissionInterval(100);
            double lastIntakePower = 0.0;
            double lastCheckedHeading = 0.0;
            boolean lastOptions = false;
            boolean lastGuide = false;
            boolean resetPending = false;
            boolean calibrating = false;
            double stoppedSeconds = 0.0;
            double calibratingSeconds = 0.0;
            DcMotor.ZeroPowerBehavior driveZeroPower = DcMotor.ZeroPowerBehavior.BRAKE;
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
                    if (pinpoint != null) {
                        pinpointRawAtZero = pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES);
                    }
                }
                lastOptions = gamepad1.options;

                // Logitech logo button (guide): FULL heading reset. Waits for the robot to stop, because the Pinpoint
                // recalibrates its gyro and a moving robot would calibrate in a wrong drift. Then the drive stays off
                // until the Pinpoint is READY again (about a quarter second).
                if (gamepad1.guide && !lastGuide) {
                    resetPending = true;
                }
                lastGuide = gamepad1.guide;
                boolean robotStopped = wheelsStill(wheels) && !sticksTouched(gamepad1);
                stoppedSeconds = robotStopped ? stoppedSeconds + dt : 0.0;
                if (resetPending && stoppedSeconds >= FULL_RESET_STILL_SEC) {
                    if (pinpoint != null) {
                        pinpoint.resetPosAndIMU();
                    }
                    if (imu != null) {
                        imu.resetYaw();
                    }
                    headingTracker.fullReset();
                    pinpointRawAtZero = 0.0;
                    resetPending = false;
                    calibratingSeconds = 0.0;
                    calibrating = pinpoint != null;
                }
                if (calibrating) {
                    calibratingSeconds += dt;
                    boolean ready = pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
                    // The first loop or two after the command can still report the old READY.
                    if ((ready && calibratingSeconds > 0.1) || calibratingSeconds > FULL_RESET_TIMEOUT_SEC) {
                        calibrating = false;
                    }
                }

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

                // BRAKE only once the sticks are released. While driving, a wheel at exactly 0 power (the inside
                // wheels when driving and turning hard) would short-brake and drag the moving robot, costing speed.
                boolean sticksActive = result.stickForward != 0.0 || result.stickRight != 0.0 || result.twist != 0.0;
                DcMotor.ZeroPowerBehavior wantedZero = sticksActive
                        ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE;
                if (wantedZero != driveZeroPower) {
                    for (DcMotorEx m : wheels) {
                        m.setZeroPowerBehavior(wantedZero);
                    }
                    driveZeroPower = wantedZero;
                }

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
                if (calibrating) {
                    wheelPowers = new double[]{0.0, 0.0, 0.0, 0.0};   // hold still while the gyro recalibrates
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
                double flywheelTicksPerSec = outtake.getVelocity();
                double flywheelTarget = outtakeLogic.update(mech.left_bumper, flywheelTicksPerSec, dt);
                if (flywheelTarget > 0.0) {
                    if (!flywheelVelocityMode) {
                        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        flywheelVelocityMode = true;
                    }
                    outtake.setVelocity(flywheelTarget);
                } else {
                    if (flywheelVelocityMode) {
                        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        flywheelVelocityMode = false;
                    }
                    outtake.setPower(0.0);
                }
                boolean flywheelReady = outtakeLogic.isReady();
                if (flywheelReady && !wasReady) {
                    mech.rumble(200);
                }
                wasReady = flywheelReady;

                // ================= TELEMETRY =================
                for (String line : shotIndicator(outtakeLogic.isOn(), flywheelReady, flywheelTicksPerSec)) {
                    telemetry.addLine(line);
                }
                telemetry.addData("Flywheel", "%s  %.0f / %.0f rpm",
                        !outtakeLogic.isOn() ? "OFF" : flywheelReady ? "READY" : "spinning up",
                        ticksPerSecToRpm(flywheelTicksPerSec), ticksPerSecToRpm(flywheelTarget));
                if (resetPending) {
                    telemetry.addLine("HEADING RESET: stop the robot to finish");
                } else if (calibrating) {
                    telemetry.addLine("HEADING RESET: hold still, recalibrating...");
                }
                telemetry.addData("Aim", aimStatus(aimer, aim, outtakeLogic.isOn()));
                telemetry.addData("Intake", "%s  power %.2f  %.1f A",
                        intakeResult.state, intakeResult.power, intakeAmps);
                telemetry.addData("Intake Servos L | R", "%.2f | %.2f", leftServo, rightServo);
                telemetry.addData("Staging", "%.2f", staging.getPower());
                telemetry.addData("Slow Mode", result.slowMode ? "ON" : "OFF");
                telemetry.addData("Stop Assist", result.braking ? "BRAKING" : "off");
                telemetry.addLine("--- heading ---");
                telemetry.addData("Heading (deg)", "%.1f", heading);
                telemetry.addData("Facing (vs start)", snapToCardinal(heading));
                telemetry.addData("Driver Position", driverPosition + "  (D-pad)");
                telemetry.addData("Sensors", headingTracker.sensorReport());
                if (pinpoint != null) {
                    // The Pinpoint's own turn count since the last zero, without PINPOINT_TURN_SCALE. If it matches
                    // "main" divided by the scale, no turn was lost; if it is bigger, the tracker lost loops.
                    // Meaningless after a Pinpoint restart (it counts from 0 again).
                    telemetry.addData("Pinpoint", "%s  own %.1f", pinpoint.getDeviceStatus(),
                            pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES) - pinpointRawAtZero);
                }
                if (!hasHeading) {
                    telemetry.addData("IMU", imuStatus);
                }
                telemetry.addLine("--- wheels ---");
                telemetry.addData("Twist", "%.2f", result.twist);
                telemetry.addData("FL | FR", "%.2f | %.2f", wheelPowers[0], wheelPowers[1]);
                telemetry.addData("BL | BR", "%.2f | %.2f", wheelPowers[2], wheelPowers[3]);
                telemetry.addData("FL Vel | FR Vel", "%.0f | %.0f", params.flVelocity, params.frVelocity);
                telemetry.addData("BL Vel | BR Vel", "%.0f | %.0f", params.blVelocity, params.brVelocity);
                telemetry.addData("Loop", "%.1f ms", dt * 1000.0);
                if (DEBUG_TELEMETRY) {
                    telemetry.addLine("--- debug ---");
                    telemetry.addData("Drive Check", driveCheck.report());
                    telemetry.addData("Still Check", stillReport(wheels, gamepad1, still,
                            aimerMayTurn(aimer, outtakeLogic)));
                    telemetry.addData("Flywheel PIDF (hub)", "P %.1f  I %.1f  D %.1f  F %.1f",
                            shootCommand.VELOCITY_PIDF.p, shootCommand.VELOCITY_PIDF.i,
                            shootCommand.VELOCITY_PIDF.d, shootCommand.VELOCITY_PIDF.f);
                    telemetry.addData("Cmd Fwd | Right", "%.2f | %.2f", result.stickForward, result.stickRight);
                    telemetry.addData("Robot Fwd | Right", "%.2f | %.2f", result.robotForward, result.robotRight);
                }
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
    /**
     * Big enough to see from the driver's spot: a solid white block when the flywheel is at speed, an empty box when
     * it is not (with how far up to speed it is), one short line when it is off.
     */
    static String[] shotIndicator(boolean on, boolean ready, double ticksPerSec) {
        if (!on) {
            return new String[]{"\u2610 FLYWHEEL OFF"};
        }
        String solid = repeat('\u2588', 22);
        if (ready) {
            return new String[]{solid, solid, "\u2588\u2588\u2588  READY TO SHOOT  \u2588\u2588\u2588", solid, solid};
        }
        String percent = String.format(java.util.Locale.US, "NOT READY  %.0f%%",
                100.0 * Math.max(0.0, ticksPerSec) / FLYWHEEL_TARGET_TPS);
        return new String[]{
                "\u250C" + repeat('\u2500', 20) + "\u2510",
                "\u2502" + repeat(' ', 30) + "\u2502",
                "\u2502      " + percent + "      \u2502",
                "\u2502" + repeat(' ', 30) + "\u2502",
                "\u2514" + repeat('\u2500', 20) + "\u2518"};
    }

    private static String repeat(char c, int n) {
        StringBuilder b = new StringBuilder(n);
        for (int i = 0; i < n; i++) {
            b.append(c);
        }
        return b.toString();
    }

    static double ticksPerSecToRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / FLYWHEEL_TICKS_PER_REV;
    }

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
    /**
     * Normal (not slow mode) drive speed. 1.0 = full power at full stick. 1.5 = 50 % more power at every stick
     * position, so full power is reached at two-thirds stick (it can't go past full power: the motors' limit).
     */
    static final double DRIVE_MULT               = 1.5;
    /** Normal (not slow mode) turn speed, same idea: 1.5 = full-power turn at two-thirds stick. */
    static final double TWIST_MULT               = 1.5;
    /**
     * Driving and turning at once can ask a wheel for more than full power; this decides who gives way.
     * 0.5 = both slow down by the same percentage (full stick + full twist = 50 % drive + 50 % turn).
     * 1.0 = turning keeps its speed and driving gets what's left. 0.0 = driving keeps its speed.
     */
    static final double TURN_PRIORITY            = 0.5;
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
            double driveMult = slow ? SLOW_MODE_DRIVE_MULT : DRIVE_MULT;
            double twistMult = slow ? SLOW_MODE_TWIST_MULT : TWIST_MULT;
            double d = robotForward * driveMult;
            double s = robotRight * STRAFE_COMPENSATION * driveMult;
            double t = Math.max(-1.0, Math.min(1.0, twist * twistMult));   // full turn power at most

            boolean released = stick[0] == 0.0 && stick[1] == 0.0 && twist == 0.0;
            double[] out = new double[4];
            boolean braking = false;
            if (!released) {
                // ================= MECANUM MIXER (positive t = clockwise) =================
                // A wheel can't go past full power, and driving and turning share it: full stick + full twist asks
                // every wheel for 200 %, so something has to give. TURN_PRIORITY picks what: at 0.5 both are cut by
                // the same percentage (the curve the sticks ask for is kept exactly), higher keeps more turn,
                // lower keeps more drive. Below full power on every wheel nothing is cut at all.
                double[] move = {d + s, d - s, d - s, d + s};
                double moveMax = Math.max(1.0, Math.max(Math.abs(d + s), Math.abs(d - s)));
                for (int k = 0; k < 4; k++) {
                    move[k] /= moveMax;
                }
                double moveBiggest = Math.max(Math.abs(move[0]), Math.abs(move[1]));
                double turn = t;
                double asked = moveBiggest + Math.abs(t);
                if (asked > 1.0) {
                    double even = Math.abs(t) / asked;                  // both cut by the same percentage
                    double turnFirst = Math.min(Math.abs(t), 1.0);      // turn keeps all it can
                    double driveFirst = Math.max(0.0, 1.0 - moveBiggest); // turn gets only what drive leaves
                    double p = Math.max(0.0, Math.min(1.0, TURN_PRIORITY));
                    double turnShare = p >= 0.5
                            ? even + (turnFirst - even) * (p - 0.5) * 2.0
                            : even + (driveFirst - even) * (0.5 - p) * 2.0;
                    turn = Math.copySign(turnShare, t);
                    double moveRoom = 1.0 - turnShare;
                    for (int k = 0; k < 4; k++) {
                        move[k] = moveBiggest > 0.0 ? move[k] * moveRoom / moveBiggest : 0.0;
                    }
                }
                double[] mix = {
                        frictionBoost(move[0] + turn),
                        frictionBoost(move[1] - turn),
                        frictionBoost(move[2] + turn),
                        frictionBoost(move[3] - turn)
                };
                // The friction boost can nudge a wheel past full power; scale back so the direction is kept.
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
    //  - A bad read (FAULT_BAD_READ) is NOT a gap: the SDK driver then keeps the previous heading, or the new one if
    //    only another value (a velocity) was bad, so the next reading's change from the last one is exactly the turn
    //    made meanwhile and is counted. Treating it as a gap threw that turn away on every bad read, and the turn
    //    scale measured with the turns missing (4 % "under-read") then over-counted every turn the robot made.
    //  - A blank read (firmware v3+: a failed I2C read arrives as zeros, so status NOT_READY and heading 0; the
    //    driver's own check does not catch it) is a short gap, not a restart. When the next good reading continues
    //    from the last one, the turn across the gap is counted (less whatever the backup already stood in for).
    //    Taking it as a new baseline lost a loop's turn at every failed read, always in the direction of the turn,
    //    so the heading fell further behind the robot with every turn and the drive slowly rotated wrong.
    //  - A loop where the Pinpoint is frozen (restarting or calibrating: it re-zeros) or turns faster than any robot
    //    can uses the Control Hub IMU's turn instead - once the
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
    // 2026-11-08: 3453 for ten turns (4.1 % short), with bad and blank reads still losing turn.
    // 2026-11-13: twelve turns, blank reads bridged: main 4154.2 (3.8 % short), backup 4283.9 (0.8 % short) for 4320.
    // The Pinpoint is short by the same amount both times, so it is its scale, not lost loops. (A gyro tilted 16
    // degrees off level would also read 3.8 % short: check the INIT Level Check line with DEBUG_TELEMETRY.)
    // The Sensors line's "pinpoint own" total tells the two apart: it is the Pinpoint's own count, unscaled.
    static final double PINPOINT_TURN_SCALE = 4320.0 / 4154.2;
    static final double HUB_IMU_TURN_SCALE = 4320.0 / 4283.9;

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
    /**
     * After loops of bad reads (heading held) the next reading carries all the turn made meanwhile, so the rate limit
     * allows for that time too, up to this much.
     */
    static final double MAX_HELD_READING_SEC = 0.5;
    /**
     * A failed I2C read of a Pinpoint on firmware v3 or later comes back as all zeros, and the SDK driver passes it
     * on as status NOT_READY with heading 0: a blank loop, not a restart. If the next good reading continues from the
     * last one within this long, the turn made across the blank is counted. A real restart takes longer and passes
     * through CALIBRATING, so it is never bridged.
     */
    static final double MAX_BLANK_GAP_SEC = 0.15;
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
        /** How long the main sensor has been reporting bad reads in a row (heading held): widens the rate limit. */
        private double primaryHeldSeconds = 0.0;
        /** True when this loop's main reading is a bad read (FAULT_BAD_READ). Set before update(). */
        private boolean badRead = false;
        /** True when this loop's main reading is a blank failed read (NOT_READY, heading 0). Set before update(). */
        private boolean blankRead = false;
        /** The current gap in the main sensor: how long, whether it is only blank reads, and the backup used in it. */
        private double gapSeconds = 0.0;
        private boolean gapBridgeable = false;
        private double backupUsedInGap = 0.0;
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
                badRead = status == GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ;
                blankRead = status == GoBildaPinpointDriver.DeviceStatus.NOT_READY;
                return update(signed(pinpoint.getHeading(AngleUnit.DEGREES), PINPOINT_HEADING_REVERSED),
                        freshHeading(status), imuYaw, imuRead, dtSeconds);
            }
            primaryScale = HUB_IMU_TURN_SCALE;
            blankRead = false;
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
            double primaryLimit = MAX_TURN_RATE_DEG_PER_SEC * (dt + primaryHeldSeconds) + TURN_JUMP_SLACK_DEG;
            boolean firstPrimary = Double.isNaN(lastPrimary);
            hasPrimary = isReading(primaryDeg);
            hasBackup = isReading(backupDeg);

            double backupTurn = backupHealthy && !backupGap ? turn(lastBackup, backupDeg) * backupScale : Double.NaN;
            boolean backupOk = !Double.isNaN(backupTurn) && Math.abs(backupTurn) <= limit;
            boolean fresh = hasPrimary && primaryFresh;
            // After a frozen stretch the main sensor's first reading is a new baseline, not a turn: the backup (or
            // nothing) already stood in for the motion it missed, and adding its catch-up would count it twice.
            double primaryTurn = fresh && !primaryGap ? turn(lastPrimary, primaryDeg) * primaryScale : Double.NaN;
            boolean primaryOk = !Double.isNaN(primaryTurn) && Math.abs(primaryTurn) <= primaryLimit;
            // Except after a short gap of blank reads only: the Pinpoint never stopped measuring, so a reading that
            // continues from the last good one carries the turn made across the gap. Dropping it (as a restart's
            // new baseline would) loses a loop's turn at every failed read, always the way the robot was turning.
            boolean bridged = false;
            if (fresh && primaryGap && gapBridgeable && gapSeconds + dt <= MAX_BLANK_GAP_SEC) {
                double across = turn(lastPrimary, primaryDeg) * primaryScale;
                if (!Double.isNaN(across)
                        && Math.abs(across) <= MAX_TURN_RATE_DEG_PER_SEC * (gapSeconds + dt) + TURN_JUMP_SLACK_DEG) {
                    primaryTurn = across;
                    primaryOk = true;
                    bridged = true;
                }
            }

            if (primaryOk && !bridged && backupOk && Math.abs(primaryTurn) >= BACKUP_JUDGE_MIN_TURN_DEG) {
                sumPrimarySquared += primaryTurn * primaryTurn;
                sumPrimaryTimesBackup += primaryTurn * backupTurn;
                judgedTurn += Math.abs(primaryTurn);
            }

            usedBackup = false;
            double turned = 0.0;
            boolean measured = false;
            if (primaryOk) {
                // A bridged turn includes what the backup already stood in for during the gap: only the rest is new.
                turned = bridged ? primaryTurn - backupUsedInGap : primaryTurn;
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
                primaryHeldSeconds = badRead ? Math.min(MAX_HELD_READING_SEC, primaryHeldSeconds + dt) : 0.0;
                lastPrimary = primaryDeg;
                primaryGap = false;
            } else if (!firstPrimary) {
                if (!primaryGap) {
                    // A gap begins. It can be bridged only if it began with a good reading to continue from.
                    gapSeconds = 0.0;
                    gapBridgeable = true;
                    backupUsedInGap = 0.0;
                }
                primaryGap = true;
                gapSeconds += dt;
                gapBridgeable = gapBridgeable && blankRead;
                if (usedBackup) {
                    backupUsedInGap += turned;
                }
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
         * Whether a Pinpoint status means its heading reading continues from the last one, so the change between
         * them is a turn. Restarting, calibrating and a runaway gyro do not (the heading re-zeros). A bad read does:
         * the driver keeps the previous heading (or takes the new one when only another value was bad), so the next
         * change still measures the turn exactly. A missing pod does too: heading comes from the gyro, not the pods.
         */
        static boolean freshHeading(GoBildaPinpointDriver.DeviceStatus status) {
            return status == GoBildaPinpointDriver.DeviceStatus.READY
                    || status == GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ
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

        /** For tests: says the next main reading is a bad read, as update(pinpoint, imu) does from the status. */
        void setBadRead(boolean badRead) {
            this.badRead = badRead;
        }

        /** For tests: says the next main reading is a blank read (NOT_READY), as update(pinpoint, imu) does. */
        void setBlankRead(boolean blankRead) {
            this.blankRead = blankRead;
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

        /**
         * Forgets everything: heading, both sensors' baselines, drift, holds, glitch count and the backup's
         * agreement record. Call it together with resetting the sensors themselves (the Logitech logo button does): the
         * next good reading of each sensor starts fresh, with the robot's current front as NORTH.
         */
        void fullReset() {
            heading = 0.0;
            primarySinceZero = 0.0;
            backupSinceZero = 0.0;
            lastPrimary = Double.NaN;
            primaryHeldSeconds = 0.0;
            gapSeconds = 0.0;
            gapBridgeable = false;
            backupUsedInGap = 0.0;
            lastBackup = Double.NaN;
            primaryGap = true;
            backupGap = true;
            rejected = 0;
            usedBackup = false;
            sumPrimarySquared = 0.0;
            sumPrimaryTimesBackup = 0.0;
            judgedTurn = 0.0;
            lastFault = null;
            stillSeconds = 0.0;
            driftDegPerSec = 0.0;
            heldTurn = 0.0;
            heldSeconds = 0.0;
            sampleTurn = 0.0;
            sampleSeconds = 0.0;
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
    // FLYWHEEL (OUTTAKE)
    //
    // One press turns the flywheel on, the next turns it off. While ON the hub's built-in velocity PIDF holds
    // FLYWHEEL_TARGET_TPS (encoder ticks per second) with shootCommand.VELOCITY_PIDF, the same gains every AUTO uses;
    // while OFF the motor is in power mode at 0 and coasts (FLOAT). Outtake only decides ON/OFF and READY.
    //
    // FlywheelPIDF below is the software loop "Flywheel PIDF Tuning" runs; no match OpMode uses it.
    // =====================================================================================================

    /** Flywheel speed while ON, in encoder ticks per second (the 6000 rpm motor's free speed is about 2800). */
    static final double FLYWHEEL_TARGET_TPS = 1349.0;   // = 2891 rpm
    /** Encoder ticks per flywheel turn: the 6000 rpm motor's 28, geared 1:1. */
    static final double FLYWHEEL_TICKS_PER_REV = 28.0;
    /** Power to break static friction, at 12 V. */
    static final double FLYWHEEL_KS = 0.03;
    /** Power per tick/s at 12 V. 1 / 2800 would be exactly the motor's free speed; a loaded wheel needs a bit more. */
    static final double FLYWHEEL_KV = 0.0003;
    /** Power per tick/s of error. 100 ticks/s low = +0.05 power. */
    static final double FLYWHEEL_KP = 0.0005;   // tuned on the robot
    /** Power per (tick/s x second) of summed error. */
    static final double FLYWHEEL_KI = 0.0002;
    /** Power per tick/s/s of measured acceleration. */
    static final double FLYWHEEL_KD = 0.0;
    /** The I term only builds up this close to the target, so spin-up and shot dips don't wind it up. */
    static final double FLYWHEEL_I_ZONE_TPS = 150.0;
    /** The I term never adds or removes more power than this. */
    static final double FLYWHEEL_I_MAX_POWER = 0.15;
    /** "READY" once the speed has been within this of the target for FLYWHEEL_READY_SEC. */
    static final double FLYWHEEL_READY_TOLERANCE_TPS = 50.0;
    static final double FLYWHEEL_READY_SEC = 0.15;
    /** Voltage the gains above were tuned at. */
    static final double FLYWHEEL_NOMINAL_VOLTS = 12.0;

    /** A velocity PIDF controller with gains that can be changed live (the tuning OpMode does). */
    static class FlywheelPIDF {
        double kS = FLYWHEEL_KS;
        double kV = FLYWHEEL_KV;
        double kP = FLYWHEEL_KP;
        double kI = FLYWHEEL_KI;
        double kD = FLYWHEEL_KD;

        private double integral = 0.0;          // in power, already multiplied by kI
        private double lastVelocity = Double.NaN;
        private double accel = 0.0;             // filtered, ticks/s/s
        private double readySeconds = 0.0;
        private double error = 0.0;
        private double feedforward = 0.0;
        private double proportional = 0.0;
        private double derivative = 0.0;

        /**
         * @param targetTps   wanted speed, ticks per second (0 or less = off)
         * @param measuredTps the encoder's speed, ticks per second
         * @param volts       battery voltage (NaN or nonsense = assume nominal)
         * @param dtSeconds   time since the previous call
         * @return motor power, 0 to 1
         */
        double update(double targetTps, double measuredTps, double volts, double dtSeconds) {
            double dt = Math.max(0.001, Math.min(0.1, finite(dtSeconds, 0.02)));
            double target = finite(targetTps, 0.0);
            double measured = finite(measuredTps, Double.isNaN(lastVelocity) ? 0.0 : lastVelocity);
            double v = finite(volts, FLYWHEEL_NOMINAL_VOLTS);
            if (v < 7.0 || v > 16.0) {
                v = FLYWHEEL_NOMINAL_VOLTS;
            }

            if (!Double.isNaN(lastVelocity)) {
                double rawAccel = (measured - lastVelocity) / dt;
                accel += (rawAccel - accel) * 0.3;
            }
            lastVelocity = measured;

            if (target <= 0.0) {
                reset();
                lastVelocity = measured;
                return 0.0;
            }

            error = target - measured;
            double voltScale = FLYWHEEL_NOMINAL_VOLTS / v;
            feedforward = (kS + kV * target) * voltScale;
            proportional = kP * error;
            derivative = -kD * accel;

            if (Math.abs(error) < FLYWHEEL_I_ZONE_TPS) {
                double unclamped = feedforward + proportional + derivative + integral;
                boolean pushingPastLimit = (unclamped >= 1.0 && error > 0) || (unclamped <= 0.0 && error < 0);
                if (!pushingPastLimit) {
                    integral += kI * error * dt;
                }
                integral = Math.max(-FLYWHEEL_I_MAX_POWER, Math.min(FLYWHEEL_I_MAX_POWER, integral));
            }

            readySeconds = Math.abs(error) <= FLYWHEEL_READY_TOLERANCE_TPS ? readySeconds + dt : 0.0;

            double power = feedforward + proportional + integral + derivative;
            return Math.max(0.0, Math.min(1.0, finite(power, 0.0)));
        }

        /** Forget the I term and the ready timer (called whenever the flywheel is off). */
        void reset() {
            integral = 0.0;
            readySeconds = 0.0;
            error = 0.0;
            feedforward = proportional = derivative = 0.0;
        }

        boolean isReady() {
            return readySeconds >= FLYWHEEL_READY_SEC;
        }

        double error() {
            return error;
        }

        /** "F 0.79  P 0.01  I 0.02  D 0.00": what each term is adding right now. */
        String terms() {
            return String.format(java.util.Locale.US, "F %.2f  P %.2f  I %.2f  D %.2f",
                    feedforward, proportional, integral, derivative);
        }
    }

    /**
     * Flywheel state carried between loops: the ON/OFF toggle and the READY timer. The speed itself is held by the
     * hub's velocity PIDF, so this only says what to command.
     */
    static class Outtake {
        private boolean on = false;
        private boolean lastButton = false;
        private double readySeconds = 0.0;

        /**
         * @param toggleButton true while the flywheel button is held; each new press flips ON/OFF
         * @param measuredTps  flywheel encoder speed, ticks per second
         * @param dtSeconds    time since the previous loop
         * @return the speed to command, ticks per second; 0 means OFF (coast)
         */
        double update(boolean toggleButton, double measuredTps, double dtSeconds) {
            if (toggleButton && !lastButton) {
                on = !on;
            }
            lastButton = toggleButton;
            boolean close = on && Math.abs(FLYWHEEL_TARGET_TPS - measuredTps) <= FLYWHEEL_READY_TOLERANCE_TPS;
            readySeconds = close ? readySeconds + Math.max(0.0, finite(dtSeconds, 0.0)) : 0.0;
            return on ? FLYWHEEL_TARGET_TPS : 0.0;
        }

        boolean isOn() {
            return on;
        }

        /** True when ON and the speed has held within tolerance for FLYWHEEL_READY_SEC: safe to feed a ball. */
        boolean isReady() {
            return on && readySeconds >= FLYWHEEL_READY_SEC;
        }
    }
}
