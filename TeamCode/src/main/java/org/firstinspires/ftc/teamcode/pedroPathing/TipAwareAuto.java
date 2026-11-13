package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.IntakeCommands.startIntake;
import static org.firstinspires.ftc.teamcode.IntakeCommands.stopIntake;
import static org.firstinspires.ftc.teamcode.pedroPathing.AllianceAuto.AT_FLOWER;
import static org.firstinspires.ftc.teamcode.pedroPathing.AllianceAuto.FLOWER_DWELL_MS;
import static org.firstinspires.ftc.teamcode.pedroPathing.AllianceAuto.FRONT_START;
import static org.firstinspires.ftc.teamcode.pedroPathing.AllianceAuto.FRONT_WALL_SHOT;
import static org.firstinspires.ftc.teamcode.pedroPathing.AllianceAuto.GARDEN_END;
import static org.firstinspires.ftc.teamcode.pedroPathing.AllianceAuto.GARDEN_ENTRY;
import static org.firstinspires.ftc.teamcode.pedroPathing.AllianceAuto.METRES_PER_INCH;
import static org.firstinspires.ftc.teamcode.pedroPathing.AllianceAuto.PARK;
import static org.firstinspires.ftc.teamcode.pedroPathing.AllianceAuto.PATH_TIMEOUT_MS;
import static org.firstinspires.ftc.teamcode.pedroPathing.AllianceAuto.PRELOAD_RIGHT_DELAY_MS;
import static org.firstinspires.ftc.teamcode.pedroPathing.AllianceAuto.REAR_CLEAR;
import static org.firstinspires.ftc.teamcode.pedroPathing.AllianceAuto.REAR_START;
import static org.firstinspires.ftc.teamcode.pedroPathing.AllianceAuto.REAR_WALL_SHOT;
import static org.firstinspires.ftc.teamcode.pedroPathing.AllianceAuto.SHOOT;
import static org.firstinspires.ftc.teamcode.pedroPathing.AllianceAuto.direction;
import static org.firstinspires.ftc.teamcode.pedroPathing.AllianceAuto.pose;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CellAimer;
import org.firstinspires.ftc.teamcode.LaunchGeometry;
import org.firstinspires.ftc.teamcode.LensMount;
import org.firstinspires.ftc.teamcode.RaisedCellWatch;
import org.firstinspires.ftc.teamcode.iamyou;
import org.firstinspires.ftc.teamcode.shootCommand;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * {@link AllianceAuto}'s full routine, shooting at whichever CELL of the alliance's HIVE is
 * raised rather than assuming the one the match starts with.
 *
 * <p>Two shooting spots, one each side of the HIVE: {@link AllianceAuto#SHOOT} on the audience
 * side (for red; blue's are turned half a turn, so "front" is the side its default raised CELL
 * faces) and {@link AllianceAuto#REAR_START}, against the rear wall, behind it. The camera reads
 * which side the raised CELL faces through INIT ({@link RaisedCellWatch}), and the robot commits
 * at START. Before every shot it looks again, and if the CELL has turned away &mdash; a TIP,
 * ours or anyone's &mdash; it drives round to the other side when at least
 * {@link #SWITCH_SIDES_MIN_REMAINING_MS} of the AUTO is left, and otherwise parks without
 * shooting at a CELL that scores nothing. A dead camera never turns the robot round: with no
 * frames it keeps to the match-start arrangement, or to whatever it last saw.</p>
 *
 * <p>{@link Routine#FULL} collects from the GARDEN and the alliance-wall FLOWER one at a time,
 * shooting after each, and only when shooting from the front: both are on that side, and a trip
 * from the rear does not fit in an AUTO. Each trip starts only if it, its shot and the park still
 * fit ({@link #GARDEN_TRIP_MS}, {@link #FLOWER_TRIP_MS}, {@link #PARK_MS}).
 * {@link Routine#PRELOAD_AND_PARK} shoots the pre-loads and parks; a Right start drives round
 * first if it must, then holds its first shot until 15 s after START.</p>
 *
 * <p>Every shot is taken pressed against the wall behind the shooting spot.</p>
 */
public abstract class TipAwareAuto extends LinearOpMode {

    /** Where the robot is set down. */
    public enum Start {
        /** {@link AllianceAuto#FRONT_START}: the side the HIVE's raised CELL faces at the start. */
        FRONT,
        /** {@link AllianceAuto#REAR_START}: behind the HIVE. */
        REAR
    }

    /** What it does once it knows which side to shoot from. */
    public enum Routine {
        /** Shoot, then a GARDEN and a FLOWER trip if they fit, each followed by a shot; PARK. */
        FULL,
        /** Shoot the pre-loads and PARK. A Right start shoots no earlier than 15 s in. */
        PRELOAD_AND_PARK
    }

    /** An AUTO is 30 s; everything is timed against this from START. */
    static final long AUTO_MS = 30_000;

    /**
     * Driving round the HIVE takes 7&ndash;8 s, the look up to 1 s, a shot 5 s and parking
     * 3&ndash;5 s (measured in the sim), so a switch with less than this left would not PARK.
     */
    static final long SWITCH_SIDES_MIN_REMAINING_MS = 18_000;

    /**
     * One trip to the GARDEN, the sweep, back to {@link AllianceAuto#SHOOT}, the look and the
     * shot, in milliseconds: 12&ndash;19 s in the sim, the long end when the sweep jams POLLEN
     * against the wall and runs to its timeout.
     */
    static final long GARDEN_TRIP_MS = 17_000;

    /**
     * The same for the FLOWER: there, 1.5 s of intake, back, the look and the shot. Estimated
     * from the legs' sim times (about 3.5 s each way), not yet measured as a whole: after a
     * GARDEN trip there is never this much left, so no test reaches it.
     */
    static final long FLOWER_TRIP_MS = 15_000;

    /** From {@link AllianceAuto#SHOOT} to the LOADING ZONE, in milliseconds, rounded up. */
    static final long PARK_MS = 5_000;

    /**
     * Longest the drive round the HIVE, one leg of it, may take, in milliseconds.
     *
     * <p>Longer than {@link AllianceAuto#PATH_TIMEOUT_MS}: its long leg takes about 5 s on its
     * own, and a leg cut short leaves the robot shooting from wherever it got to.</p>
     */
    static final double AROUND_HIVE_TIMEOUT_MS = 8000.0;

    /** How long the look before a shot may take, milliseconds, if the camera says nothing. */
    private static final long RECHECK_TIMEOUT_MS = 2 * RaisedCellWatch.LOOK_MS;

    private final PoseFactory alliance;
    private final boolean red;
    private final Start startSpot;
    private final Routine routine;

    private Follower follower;
    private DcMotorEx flywheel;
    private DcMotorEx intake;
    private CRServo staging;
    private CRServo intakeServoLeft;
    private CRServo intakeServoRight;
    private AprilTagProcessor tags;
    private final LensMount mount = LaunchGeometry.camera();
    private VisionPortal camera;
    private RaisedCellWatch watch;
    private final ElapsedTime clock = new ElapsedTime();
    private ElapsedTime sinceStart;

    /** Which side the raised CELL is believed to face; true is the front. */
    private boolean raisedFront = true;
    /** Which side the robot is on: true is the front. */
    private boolean atFront;
    /** Where the robot is standing between moves. */
    private Pose here;

    private String step = "INIT";
    private String decision = "";

    /**
     * @param alliance {@link AllianceAuto#RED} or {@link AllianceAuto#BLUE}
     * @param startSpot where the robot is set down
     * @param routine what it does
     */
    protected TipAwareAuto(PoseFactory alliance, Start startSpot, Routine routine) {
        this.alliance = alliance;
        this.red = alliance == AllianceAuto.RED;
        this.startSpot = startSpot;
        this.routine = routine;
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        flywheel = hardwareMap.get(DcMotorEx.class, iamyou.OUTTAKE_NAME);
        staging = hardwareMap.get(CRServo.class, iamyou.STAGING_NAME);
        intake = hardwareMap.get(DcMotorEx.class, iamyou.INTAKE_NAME);
        intakeServoLeft = hardwareMap.get(CRServo.class, iamyou.INTAKE_SERVO_LEFT_NAME);
        intakeServoRight = hardwareMap.get(CRServo.class, iamyou.INTAKE_SERVO_RIGHT_NAME);
        flywheel.setDirection(direction(iamyou.OUTTAKE_REVERSED));
        shootCommand.configure(flywheel);
        staging.setDirection(direction(iamyou.STAGING_REVERSED));
        intake.setDirection(direction(iamyou.INTAKE_REVERSED));

        atFront = startSpot == Start.FRONT;
        here = atFront ? pose(alliance, FRONT_START) : pose(alliance, REAR_START);
        follower.setPose(here);
        follower.update();

        camera = openCamera();
        watch = new RaisedCellWatch(now());

        // Read through INIT, so START costs no time; what counts is the last half-second of it.
        while (opModeInInit()) {
            look();
            RaisedCellWatch.Reading reading = watch.reading(now());
            telemetry.addData("start on", "x %.1f  y %.1f  heading %.0f (Pedro inches)",
                    here.x(), here.y(), Math.toDegrees(here.heading()));
            telemetry.addData("dashboard", "x %.3f  y %.3f  heading %.0f (metres)",
                    (here.x() - 72.0) * METRES_PER_INCH, (here.y() - 72.0) * METRES_PER_INCH,
                    Math.toDegrees(here.heading()));
            telemetry.addData("camera", describe(reading));
            telemetry.addData("plan", "shoot from the %s",
                    sideName(raisedFrontFrom(reading, defaultRaisedFront())));
            telemetry.update();
            idle();
        }
        if (!opModeIsActive()) {
            closeCamera();
            return;
        }
        sinceStart = new ElapsedTime();
        // Detection is heavy work for the Control Hub and slows every loop the follower runs, so
        // after START it runs only for the looks before a shot.
        detect(false);
        RaisedCellWatch.Reading atStart = watch.reading(now());
        raisedFront = raisedFrontFrom(atStart, defaultRaisedFront());
        decision = describe(atStart) + " at START";

        runRoutine();

        follower.manual(0.0, 0.0, 0.0);
        follower.update();
        closeCamera();
        step = "done";
        report();
    }

    private void runRoutine() {
        if (raisedFront != atFront) {
            mark("driving round to the " + sideName(raisedFront));
            driveToOtherSide();
        }
        if (routine == Routine.PRELOAD_AND_PARK) {
            if (startSpot == Start.REAR) {
                long wait = (long) PRELOAD_RIGHT_DELAY_MS - (AUTO_MS - remainingMs());
                if (wait > 0) {
                    mark("waiting until 15 s");
                    run(waitMs(wait));
                }
            }
            lookAndShoot("shooting the pre-loads");
            park();
            return;
        }
        if (!lookAndShoot("shooting the pre-loads")) {
            park();
            return;
        }
        // One collection per shot, GARDEN first, each only if it, its shot and the park still fit.
        // Only from the front: both are on that side, and a trip from the rear does not fit.
        if (canCollect(GARDEN_TRIP_MS)) {
            collectFromGarden();
            if (!lookAndShoot("shooting the GARDEN's POLLEN")) {
                park();
                return;
            }
        }
        if (canCollect(FLOWER_TRIP_MS)) {
            collectFromFlower();
            lookAndShoot("shooting the FLOWER's POLLEN");
        }
        park();
    }

    /**
     * Looks again, then shoots, drives round and shoots, or gives up.
     *
     * @return false if it gave up on this shot for want of time, so the caller should park
     */
    private boolean lookAndShoot(String label) {
        for (int switched = 0; ; switched++) {
            // A drive can end a few inches short of its spot, and this robot cannot shoot on the
            // move: settle on it first.
            run(AllianceAuto.settleAt(follower, wallShot()));
            RaisedCellWatch.Reading reading = recheck();
            boolean facing = reading == RaisedCellWatch.Reading.FACING
                    || reading == RaisedCellWatch.Reading.NO_CAMERA;
            if (!facing) {
                raisedFront = !atFront;
            }
            decision = describe(reading) + " from the " + sideName(atFront);
            if (facing) {
                mark(label);
                run(new shootCommand(flywheel, staging));
                return true;
            }
            // One drive round per shot: a CELL that is not there from either side is behind
            // something, and another lap would only use up the AUTO.
            if (switched > 0 || remainingMs() < SWITCH_SIDES_MIN_REMAINING_MS) {
                mark("the raised CELL faces the " + sideName(raisedFront)
                        + " and there is no time to get there");
                return false;
            }
            mark("the raised CELL turned to the " + sideName(raisedFront) + "; driving round");
            driveToOtherSide();
        }
    }

    /** Up to {@link #RECHECK_TIMEOUT_MS} of looking from where the robot stands. */
    private RaisedCellWatch.Reading recheck() {
        mark("looking for the raised CELL");
        detect(true);
        long started = now();
        watch.restart(started);
        while (opModeIsActive() && !watch.decided(now())
                && now() - started < RECHECK_TIMEOUT_MS) {
            look();
            follower.update();
            report();
            idle();
        }
        detect(false);
        return watch.reading(now());
    }

    private void detect(boolean on) {
        if (camera != null) {
            camera.setProcessorEnabled(tags, on);
        }
    }

    private boolean canCollect(long tripMs) {
        return raisedFront && atFront && remainingMs() >= tripMs + PARK_MS;
    }

    /** From the front shooting spot: the GARDEN's line of POLLEN, and back. */
    private void collectFromGarden() {
        Pose gardenEntry = pose(alliance, GARDEN_ENTRY);
        Pose gardenEnd = pose(alliance, GARDEN_END);
        Pose clearOfWall = alliance.of(30, 22, 180);
        Pose shoot = pose(alliance, SHOOT);

        mark("sweeping the GARDEN");
        drive(Paths.curve(here, alliance.of(56, 30, 0), alliance.of(34, 26, 0), gardenEntry)
                .linear(here, gardenEntry));
        run(startIntake(intake, intakeServoLeft, intakeServoRight));
        drive(Paths.line(gardenEntry, gardenEnd).constant(gardenEntry),
                AllianceAuto.SWEEP_TIMEOUT_MS);
        run(stopIntake(intake, intakeServoLeft, intakeServoRight));

        mark("returning to shoot");
        // Straight up off the audience wall before turning: turning needs 10.9 in of room and
        // the sweep ends with the chassis's centre 8 in off it.
        drive(Paths.line(gardenEnd, clearOfWall).constant(gardenEnd));
        drive(Paths.curve(clearOfWall, alliance.of(50, 28, 0), shoot)
                .linear(clearOfWall, shoot));
        arrive(true, shoot);
    }

    /** From the front shooting spot: the alliance-wall FLOWER's retrieval opening, and back. */
    private void collectFromFlower() {
        Pose atFlower = pose(alliance, AT_FLOWER);
        Pose shoot = pose(alliance, SHOOT);

        mark("collecting from the FLOWER");
        drive(Paths.curve(here, alliance.of(44, 26, 0), alliance.of(30, 44, 0), atFlower)
                .linear(here, atFlower));
        run(startIntake(intake, intakeServoLeft, intakeServoRight));
        run(waitMs(FLOWER_DWELL_MS));
        run(stopIntake(intake, intakeServoLeft, intakeServoRight));

        mark("returning to shoot");
        drive(Paths.curve(atFlower, alliance.of(30, 44, 0), alliance.of(44, 26, 0), shoot)
                .linear(atFlower, shoot));
        arrive(true, shoot);
    }

    /** From whichever shooting spot the robot is at, round the HIVE to the other one. */
    private void driveToOtherSide() {
        Pose rearClear = pose(alliance, REAR_CLEAR);
        if (atFront) {
            Pose rearSpot = pose(alliance, REAR_START);
            Pose turned = alliance.of(26, 44, 180);
            // Up off the wall and over to the alliance-wall side, turning to face it...
            drive(Paths.curve(here, alliance.of(56, 28, 0), alliance.of(34, 34, 0), turned)
                    .linear(here, turned));
            // ...up that side at x 26, turning to face the rear wall...
            drive(Paths.curve(turned, alliance.of(26, 100, 0), rearClear)
                    .linear(turned, rearClear), AROUND_HIVE_TIMEOUT_MS);
            // ...and straight back onto it.
            drive(ontoRearWall(rearClear, rearSpot));
            arrive(false, rearSpot);
        } else {
            Pose shoot = pose(alliance, SHOOT);
            drive(leaveRearWall(rearClear));
            drive(Paths.curve(rearClear, alliance.of(26, 100, 0), alliance.of(26, 30, 0), shoot)
                    .linear(rearClear, shoot), AROUND_HIVE_TIMEOUT_MS);
            arrive(true, shoot);
        }
    }

    private void park() {
        mark("parking in the LOADING ZONE");
        Pose park = pose(alliance, AllianceAuto.PARK_APPROACH);
        if (atFront) {
            drive(Paths.curve(here, alliance.of(30, 30, 0), alliance.of(20, 90, 0), park)
                    .constant(here), AllianceAuto.PARK_TIMEOUT_MS);
        } else {
            drive(Paths.curve(here, alliance.of(62, 120, 0), alliance.of(30, 112, 0), park)
                    .constant(here), AllianceAuto.PARK_TIMEOUT_MS);
        }
        run(AllianceAuto.turnTo(follower, pose(alliance, PARK)));
    }

    /** Straight off the rear wall first, so the turn happens away from it and the FLOWER. */
    private Path leaveRearWall(Pose rearClear) {
        return Paths.curve(here, alliance.of(62, 124, 0), rearClear).constant(here);
    }

    private Path ontoRearWall(Pose rearClear, Pose rearSpot) {
        return Paths.curve(rearClear, alliance.of(62, 124, 0), rearSpot).constant(rearSpot);
    }

    private void arrive(boolean front, Pose spot) {
        atFront = front;
        here = spot;
    }

    /** Pressed against the wall behind the side the robot is on. */
    private Pose wallShot() {
        return pose(alliance, atFront ? FRONT_WALL_SHOT : REAR_WALL_SHOT);
    }

    /** Follows a path to its end, or for {@link AllianceAuto#PATH_TIMEOUT_MS}. */
    private void drive(Path path) {
        drive(path, PATH_TIMEOUT_MS);
    }

    /** Follows a path to its end, or for that many milliseconds if something holds it short. */
    private void drive(Path path, double timeoutMs) {
        run(race(follow(follower, path), waitMs(timeoutMs)));
    }

    /** Runs one command to its end, keeping the follower and the scheduler turning. */
    private void run(Command command) {
        boolean[] done = {false};
        schedule(sequential(command, instant(() -> done[0] = true)));
        while (opModeIsActive() && !done[0]) {
            follower.update();
            Scheduler.execute();
            report();
            idle();
        }
    }

    /** Hands the newest camera frame, if there is one, to the watch. */
    private void look() {
        if (tags == null) {
            return;
        }
        List<AprilTagDetection> fresh = tags.getFreshDetections();
        if (fresh != null) {
            watch.frame(now(), CellAimer.raisedCell(fresh, red, mount) != null);
        }
    }

    private VisionPortal openCamera() {
        WebcamName webcam = hardwareMap.tryGet(WebcamName.class, iamyou.WEBCAM_NAME);
        if (webcam == null) {
            return null;
        }
        tags = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();
        return new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(tags)
                .enableLiveView(false)
                .build();
    }

    private void closeCamera() {
        if (camera != null) {
            camera.close();
        }
    }

    /**
     * The side the raised CELL faces, given a reading taken from where the robot stands.
     *
     * @param otherwise what to believe if the camera has nothing to say
     */
    private boolean raisedFrontFrom(RaisedCellWatch.Reading reading, boolean otherwise) {
        switch (reading) {
            case FACING:
                return atFront;
            case BEHIND:
                return !atFront;
            default:
                return otherwise;
        }
    }

    /**
     * At the start of a match the raised CELL faces the front for both alliances: red's audience
     * CELL and blue's rear one, which is the same spot turned half a turn.
     */
    private static boolean defaultRaisedFront() {
        return true;
    }

    private String describe(RaisedCellWatch.Reading reading) {
        String hive = red ? "RED" : "BLUE";
        switch (reading) {
            case FACING:
                return hive + " raised CELL in view (" + watch.framesWithCell() + " frames)";
            case BEHIND:
                return "no " + hive + " raised CELL in view: it faces the other way";
            case LOOKING:
                return "looking";
            default:
                return "no camera frames: assuming the match-start arrangement";
        }
    }

    private String sideName(boolean front) {
        if (front) {
            return red ? "audience side" : "rear side";
        }
        return red ? "rear side" : "audience side";
    }

    private long remainingMs() {
        return AUTO_MS - (long) sinceStart.milliseconds();
    }

    private long now() {
        return (long) clock.milliseconds();
    }

    private void mark(String label) {
        step = label;
        report();
    }

    private void report() {
        telemetry.addData("step", step);
        telemetry.addData("camera", decision);
        telemetry.addData("raised CELL", "faces the %s", sideName(raisedFront));
        if (sinceStart != null) {
            telemetry.addData("time", "%.1f s", sinceStart.seconds());
        }
        Pose pose = follower.pose();
        telemetry.addData("pose", "(%.1f, %.1f) %.0f", pose.x(), pose.y(),
                Math.toDegrees(pose.heading()));
        telemetry.update();
    }
}
