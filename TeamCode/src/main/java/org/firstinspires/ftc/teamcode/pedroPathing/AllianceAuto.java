package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.IntakeCommands.startIntake;
import static org.firstinspires.ftc.teamcode.IntakeCommands.stopIntake;

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
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.iamyou;
import org.firstinspires.ftc.teamcode.shootCommand;

import java.util.ArrayList;
import java.util.List;

/**
 * The AUTOs all four starts share. {@link Routine#FULL}: shoot the pre-loads, sweep the alliance's
 * GARDEN along its length, shoot again, then PARK in the LOADING ZONE. One collection per shot,
 * and the FLOWER trip that would come next does not fit in 30 s, so it is left out. {@link Routine#PRELOAD_AND_PARK}: shoot the
 * pre-loads and PARK; a Right start waits 15 s first. The OpMode ends when the routine does.
 *
 * <h2>Frame</h2>
 *
 * <p>Pedro inches, which is the FTC field frame moved so its corner is the origin:
 * {@code pedro = field + 72}, heading unchanged. Red is the {@code x < 72} half, the audience is
 * {@code y = 0}. The interior is 141&nbsp;in, not 144 ({@code robot-config/field.json}), so the
 * walls are at 1.5 and 142.5. Every pose below is written for red; blue is the same route turned
 * half a turn about the field's centre, which is the field's own symmetry.</p>
 *
 * <h2>Where things are, for red (manual figure 9-2)</h2>
 *
 * <ul>
 *   <li>The raised CELL opens toward the audience; the lowered one, toward the rear wall.</li>
 *   <li>GARDEN: the audience-wall corner, x 1.5&ndash;24. Its four POLLEN start in a line against
 *       the audience wall from the corner, centres 2.9&nbsp;in off it.</li>
 *   <li>LOADING ZONE: on the red wall, y 96&ndash;120, 11&nbsp;in deep.</li>
 *   <li>FLOWERs: (4, 48.6) on the red wall, retrieval opening facing +x, and (48.6, 140) on the
 *       rear wall. Only the red-wall one is collected from; nothing else is driven near.</li>
 *   <li>HIVE frame: its foot bar runs along x&nbsp;=&nbsp;48 from y&nbsp;52 to 92. Every path keeps
 *       the whole chassis, turning included, out of x&nbsp;47&ndash;97 over that span.</li>
 * </ul>
 *
 * <p>No path puts any corner of the chassis past x&nbsp;=&nbsp;72.</p>
 */
public abstract class AllianceAuto extends LinearOpMode {

    /** Where this OpMode is set down, which decides whether it must first drive to its shot. */
    public enum Start {
        /** Against the audience wall facing the raised CELL: the first shot is from here. */
        FACING_RAISED_CELL,
        /** Against the rear wall facing the lowered CELL: it drives round the HIVE to shoot. */
        BEHIND_LOWERED_CELL
    }

    /** What the OpMode does once it is moving. */
    public enum Routine {
        /** Shoot, sweep the GARDEN, shoot again, PARK. */
        FULL,
        /**
         * Shoot the pre-loads and PARK, nothing else, shooting from where the robot was set down.
         * A Right start first waits {@link #PRELOAD_RIGHT_DELAY_MS}, leaving the field to its
         * partner.
         */
        PRELOAD_AND_PARK
    }

    /** Blue's starts and paths are red's turned half a turn about the field's centre. */
    protected static final PoseFactory RED = PoseFactory.degrees();
    protected static final PoseFactory BLUE = PoseFactory.degrees().mirrorAroundPoint(72.0, 72.0);

    // Red poses. A chassis is 15.75 in long and 15 in wide, so its centre is 7.9 in off a wall it
    // backs onto and 7.5 in off one it is side-on to; turning in place needs 10.9 in of room.

    /** Against the audience wall, facing the raised CELL: RedLeft's start and its first shot. */
    static final double[] FRONT_START = {56.0, 9.5, 90.0};

    /**
     * Where a drive back to the audience-side shot ends: an inch off the wall, so a path can
     * finish without driving into it. The shot itself is taken from {@link #FRONT_WALL_SHOT}.
     */
    static final double[] SHOOT = {56.0, 10.5, 90.0};

    /**
     * Every audience-side shot is taken here: 0.4 in inside the wall, so the follower holds the
     * chassis pressed against it rather than balancing an inch off it. The wall, not the
     * controller, then fixes the range and the heading for the shot.
     */
    static final double[] FRONT_WALL_SHOT = {56.0, 9.0, 90.0};

    /** The same against the rear wall, behind {@link #REAR_START}. */
    static final double[] REAR_WALL_SHOT = {62.0, 135.0, 270.0};

    /** Against the rear wall facing the lowered CELL, clear of the rear-wall FLOWER at x 48.6. */
    static final double[] REAR_START = {62.0, 134.5, 270.0};

    /** Far enough off the rear wall and the FLOWER to turn, and left of the HIVE frame. */
    static final double[] REAR_CLEAR = {40.0, 116.0, 270.0};

    /**
     * Facing the red wall at the GARDEN's open end, side 2 in off the audience wall.
     *
     * <p>The sweep drives forward along the line of POLLEN. The chassis is wider than the sim's
     * intake (15 in against 11), so a centred intake running beside a wall cannot reach a ball's
     * centre: here the line sits under the chassis's left 3 in, and is collected only if the real
     * intake spans that far.</p>
     */
    static final double[] GARDEN_ENTRY = {28.0, 9.5, 180.0};

    /** Driven forward along the line until the bumper is an inch off the red wall. */
    static final double[] GARDEN_END = {10.4, 9.5, 180.0};

    /**
     * Facing the red-wall FLOWER, bumper half an inch off its rings.
     *
     * <p>The intake reaches 0.4&ndash;2.7&nbsp;in past the bumper, which puts it across the
     * retrieval opening at the bottom where the FLOWER's POLLEN rest.</p>
     */
    static final double[] AT_FLOWER = {15.3, 48.6, 180.0};

    /** How long the intake runs at the FLOWER, in milliseconds, to empty its retrieval opening. */
    static final double FLOWER_DWELL_MS = 1500.0;

    /**
     * Where the drive to the LOADING ZONE ends, half the chassis inside it. The robot keeps the
     * heading it shot with (along or against the wall) and only turns in place at the end, a
     * quarter turn: turning while crossing the field slowed the path until it timed out short, and
     * a half turn in place took longer than the AUTO had left.
     */
    static final double[] PARK_APPROACH = {14.0, 108.0, 90.0};

    /**
     * The final PARK: the same spot turned in place, intake facing away from the alliance wall and
     * so from the drivers behind it, so the robot drives straight out at the start of TELEOP.
     * Centre 12.5 in off the wall leaves the 10.9 in a turn in place needs.
     */
    static final double[] PARK = {14.0, 108.0, 0.0};

    /** Longest the turn to {@link #PARK} may take, in milliseconds. */
    static final long TURN_MAX_MS = 2000;

    /** Holds a pose until the heading is within 5 degrees of it, or {@link #TURN_MAX_MS}. */
    static Command turnTo(Follower follower, Pose target) {
        long[] started = {0};
        return Command.build()
                .setStart(() -> {
                    follower.hold(target);
                    started[0] = System.currentTimeMillis();
                })
                .setDone(() -> Math.abs(Math.IEEEremainder(
                        follower.pose().heading() - target.heading(), 2 * Math.PI))
                        < Math.toRadians(5)
                        || System.currentTimeMillis() - started[0] >= TURN_MAX_MS);
    }

    /**
     * Longest any one path may take, in milliseconds: about twice what each takes in the sim.
     *
     * <p>A path ends when the follower reaches the end of it, and a robot set down a few inches
     * off its start square can be held short of that end by a wall forever. Each path gives up
     * after this long so the routine always runs to its end.</p>
     */
    static final double PATH_TIMEOUT_MS = 5000.0;

    /**
     * Longest the GARDEN sweep may take, in milliseconds. It drives 18 in in about 1.3 s, but the
     * line of POLLEN jams between the bumper and the alliance wall and holds the robot about
     * 1.5 in short of the end: under {@link #PATH_TIMEOUT_MS} it then sat pushing for 5 s, which
     * was most of a Right start's margin on 30 s.
     */
    static final double SWEEP_TIMEOUT_MS = 2000.0;

    /**
     * Longest the long leg round the HIVE may take, in milliseconds: it takes about 5 s on its
     * own, and cut short at {@link #PATH_TIMEOUT_MS} it left the robot shooting from wherever it
     * had got to.
     */
    static final double AROUND_HIVE_TIMEOUT_MS = 8000.0;

    /**
     * Longest the drive to the LOADING ZONE may take, in milliseconds. It is the longest path,
     * about 4 s, and a park cut short scores nothing; the end of the AUTO is the real limit.
     */
    static final double PARK_TIMEOUT_MS = 8000.0;

    /** Close enough to a shooting spot to shoot from, in inches. */
    static final double SETTLED_INCHES = 0.75;

    /** How long it must stay that close, in milliseconds, to count as settled. */
    static final long SETTLED_FOR_MS = 250;

    /** Longest it waits to settle on a shooting spot, in milliseconds. */
    static final long SETTLE_MAX_MS = 1500;

    /**
     * How long a Right start waits before it moves on {@link Routine#PRELOAD_AND_PARK}.
     *
     * <p>15 s, 5 s of shot from the start and 3 s to PARK: 23 s of a 30 s AUTO.</p>
     */
    static final double PRELOAD_RIGHT_DELAY_MS = 15000.0;

    static final double METRES_PER_INCH = 0.0254;

    private final Start startSpot;
    private final Routine routine;
    private final Pose start;
    private final Path leaveRearWall;
    private final Path aroundHive;
    private final Path toGarden;
    private final Path sweepGarden;
    private final Path gardenToClear;
    private final Path clearToShot;
    private final Pose firstShot;
    private final boolean firstShotFront;
    private final Pose frontWallShot;
    private final Pose rearWallShot;
    private final Path toLoadingZone;
    private final Pose finalPark;

    private Follower follower;
    private DcMotorEx flywheel;
    private DcMotorEx intake;
    private CRServo staging;
    private CRServo intakeServoLeft;
    private CRServo intakeServoRight;

    private volatile String step = "waiting for START";
    private volatile boolean finished;

    /**
     * @param alliance {@link #RED} or {@link #BLUE}
     * @param startSpot where the robot is set down
     * @param routine what it does
     */
    protected AllianceAuto(PoseFactory alliance, Start startSpot, Routine routine) {
        this.startSpot = startSpot;
        this.routine = routine;
        Pose frontStart = pose(alliance, FRONT_START);
        Pose shoot = pose(alliance, SHOOT);
        Pose rearStart = pose(alliance, REAR_START);
        Pose rearClear = pose(alliance, REAR_CLEAR);
        Pose gardenEntry = pose(alliance, GARDEN_ENTRY);
        Pose gardenEnd = pose(alliance, GARDEN_END);
        Pose clearOfWall = alliance.of(30, 22, 180);
        Pose park = pose(alliance, PARK_APPROACH);
        finalPark = pose(alliance, PARK);

        start = startSpot == Start.FACING_RAISED_CELL ? frontStart : rearStart;
        // A pre-load routine shoots from wherever it was set down.
        firstShot = routine == Routine.PRELOAD_AND_PARK ? start
                : startSpot == Start.FACING_RAISED_CELL ? frontStart : shoot;
        firstShotFront = firstShot != rearStart;
        frontWallShot = pose(alliance, FRONT_WALL_SHOT);
        rearWallShot = pose(alliance, REAR_WALL_SHOT);
        // A pre-load routine shoots from where it was set down and parks from there.
        Pose lastShot = routine == Routine.FULL ? shoot : start;

        // Straight off the rear wall first, so the turn happens away from it and the FLOWER.
        leaveRearWall = Paths.curve(rearStart, alliance.of(62, 124, 0), rearClear)
                .constant(rearStart);
        // Down the red wall side at x 26, well left of the frame, turning to face the HIVE.
        aroundHive = Paths.curve(rearClear, alliance.of(26, 100, 0), alliance.of(26, 30, 0), shoot)
                .linear(rearClear, shoot);
        // Up off the wall to turn round, then back down to it beside the line of POLLEN.
        toGarden = Paths.curve(firstShot, alliance.of(56, 30, 0), alliance.of(34, 26, 0), gardenEntry)
                .linear(firstShot, gardenEntry);
        sweepGarden = Paths.line(gardenEntry, gardenEnd).constant(gardenEntry);
        // Straight up off the audience wall before turning: turning needs 10.9 in of room and the
        // sweep ends with the chassis's centre 8 in off it.
        gardenToClear = Paths.line(gardenEnd, clearOfWall).constant(gardenEnd);
        clearToShot = Paths.curve(clearOfWall, alliance.of(50, 28, 0), shoot)
                .linear(clearOfWall, shoot);
        // Out to x 30 before climbing past the FLOWER and the HIVE frame. From the rear wall:
        // straight off it first, then across behind the frame to the LOADING ZONE.
        toLoadingZone = lastShot == rearStart
                ? Paths.curve(rearStart, alliance.of(62, 120, 0), alliance.of(30, 112, 0), park)
                        .constant(rearStart)
                : Paths.curve(lastShot, alliance.of(30, 30, 0), alliance.of(20, 90, 0), park)
                        .constant(lastShot);
    }

    static Pose pose(PoseFactory alliance, double[] red) {
        return alliance.of(red[0], red[1], red[2]);
    }

    public Command autoRoutine() {
        List<Command> steps = new ArrayList<>();
        if (routine == Routine.PRELOAD_AND_PARK && startSpot == Start.BEHIND_LOWERED_CELL) {
            steps.add(mark("waiting 15 s"));
            steps.add(waitMs(PRELOAD_RIGHT_DELAY_MS));
        }
        if (routine == Routine.FULL && startSpot == Start.BEHIND_LOWERED_CELL) {
            steps.add(mark("driving round the HIVE"));
            steps.add(drive(leaveRearWall));
            steps.add(drive(aroundHive, AROUND_HIVE_TIMEOUT_MS));
        }
        steps.add(mark("shooting the pre-loads"));
        steps.add(shoot(firstShotFront));
        if (routine == Routine.FULL) {
            addCollectAndShoot(steps);
        }
        steps.add(mark("parking in the LOADING ZONE"));
        steps.add(drive(toLoadingZone, PARK_TIMEOUT_MS));
        steps.add(turnTo(follower, finalPark));
        steps.add(instant(() -> {
            step = "done";
            finished = true;
        }));
        return sequential(steps.toArray(new Command[0]));
    }

    /** The GARDEN sweep and the second shot, ending back at {@link #SHOOT}. */
    private void addCollectAndShoot(List<Command> steps) {
        steps.add(mark("sweeping the GARDEN"));
        steps.add(drive(toGarden));
        steps.add(startIntake(intake, intakeServoLeft, intakeServoRight));
        steps.add(drive(sweepGarden, SWEEP_TIMEOUT_MS));
        steps.add(stopIntake(intake, intakeServoLeft, intakeServoRight));
        steps.add(mark("returning to shoot"));
        steps.add(drive(gardenToClear));
        steps.add(drive(clearToShot));
        steps.add(mark("shooting the GARDEN's POLLEN"));
        steps.add(shoot(true));
    }

    /** Settles pressed against the wall on that side, then shoots. */
    private Command shoot(boolean front) {
        return sequential(settleAt(follower, front ? frontWallShot : rearWallShot),
                new shootCommand(flywheel, staging));
    }

    /**
     * Holds a spot until the robot has stayed within {@link #SETTLED_INCHES} of it for
     * {@link #SETTLED_FOR_MS}, or for {@link #SETTLE_MAX_MS}. A path can end, or time out, a few
     * inches short of its end, and this robot cannot shoot on the move. Staying inside, not just
     * passing through: a robot arriving at speed crosses the spot and overshoots.
     */
    static Command settleAt(Follower follower, Pose spot) {
        long[] started = {0};
        long[] insideSince = {-1};
        return Command.build()
                .setStart(() -> {
                    follower.hold(spot);
                    started[0] = System.currentTimeMillis();
                    insideSince[0] = -1;
                })
                .setDone(() -> {
                    long now = System.currentTimeMillis();
                    boolean inside = Math.hypot(follower.pose().x() - spot.x(),
                            follower.pose().y() - spot.y()) < SETTLED_INCHES;
                    if (!inside) {
                        insideSince[0] = -1;
                    } else if (insideSince[0] < 0) {
                        insideSince[0] = now;
                    }
                    return (inside && now - insideSince[0] >= SETTLED_FOR_MS)
                            || now - started[0] >= SETTLE_MAX_MS;
                });
    }

    /** Follows a path to its end, or for {@link #PATH_TIMEOUT_MS} if something holds it short. */
    private Command drive(Path path) {
        return drive(path, PATH_TIMEOUT_MS);
    }

    private Command drive(Path path, double timeoutMs) {
        return race(follow(follower, path), waitMs(timeoutMs));
    }

    private Command mark(String label) {
        return instant(() -> step = label);
    }

    static DcMotorSimple.Direction direction(boolean reversed) {
        return reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        // hardwareMap is only set once the OpMode has been constructed, so these cannot be
        // field initialisers.
        follower = Constants.createFollower(hardwareMap);
        flywheel = hardwareMap.get(DcMotorEx.class, iamyou.OUTTAKE_NAME);
        staging = hardwareMap.get(CRServo.class, iamyou.STAGING_NAME);
        intake = hardwareMap.get(DcMotorEx.class, iamyou.INTAKE_NAME);
        intakeServoLeft = hardwareMap.get(CRServo.class, iamyou.INTAKE_SERVO_LEFT_NAME);
        intakeServoRight = hardwareMap.get(CRServo.class, iamyou.INTAKE_SERVO_RIGHT_NAME);
        // The same wiring the TeleOp drives, so +1 means the same thing in both: staging feeds the
        // flywheel, the flywheel launches, the intake pulls in. Left at FORWARD, staging ran away
        // from the flywheel through every shot.
        flywheel.setDirection(direction(iamyou.OUTTAKE_REVERSED));
        shootCommand.configure(flywheel);
        staging.setDirection(direction(iamyou.STAGING_REVERSED));
        intake.setDirection(direction(iamyou.INTAKE_REVERSED));
        follower.setPose(start);
        follower.update();

        telemetry.addData("start on", "x %.1f  y %.1f  heading %.0f (Pedro inches)",
                start.x(), start.y(), Math.toDegrees(start.heading()));
        telemetry.addData("dashboard", "x %.3f  y %.3f  heading %.0f (metres)",
                (start.x() - 72.0) * METRES_PER_INCH, (start.y() - 72.0) * METRES_PER_INCH,
                Math.toDegrees(start.heading()));
        telemetry.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive() && !finished) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("step", step);
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", Math.toDegrees(follower.pose().heading()));
            telemetry.addData("Staging", "%.2f", staging.getPower());
            telemetry.addData("Flywheel", "%.0f ticks/s", flywheel.getVelocity());
            if (follower.currentPath() != null) {
                telemetry.addData("distance remaining", follower.distanceToEndpoint());
            }
            telemetry.update();
            idle();
        }
        // The routine is over: stop the drive rather than leave the last command on the motors.
        follower.manual(0.0, 0.0, 0.0);
        follower.update();
        telemetry.addData("step", step);
        telemetry.update();
    }
}
