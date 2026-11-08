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

import org.firstinspires.ftc.teamcode.shootCommand;

import java.util.ArrayList;
import java.util.List;

/**
 * The AUTO all four starts share: shoot the pre-loads, sweep the alliance's GARDEN, shoot again,
 * then finish beside the alliance-wall FLOWER (Left) or in the LOADING ZONE (Right). The OpMode
 * ends when the routine does.
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
 *       the audience wall from the corner. It is the only place on red's half a robot may collect
 *       from: the FLOWERs' POLLEN are not for taking, and there are no other lines of balls.</li>
 *   <li>LOADING ZONE: on the red wall, y 96&ndash;120, 11&nbsp;in deep.</li>
 *   <li>FLOWERs: (4, 48.6) on the red wall and (48.6, 140) on the rear wall.</li>
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

    /** Blue's starts and paths are red's turned half a turn about the field's centre. */
    protected static final PoseFactory RED = PoseFactory.degrees();
    protected static final PoseFactory BLUE = PoseFactory.degrees().mirrorAroundPoint(72.0, 72.0);

    // Red poses. A chassis is 15.75 in long and 15 in wide, so its centre is 7.9 in off a wall it
    // backs onto and 7.5 in off one it is side-on to; turning in place needs 10.9 in of room.

    /** Against the audience wall, facing the raised CELL: RedLeft's start and its first shot. */
    private static final double[] FRONT_START = {56.0, 9.5, 90.0};

    /**
     * Every other red shot: the start, an inch off the wall.
     *
     * <p>Arriving at the start itself leaves the follower holding the chassis against the wall,
     * rattling on it through the five-second shot. One inch of range costs nothing on a lob.</p>
     */
    private static final double[] SHOOT = {56.0, 10.5, 90.0};

    /** Against the rear wall facing the lowered CELL, clear of the rear-wall FLOWER at x 48.6. */
    private static final double[] REAR_START = {62.0, 134.5, 270.0};

    /** Far enough off the rear wall and the FLOWER to turn, and left of the HIVE frame. */
    private static final double[] REAR_CLEAR = {40.0, 116.0, 270.0};

    /**
     * Facing the audience wall at the GARDEN's open end, intake over the line of POLLEN.
     *
     * <p>A ball against the wall has its centre 2.9 in from it. Facing the wall from 13.2 in puts
     * that centre 10.3 in ahead of the robot's, inside the intake, with the bumper an inch clear
     * of the ball so the line is collected rather than pushed.</p>
     */
    private static final double[] GARDEN_ENTRY = {26.0, 13.2, 270.0};

    /** The same, slid along the line until the robot's side is 3.5 in off the red wall. */
    private static final double[] GARDEN_END = {11.0, 13.2, 270.0};

    /** A Left start finishes here: facing the red-wall FLOWER, bumper an inch short of its rings. */
    private static final double[] AT_FLOWER = {15.8, 48.6, 180.0};

    /** A Right start finishes here: half the chassis inside the LOADING ZONE, facing the red wall. */
    private static final double[] PARK = {14.0, 108.0, 180.0};

    /**
     * Longest any one path may take, in milliseconds: about twice what each takes in the sim.
     *
     * <p>A path ends when the follower reaches the end of it, and a robot set down a few inches
     * off its start square can be held short of that end by a wall forever. Each path gives up
     * after this long so the routine always runs to its end.</p>
     */
    private static final double PATH_TIMEOUT_MS = 5000.0;

    private static final double METRES_PER_INCH = 0.0254;

    private final Start startSpot;
    private final Pose start;
    private final Path leaveRearWall;
    private final Path aroundHive;
    private final Path toGarden;
    private final Path sweepGarden;
    private final Path gardenToShot;
    private final Path toFinish;

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
     */
    protected AllianceAuto(PoseFactory alliance, Start startSpot) {
        this.startSpot = startSpot;
        Pose frontStart = pose(alliance, FRONT_START);
        Pose shoot = pose(alliance, SHOOT);
        Pose rearStart = pose(alliance, REAR_START);
        Pose rearClear = pose(alliance, REAR_CLEAR);
        Pose gardenEntry = pose(alliance, GARDEN_ENTRY);
        Pose gardenEnd = pose(alliance, GARDEN_END);
        Pose finish = pose(alliance, startSpot == Start.FACING_RAISED_CELL ? AT_FLOWER : PARK);

        start = startSpot == Start.FACING_RAISED_CELL ? frontStart : rearStart;
        Pose firstShot = startSpot == Start.FACING_RAISED_CELL ? frontStart : shoot;

        // Straight off the rear wall first, so the turn happens away from it and the FLOWER.
        leaveRearWall = Paths.curve(rearStart, alliance.of(62, 124, 0), rearClear)
                .constant(rearStart);
        // Down the red wall side at x 26, well left of the frame, turning to face the HIVE.
        aroundHive = Paths.curve(rearClear, alliance.of(26, 100, 0), alliance.of(26, 30, 0), shoot)
                .linear(rearClear, shoot);
        // Up off the wall before turning round, or the corners hit it.
        toGarden = Paths.curve(firstShot, alliance.of(56, 30, 0), alliance.of(26, 30, 0), gardenEntry)
                .linear(firstShot, gardenEntry);
        sweepGarden = Paths.line(gardenEntry, gardenEnd).constant(gardenEntry);
        gardenToShot = Paths.curve(gardenEnd, alliance.of(30, 30, 0), alliance.of(56, 30, 0), shoot)
                .linear(gardenEnd, shoot);
        // Out to x 30 before climbing past the HIVE frame, and for a Right start past the FLOWER.
        toFinish = startSpot == Start.FACING_RAISED_CELL
                ? Paths.curve(shoot, alliance.of(30, 30, 0), alliance.of(28, 48.6, 0), finish)
                        .linear(shoot, finish)
                : Paths.curve(shoot, alliance.of(30, 30, 0), alliance.of(20, 90, 0), finish)
                        .linear(shoot, finish);
    }

    private static Pose pose(PoseFactory alliance, double[] red) {
        return alliance.of(red[0], red[1], red[2]);
    }

    public Command autoRoutine() {
        List<Command> steps = new ArrayList<>();
        if (startSpot == Start.BEHIND_LOWERED_CELL) {
            steps.add(mark("driving round the HIVE"));
            steps.add(drive(leaveRearWall));
            steps.add(drive(aroundHive));
        }
        steps.add(mark("shooting the pre-loads"));
        steps.add(new shootCommand(flywheel, staging));
        steps.add(mark("sweeping the GARDEN"));
        steps.add(startIntake(intake, intakeServoLeft, intakeServoRight));
        steps.add(drive(toGarden));
        steps.add(drive(sweepGarden));
        steps.add(stopIntake(intake, intakeServoLeft, intakeServoRight));
        steps.add(mark("returning to shoot"));
        steps.add(drive(gardenToShot));
        steps.add(mark("shooting the GARDEN's POLLEN"));
        steps.add(new shootCommand(flywheel, staging));
        steps.add(mark(startSpot == Start.FACING_RAISED_CELL
                ? "driving to the FLOWER" : "parking in the LOADING ZONE"));
        steps.add(drive(toFinish));
        steps.add(instant(() -> {
            step = "done";
            finished = true;
        }));
        return sequential(steps.toArray(new Command[0]));
    }

    /** Follows a path to its end, or for {@link #PATH_TIMEOUT_MS} if something holds it short. */
    private Command drive(Path path) {
        return race(follow(follower, path), waitMs(PATH_TIMEOUT_MS));
    }

    private Command mark(String label) {
        return instant(() -> step = label);
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        // hardwareMap is only set once the OpMode has been constructed, so these cannot be
        // field initialisers.
        follower = Constants.createFollower(hardwareMap);
        flywheel = hardwareMap.get(DcMotorEx.class, "Outtake");
        staging = hardwareMap.get(CRServo.class, "Staging");
        intake = hardwareMap.get(DcMotorEx.class, "IntakeM");
        intakeServoLeft = hardwareMap.get(CRServo.class, "IntakeSL");
        intakeServoRight = hardwareMap.get(CRServo.class, "IntakeSR");
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
            if (follower.currentPath() != null) {
                telemetry.addData("distance remaining", follower.distanceToEndpoint());
            }
            telemetry.update();
        }
        // The routine is over: stop the drive rather than leave the last command on the motors.
        follower.manual(0.0, 0.0, 0.0);
        follower.update();
        telemetry.addData("step", step);
        telemetry.update();
    }
}
