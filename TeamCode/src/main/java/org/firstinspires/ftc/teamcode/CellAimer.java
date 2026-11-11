package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Turns the robot onto its alliance's raised CELL, using the AprilTag cluster hanging under it.
 *
 * <p>Two ways in. {@link #update()} reads the camera and says how hard to turn, and touches no
 * motor, so a TeleOp can mix the turn into its own drive and leave the driver the strafe.
 * {@link #aim} is the same loop run to completion, turning in place, for an AUTO.</p>
 *
 * <h2>Which way the robot faces when it is aimed</h2>
 *
 * <p>At the CELL from the robot's centre. The launcher sits on the centre line and the robot
 * turns about its centre, so a CELL dead ahead of the centre is dead ahead of the muzzle too
 * &mdash; there is no allowance for the muzzle's reach to make.</p>
 *
 * <h2>Where it can see from</h2>
 *
 * <p>The camera looks forward and 45&deg; up (see {@link LaunchGeometry}), so the cluster is in
 * view from about 0.46&nbsp;m to about 2&nbsp;m short of the opening, which covers the
 * launcher's whole range: a robot standing where it can aim can also shoot.</p>
 *
 * <p>The alliance is fixed when this is built, so an OpMode that picks it during INIT builds
 * this once the pick is final.</p>
 */
public class CellAimer {

    /**
     * How hard it turns, in power per radian of error, and the least and most it bothers with.
     *
     * <p>Proportional only: the error is measured afresh from the tags every frame, so a
     * steady-state offset cannot accumulate unseen. The floor is what a mecanum drive needs to
     * turn at all; without it the aimer stalls a few degrees short and never settles.</p>
     */
    public static final double TURN_GAIN_PER_RADIAN = 1.2;
    public static final double TURN_FLOOR = 0.08;
    public static final double TURN_CEILING = 0.35;

    /**
     * Within this of the CELL, in degrees, the robot is pointed.
     *
     * <p>Two degrees at a metre is 3.5&nbsp;cm of sideways miss against an opening 50&nbsp;cm
     * wide, and about what the tag solve wanders by frame to frame.</p>
     */
    public static final double READY_DEGREES = 2.0;

    /** Pointed for this many camera frames running is settled: one frame can be noise. */
    public static final int SETTLED_FRAMES = 3;

    /**
     * {@link #aim} gives up after this many camera frames running without the CELL.
     *
     * <p>Half a second at the webcam's 30 frames a second. Counted in frames rather than timed,
     * because what it guards against is the CELL being out of view, and a camera that delivers
     * frames slowly has not looked for it any less often.</p>
     */
    public static final int LOST_FRAMES = 15;

    private final AprilTagProcessor tags;
    private final DriveHardware drive;
    private final boolean red;
    private final LensMount mount = LaunchGeometry.camera();

    private int pointedFrames;
    private int framesWithoutCell;
    private Aim latest = Aim.none();

    /**
     * @param tags  the processor the camera's frames go through; this consumes its fresh
     *              detections, so nothing else should read {@code getFreshDetections()} from it
     * @param drive the chassis {@link #aim} turns; {@link #update()} never touches it
     * @param red   which alliance's CELLs to aim at
     */
    public CellAimer(AprilTagProcessor tags, DriveHardware drive, boolean red) {
        this.tags = tags;
        this.drive = drive;
        this.red = red;
    }

    /** Forgets how long the robot has been pointed and how long since it saw the CELL. */
    public void reset() {
        pointedFrames = 0;
        framesWithoutCell = 0;
        latest = Aim.none();
    }

    /**
     * What to do this loop: how hard to turn, and whether the robot is on the CELL.
     *
     * <p>Counts camera frames rather than calls, because a loop runs several times per frame and
     * three calls on one frame are one measurement. Between frames it returns the last answer.</p>
     */
    public Aim update() {
        List<AprilTagDetection> fresh = tags.getFreshDetections();
        if (fresh == null) {
            return latest;
        }

        LensMount.Sighting cell = raisedCell(fresh, red, mount);
        if (cell == null) {
            pointedFrames = 0;
            framesWithoutCell++;
            latest = Aim.none();
            return latest;
        }

        framesWithoutCell = 0;
        double error = errorRadians(cell.forwardMetres(), cell.leftMetres());
        boolean pointed = Math.abs(Math.toDegrees(error)) < READY_DEGREES;
        pointedFrames = pointed ? pointedFrames + 1 : 0;
        latest = new Aim(true, pointed ? 0.0 : turnPower(error), error,
                Math.hypot(cell.forwardMetres(), cell.leftMetres()), pointed,
                pointedFrames >= SETTLED_FRAMES);
        return latest;
    }

    /**
     * Turns in place until settled on the CELL.
     *
     * @return true once settled; false on timeout, on STOP, or after {@link #LOST_FRAMES}
     *     frames without the CELL in view. The motors are stopped whichever way it ends.
     */
    public boolean aim(LinearOpMode opMode, double timeoutSeconds) {
        reset();
        ElapsedTime elapsed = new ElapsedTime();
        try {
            while (opMode.opModeIsActive() && elapsed.seconds() < timeoutSeconds) {
                Aim aim = update();
                if (aim.settled()) {
                    return true;
                }
                if (framesWithoutCell >= LOST_FRAMES) {
                    return false;
                }
                double turn = aim.turnPower();
                // Counter-clockwise: left wheels back, right wheels forward.
                drive.setPowers(new ChassisMotors(-turn, turn, -turn, turn));
                opMode.idle();
            }
            return false;
        } finally {
            drive.stop();
        }
    }

    /**
     * The raised CELL of an alliance, in the robot's frame, or null if none is in view.
     *
     * <p>Only cluster detections: on BioBuzz every tag belongs to one, so a lone tag is from
     * somebody's practice field. Raised by height, because a HIVE's two CELLs share a normal and
     * the camera can see both; the higher of two if both somehow pass.</p>
     */
    public static LensMount.Sighting raisedCell(List<AprilTagDetection> detections, boolean red,
                                                LensMount mount) {
        LensMount.Sighting best = null;
        for (AprilTagDetection detection : detections) {
            if (!(detection instanceof AprilTagClusterDetection)) {
                continue;
            }
            String name = ((AprilTagClusterDetection) detection).metadata.name;
            if (name == null || !name.startsWith(red ? "RED" : "BLUE")) {
                continue;
            }
            LensMount.Sighting sighting = mount.sight(
                    detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);
            if (sighting.heightMetres() < LaunchGeometry.RAISED_CELL_HEIGHT_METRES) {
                continue;
            }
            if (best == null || sighting.heightMetres() > best.heightMetres()) {
                best = sighting;
            }
        }
        return best;
    }

    /** How far to turn, in radians, counter-clockwise positive, to face a point on the robot. */
    static double errorRadians(double forwardMetres, double leftMetres) {
        return Math.atan2(leftMetres, forwardMetres);
    }

    /** Turn power for an error, floored so it still moves and capped so it does not spin. */
    static double turnPower(double errorRadians) {
        double power = Math.abs(errorRadians) * TURN_GAIN_PER_RADIAN;
        power = Math.max(TURN_FLOOR, Math.min(TURN_CEILING, power));
        return errorRadians > 0.0 ? power : -power;
    }

    /** One camera frame's verdict. */
    public static final class Aim {

        private final boolean hasTarget;
        private final double turnPower;
        private final double errorRadians;
        private final double rangeMetres;
        private final boolean pointed;
        private final boolean settled;

        private Aim(boolean hasTarget, double turnPower, double errorRadians, double rangeMetres,
                    boolean pointed, boolean settled) {
            this.hasTarget = hasTarget;
            this.turnPower = turnPower;
            this.errorRadians = errorRadians;
            this.rangeMetres = rangeMetres;
            this.pointed = pointed;
            this.settled = settled;
        }

        private static Aim none() {
            return new Aim(false, 0.0, 0.0, 0.0, false, false);
        }

        /** Whether this alliance's raised CELL is in view. */
        public boolean hasTarget() {
            return hasTarget;
        }

        /** Turn power, counter-clockwise positive; zero when pointed or with nothing in view. */
        public double turnPower() {
            return turnPower;
        }

        /** How far off the CELL the nose is, in radians, counter-clockwise positive. */
        public double errorRadians() {
            return errorRadians;
        }

        /** Level distance from the robot's centre to the cluster, in metres. */
        public double rangeMetres() {
            return rangeMetres;
        }

        /** Within {@link #READY_DEGREES} on this frame. */
        public boolean pointed() {
            return pointed;
        }

        /** Pointed for {@link #SETTLED_FRAMES} frames running. */
        public boolean settled() {
            return settled;
        }
    }
}
