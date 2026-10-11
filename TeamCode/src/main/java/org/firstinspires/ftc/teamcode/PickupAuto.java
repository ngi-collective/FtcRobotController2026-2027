package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

/**
 * The autonomous pickup loop: see a ball, chase it, drive over it, and park when time runs short.
 *
 * <p>The routine is the one decision per tick the grilling settled on: own-colour nectar first,
 * then pollen, never the opponent's decoy &mdash; and, because a floor camera sees only a slice of
 * the field, spin in place to scan when nothing is in frame. Detection is {@link BallDetector}
 * (pure Java), drive is PedroPathing's follower, and the chase is vision-as-absolute with the
 * localizer only smoothing between frames.</p>
 *
 * <p>Subclasses pin the alliance colour; the rest of the loop is identical for both.</p>
 */
public abstract class PickupAuto extends LinearOpMode {

    protected static final double INCHES_PER_METRE = 39.3701;

    /** How many seconds before the end of AUTO to stop chasing and go park. */
    private static final double PARK_MARGIN_SECONDS = 8.0;

    /** How long the 30 s AUTO period is, for the park timing. */
    private static final double AUTO_SECONDS = 30.0;

    /** The ball is at the mouth once its forward distance is inside this, in metres. */
    private static final double MOUTH_FORWARD_METRES = 0.24;

    /** The follow-through nudge past the mouth, in metres, that makes a pickup stick. */
    private static final double NUDGE_METRES = 0.12;

    /** The pickup robot's own camera mount, copied from {@code pickup.json}. */
    private static final LensMount MOUNT = LensMount.of(0.16, 0.0, 0.25, 0.0, -10.0, 0.0);

    protected abstract PickupLogic.OwnColor ownColor();

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        BallDetectorProcessor detector = new BallDetectorProcessor();
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(detector)
                .build();

        follower.setStartingPose(startingPose());

        waitForStart();

        while (opModeIsActive()) {
            List<BallDetector.Detection> detections = detector.getDetections();
            BallDetector.Detection target = PickupLogic.selectTarget(detections, ownColor());

            if (secondsRemaining() > PARK_MARGIN_SECONDS) {
                if (target == null) {
                    scan(follower);
                } else {
                    chase(follower, target);
                }
            } else {
                park(follower);
            }

            follower.update();
            telemetry.addData("target", target == null ? "none" : target.kind());
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.update();
        }
        portal.close();
    }

    /** Turns in place so the floor camera sweeps the field looking for a ball. */
    private void scan(Follower follower) {
        follower.setTeleOpDrive(0.0, 0.0, 0.6, false);
    }

    /** Drives toward the chosen ball, re-aiming every tick off the fresh sighting. */
    private void chase(Follower follower, BallDetector.Detection target) {
        LensMount.Sighting sighting = BallDetector.toSighting(target, MOUNT);
        Pose here = follower.getPose();
        double heading = here.getHeading();
        double xMetres = here.getX() / INCHES_PER_METRE;
        double yMetres = here.getY() / INCHES_PER_METRE;

        double targetHeading = heading + Math.atan2(sighting.leftMetres(), sighting.forwardMetres());
        double forward = sighting.forwardMetres();
        // Once the ball is at the mouth, drive a little further so it is gathered, not nudged.
        double travel = forward < MOUTH_FORWARD_METRES ? forward + NUDGE_METRES : forward;

        double targetX = xMetres + travel * Math.cos(targetHeading);
        double targetY = yMetres + travel * Math.sin(targetHeading);
        follower.holdPoint(new Pose(targetX * INCHES_PER_METRE, targetY * INCHES_PER_METRE,
                targetHeading));
    }

    /** Drives back to the alliance loading zone to claim the AUTO park. */
    private void park(Follower follower) {
        Pose park = parkingPose();
        follower.holdPoint(park);
    }

    /** Seconds left in AUTO; never negative. */
    private double secondsRemaining() {
        return Math.max(0.0, AUTO_SECONDS - getRuntime());
    }

    /** Where the robot starts, in PedroPathing's field frame (inches, radians). */
    protected abstract Pose startingPose();

    /** The alliance loading zone, where the robot parks at the end of AUTO. */
    protected abstract Pose parkingPose();
}
