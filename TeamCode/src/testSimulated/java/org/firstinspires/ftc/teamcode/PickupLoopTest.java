package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.simulated.VerityRobot;
import org.junit.jupiter.api.Test;
import org.ngicollective.ftcsim.camera.CameraIntrinsics;
import org.ngicollective.ftcsim.camera.GameElement;
import org.ngicollective.ftcsim.camera.Pose3d;
import org.ngicollective.ftcsim.camera.SceneFrameSource;
import org.ngicollective.ftcsim.camera.SimulatedCamera;
import org.ngicollective.ftcsim.camera.SimulatedScene;
import org.ngicollective.ftcsim.camera.SyntheticFrame;
import org.ngicollective.ftcsim.camera.Vec3;
import org.ngicollective.ftcsim.hardware.FakeHardwareMap;
import org.ngicollective.ftcsim.physics.FieldPhysics;
import org.ngicollective.ftcsim.season.BioBuzzElements;
import org.ngicollective.ftcsim.sim.FieldConfig;
import org.ngicollective.ftcsim.sim.Pose2d;
import org.ngicollective.ftcsim.sim.RobotConfig;
import org.ngicollective.ftcsim.sim.SimConfigFiles;

import java.util.Collections;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The whole pickup loop, end to end on a plain JVM: detect a staged ball, choose it, chase it with
 * PedroPathing's follower re-aimed off a fresh sighting every tick, and drive over it until the
 * mouth collects it.
 *
 * <p>This is the loop {@link PickupAuto} runs, minus the EasyOpenCV glue that cannot live on a
 * plain JVM. The pieces &mdash; {@link BallDetector}, {@link PickupLogic}, the follower and the
 * capture &mdash; are all here and all real.</p>
 */
class PickupLoopTest {

    private static final double TICK = 0.02;

    private static final double INCHES_PER_METRE = 39.3701;

    private static final CameraIntrinsics LENS = CameraIntrinsics.approximate(640, 480);

    private static final SimulatedCamera CAMERA = new SimulatedCamera("Webcam 1", LENS,
            Pose3d.ofDegrees(new Vec3(0.16, 0.0, 0.25), 0.0, -10.0, 0.0));

    private static final LensMount MOUNT = LensMount.of(0.16, 0.0, 0.25, 0.0, -10.0, 0.0);

    @Test
    void thePickupLoopDrivesOverAndCollectsAStagedBall() {
        RobotConfig config = SimConfigFiles.robot("pickup");
        GameElement ball = BioBuzzElements.pollen(0.8, 0.0);
        SimulatedScene scene = SimulatedScene.of().withElements(Collections.singletonList(ball));

        FakeHardwareMap hardware = new VerityRobot(config, FieldConfig.standard(), scene).create();
        FieldPhysics world = FieldPhysics.of(scene.elements(), scene.structures(),
                FieldConfig.standard(), config);
        hardware.setPhysics(world);
        hardware.drive().setPose(Pose2d.ORIGIN);

        SceneFrameSource frames = new SceneFrameSource(scene, CAMERA,
                new SceneFrameSource.PoseSource() {
                    @Override
                    public Pose2d pose() {
                        return hardware.drive().pose();
                    }
                });
        Follower follower = Constants.createFollower(hardware);
        follower.setStartingPose(new Pose(0.0, 0.0, 0.0));
        follower.update();

        SyntheticFrame frame = new SyntheticFrame(LENS.width(), LENS.height());
        for (int tick = 0; tick < 800 && !world.elements().isEmpty(); tick++) {
            frames.render(frame);
            List<BallDetector.Detection> detections = BallDetector.detect(
                    frame.rgba(), frame.width(), frame.height(),
                    LENS.focalX(), LENS.focalY(), LENS.centreX(), LENS.centreY());
            BallDetector.Detection target = PickupLogic.selectTarget(detections,
                    PickupLogic.OwnColor.RED);
            if (target != null) {
                chase(follower, target);
            }
            follower.update();
            hardware.advance(TICK);
        }

        assertTrue(world.elements().isEmpty(),
                "the loop should drive over the ball and the mouth should collect it");
    }

    /** The same re-aiming {@link PickupAuto#chase} does: a fresh sighting each tick. */
    private void chase(Follower follower, BallDetector.Detection target) {
        LensMount.Sighting sighting = BallDetector.toSighting(target, MOUNT);
        Pose here = follower.getPose();
        double heading = here.getHeading();
        double xMetres = here.getX() / INCHES_PER_METRE;
        double yMetres = here.getY() / INCHES_PER_METRE;
        double targetHeading = heading + Math.atan2(sighting.leftMetres(), sighting.forwardMetres());
        double targetX = xMetres + sighting.forwardMetres() * Math.cos(targetHeading);
        double targetY = yMetres + sighting.forwardMetres() * Math.sin(targetHeading);
        follower.holdPoint(new Pose(targetX * INCHES_PER_METRE, targetY * INCHES_PER_METRE,
                targetHeading));
    }
}
