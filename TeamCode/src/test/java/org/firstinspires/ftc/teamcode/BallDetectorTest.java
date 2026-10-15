package org.firstinspires.ftc.teamcode;

import org.junit.jupiter.api.Test;
import org.ngicollective.testframework.camera.CameraIntrinsics;
import org.ngicollective.testframework.camera.CameraView;
import org.ngicollective.testframework.camera.GameElement;
import org.ngicollective.testframework.camera.Pixel;
import org.ngicollective.testframework.camera.Pose3d;
import org.ngicollective.testframework.camera.SceneFrameSource;
import org.ngicollective.testframework.camera.SimulatedCamera;
import org.ngicollective.testframework.camera.SimulatedScene;
import org.ngicollective.testframework.camera.SyntheticFrame;
import org.ngicollective.testframework.camera.Vec3;
import org.ngicollective.testframework.sim.Pose2d;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * The detector's contract, proven against ground truth the renderer provides.
 *
 * <p>Every ball here is placed at a known position; the same pinhole model the detector inverts is
 * what draws the ball, so the true depth and bearing come from {@link CameraView} and the
 * assertion is exact. This is a plain-JVM test on purpose: no OpenCV, no emulator, milliseconds.</p>
 */
class BallDetectorTest {

    private static final CameraIntrinsics LENS = CameraIntrinsics.approximate(640, 480);

    /** A ball-hunting camera: 0.25 m up, facing +X level, at the field origin. */
    private static final SimulatedCamera CAMERA =
            new SimulatedCamera("Ball Cam", LENS, Pose3d.facingForward(new Vec3(0.0, 0.0, 0.25)));

    /** How close a computed range has to come to the true depth, in metres. */
    private static final double RANGE_TOLERANCE = 0.03;

    /** How close a computed bearing has to come, in radians. */
    private static final double BEARING_TOLERANCE = 0.02;

    private static List<BallDetector.Detection> detect(GameElement ball) {
        SimulatedScene scene = SimulatedScene.of().withElements(Collections.singletonList(ball));
        return detect(scene);
    }

    private static List<BallDetector.Detection> detect(List<GameElement> balls) {
        return detect(SimulatedScene.of().withElements(balls));
    }

    private static List<BallDetector.Detection> detect(SimulatedScene scene) {
        SyntheticFrame frame = new SyntheticFrame(LENS.width(), LENS.height());
        new SceneFrameSource(scene, CAMERA).render(frame);
        return BallDetector.detect(frame.rgba(), frame.width(), frame.height(),
                LENS.focalX(), LENS.focalY(), LENS.centreX(), LENS.centreY());
    }

    private static double depthOf(GameElement ball) {
        return CAMERA.viewFrom(Pose2d.ORIGIN).depthOf(ball.centre());
    }

    private static double bearingOf(GameElement ball) {
        CameraView view = CAMERA.viewFrom(Pose2d.ORIGIN);
        Pixel projected = view.project(ball.centre());
        return Math.atan2(projected.x() - LENS.centreX(), LENS.focalX());
    }

    @Test
    void aPollenBallIsYellowAndRangesToItsTrueDepth() {
        GameElement pollen = GameElement.pollen(1.0, 0.0);

        List<BallDetector.Detection> found = detect(pollen);
        assertEquals(1, found.size(), "exactly the one ball in frame should be detected");
        assertEquals(BallDetector.Kind.POLLEN, found.get(0).kind());
        assertEquals(depthOf(pollen), found.get(0).rangeMetres(), RANGE_TOLERANCE);
        assertEquals(bearingOf(pollen), found.get(0).bearingRadians(), BEARING_TOLERANCE);
    }

    @Test
    void aRedNectarBallIsRedNectarAtItsTrueRange() {
        GameElement nectar = GameElement.redNectar(1.2, -0.15);

        List<BallDetector.Detection> found = detect(nectar);
        assertEquals(1, found.size());
        assertEquals(BallDetector.Kind.RED_NECTAR, found.get(0).kind());
        assertEquals(depthOf(nectar), found.get(0).rangeMetres(), RANGE_TOLERANCE);
        assertEquals(bearingOf(nectar), found.get(0).bearingRadians(), BEARING_TOLERANCE);
    }

    @Test
    void aBlueNectarBallIsBlueNectarAtItsTrueRange() {
        GameElement nectar = GameElement.blueNectar(0.9, 0.20);

        List<BallDetector.Detection> found = detect(nectar);
        assertEquals(1, found.size());
        assertEquals(BallDetector.Kind.BLUE_NECTAR, found.get(0).kind());
        assertEquals(depthOf(nectar), found.get(0).rangeMetres(), RANGE_TOLERANCE);
        assertEquals(bearingOf(nectar), found.get(0).bearingRadians(), BEARING_TOLERANCE);
    }

    @Test
    void threeBallsAreEachClassifiedAndRanged() {
        GameElement pollen = GameElement.pollen(0.8, -0.25);
        GameElement red = GameElement.redNectar(1.3, 0.0);
        GameElement blue = GameElement.blueNectar(1.1, 0.30);
        List<GameElement> balls = Arrays.asList(pollen, red, blue);

        List<BallDetector.Detection> found = detect(balls);
        assertEquals(3, found.size(), "three balls in frame, three detections");
        for (GameElement ball : balls) {
            BallDetector.Detection nearest = nearestTo(found, ball);
            assertEquals(expectedKind(ball), nearest.kind());
            assertEquals(depthOf(ball), nearest.rangeMetres(), RANGE_TOLERANCE);
            assertEquals(bearingOf(ball), nearest.bearingRadians(), BEARING_TOLERANCE);
        }
    }

    @Test
    void aBallConvertsToARobotFrameSighting() {
        // The pickup robot's own mount, copied from pickup.json: 0.16 m ahead, 0.25 m up, pitched
        // 10 degrees down at the floor ahead of the mouth.
        SimulatedCamera camera = new SimulatedCamera("Ball Cam", LENS,
                Pose3d.ofDegrees(new Vec3(0.16, 0.0, 0.25), 0.0, -10.0, 0.0));
        GameElement ball = GameElement.pollen(1.0, 0.20);

        SimulatedScene scene = SimulatedScene.of().withElements(Collections.singletonList(ball));
        SyntheticFrame frame = new SyntheticFrame(LENS.width(), LENS.height());
        new SceneFrameSource(scene, camera).render(frame);
        List<BallDetector.Detection> found = BallDetector.detect(frame.rgba(), frame.width(),
                frame.height(), LENS.focalX(), LENS.focalY(), LENS.centreX(), LENS.centreY());

        assertEquals(1, found.size());
        LensMount mount = LensMount.of(0.16, 0.0, 0.25, 0.0, -10.0, 0.0);
        LensMount.Sighting sighting = BallDetector.toSighting(found.get(0), mount);

        // The robot is at the origin facing +X, so the ball's robot-frame place is its field place.
        assertEquals(1.0, sighting.forwardMetres(), RANGE_TOLERANCE);
        assertEquals(0.20, sighting.leftMetres(), RANGE_TOLERANCE);
        assertEquals(ball.centre().z(), sighting.heightMetres(), RANGE_TOLERANCE);
    }

    private static BallDetector.Detection nearestTo(List<BallDetector.Detection> detections,
                                                    GameElement ball) {
        double depth = depthOf(ball);
        double bearing = bearingOf(ball);
        BallDetector.Detection best = null;
        double bestDistance = Double.MAX_VALUE;
        for (BallDetector.Detection detection : detections) {
            double distance = Math.hypot(detection.rangeMetres() - depth,
                    detection.bearingRadians() - bearing);
            if (distance < bestDistance) {
                bestDistance = distance;
                best = detection;
            }
        }
        return best;
    }

    private static BallDetector.Kind expectedKind(GameElement ball) {
        if (ball.name().equals("POLLEN")) {
            return BallDetector.Kind.POLLEN;
        }
        if (ball.name().equals("RED NECTAR")) {
            return BallDetector.Kind.RED_NECTAR;
        }
        if (ball.name().equals("BLUE NECTAR")) {
            return BallDetector.Kind.BLUE_NECTAR;
        }
        throw new IllegalArgumentException("unexpected ball " + ball.name());
    }
}
