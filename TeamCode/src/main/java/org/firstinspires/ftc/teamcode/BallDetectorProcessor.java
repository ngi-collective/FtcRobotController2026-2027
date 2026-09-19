package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

import java.util.Collections;
import java.util.List;

/**
 * The EasyOpenCV seam that feeds camera frames into {@link BallDetector}.
 *
 * <p>Thin on purpose: the OpenCV work here is a copy of the RGBA frame out of the {@code Mat} and
 * nothing more. Detection, classification and ranging live in {@link BallDetector}, which is pure
 * Java, so the whole pipeline is exercised on a plain JVM by {@code BallDetectorTest} and this
 * class is the only part that ever touches a device.</p>
 */
public final class BallDetectorProcessor implements VisionProcessor {

    private double focalX;
    private double focalY;
    private double centreX;
    private double centreY;
    private byte[] rgba;

    private volatile List<BallDetector.Detection> detections = Collections.emptyList();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.focalX = calibration.focalLengthX;
        this.focalY = calibration.focalLengthY;
        this.centreX = calibration.principalPointX;
        this.centreY = calibration.principalPointY;
        this.rgba = new byte[width * height * 4];
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // The SDK hands processors RGBA (CV_8UC4); four bytes per pixel, row-major, exactly the
        // layout SyntheticFrame renders and BallDetector consumes.
        frame.get(0, 0, rgba);
        detections = BallDetector.detect(rgba, frame.cols(), frame.rows(),
                focalX, focalY, centreX, centreY);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Nothing to draw: the detections are the output, not an overlay.
    }

    /** Every ball in the most recent frame, in the camera's frame. */
    public List<BallDetector.Detection> getDetections() {
        return detections;
    }
}
