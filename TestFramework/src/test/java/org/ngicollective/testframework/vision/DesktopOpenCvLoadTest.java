package org.ngicollective.testframework.vision;

import org.junit.jupiter.api.Test;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * That an OpenCV Mat can actually be constructed on a plain x86_64 JVM.
 *
 * <p>The FTC SDK's OpenCV natives ship arm64/armeabi only, so the vision code lives behind the
 * plain-JVM seam because a desktop test has no OpenCV to load. This one comes from
 * {@code org.openpnp:opencv}, which *does* bundle an osx/x86_64 build, so it is the proof that the
 * vision pipeline is testable on a developer machine at all.</p>
 */
class DesktopOpenCvLoadTest {

    @Test
    void aMatIsConstructed() {
        Mat mat = new Mat(480, 640, CvType.CV_8UC4);
        assertTrue(mat.rows() == 480 && mat.cols() == 640, "odd shape");
        mat.setTo(new Scalar(3, 7, 11, 255));
        assertTrue(mat.get(0, 0).length == 4, "channel count wrong");
        mat.release();
    }
}
