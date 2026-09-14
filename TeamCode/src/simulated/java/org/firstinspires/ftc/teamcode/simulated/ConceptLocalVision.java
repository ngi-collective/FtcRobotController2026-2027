package org.firstinspires.ftc.teamcode.simulated;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.ngicollective.testframework.vision.LocalVisionHost;
import org.opencv.core.RotatedRect;

import java.util.List;

/**
 * Runs the SDK's real vision processors against the development machine's webcam.
 *
 * <p>This OpMode only exists in the {@code simulated} build, and it is the one place this
 * framework asks an OpMode to be written differently: it builds a {@link LocalVisionHost} where a
 * competition OpMode builds a {@code VisionPortal}. The reason is structural rather than a
 * shortcut &mdash; {@code VisionPortal} finds a camera by USB enumeration, and an emulator hands
 * the host's webcam to the guest through Camera2 without ever creating a USB device, so
 * {@code WebcamName} lookup cannot succeed there no matter how the AVD is configured.</p>
 *
 * <p>Everything below the camera is the real thing: {@link AprilTagProcessor} and
 * {@link ColorBlobLocatorProcessor} are the same unmodified SDK classes a competition OpMode
 * uses, so tuning a color range or a tag family here transfers directly.</p>
 *
 * <p>Requires an emulator whose AVD was built from a camera-capable system image, with
 * {@code hw.camera.back} set to a host webcam &mdash; see {@code mise.toml}.</p>
 */
@TeleOp(name = "Concept: Local Vision (webcam)", group = "Simulated")
public class ConceptLocalVision extends LinearOpMode {

    /** Matches the sample OpModes: small frames keep latency down and detection is unaffected. */
    private static final Size RESOLUTION = new Size(640, 480);

    @Override
    public void runOpMode() {
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .build();

        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        LocalVisionHost vision = LocalVisionHost.builder()
                .addProcessor(aprilTag)
                .addProcessor(colorLocator)
                .setCameraResolution(RESOLUTION)
                .build();

        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        try {
            // Init as well as active, so the camera can be aimed before the match starts.
            while (opModeIsActive() || opModeInInit()) {
                telemetry.addData("camera", "%s @ %.1f fps, %d frames",
                        vision.getCameraResolution(), vision.getFps(), vision.getFrameCount());

                // BioBuzz puts every tag in a four-member cluster, so a cluster detection is the
                // normal case and a single tag is the exception. A cluster reports one pose for
                // the whole sticker, resolves even when members are occluded, and aims at the
                // CELL opening rather than at any one tag face.
                List<AprilTagDetection> detections = aprilTag.getDetections();
                telemetry.addData("april tags", detections.size());
                for (AprilTagDetection detection : detections) {
                    telemetry.addLine("  " + describe(detection) + "  " + describePose(detection));
                }

                List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
                ColorBlobLocatorProcessor.Util.filterByCriteria(
                        ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 50, 20000, blobs);
                telemetry.addData("blue blobs", blobs.size());
                for (ColorBlobLocatorProcessor.Blob blob : blobs) {
                    RotatedRect boxFit = blob.getBoxFit();
                    telemetry.addLine(String.format("  (%3d,%3d) area %5d",
                            (int) boxFit.center.x, (int) boxFit.center.y, blob.getContourArea()));
                }

                telemetry.update();
                sleep(100);
            }
        } finally {
            vision.close();
        }
    }

    /** What was seen: a named cluster and how much of it resolved, or a lone tag's ID. */
    private static String describe(AprilTagDetection detection) {
        if (detection instanceof AprilTagClusterDetection) {
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            return String.format("%-14s %3d%% found",
                    cluster.metadata.shortName, cluster.percentClusterFound);
        }
        return String.format("tag %-10d", ((AprilTagSingleDetection) detection).id);
    }

    /**
     * Where it is, relative to the camera. Absolute field position is deliberately not reported:
     * BioBuzz tags ride on the HIVE, which tips during a match, so FIRST states they are not
     * suitable for absolute field localization.
     */
    private static String describePose(AprilTagDetection detection) {
        if (detection.ftcPose == null) {
            return "no pose (tag not in the active library)";
        }
        return String.format("range %5.1f  bearing %5.1f  yaw %5.1f",
                detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw);
    }
}
