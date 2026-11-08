package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing 3.x: a {@link Follower} is a localizer, a drivetrain and an algorithm.
 *
 * <p>Units are inches and radians, Pedro's defaults.</p>
 */
public class Constants {

    public static MecanumConfig driveConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FL");
        c.frontRightName.set("FR");
        c.backLeftName.set("BL");
        c.backRightName.set("BR");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // PLACEHOLDER: the offsets are the SDK sample's (SensorGoBildaPinpoint), and
    // robot-config/*.json mounts the simulated pods there to match. Replace with PinpointTuner's
    // output from the real robot, and update the robot-config "pinpoint" blocks to the same mount.
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-3.7887642327256095);
        c.yPodOffset.set(-1.1133000231164647);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // TUNED on the real robot with ForesightTuner, 2026-11-07. Re-run the tuner and replace this whole block
    // after any change to the drivetrain, weight or wheels.
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.24109671432838614);
        Controller secondaryTranslationalForward = Controller.proportional(0.08907879372148371);
        Controller primaryTranslationalLateral = Controller.proportional(0.29662096131391186);
        Controller secondaryTranslationalLateral = Controller.proportional(0.10959351934743151);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward)
                .put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral)
                .put(2.5, primaryTranslationalLateral));

        c.coast.set(Controller.proportionalFeedforward(0.016629839359064622));
        c.brake.set(Controller.proportionalFeedforward(0.014135363455204929));

        c.headingFeedback.set(Controller.proportional(3.9353403428397735));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.058440552307153205, 0.0037273534349095347));

        c.linearBrakeCoefficients.set(Matrix.diag(0.077822495746091, 0.0640327054313192));
        c.quadraticBrakeCoefficients.set(Matrix.diag(8.122631246593916E-4, 0.0010263014367774678));

        c.maxAchievableForwardVelocity.set(59.90868640419559);
        c.maxAchievableStrafeVelocity.set(50.621265628312756);
        c.naturalForwardDeceleration.set(35.301751211680944);
        c.naturalStrafeDeceleration.set(54.247378322048235);
    });

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, driveConfig),
                new Foresight(foresightConfig));
    }
}
