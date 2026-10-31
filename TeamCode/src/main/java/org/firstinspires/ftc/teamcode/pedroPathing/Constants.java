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
        c.xPodOffset.set(-84.0);
        c.yPodOffset.set(-168.0);
        c.offsetUnits.set(DistanceUnit.MM);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
    });

    // UNTUNED: starting values that let the simulated robot follow a path, not numbers measured on
    // the real one. Replace the whole block with ForesightTuner's output.
    //   - velocities: the sim's free speed, 1.57 m/s = 61.8 in/s, and 80% of it sideways
    //     (robot-config strafeEfficiency 0.8)
    //   - decelerations: below the sim's grip limit of about 0.9 g = 347 in/s^2
    //   - gains and brake coefficients: guesses, checked only by PedroPathingSimTest
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.1);
        Controller secondaryTranslationalForward = Controller.proportional(0.05);
        Controller primaryTranslationalLateral = Controller.proportional(0.1);
        Controller secondaryTranslationalLateral = Controller.proportional(0.05);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward)
                .put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral)
                .put(2.5, primaryTranslationalLateral));

        c.coast.set(Controller.proportionalFeedforward(0.01));
        c.brake.set(Controller.proportionalFeedforward(0.02));

        c.headingFeedback.set(Controller.proportional(1.0));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.1, 0.01));

        c.linearBrakeCoefficients.set(Matrix.diag(0.05, 0.05));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.002, 0.002));

        c.maxAchievableForwardVelocity.set(61.8);
        c.maxAchievableStrafeVelocity.set(49.4);
        c.naturalForwardDeceleration.set(250.0);
        c.naturalStrafeDeceleration.set(200.0);
    });

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, driveConfig),
                new Foresight(foresightConfig));
    }
}
