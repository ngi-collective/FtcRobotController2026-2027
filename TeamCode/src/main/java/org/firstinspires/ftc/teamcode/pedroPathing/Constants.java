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
import com.pedropathing.tuning.autotune.*;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.*;

/**
 * Pedro Pathing 3.x: a {@link Follower} is a localizer, a drivetrain and an algorithm.
 *
 * <p>Units are inches and radians, Pedro's defaults.</p>
 */
public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FL");
        c.frontRightName.set("FR");
        c.backLeftName.set("BL");
        c.backRightName.set("BR");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });
    // The simulated robot mirrors this: robot-config/*.json mounts its pods at these offsets
    // (xPodLeftMetres, yPodForwardMetres) with the strafe pod reversed, and mirrors the right-side
    // drive motors to match drivetrainConfig. Change one, change the other.
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-1.753755404254583);
        c.yPodOffset.set(-0.8289895095224457);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // UNTUNED: starting values that let the simulated robot follow a path, not numbers measured on
    // the real one. Replace the whole block with ForesightTuner's output.
    //   - velocities: the sim's free speed, 1.57 m/s = 61.8 in/s, and 80% of it sideways
    //     (robot-config strafeEfficiency 0.8)
    //   - decelerations: below the sim's grip limit of about 0.9 g = 347 in/s^2
    //   - gains and brake coefficients: guesses, checked only by PedroPathingSimTest
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.21245178498483241);
                Controller secondaryTranslationalForward = Controller.proportional(0.07849525773564937);
                Controller primaryTranslationalLateral = Controller.proportional(0.24467089780674908);
                Controller secondaryTranslationalLateral = Controller.proportional(0.09039935901279737);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.015898311726797985));
                c.brake.set(Controller.proportionalFeedforward(0.013513564967778287));

                c.headingFeedback.set(Controller.proportional(3.9498887575127464));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05469251605760781, 0.00428766173467247));

                c.linearBrakeCoefficients.set(Matrix.diag(0.08154348046058826, 0.08123698262551923));
                c.quadraticBrakeCoefficients.set(Matrix.diag(9.95468732016005E-4, 9.58694748903988E-4));

                c.maxAchievableForwardVelocity.set(66.32158222045105);
                c.maxAchievableStrafeVelocity.set(56.0235802155021);
                c.naturalForwardDeceleration.set(31.36138640065747);
                c.naturalStrafeDeceleration.set(56.756746272602705);
            }
    );
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}
