package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

/**
 * Pins the robot where it stands, heading included, for a TeleOp that drives its own motors.
 *
 * <p>{@link #engage()} records the Pinpoint's pose and hands it to Pedro Pathing's hold-endpoint
 * ({@link Follower#hold}), with the gains in {@link Constants#foresightConfig}. While engaged,
 * {@link #update} returns the wheel powers that push the chassis back toward that pose: a shove is
 * met with a push the other way that grows with how far the robot has been moved, and once it stops
 * the robot returns to the spot and the heading it was braked at.</p>
 *
 * <p>Pedro is given neither the motors nor the Pinpoint to own. {@link Constants#createFollower}
 * would build a {@code Mecanum}, which caches each motor's last power and skips writes that look
 * unchanged (stale the moment the TeleOp writes a motor itself), and a {@code PinpointLocalizer},
 * which recalibrates the gyro and sleeps half a second. So the follower here drives
 * {@link WheelPowers}, which only works out powers for the TeleOp to apply, and reads
 * {@link PinpointReader}, which reads the Pinpoint the TeleOp has already set up.</p>
 *
 * <p>The Pinpoint is read, not refreshed: the TeleOp must call {@code pinpoint.update()} each loop
 * before {@link #engage()} or {@link #update}, as {@code iamyou}'s heading tracker already does.
 * Its raw heading is used, not the TeleOp's scaled one: the brake only needs how far the robot has
 * turned since it engaged, and a few percent of a shove's worth of turn is a fraction of a degree.</p>
 */
public class SoftwareBrake {

    private final PinpointReader localizer;
    private final WheelPowers drivetrain = new WheelPowers();
    private final Follower follower;
    private Pose holdPose = null;

    public SoftwareBrake(GoBildaPinpointDriver pinpoint) {
        localizer = new PinpointReader(pinpoint);
        follower = new Follower(localizer, drivetrain, new Foresight(Constants.foresightConfig));
    }

    /** Holds the robot at the pose the Pinpoint reads now. */
    public void engage() {
        localizer.update();
        holdPose = localizer.pose();
        follower.hold(holdPose);
    }

    /** Lets go: {@link #update} stops returning powers until the next {@link #engage()}. */
    public void release() {
        holdPose = null;
        follower.stop();
    }

    public boolean isEngaged() {
        return holdPose != null;
    }

    /** Where the robot is being held, in Pedro inches and radians; null when released. */
    public Pose holdPose() {
        return holdPose;
    }

    /** Where the Pinpoint last read the robot, in Pedro inches and radians. */
    public Pose pose() {
        return localizer.pose();
    }

    /** One telemetry line for the software brake: off, or how far the robot is from the pose it holds. */
    static String status(SoftwareBrake brake) {
        if (brake == null) {
            return "unavailable (no Pinpoint)";
        }
        if (!brake.isEngaged()) {
            return "off (Y to engage)";
        }
        Pose hold = brake.holdPose();
        Pose now = brake.pose();
        return String.format(Locale.US, "ON (Y to release)  off by %.1f in, %.1f deg",
                Math.hypot(now.x() - hold.x(), now.y() - hold.y()),
                Math.toDegrees(AngleUnit.normalizeRadians(now.heading() - hold.heading())));
    }

    /**
     * One control step: reads the Pinpoint and returns the FL, FR, BL, BR powers that hold the
     * robot, already scaled so none exceeds 1. All zero when released.
     */
    public double[] update(double dtSeconds) {
        if (!isEngaged()) {
            return new double[4];
        }
        follower.update(dtSeconds);
        return drivetrain.powers.clone();
    }

    /**
     * A Pedro {@link Drivetrain} that writes no motors: it works out mecanum wheel powers with the
     * same mix and scaling as Pedro's {@code Mecanum} and keeps them for the TeleOp to apply.
     */
    static final class WheelPowers implements Drivetrain {

        /** FL, FR, BL, BR, as last commanded. */
        final double[] powers = new double[4];
        private DrivePowers last = DrivePowers.zero();

        /** Pedro's mecanum mix: forward, strafe positive LEFT, turn positive counter-clockwise. */
        static double[] unnormalized(DrivePowers p) {
            return new double[]{
                    p.forward() - p.strafe() - p.turn(),
                    p.forward() + p.strafe() + p.turn(),
                    p.forward() + p.strafe() - p.turn(),
                    p.forward() - p.strafe() + p.turn(),
            };
        }

        @Override
        public void drive(DrivePowers drivePowers, boolean manual) {
            last = drivePowers;
            double[] raw = unnormalized(drivePowers);
            double max = 1.0;
            for (double power : raw) {
                max = Math.max(max, Math.abs(power));
            }
            for (int i = 0; i < raw.length; i++) {
                powers[i] = raw[i] / max;
            }
        }

        /** Copied from Pedro's {@code Mecanum}: how far along {@code delta} the wheels stay within +/-1. */
        @Override
        public double maxScaling(DrivePowers current, DrivePowers delta) {
            double lambda = 1.0;
            double[] currentPowers = unnormalized(current);
            double[] deltaPowers = unnormalized(delta);
            for (int i = 0; i < 4; i++) {
                double a = currentPowers[i];
                double b = deltaPowers[i];
                if (Math.abs(b) < 1e-9) continue;
                double t1 = (1.0 - a) / b;
                double t2 = (-1.0 - a) / b;
                if (t1 >= 0.0 && t1 < lambda) lambda = t1;
                if (t2 >= 0.0 && t2 < lambda) lambda = t2;
            }
            return Math.max(0.0, Math.min(1.0, lambda));
        }

        @Override
        public void stop() {
            stop(true);
        }

        @Override
        public void stop(boolean brake) {
            last = DrivePowers.zero();
            java.util.Arrays.fill(powers, 0.0);
        }

        /** Copied from Pedro's {@code Mecanum}. */
        @Override
        public double interpolateVelocity(double xRadius, double yRadius, double theta) {
            return 1.0 / (Math.abs(Math.cos(theta)) / xRadius + Math.abs(Math.sin(theta)) / yRadius);
        }

        @Override
        public Map<String, Object> debug() {
            Map<String, Object> map = new HashMap<>();
            map.put("forward", last.forward());
            map.put("strafe", last.strafe());
            map.put("turn", last.turn());
            return map;
        }
    }

    /**
     * A Pedro {@link Localizer} over a Pinpoint someone else configures and refreshes: reads its
     * pose and velocity in inches and radians, and never resets or recalibrates it.
     */
    static final class PinpointReader implements Localizer {

        private final GoBildaPinpointDriver pinpoint;
        private MotionState state = MotionState.zero();

        PinpointReader(GoBildaPinpointDriver pinpoint) {
            this.pinpoint = pinpoint;
        }

        @Override
        public void update() {
            state = MotionState.ofVelocity(
                    new Pose(pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH),
                            pinpoint.getHeading(AngleUnit.RADIANS)),
                    new Velocity(pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH),
                            pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)));
        }

        @Override
        public MotionState state() {
            return state;
        }

        @Override
        public void setPose(Pose pose) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, pose.x(), pose.y(),
                    AngleUnit.RADIANS, pose.heading()));
            state = state.withPose(pose);
        }

        /** Nothing to do: the TeleOp that owns the Pinpoint decides when it resets. */
        @Override
        public void reset() {
        }
    }
}
