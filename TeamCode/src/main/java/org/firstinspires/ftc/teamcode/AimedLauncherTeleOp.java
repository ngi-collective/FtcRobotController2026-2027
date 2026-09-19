package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Look at a CELL, work out how hard to throw, and throw that hard.
 *
 * <p>The closed loop the season is about. {@code LauncherTeleOp} leaves the speed on the D-pad for
 * a driver to guess at, which works from one marked spot on the tiles and nowhere else. Here the
 * AprilTag Cluster under the raised CELL supplies the range, the range supplies the wheel speed,
 * and the driver's job shrinks to holding the assist and feeding balls.</p>
 *
 * <h2>What the driver does</h2>
 *
 * <ul>
 *   <li><b>Sticks</b>: drive, as ever, except while the assist has the robot turning.</li>
 *   <li><b>Left bumper</b>: assist. Turns the robot onto the CELL and spins the wheel to the
 *       solved speed. Held, not latched, because it takes the sticks away.</li>
 *   <li><b>Right trigger</b>: feed. Only once the telemetry says READY, which means the wheel has
 *       arrived <em>and</em> the robot is pointed.</li>
 *   <li><b>Left trigger</b>: eject, for a jam.</li>
 *   <li><b>D-pad left / right, before START</b>: which alliance. There is no way to detect it and
 *       putting a ball in the other alliance's CELL scores for them.</li>
 * </ul>
 *
 * <h2>Why the wheel spins before the ball arrives</h2>
 *
 * <p>Nothing holds a ball back from a spinning wheel, so a ball fed into a wheel that is still
 * climbing leaves at whatever speed the wheel has reached and lands a foot away. The assist
 * therefore spins up on its own, and READY is the permission to feed. This is the single most
 * expensive mistake available with a flywheel and it looks exactly like a mis-calibration.</p>
 *
 * <h2>Watching the tags swap</h2>
 *
 * <p>Enough balls in a raised CELL and the HIVE tips, which is the point of aiming at it. When it
 * does, both of that HIVE's plates swing to face the <em>other</em> side of the field, so a robot
 * standing where it just scored from loses its target completely &mdash; the telemetry goes from a
 * named CELL and a range to "no CELL in view". That is not a failure to recover from; it is the
 * game telling the driver to go round. The newly raised CELL is the other one, with the other four
 * ids, and it is visible from the far side.</p>
 */
@TeleOp(name = "Aimed Launcher", group = "Linear OpMode")
public class AimedLauncherTeleOp extends LinearOpMode {

    /** Below this the trigger is noise rather than intent. */
    private static final double TRIGGER_DEADBAND = 0.1;

    /** Within this of the solved speed, in rev/min, the wheel is worth feeding. */
    private static final double READY_REVOLUTIONS_PER_MINUTE = 60.0;

    /**
     * And within this of pointing at the CELL, in degrees.
     *
     * <p>Two degrees at a metre is 3.5&nbsp;cm of sideways miss, against a CELL opening 50&nbsp;cm
     * wide. Tighter would be spurious &mdash; the tag solve itself moves by about that much frame
     * to frame &mdash; and would leave the assist hunting instead of arriving.</p>
     */
    private static final double READY_DEGREES = 2.0;

    /**
     * How hard the assist turns, in power per radian of error, and the least it bothers with.
     *
     * <p>Proportional only. An integral term on a turn this short winds up while the wheel is
     * spinning and overshoots on release, and there is nothing to hold against: the error is
     * measured afresh from the tags every frame, so a steady-state offset cannot accumulate
     * unseen.</p>
     */
    private static final double TURN_GAIN_PER_RADIAN = 1.2;
    private static final double TURN_FLOOR = 0.08;
    private static final double TURN_CEILING = 0.35;

    private final MecanumMovement movement = new MecanumMovement();

    @Override
    public void runOpMode() {
        DriveHardware drive = DriveHardware.initWithoutEncoders(hardwareMap);

        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        // Commanded by speed, not power: a tiring battery drops a power-commanded wheel by a
        // couple of hundred rev/min, and every shot drops short with it. The solver's answer is a
        // speed, so the mechanism had better be able to hold one.
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        TouchSensor loaded = hardwareMap.get(TouchSensor.class, "intakeTouch");

        // Metres and degrees out of the processor. The default is inches, and this whole loop is
        // metric because gravity is: one conversion here beats one at every use.
        AprilTagProcessor tags = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();
        VisionPortal camera = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tags)
                .build();

        LensMount mount = LaunchGeometry.camera();
        ShotSolver solver = LaunchGeometry.solver();

        JoystickInput leftStick = new JoystickInput();
        JoystickInput rightStick = new JoystickInput();

        boolean red = true;
        while (opModeInInit()) {
            if (gamepad1.dpad_left) {
                red = true;
            } else if (gamepad1.dpad_right) {
                red = false;
            }
            telemetry.addData("alliance", red ? "RED  (dpad right for blue)"
                    : "BLUE  (dpad left for red)");
            telemetry.addData("camera", camera.getCameraState());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            Target target = bestTarget(tags.getDetections(), mount, solver, red);
            boolean assisting = gamepad1.left_bumper && target != null;

            double turnPower = 0.0;
            boolean pointed = false;
            if (assisting) {
                double error = target.shot.turnRadians();
                pointed = Math.abs(Math.toDegrees(error)) < READY_DEGREES;
                turnPower = pointed ? 0.0 : turnTowards(error);
            }

            leftStick.update(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            // The assist owns the turn while it is held; the left stick still strafes and drives,
            // so a driver can close the range without losing the aim they already have.
            rightStick.update(assisting ? (float) -turnPower : gamepad1.right_stick_x,
                    gamepad1.right_stick_y);
            drive.setPowers(movement.calculate(leftStick, rightStick));

            double commandedRpm =
                    assisting && target.shot.reachable() ? target.shot.revolutionsPerMinute() : 0.0;
            flywheel.setVelocity(commandedRpm * 360.0 / 60.0, AngleUnit.DEGREES);
            double actualRpm = flywheel.getVelocity(AngleUnit.DEGREES) * 60.0 / 360.0;
            boolean upToSpeed = commandedRpm > 0.0
                    && Math.abs(actualRpm - commandedRpm) < READY_REVOLUTIONS_PER_MINUTE;

            intake.setPower(gamepad1.right_trigger > TRIGGER_DEADBAND ? 1.0
                    : gamepad1.left_trigger > TRIGGER_DEADBAND ? -1.0 : 0.0);

            telemetry.addData("alliance", red ? "RED" : "BLUE");
            if (target == null) {
                telemetry.addData("target", "no %s CELL in view", red ? "RED" : "BLUE");
                telemetry.addLine("a HIVE that has just tipped faces the other way: drive round");
            } else {
                telemetry.addData("target", "%s  %d%% of the cluster",
                        target.cell, target.percentFound);
                telemetry.addData("range", "%.2f m to a mouth %.2f m up",
                        target.shot.distanceMetres(), target.shot.targetHeightMetres());
                telemetry.addData("turn", "%.1f deg", Math.toDegrees(target.shot.turnRadians()));
                telemetry.addData("solved", target.shot.reachable()
                        ? String.format("%.0f rpm", target.shot.revolutionsPerMinute())
                        : target.shot.reach().toString());
            }
            telemetry.addData("flywheel", "%.0f / %.0f rpm", actualRpm, commandedRpm);
            telemetry.addData("state", !assisting ? "hold LB to aim"
                    : !target.shot.reachable() ? target.shot.reach().toString()
                    : !pointed ? "turning"
                    : !upToSpeed ? "spinning up"
                    : "READY");
            telemetry.addData("intake", loaded.isPressed() ? "LOADED" : "empty");
            telemetry.update();
        }

        flywheel.setVelocity(0.0);
        intake.setPower(0.0);
        drive.stop();
        camera.close();
    }

    /**
     * The CELL worth shooting at, out of everything the camera can see, or null.
     *
     * <p>Three filters, each throwing away a detection that is a perfectly good aim point at
     * something not worth aiming at: the opponent's HIVE, the CELL facing the floor, and anything
     * the launcher cannot reach. The last of those is kept if nothing better is available, because
     * "too close, back up" is the most useful thing the telemetry can say to a driver who has
     * driven under the HIVE.</p>
     */
    private Target bestTarget(List<AprilTagDetection> detections, LensMount mount,
                              ShotSolver solver, boolean red) {
        Target best = null;
        for (AprilTagDetection detection : detections) {
            if (!(detection instanceof AprilTagClusterDetection)) {
                // On BioBuzz every tag belongs to a cluster, so a single detection is a tag from
                // somebody's practice field or the sample library.
                continue;
            }
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            String cell = cluster.metadata.name;
            if (cell == null || !cell.startsWith(red ? "RED" : "BLUE")) {
                continue;
            }

            ShotSolver.Shot shot = solver.solve(mount.sight(
                    detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
            if (shot.targetHeightMetres() < LaunchGeometry.RAISED_CELL_HEIGHT_METRES) {
                continue;
            }
            // The higher of the two if both somehow pass: a raised CELL is the taller thing.
            if (best == null || shot.targetHeightMetres() > best.shot.targetHeightMetres()) {
                best = new Target(cell, cluster.percentClusterFound, shot);
            }
        }
        return best;
    }

    /**
     * Turn power for a given error, floored so it still moves and capped so it does not spin.
     *
     * <p>The floor is not a nicety: a mecanum drive at 6&nbsp;% power does not turn at all, so
     * without it the assist stops at whatever error happens to fall below the stiction and reports
     * itself as still turning forever.</p>
     */
    private static double turnTowards(double errorRadians) {
        double power = Math.abs(errorRadians) * TURN_GAIN_PER_RADIAN;
        power = Math.max(TURN_FLOOR, Math.min(TURN_CEILING, power));
        return errorRadians > 0.0 ? power : -power;
    }

    /** One CELL the camera can see, and what it would take to score in it. */
    private static final class Target {

        private final String cell;
        private final int percentFound;
        private final ShotSolver.Shot shot;

        private Target(String cell, int percentFound, ShotSolver.Shot shot) {
            this.cell = cell;
            this.percentFound = percentFound;
            this.shot = shot;
        }
    }
}
