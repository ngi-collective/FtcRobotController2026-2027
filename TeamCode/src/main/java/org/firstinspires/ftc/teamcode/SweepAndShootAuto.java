package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Nine seconds of AUTONOMOUS: spin up, slide across two rows of POLLEN, and tip the HIVE.
 *
 * <p>Deliberately the simplest thing that scores. Five steps on a timer &mdash; no odometry, no
 * vision, no path follower &mdash; which is what a first AUTO looks like, and what to run in the
 * dashboard to watch the whole game work at once. Load the {@code auto-sweep} scenario, drag the
 * robot onto the start square the INIT telemetry names, and press START.</p>
 *
 * <h2>Why it aims without looking</h2>
 *
 * <p>An AUTO starts on a known square of tiles facing a known direction, so where the CELL is
 * relative to the robot comes off the field drawing. That is not a weaker way of aiming than
 * {@link AimedLauncherTeleOp}'s: it works before the robot has moved and with no tag in view, and
 * it goes through the same {@link ShotSolver}. Only the source of the range differs &mdash; which
 * is also the weakness, and {@code SweepAndShootAutoTest} pins it: set the robot down half a metre
 * off its square and this routine throws every ball onto the tiles, cheerfully.</p>
 *
 * <p>The start square puts the muzzle near <b>twice the minimum distance</b>, where a steep lob
 * needs the least speed and cares least about being wrong. That is why both rows score: they are
 * 7.5&nbsp;cm apart in range and the curve through there is flat enough that the difference is a
 * couple of rev/min. See {@link ShotSolver} for the shape of it.</p>
 *
 * <h2>Why it slides sideways instead of driving at the balls</h2>
 *
 * <p>The mouth of this robot <em>is</em> its flywheel: a ball comes within reach and goes, at
 * whatever speed the wheel is running. So a shot has to be taken with the chassis still &mdash;
 * except in the one direction where being wrong is cheap. Driving <em>forward</em> into a ball adds
 * the chassis's speed to the flight and sails it over the far lip, which is 9&nbsp;cm past the near
 * one. Sliding <em>sideways</em> into it costs the same 13&nbsp;cm against an opening 50&nbsp;cm
 * wide.</p>
 *
 * <p>Hence the shape of the routine: start beside the row, slide along it firing, then nudge
 * forward onto the second row from the end where it is still clear of the mouth, and slide back.
 * Two rows because only one fits at a time &mdash; the mouth reaches 24&nbsp;cm out and the bumper
 * is at 20, so the band a ball can sit in without being shoved along by the chassis is one ball
 * wide.</p>
 *
 * <p>Six POLLEN on top of the three NECTAR a MATCH stages in the CELL (&sect;10.3.1) is what it
 * takes to tip the HIVE, which is worth more than all six balls that did it.</p>
 */
@Autonomous(name = "Auto: sweep and shoot", group = "BioBuzz")
public class SweepAndShootAuto extends LinearOpMode {

    /**
     * Where the red HIVE's raised CELL opening is, in the field frame.
     *
     * <p>The audience-side CELL of an audience-up HIVE: its opening centre is 1.51&nbsp;m above the
     * tiles and 0.40&nbsp;m out from the field's centre line on the side it faces. Manual figure
     * 9-10 and the season CAD, via {@code BioBuzzHive}.</p>
     */
    private static final double CELL_OPENING_Y_METRES = -0.4010;
    private static final double CELL_OPENING_HEIGHT_METRES = 1.5100;

    /**
     * The start square, in the field frame, facing the HIVE.
     *
     * <p>{@code Y} puts the near row of POLLEN just past the bumper and inside the mouth, with the
     * muzzle 0.76&nbsp;m from the opening. {@code X} is far enough to the audience-left of the row
     * that none of it is in the wheel before the robot starts moving.</p>
     */
    private static final double START_X_METRES = -0.55;
    private static final double START_Y_METRES = -1.431;

    /** Where the two rows of POLLEN lie, in the field frame: 7.5 cm apart, which is a ball. */
    private static final double NEAR_ROW_Y_METRES = -1.161;
    private static final double FAR_ROW_Y_METRES = -1.086;

    /**
     * How hard to slide, and for how long.
     *
     * <p>0.16 of a mecanum's strafe is about 0.20&nbsp;m/s, which carries a ball 13&nbsp;cm off
     * line: a quarter of the opening, and the price of collecting without stopping. Faster starts
     * putting balls on the lip; slower spends AUTO. The duration covers the row plus enough
     * overrun to leave the far row clear of the mouth before the nudge forward.</p>
     */
    private static final double SWEEP_POWER = 0.16;
    private static final double SWEEP_SECONDS = 2.4;

    /**
     * And the nudge between rows: forward far enough to bring the far row into the mouth.
     *
     * <p>Taken at the right-hand end of the sweep, where the far row is still a hand's width
     * outside the mouth, so nothing is in the wheel while the chassis is moving forward. That is
     * the one direction a shot cannot survive.</p>
     */
    private static final double NUDGE_POWER = 0.15;
    private static final double NUDGE_SECONDS = 0.31;

    /**
     * Long enough for a 4 in wheel on a bare motor to arrive, with a little to spare.
     *
     * <p>A ceiling rather than a wait: the loop breaks as soon as the wheel is at speed. A launcher
     * with a jammed roller therefore costs two seconds of a thirty-second AUTO rather than all of
     * it.</p>
     */
    private static final double SPIN_UP_SECONDS = 2.0;

    /** Within this of the commanded speed, in rev/min, the wheel is worth feeding. */
    private static final double READY_REVOLUTIONS_PER_MINUTE = 60.0;

    /** Time for the last ball of the sweep to land and for the HIVE to finish swinging. */
    private static final double SETTLE_SECONDS = 1.5;

    private static final double STEP_SECONDS = 0.02;

    private final MecanumMovement movement = new MecanumMovement();
    private final JoystickInput left = new JoystickInput();
    private final JoystickInput right = new JoystickInput();

    private DriveHardware drive;
    private DcMotorEx flywheel;
    private String step = "init";

    @Override
    public void runOpMode() {
        drive = DriveHardware.initWithoutEncoders(hardwareMap);

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        // Commanded by speed, not power: the solver's answer is a speed, so the mechanism had
        // better be able to hold one as the battery tires.
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Both shots, worked out once from the field drawing. The sideways offset is deliberately
        // left out: this routine never turns, and the sweep keeps every ball inside the CELL's own
        // 20 in width, so a turn to ask for would only be one nobody is going to make.
        ShotSolver solver = LaunchGeometry.solver();
        ShotSolver.Shot nearRow = solver.solve(shotFrom(NEAR_ROW_Y_METRES), 0.0,
                CELL_OPENING_HEIGHT_METRES);
        ShotSolver.Shot farRow = solver.solve(shotFrom(FAR_ROW_Y_METRES), 0.0,
                CELL_OPENING_HEIGHT_METRES);

        telemetry.addData("start on", "x %.2f  y %.2f  heading 90 (facing the red HIVE)",
                START_X_METRES, START_Y_METRES);
        telemetry.addData("scenario", "auto-sweep");
        telemetry.addData("near row", nearRow.toString());
        telemetry.addData("far row", farRow.toString());
        telemetry.update();

        waitForStart();

        spinUpTo(nearRow);
        slide(SWEEP_POWER, "sweeping the near row");

        // Forward onto the far row, then back the way it came. The wheel stays at speed through
        // the nudge: there is nothing in the mouth to throw badly.
        nudgeForward();
        command(farRow);
        slide(-SWEEP_POWER, "sweeping the far row");

        drive.stop();
        flywheel.setVelocity(0.0);
        hold(SETTLE_SECONDS, "settling");

        // And then stand there. A real AUTO has nothing left to do but wait for the buzzer, and
        // returning early would be worse than useless here: a dashboard session rebuilds the
        // field when an OpMode stops, so a routine that finished by finishing would undo the TIP
        // it just earned about a second after earning it.
        step = "done";
        while (opModeIsActive()) {
            report();
            sleep((long) (STEP_SECONDS * 1000.0));
        }
    }

    /** How far ahead of the robot's centre the CELL's opening is, for a ball fired from a row. */
    private static double shotFrom(double rowY) {
        // Measured from the row rather than from the chassis: a ball leaves from where it is
        // lying, which is the mouth, and the solver takes the muzzle's own reach off again.
        return CELL_OPENING_Y_METRES - rowY + LaunchGeometry.muzzleForwardMetres();
    }

    private void spinUpTo(ShotSolver.Shot shot) {
        command(shot);
        step = "spinning up";
        for (double waited = 0.0; waited < SPIN_UP_SECONDS && opModeIsActive();
                waited += STEP_SECONDS) {
            if (Math.abs(revolutionsPerMinute() - commanded(shot)) < READY_REVOLUTIONS_PER_MINUTE) {
                return;
            }
            report();
            sleep((long) (STEP_SECONDS * 1000.0));
        }
    }

    private void command(ShotSolver.Shot shot) {
        flywheel.setVelocity(commanded(shot) * 360.0 / 60.0, AngleUnit.DEGREES);
    }

    private static double commanded(ShotSolver.Shot shot) {
        return shot.reachable() ? shot.revolutionsPerMinute() : 0.0;
    }

    private double revolutionsPerMinute() {
        return flywheel.getVelocity(AngleUnit.DEGREES) * 60.0 / 360.0;
    }

    /** Slides across the field at a given power: positive is to the robot's right. */
    private void slide(double power, String what) {
        left.update(power, 0.0);
        right.update(0.0, 0.0);
        drive.setPowers(movement.calculate(left, right));
        hold(SWEEP_SECONDS, what);
        drive.stop();
    }

    private void nudgeForward() {
        left.update(0.0, NUDGE_POWER);
        right.update(0.0, 0.0);
        drive.setPowers(movement.calculate(left, right));
        hold(NUDGE_SECONDS, "moving up to the far row");
        drive.stop();
    }

    private void hold(double seconds, String what) {
        step = what;
        for (double held = 0.0; held < seconds && opModeIsActive(); held += STEP_SECONDS) {
            report();
            sleep((long) (STEP_SECONDS * 1000.0));
        }
    }

    private void report() {
        telemetry.addData("step", step);
        telemetry.addData("flywheel", "%.0f rpm", revolutionsPerMinute());
        telemetry.update();
    }
}
