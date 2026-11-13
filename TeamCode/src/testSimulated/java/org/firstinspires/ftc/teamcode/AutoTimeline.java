package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.ngicollective.ftcsim.hardware.FakeHardwareMap;
import org.ngicollective.ftcsim.sim.Pose2d;

import java.util.ArrayList;
import java.util.List;

/**
 * What an AUTO did with its flywheel, staging and intake, tick by tick, and the rules it must keep.
 *
 * <p>Sampled off the hardware, not the telemetry, so a step label that says "shooting" while the
 * staging servo sits still does not pass. Times are wall-clock seconds since START, which is the
 * clock both the OpMode and the match run on.</p>
 */
public final class AutoTimeline {

    private static final double METRES_PER_INCH = 0.0254;

    /** Least time the flywheel runs before staging feeds it: a spin-up, not a coincidence. */
    private static final double MIN_SPIN_UP_SECONDS = 0.3;

    /** Furthest the robot may move while staging feeds, in inches: it cannot shoot on the move. */
    private static final double MAX_DRIFT_WHILE_FEEDING_INCHES = 1.0;

    private final FakeHardwareMap hardware;
    private final DcMotorEx flywheel;
    private final DcMotorEx intake;
    private final CRServo staging;
    private final long startNanos = System.nanoTime();

    private final List<String> events = new ArrayList<>();
    private final List<String> violations = new ArrayList<>();
    private String last = "";

    private double flywheelOnSince = -1;
    private Pose2d feedingFrom;
    private boolean shotYet;
    private boolean intakeWasOn;
    private boolean carrying;
    private int collections;
    private int shots;
    private boolean feeding;

    public AutoTimeline(FakeHardwareMap hardware) {
        this.hardware = hardware;
        this.flywheel = hardware.get(DcMotorEx.class, iamyou.OUTTAKE_NAME);
        this.intake = hardware.get(DcMotorEx.class, iamyou.INTAKE_NAME);
        this.staging = hardware.get(CRServo.class, iamyou.STAGING_NAME);
    }

    public double seconds() {
        return (System.nanoTime() - startNanos) / 1e9;
    }

    /** One tick: call after each world step, with the OpMode's current step label. */
    public void sample(String step) {
        double t = seconds();
        boolean flywheelOn = Math.abs(flywheel.getPower()) > 0.05;
        boolean stagingOn = Math.abs(staging.getPower()) > 0.05;
        boolean intakeOn = Math.abs(intake.getPower()) > 0.05;
        Pose2d pose = hardware.drive().pose();

        if (flywheelOn && flywheelOnSince < 0) {
            flywheelOnSince = t;
        } else if (!flywheelOn) {
            flywheelOnSince = -1;
        }
        if (stagingOn && !feeding) {
            feeding = true;
            feedingFrom = pose;
            if (!flywheelOn) {
                violations.add(String.format("%.1fs staging fed a stopped flywheel", t));
            } else if (t - flywheelOnSince < MIN_SPIN_UP_SECONDS) {
                violations.add(String.format("%.1fs staging fed %.2f s after the flywheel started,"
                        + " before it could spin up", t, t - flywheelOnSince));
            }
            if (intakeOn) {
                violations.add(String.format("%.1fs shooting with the intake running", t));
            }
            shots++;
            shotYet = true;
            carrying = false;
        }
        if (stagingOn && feedingFrom != null && inches(pose, feedingFrom)
                > MAX_DRIFT_WHILE_FEEDING_INCHES) {
            violations.add(String.format("%.1fs moved %.1f in while shooting", t,
                    inches(pose, feedingFrom)));
            feedingFrom = null;
        }
        if (!stagingOn) {
            feeding = false;
        }
        if (intakeOn && !intakeWasOn) {
            if (!shotYet) {
                violations.add(String.format("%.1fs collected before shooting the pre-loads", t));
            }
            if (carrying) {
                violations.add(String.format("%.1fs collected again without shooting first", t));
            }
            if (flywheelOn) {
                violations.add(String.format("%.1fs intake on with the flywheel running", t));
            }
            carrying = true;
            collections++;
        }
        intakeWasOn = intakeOn;

        String state = String.format("%-36s fly %s stg %s in %s", step, on(flywheelOn),
                on(stagingOn), on(intakeOn));
        if (!state.equals(last)) {
            events.add(String.format("%5.1fs %s (%.1f, %.1f) %.0f", t, state,
                    pose.x() / METRES_PER_INCH + 72, pose.y() / METRES_PER_INCH + 72,
                    Math.toDegrees(pose.heading())));
            last = state;
        }
    }

    /**
     * The routine reported itself done at {@code doneSeconds}: check everything.
     *
     * @param red whose LOADING ZONE it must be parked in
     */
    public void assertKeptTheRules(String name, double doneSeconds, boolean red) {
        Pose2d pose = hardware.drive().pose();
        double x = pose.x() / METRES_PER_INCH + 72;
        double y = pose.y() / METRES_PER_INCH + 72;
        // Half the chassis in the 11 in deep zone: centre within 20.5 in of the alliance wall.
        boolean parked = red
                ? x <= 20.5 && y >= 96 && y <= 120
                : x >= 121.5 && y >= 24 && y <= 48;
        if (!parked) {
            violations.add(String.format("ended at (%.1f, %.1f), outside the LOADING ZONE", x, y));
        }
        // Intake (the front) facing away from the drivers, who stand behind their alliance wall:
        // red's at x = 0, so heading 0; blue's at x = 144, so heading 180.
        double awayDegrees = red ? 0.0 : 180.0;
        double off = Math.abs(Math.IEEEremainder(Math.toDegrees(pose.heading()) - awayDegrees, 360.0));
        if (off > 20.0) {
            violations.add(String.format("parked facing %.0f deg, intake not away from the drivers",
                    Math.toDegrees(pose.heading())));
        }
        if (!(doneSeconds <= 30.0)) {
            violations.add(String.format("finished at %.1f s, after the 30 s AUTO", doneSeconds));
        }
        if (shots == 0) {
            violations.add("never shot");
        }
        if (carrying) {
            violations.add("collected and never shot what it collected");
        }
        if (Math.abs(intake.getPower()) > 0.05 || Math.abs(staging.getPower()) > 0.05
                || Math.abs(flywheel.getPower()) > 0.05) {
            violations.add("left a mechanism running at the end");
        }
        assertTrue(violations.isEmpty(), name + ": " + violations + "\n  " + timeline());
    }

    public String timeline() {
        return String.join("\n  ", events);
    }

    public int collections() {
        return collections;
    }

    private static String on(boolean on) {
        return on ? "ON " : "off";
    }

    private static double inches(Pose2d a, Pose2d b) {
        return Math.hypot(a.x() - b.x(), a.y() - b.y()) / METRES_PER_INCH;
    }
}
