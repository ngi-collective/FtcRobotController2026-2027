package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.Set;

/**
 * One shot of everything the robot holds: spin the flywheel up, then run staging to feed it.
 *
 * <p>Spin up first: a ball fed to a wheel still coming up to speed leaves slow and falls short.
 * Staging starts once the flywheel has run for {@link #SPIN_UP_MIN_MS} and reads
 * {@link #READY_FRACTION} of its target, or after {@link #SPIN_UP_MAX_MS} if the encoder never
 * says so, and then feeds for {@link #FEED_MS}. The minimum also lets the chassis settle on its
 * spot before anything leaves.</p>
 */
public class shootCommand implements Command {

    /** Flywheel target, encoder ticks per second. */
    public static final double TARGET_TICKS_PER_SECOND = 1200.0;

    /** Up to speed at this fraction of the target. */
    public static final double READY_FRACTION = 0.9;

    /** Least it spins up before feeding, in milliseconds, whatever the encoder reads. */
    public static final long SPIN_UP_MIN_MS = 500;

    /** Longest it waits for the flywheel before feeding anyway, in milliseconds. */
    public static final long SPIN_UP_MAX_MS = 1500;

    /** How long staging feeds once the flywheel is up to speed, in milliseconds. */
    public static final long FEED_MS = 3000;

    /**
     * The hub's velocity loop gains, set explicitly rather than trusted to be there.
     *
     * <p>These are the REV hub's own defaults for {@code RUN_USING_ENCODER}. They are written on
     * every AUTO's INIT because the hub keeps whatever was last set until it loses power, and
     * {@code FlywheelPIDFTuning} starts by setting P and F to zero: an AUTO run after it would
     * otherwise command 1200 ticks/s through a loop that does nothing. Replace them with the
     * tuner's numbers once it has produced some.</p>
     */
    public static final PIDFCoefficients VELOCITY_PIDF = new PIDFCoefficients(10.0, 3.0, 0.0, 0.0);

    /**
     * Puts the flywheel in the hub's closed-loop velocity mode with {@link #VELOCITY_PIDF}.
     *
     * <p>{@code setVelocity} is only a speed in {@code RUN_USING_ENCODER}; the TeleOp drives the
     * same motor open loop in {@code RUN_WITHOUT_ENCODER}, and the hub keeps that mode into an
     * AUTO run straight after it. Call once at INIT, after the direction is set.</p>
     */
    public static void configure(DcMotorEx flywheel) {
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, VELOCITY_PIDF);
    }

    private final DcMotorEx flywheel;
    private final CRServo staging;
    private long startTime;
    private long feedingSince;

    public shootCommand(DcMotorEx flywheel, CRServo staging) {
        this.flywheel = flywheel;
        this.staging = staging;
    }

    @Override
    public Set<Object> requirements() {
        return Set.of(flywheel, staging);
    }

    @Override
    public int priority() {
        return 0;
    }

    @Override
    public InterruptedBehavior interruptedBehavior() {
        return InterruptedBehavior.END;
    }

    @Override
    public ConflictBehavior conflictBehavior() {
        return ConflictBehavior.OVERRIDE;
    }

    @Override
    public BlockedBehavior blockedBehavior() {
        return BlockedBehavior.CANCEL;
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        feedingSince = -1;
    }

    @Override
    public void execute() {
        long now = System.currentTimeMillis();
        flywheel.setVelocity(TARGET_TICKS_PER_SECOND);
        boolean upToSpeed =
                Math.abs(flywheel.getVelocity()) >= READY_FRACTION * TARGET_TICKS_PER_SECOND;
        long spinning = now - startTime;
        if (feedingSince < 0 && ((upToSpeed && spinning >= SPIN_UP_MIN_MS)
                || spinning >= SPIN_UP_MAX_MS)) {
            feedingSince = now;
        }
        staging.setPower(feedingSince < 0 ? 0.0 : 1.0);
    }

    @Override
    public boolean done() {
        return feedingSince >= 0 && System.currentTimeMillis() - feedingSince >= FEED_MS;
    }

    @Override
    public void end(EndCondition endCondition) {
        flywheel.setVelocity(0);
        staging.setPower(0);
    }
}
