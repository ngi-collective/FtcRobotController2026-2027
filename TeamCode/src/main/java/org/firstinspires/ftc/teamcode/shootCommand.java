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
 * Staging only ever feeds while the flywheel reads {@value #MIN_READY_FRACTION}&ndash;
 * {@value #MAX_READY_FRACTION} of its target (see {@link #flywheelReady}), and pauses through any
 * dip out of that window. Feeding starts once the flywheel has run for {@link #SPIN_UP_MIN_MS}
 * and is in the window, and lasts {@link #FEED_MS}. The minimum also lets
 * the chassis settle on its spot before anything leaves.</p>
 *
 * <p>A flywheel that has not reached the window after {@link #SPIN_UP_MAX_MS} gives up the shot
 * rather than feed it: the routine moves on with the balls still aboard.</p>
 */
public class shootCommand implements Command {

    /** Flywheel target, encoder ticks per second. */
    public static final double TARGET_TICKS_PER_SECOND = 1350;

    /** Least fraction of its target the flywheel may read while staging feeds it. */
    public static final double MIN_READY_FRACTION = 0.96;

    /** Greatest fraction of its target the flywheel may read while staging feeds it. */
    public static final double MAX_READY_FRACTION = 1.02;

    /** Least it spins up before feeding, in milliseconds, whatever the encoder reads. */
    public static final long SPIN_UP_MIN_MS = 500;

    /**
     * Longest it waits for the flywheel to reach the feeding window, in milliseconds. A loaded
     * wheel coming up from rest on the real hub can take more than a second and a half to settle
     * inside it.
     */
    public static final long SPIN_UP_MAX_MS = 2500;

    /** How long staging feeds once the flywheel is up to speed, in milliseconds. */
    public static final long FEED_MS = 7000;

    /**
     * The hub's velocity loop gains, written on every INIT because the hub keeps whatever was
     * last set until it loses power.
     *
     * <p>Tuned from P=50, F=3, which overshot the first balls and undershot the later ones. F is
     * the feedforward and was far too low: the 6000 rpm motor's free speed is about 2800 ticks/s,
     * so holding a speed on its own wants F ≈ 32767 / 2800 ≈ 11.7, and the loaded wheel tuned in
     * {@code iamyou} ({@code FLYWHEEL_KV}, {@code FLYWHEEL_KS}) works out to about 11. With F=3 the
     * P term had to supply most of the power, which only happens with a large error: P=50 shot
     * the wheel past the target on spin-up (early balls long), then once balls started taking
     * energy out it could only hold the wheel well below the target (later balls short). With F
     * carrying the speed, P only has to recover each ball's dip, and 15 does that without
     * overshooting.</p>
     */
    public static final PIDFCoefficients VELOCITY_PIDF = new PIDFCoefficients(40.0, 10.0, 0.0, 11.0);

    /**
     * Puts the flywheel in the hub's closed-loop velocity mode with {@link #VELOCITY_PIDF}.
     *
     * <p>{@code setVelocity} is only a speed in {@code RUN_USING_ENCODER}; {@code iamyou} puts the
     * motor in {@code RUN_WITHOUT_ENCODER} whenever its flywheel is off, so it coasts, and the hub
     * keeps that mode into an AUTO run straight after it. Call once at INIT, after the direction is set.</p>
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

    /**
     * Whether a flywheel reading {@code ticksPerSecond} may be fed: within
     * {@link #MIN_READY_FRACTION}&ndash;{@link #MAX_READY_FRACTION} of a positive target. A
     * flywheel with no target is never ready.
     */
    public static boolean flywheelReady(double ticksPerSecond, double targetTicksPerSecond) {
        return targetTicksPerSecond > 0.0
                && ticksPerSecond >= MIN_READY_FRACTION * targetTicksPerSecond
                && ticksPerSecond <= MAX_READY_FRACTION * targetTicksPerSecond;
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
        boolean ready = flywheelReady(flywheel.getVelocity(), TARGET_TICKS_PER_SECOND);
        if (feedingSince < 0 && now - startTime >= SPIN_UP_MIN_MS && ready) {
            feedingSince = now;
        }
        staging.setPower(feedingSince >= 0 && ready ? 1.0 : 0.0);
    }

    @Override
    public boolean done() {
        long now = System.currentTimeMillis();
        return feedingSince >= 0 ? now - feedingSince >= FEED_MS
                : now - startTime >= SPIN_UP_MAX_MS;
    }

    @Override
    public void end(EndCondition endCondition) {
        flywheel.setVelocity(0);
        staging.setPower(0);
    }
}
