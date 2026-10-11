package org.firstinspires.ftc.teamcode;

/**
 * Intake control math. No FTC SDK dependencies, so it can be compiled and tested on a laptop.
 *
 * Power convention: positive = pulling game pieces IN. (IntakeHardware flips the motor direction
 * if the gear mesh makes positive spit them out, so this file never needs to change for that.)
 *
 * Built for the goBILDA starter-kit intake: a large gear on the motor driving a small gear on the roller.
 * That is a gear-UP, so the roller spins faster than the motor and the gears take shock loads. This logic therefore:
 *  - ramps power up and down instead of jumping, which protects the gear teeth and the battery,
 *  - caps power below 1.0 so the roller doesn't fling pieces,
 *  - watches motor current and briefly reverses to clear a jam instead of stalling the motor.
 */
public class IntakeLogic {

    // ---- Tunable constants ----
    /** Power at full trigger pull while intaking. */
    public static final double INTAKE_POWER        = 1.00;
    /** Power while the eject button is held. */
    public static final double EJECT_POWER         = 0.80;
    /** Power change limit in power-per-second. 8.0 = 0 to full in about 0.12 s; a full direction flip takes about 0.2 s. */
    public static final double RAMP_PER_SEC        = 8.0;
    public static final double TRIGGER_DEADZONE    = 0.05;

    // ---- Jam protection ----
    public static final boolean JAM_PROTECTION     = true;
    /** Motor current that counts as stalled. goBILDA 5203 motors stall around 9 A, so only a true stall should reach this. */
    public static final double STALL_CURRENT_AMPS  = 8.0;
    /** Current must stay above the threshold this long before reversing. */
    public static final double STALL_TIME_SEC      = 0.50;
    /** Ignore current for this long after spin-up begins (startup inrush is not a jam). */
    public static final double SPINUP_GRACE_SEC    = 0.35;
    public static final double UNJAM_TIME_SEC      = 0.35;
    public static final double UNJAM_POWER         = 0.60;
    /** Ramp is this many times faster while clearing a jam. */
    public static final double UNJAM_RAMP_MULT     = 3.0;

    public enum State { IDLE, INTAKING, EJECTING, UNJAMMING }

    public static class Result {
        public final double power;
        public final State state;

        public Result(double power, State state) {
            this.power = power;
            this.state = state;
        }
    }

    private double power = 0.0;        // actual (ramped) power being sent
    private double spinupTimer = 0.0;  // how long the roller has been spinning inward
    private double stallTimer = 0.0;   // how long current has been over the stall threshold
    private double unjamTimer = 0.0;   // time left in the current unjam reverse

    /**
     * @param intakeTrigger 0..1 analog trigger value (hold to intake)
     * @param eject         true while the eject button is held (wins over the trigger)
     * @param currentAmps   motor current in amps; pass 0 if not measured this loop
     * @param dtSeconds     time since the previous loop
     */
    public Result update(double intakeTrigger, boolean eject, double currentAmps, double dtSeconds) {
        double dt = Math.max(0.001, Math.min(0.1, dtSeconds));

        // ---- What the driver is asking for ----
        double requested = 0.0;
        if (eject) {
            requested = -EJECT_POWER;
        } else if (intakeTrigger > TRIGGER_DEADZONE) {
            double scaled = (Math.min(intakeTrigger, 1.0) - TRIGGER_DEADZONE) / (1.0 - TRIGGER_DEADZONE);
            requested = INTAKE_POWER * scaled;
        }

        // Releasing the trigger, or ejecting, cancels any unjam and restarts the jam timers.
        if (requested <= 0.0) {
            unjamTimer = 0.0;
            stallTimer = 0.0;
        }

        // ---- Jam detection (intake direction only) ----
        boolean unjamming = unjamTimer > 0.0;
        if (JAM_PROTECTION && requested > 0.0 && !unjamming) {
            spinupTimer = power > 0.1 ? spinupTimer + dt : 0.0;
            if (spinupTimer > SPINUP_GRACE_SEC && currentAmps > STALL_CURRENT_AMPS) {
                stallTimer += dt;
            } else {
                stallTimer = Math.max(0.0, stallTimer - dt);
            }
            if (stallTimer >= STALL_TIME_SEC) {
                unjamTimer = UNJAM_TIME_SEC;
                stallTimer = 0.0;
                spinupTimer = 0.0;
                unjamming = true;
            }
        } else if (requested <= 0.0) {
            spinupTimer = 0.0;
        }

        // ---- Pick the target ----
        double target = requested;
        State state;
        double rate = RAMP_PER_SEC;
        if (unjamming) {
            target = -UNJAM_POWER;
            rate = RAMP_PER_SEC * UNJAM_RAMP_MULT;
            unjamTimer -= dt;
            state = State.UNJAMMING;
        } else if (requested < 0.0) {
            state = State.EJECTING;
        } else if (requested > 0.0) {
            state = State.INTAKING;
        } else {
            state = State.IDLE;
        }

        // ---- Ramp toward it ----
        double step = rate * dt;
        power += Math.max(-step, Math.min(step, target - power));

        return new Result(power, state);
    }
}