package org.firstinspires.ftc.teamcode;

/**
 * Intake roller control: trigger-proportional intake, eject override, slew-limited power and
 * automatic unjamming on a sustained current stall. Pure logic, no hardware.
 */
public class IntakeLogic {

    public static final double INTAKE_POWER = 0.8;
    public static final double EJECT_POWER = 0.6;
    public static final double UNJAM_POWER = 0.5;
    /** Maximum power change per second. */
    public static final double RAMP_PER_SEC = 4.0;
    public static final double TRIGGER_DEADBAND = 0.1;
    public static final double STALL_CURRENT_AMPS = 6.0;
    /** Current must stay above the stall threshold this long to count as a jam. */
    public static final double STALL_TIME_SEC = 0.25;
    /** Inrush after the roller starts is ignored for this long. */
    public static final double SPINUP_GRACE_SEC = 0.3;
    public static final double UNJAM_TIME_SEC = 0.4;

    public enum State { IDLE, INTAKING, EJECTING, UNJAMMING }

    public static final class Result {
        public final double power;
        public final State state;

        Result(double power, State state) {
            this.power = power;
            this.state = state;
        }
    }

    private State state = State.IDLE;
    private double power = 0.0;
    private double intakingTime = 0.0;
    private double stallTime = 0.0;
    private double unjamTime = 0.0;

    public Result update(double trigger, boolean eject, double amps, double dt) {
        boolean intakeRequested = trigger > TRIGGER_DEADBAND;
        double target;

        if (eject) {
            state = State.EJECTING;
            target = -EJECT_POWER;
        } else if (!intakeRequested) {
            state = State.IDLE;
            target = 0.0;
        } else if (state == State.UNJAMMING) {
            unjamTime += dt;
            if (unjamTime >= UNJAM_TIME_SEC) {
                enterIntaking();
                target = trigger * INTAKE_POWER;
            } else {
                target = -UNJAM_POWER;
            }
        } else {
            if (state != State.INTAKING) {
                enterIntaking();
            }
            intakingTime += dt;
            if (intakingTime > SPINUP_GRACE_SEC && amps > STALL_CURRENT_AMPS) {
                stallTime += dt;
            } else {
                stallTime = 0.0;
            }
            if (stallTime >= STALL_TIME_SEC) {
                state = State.UNJAMMING;
                unjamTime = 0.0;
                target = -UNJAM_POWER;
            } else {
                target = Math.min(trigger, 1.0) * INTAKE_POWER;
            }
        }

        double step = RAMP_PER_SEC * dt;
        power += Math.max(-step, Math.min(step, target - power));
        return new Result(power, state);
    }

    private void enterIntaking() {
        state = State.INTAKING;
        intakingTime = 0.0;
        stallTime = 0.0;
    }
}
