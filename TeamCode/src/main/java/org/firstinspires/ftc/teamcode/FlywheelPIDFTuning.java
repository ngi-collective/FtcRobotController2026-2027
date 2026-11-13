package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

/**
 * Tunes the flywheel PIDF that I AM YOU uses ({@link iamyou.FlywheelPIDF}), live, then you copy the numbers into the
 * FLYWHEEL_* constants in iamyou.java.
 *
 * Gamepad 1:
 *   A ................... flywheel ON/OFF
 *   Y ................... next value to tune (target, kV, kS, kP, kI, kD)
 *   D-pad up / down ..... raise / lower that value by the step
 *   D-pad right / left .. bigger / smaller step
 *   Right bumper ........ hold to drop the target by 300 ticks/s (fakes a shot's dip, to watch recovery)
 *
 * Order: kP = kI = kD = 0, raise kV until it settles near the target on its own; then kP until a dip recovers fast
 * without bouncing; then a small kI only if it still settles low.
 */
@TeleOp(name = "Flywheel PIDF Tuning")
public class FlywheelPIDFTuning extends OpMode {

    private static final String[] NAMES = {"Target", "kV", "kS", "kP", "kI", "kD"};
    private static final double[] STEPS = {100, 10, 1, 0.1, 0.01, 0.001, 0.0001, 0.00001, 0.000001};

    private final iamyou.FlywheelPIDF pidf = new iamyou.FlywheelPIDF();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private DcMotorEx flywheel;
    private VoltageSensor battery;
    private double target = iamyou.FLYWHEEL_TARGET_TPS;
    private boolean on = false;
    private int selected = 0;
    private int stepIndex = 1;
    private double volts = iamyou.FLYWHEEL_NOMINAL_VOLTS;
    private double peakError = 0.0;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, iamyou.OUTTAKE_NAME);
        flywheel.setDirection(iamyou.OUTTAKE_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        battery = hardwareMap.voltageSensor.iterator().hasNext() ? hardwareMap.voltageSensor.iterator().next() : null;
        telemetry.addLine("Init complete. A = flywheel on/off.");
    }

    @Override
    public void start() {
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        if (gamepad1.aWasPressed()) {
            on = !on;
            peakError = 0.0;
        }
        if (gamepad1.yWasPressed()) {
            selected = (selected + 1) % NAMES.length;
        }
        if (gamepad1.dpadRightWasPressed()) {
            stepIndex = Math.max(0, stepIndex - 1);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            stepIndex = Math.min(STEPS.length - 1, stepIndex + 1);
        }
        if (gamepad1.dpadUpWasPressed()) {
            adjust(STEPS[stepIndex]);
        }
        if (gamepad1.dpadDownWasPressed()) {
            adjust(-STEPS[stepIndex]);
        }

        if (battery != null) {
            double v = battery.getVoltage();
            if (v > 7.0 && v < 16.0) {
                volts += (v - volts) * 0.2;
            }
        }

        double wanted = on ? (gamepad1.right_bumper ? Math.max(0.0, target - 300.0) : target) : 0.0;
        double velocity = flywheel.getVelocity();
        double power = pidf.update(wanted, velocity, volts, dt);
        flywheel.setPower(power);
        if (on) {
            peakError = Math.max(peakError, Math.abs(pidf.error()));
        }

        telemetry.addData("Flywheel", on ? (pidf.isReady() ? "READY" : "spinning up") : "OFF  (A to start)");
        telemetry.addData("Target | Actual", "%.0f | %.0f ticks/s", wanted, velocity);
        telemetry.addData("Error", "%.0f ticks/s  (worst %.0f)", pidf.error(), peakError);
        telemetry.addData("Power", "%.3f   battery %.2f V", power, volts);
        telemetry.addData("Terms", pidf.terms());
        telemetry.addLine("-----------------------------------------------");
        for (int i = 0; i < NAMES.length; i++) {
            telemetry.addData((i == selected ? ">> " : "   ") + NAMES[i], format(i));
        }
        telemetry.addData("Step", "%s  (D-pad L/R)", trim(STEPS[stepIndex]));
        telemetry.addLine("Y = next value, D-pad U/D = change it, RB = fake a shot dip");
        telemetry.addLine("Copy into iamyou.java: FLYWHEEL_TARGET_TPS, FLYWHEEL_KV, _KS, _KP, _KI, _KD");
    }

    @Override
    public void stop() {
        flywheel.setPower(0.0);
    }

    private void adjust(double step) {
        switch (selected) {
            case 0: target = Math.max(0.0, target + step); break;
            case 1: pidf.kV = Math.max(0.0, pidf.kV + step); break;
            case 2: pidf.kS = Math.max(0.0, pidf.kS + step); break;
            case 3: pidf.kP = Math.max(0.0, pidf.kP + step); break;
            case 4: pidf.kI = Math.max(0.0, pidf.kI + step); break;
            default: pidf.kD = Math.max(0.0, pidf.kD + step); break;
        }
    }

    private String format(int i) {
        switch (i) {
            case 0: return String.format(Locale.US, "%.0f ticks/s", target);
            case 1: return String.format(Locale.US, "%.7f  (= 1/%.0f)", pidf.kV, pidf.kV > 0 ? 1.0 / pidf.kV : 0.0);
            case 2: return trim(pidf.kS);
            case 3: return trim(pidf.kP);
            case 4: return trim(pidf.kI);
            default: return trim(pidf.kD);
        }
    }

    private static String trim(double v) {
        return String.format(Locale.US, "%.6f", v).replaceAll("0+$", "").replaceAll("\\.$", ".0");
    }
}
