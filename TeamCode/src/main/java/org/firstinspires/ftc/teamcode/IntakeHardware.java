package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * The intake motor: configured as "INTAKE" in the robot configuration, plugged into
 * motor port 2 of the Expansion Hub. (The code only knows the config name, so the
 * configuration must have a motor named exactly INTAKE on that port.)
 */
public class IntakeHardware {

    public static final String NAME = "INTAKE";

    /**
     * Flip this if positive power (right trigger) spits pieces out instead of pulling them in.
     * With a big-gear-drives-small-gear mesh the roller turns opposite to the motor, so
     * the first run may need this set to true.
     */
    public static final boolean REVERSED = false;

    public final DcMotorEx motor;

    private IntakeHardware(DcMotorEx motor) {
        this.motor = motor;
    }

    public static IntakeHardware init(HardwareMap hardwareMap) {
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, NAME);
        m.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        // Open-loop power: no encoder needed, and the ramping happens in IntakeLogic.
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return new IntakeHardware(m);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public double getCurrentAmps() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    public void stop() {
        motor.setPower(0.0);
    }
}