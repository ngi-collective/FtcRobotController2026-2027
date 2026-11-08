package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Set;

public class shootCommand implements Command {
    private DcMotorEx flywheel;
    private CRServo staging;
    private long startTime;

    public shootCommand(DcMotorEx flywheel, CRServo staging) {
        this.flywheel = flywheel;
        this.staging = staging;
    }

    @Override
    public Set<Object> requirements() {
        return Set.of(flywheel,staging);
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
    }

    @Override
    public void execute() {
        flywheel.setVelocity(1200);
        staging.setPower(1);
    }

    @Override
    public boolean done() {
        return System.currentTimeMillis() - startTime >= 5000;
    }

    @Override
    public void end(EndCondition endCondition) {
        flywheel.setVelocity(0);
        staging.setPower(0);
    }
}
