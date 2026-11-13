package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeCommands {
    // Instant: set the powers once and finish, so the sequence moves on with the intake running.
    // A built command's done() is false unless told otherwise, which would hold the sequence here.
    public static Command startIntake(DcMotorEx intake, CRServo intakeServoLeft, CRServo intakeServoRight) {
        return Command.build()
                .setStart(() -> {
                    intake.setPower(1);
                    intakeServoLeft.setPower(-1);
                    intakeServoRight.setPower(-1);
                })
                .setDone(() -> true);
    }
    public static Command stopIntake(DcMotorEx intake, CRServo intakeServoLeft, CRServo intakeServoRight) {
        return Command.build()
                .setStart(() -> {
                    intake.setPower(0);
                    intakeServoLeft.setPower(0);
                    intakeServoRight.setPower(0);
                })
                .setDone(() -> true);
    }

}
