package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Field-centric mecanum with Pinpoint traction control. Forward is the way the robot faced at INIT; Options makes
 * the robot's current front forward. Controls: see {@link TractionMecanumBase}.
 */
@TeleOp(name = "Traction Field Mecanum", group = "Linear OpMode")
public class TractionFieldMecanumTeleOp extends TractionMecanumBase {
    public TractionFieldMecanumTeleOp() {
        super(true);
    }
}
