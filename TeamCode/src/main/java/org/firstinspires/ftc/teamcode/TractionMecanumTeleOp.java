package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Robot-centric mecanum with Pinpoint traction control. Controls: see {@link TractionMecanumBase}. */
@TeleOp(name = "Traction Mecanum", group = "Linear OpMode")
public class TractionMecanumTeleOp extends TractionMecanumBase {
    public TractionMecanumTeleOp() {
        super(false);
    }
}
