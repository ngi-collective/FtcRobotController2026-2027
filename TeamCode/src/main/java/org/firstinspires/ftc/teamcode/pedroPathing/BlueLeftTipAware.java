package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** BlueLeft's start and full routine, shooting at whichever CELL the camera says is raised. */
@Autonomous(name = "BlueLeft Tip-Aware", group = "Autonomous")
public class BlueLeftTipAware extends TipAwareAuto {
    public BlueLeftTipAware() {
        super(AllianceAuto.BLUE, Start.FRONT);
    }
}
