package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** BlueRight's start and full routine, shooting at whichever CELL the camera says is raised. */
@Autonomous(name = "BlueRight Tip-Aware", group = "Autonomous")
public class BlueRightTipAware extends TipAwareAuto {
    public BlueRightTipAware() {
        super(AllianceAuto.BLUE, Start.REAR);
    }
}
