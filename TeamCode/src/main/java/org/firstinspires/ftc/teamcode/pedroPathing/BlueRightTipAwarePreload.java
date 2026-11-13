package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** BlueRight's start: shoots its pre-loads at whichever CELL is raised, no earlier than 15 s in, then parks. */
@Autonomous(name = "BlueRight Tip-Aware Preload", group = "Autonomous")
public class BlueRightTipAwarePreload extends TipAwareAuto {
    public BlueRightTipAwarePreload() {
        super(AllianceAuto.BLUE, Start.REAR, Routine.PRELOAD_AND_PARK);
    }
}
