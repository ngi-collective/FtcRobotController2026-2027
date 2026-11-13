package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** BlueLeft's start: shoots its pre-loads at whichever CELL is raised at once, then parks. */
@Autonomous(name = "BlueLeft Tip-Aware Preload", group = "Autonomous")
public class BlueLeftTipAwarePreload extends TipAwareAuto {
    public BlueLeftTipAwarePreload() {
        super(AllianceAuto.BLUE, Start.FRONT, Routine.PRELOAD_AND_PARK);
    }
}
