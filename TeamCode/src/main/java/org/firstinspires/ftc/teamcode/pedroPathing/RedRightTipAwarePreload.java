package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** RedRight's start: shoots its pre-loads at whichever CELL is raised, no earlier than 15 s in, then parks. */
@Autonomous(name = "RedRight Tip-Aware Preload", group = "Autonomous")
public class RedRightTipAwarePreload extends TipAwareAuto {
    public RedRightTipAwarePreload() {
        super(AllianceAuto.RED, Start.REAR, Routine.PRELOAD_AND_PARK);
    }
}
