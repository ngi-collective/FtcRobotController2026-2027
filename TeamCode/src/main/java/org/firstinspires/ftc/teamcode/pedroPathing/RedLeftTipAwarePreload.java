package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** RedLeft's start: shoots its pre-loads at whichever CELL is raised at once, then parks. */
@Autonomous(name = "RedLeft Tip-Aware Preload", group = "Autonomous")
public class RedLeftTipAwarePreload extends TipAwareAuto {
    public RedLeftTipAwarePreload() {
        super(AllianceAuto.RED, Start.FRONT, Routine.PRELOAD_AND_PARK);
    }
}
