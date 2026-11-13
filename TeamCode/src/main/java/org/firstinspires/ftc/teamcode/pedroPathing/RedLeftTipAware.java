package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** RedLeft's start and full routine, shooting at whichever CELL the camera says is raised. */
@Autonomous(name = "RedLeft Tip-Aware", group = "Autonomous")
public class RedLeftTipAware extends TipAwareAuto {
    public RedLeftTipAware() {
        super(AllianceAuto.RED, Start.FRONT, Routine.FULL);
    }
}
