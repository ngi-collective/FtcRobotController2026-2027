package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Blue, against the audience wall facing the lowered CELL: RedRight turned half a turn. */
@Autonomous(name = "BlueRight", group = "Autonomous")
public class BlueRight extends AllianceAuto {
    public BlueRight() {
        super(BLUE, Start.BEHIND_LOWERED_CELL, Routine.FULL);
    }
}
