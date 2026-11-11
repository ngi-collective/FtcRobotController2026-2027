package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** RedLeft's start: shoots the pre-loads at once, then parks. */
@Autonomous(name = "RedLeft Preload", group = "Autonomous")
public class RedLeftPreload extends AllianceAuto {
    public RedLeftPreload() {
        super(RED, Start.FACING_RAISED_CELL, Routine.PRELOAD_AND_PARK);
    }
}
