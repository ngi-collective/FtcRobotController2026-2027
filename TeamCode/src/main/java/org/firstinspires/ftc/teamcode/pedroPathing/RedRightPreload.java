package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** RedRight's start: waits 15 s, shoots the pre-loads, then parks. */
@Autonomous(name = "RedRight Preload", group = "Autonomous")
public class RedRightPreload extends AllianceAuto {
    public RedRightPreload() {
        super(RED, Start.BEHIND_LOWERED_CELL, Routine.PRELOAD_AND_PARK);
    }
}
