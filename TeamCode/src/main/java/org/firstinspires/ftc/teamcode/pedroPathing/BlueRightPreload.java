package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** BlueRight's start: waits 15 s, shoots the pre-loads, then parks. */
@Autonomous(name = "BlueRight Preload", group = "Autonomous")
public class BlueRightPreload extends AllianceAuto {
    public BlueRightPreload() {
        super(BLUE, Start.BEHIND_LOWERED_CELL, Routine.PRELOAD_AND_PARK);
    }
}
