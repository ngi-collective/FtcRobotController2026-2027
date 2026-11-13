package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.pedroPathing.BlueLeftPreload;
import org.firstinspires.ftc.teamcode.pedroPathing.BlueRightPreload;
import org.junit.jupiter.api.Test;

/** {@link AllianceAutoRun} on BlueLeftPreload, BlueRightPreload. */
class PreloadBlueAutoTimelineTest {
    @Test
    void blueLeftPreload() throws Exception {
        AllianceAutoRun.run(new BlueLeftPreload(), 88, 134.5, 270, false);
    }

    @Test
    void blueRightPreload() throws Exception {
        AllianceAutoRun.run(new BlueRightPreload(), 82, 9.5, 90, false);
    }
}
