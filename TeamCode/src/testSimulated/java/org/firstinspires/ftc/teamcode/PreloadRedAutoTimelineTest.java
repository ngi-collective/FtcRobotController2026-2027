package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.pedroPathing.RedLeftPreload;
import org.firstinspires.ftc.teamcode.pedroPathing.RedRightPreload;
import org.junit.jupiter.api.Test;

/** {@link AllianceAutoRun} on RedLeftPreload, RedRightPreload. */
class PreloadRedAutoTimelineTest {
    @Test
    void redLeftPreload() throws Exception {
        AllianceAutoRun.run(new RedLeftPreload(), 56, 9.5, 90, true);
    }

    @Test
    void redRightPreload() throws Exception {
        AllianceAutoRun.run(new RedRightPreload(), 62, 134.5, 270, true);
    }
}
