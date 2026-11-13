package org.firstinspires.ftc.teamcode.simulated;

import org.firstinspires.ftc.teamcode.pedroPathing.RedLeftTipAwarePreload;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;

/** {@link TipAwareAutoRun} on RedLeftTipAwarePreload, untipped and tipped. */
@RunWith(RobolectricTestRunner.class)
@Config(sdk = 28)
public class RedLeftTipAwarePreloadTest {
    @Test
    public void redLeftPreloadShootsAtOnceFromTheAudienceSideWhenTheHiveIsUntipped() throws Exception {
        TipAwareAutoRun.assertShootsFrom(new RedLeftTipAwarePreload(), TipAwareAutoRun.RED_FRONT, "match-setup", true,
                "audience side", TipAwareAutoRun.RED_FRONT);
    }

    @Test
    public void redLeftPreloadDrivesRoundToTheRearWhenTheHiveIsTipped() throws Exception {
        TipAwareAutoRun.assertShootsFrom(new RedLeftTipAwarePreload(), TipAwareAutoRun.RED_FRONT, "tipped-match-setup", true,
                "rear side", TipAwareAutoRun.RED_REAR);
    }
}
