package org.firstinspires.ftc.teamcode.simulated;

import org.firstinspires.ftc.teamcode.pedroPathing.RedLeftTipAware;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;

/** {@link TipAwareAutoRun} on RedLeftTipAware, untipped and tipped. */
@RunWith(RobolectricTestRunner.class)
@Config(sdk = 28)
public class RedLeftTipAwareTest {
    @Test
    public void redLeftShootsFromTheAudienceSideWhenTheHiveIsUntipped() throws Exception {
        TipAwareAutoRun.assertShootsFrom(new RedLeftTipAware(), TipAwareAutoRun.RED_FRONT, "match-setup", true,
                "audience side", TipAwareAutoRun.RED_FRONT);
    }

    @Test
    public void redLeftDrivesRoundToTheRearWhenTheHiveIsTipped() throws Exception {
        TipAwareAutoRun.assertShootsFrom(new RedLeftTipAware(), TipAwareAutoRun.RED_FRONT, "tipped-match-setup", true,
                "rear side", TipAwareAutoRun.RED_REAR);
    }
}
