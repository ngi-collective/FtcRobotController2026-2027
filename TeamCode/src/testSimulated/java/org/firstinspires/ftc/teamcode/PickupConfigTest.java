package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.simulated.VerityRobot;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.ngicollective.ftcsim.hardware.FakeHardwareMap;
import org.ngicollective.ftcsim.sim.FieldConfig;
import org.ngicollective.ftcsim.sim.RobotConfig;
import org.ngicollective.ftcsim.sim.SimConfigFiles;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * That the committed pickup configuration describes the second robot the sim can drive.
 *
 * <p>Configurations are data, and data that nothing parses until someone runs the dashboard rots
 * silently. These load the real file, so a typo in the camera pitch or a motor name fails here in
 * milliseconds rather than on a device four minutes later.</p>
 */
class PickupConfigTest {

    private static final String PROPERTY = "sim.robot";

    @AfterEach
    void clearProperty() {
        System.clearProperty(PROPERTY);
    }

    @Test
    void thePickupRobotLooksDownAtTheFloorAndThrowsNothing() {
        RobotConfig config = SimConfigFiles.robot("pickup");
        assertEquals(-10.0, config.camera().mount().pitchDegrees(), 1e-9,
                "the ball camera pitches down at the floor");
        assertEquals(0.25, config.camera().mount().heightMetres(), 1e-9,
                "the ball camera sits up where it can see the field ahead of the mouth");
        assertTrue(config.launchers().isEmpty(), "a pickup robot throws nothing");
    }

    @Test
    void thePickupRobotBuildsAWheeledCameraRobot() {
        RobotConfig config = SimConfigFiles.robot("pickup");
        FakeHardwareMap hardware = new VerityRobot(config, FieldConfig.standard()).create();
        for (String wheel : new String[]{"FL", "FR", "BL", "BR"}) {
            assertNotNull(hardware.motor(wheel), "missing wheel " + wheel);
        }
        assertNotNull(hardware.webcam("Webcam 1"), "missing the ball camera");
    }

    @Test
    void theConfigNamePropertySelectsWhichRobotDrives() {
        System.setProperty(PROPERTY, "pickup");
        assertEquals("Pickup", new VerityRobot().name());
    }
}
