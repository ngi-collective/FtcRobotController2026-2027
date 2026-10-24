package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.simulated.VerityRobot;
import org.junit.jupiter.api.Test;
import org.ngicollective.ftcsim.camera.GameElement;
import org.ngicollective.ftcsim.camera.SimulatedScene;
import org.ngicollective.ftcsim.hardware.FakeHardwareMap;
import org.ngicollective.ftcsim.physics.FieldPhysics;
import org.ngicollective.ftcsim.season.BioBuzzElements;
import org.ngicollective.ftcsim.sim.FieldConfig;
import org.ngicollective.ftcsim.sim.Pose2d;
import org.ngicollective.ftcsim.sim.RobotConfig;
import org.ngicollective.ftcsim.sim.SimConfigFiles;

import java.util.Collections;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * That a ball the pickup robot drives over leaves the field.
 *
 * <p>Collection is the world's answer, not a flag: the mouth is a box bolted to the robot, and a
 * ball whose surface reaches into it is gone from the field. This is the one mechanism the full
 * pickup Auto's confirmation leans on, so it is worth its own test.</p>
 */
class BallCaptureTest {

    private static final double TICK = 0.02;

    @Test
    void aBallAtTheMouthIsCollected() {
        RobotConfig config = SimConfigFiles.robot("pickup");
        GameElement ball = BioBuzzElements.pollen(0.24, 0.0);
        SimulatedScene scene = SimulatedScene.of().withElements(Collections.singletonList(ball));

        FakeHardwareMap hardware = new VerityRobot(config, FieldConfig.standard(), scene).create();
        FieldPhysics world = FieldPhysics.of(scene.elements(), scene.structures(),
                FieldConfig.standard(), config);
        hardware.setPhysics(world);
        hardware.drive().setPose(Pose2d.ORIGIN);

        hardware.advance(TICK);

        assertEquals(0, world.elements().size(),
                "a ball whose surface is inside the mouth should be collected");
    }

    @Test
    void aBallOutOfReachIsLeftAlone() {
        RobotConfig config = SimConfigFiles.robot("pickup");
        // Two metres ahead: nowhere near the mouth.
        GameElement ball = BioBuzzElements.pollen(2.0, 0.0);
        SimulatedScene scene = SimulatedScene.of().withElements(Collections.singletonList(ball));

        FakeHardwareMap hardware = new VerityRobot(config, FieldConfig.standard(), scene).create();
        FieldPhysics world = FieldPhysics.of(scene.elements(), scene.structures(),
                FieldConfig.standard(), config);
        hardware.setPhysics(world);
        hardware.drive().setPose(Pose2d.ORIGIN);

        hardware.advance(TICK);

        assertEquals(1, world.elements().size(),
                "a ball two metres away should stay on the field");
    }
}
