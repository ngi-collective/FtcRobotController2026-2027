package org.firstinspires.ftc.teamcode;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.ngicollective.testframework.camera.FieldTag;
import org.ngicollective.testframework.camera.GameElement;
import org.ngicollective.testframework.camera.SimulatedScene;
import org.ngicollective.testframework.camera.TagCluster;
import org.ngicollective.testframework.season.BioBuzzField;
import org.ngicollective.testframework.sim.ScenarioConfig;
import org.ngicollective.testframework.sim.SimConfigFiles;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * That the committed scenarios load and describe what their names claim.
 *
 * <p>A scenario is data, and data that nothing parses until an emulator boots is data that rots.
 * These load the real files, so a typo in a ball's kind or a malformed brace fails here in
 * milliseconds instead of on a device four minutes later.</p>
 */
class ScenarioFilesTest {

    @Test
    void everyCommittedScenarioStillLoads() throws IOException {
        List<Path> files = scenarioFiles();
        assertTrue(files.size() >= 2, "expected the committed scenarios, found " + files);

        for (Path file : files) {
            String name = file.getFileName().toString().replace(".json", "");
            ScenarioConfig scenario = SimConfigFiles.scenario(name);
            assertTrue(!scenario.name().isEmpty(), name + " should name itself");

            // Every scenario is still the BioBuzz field, however it is arranged.
            SimulatedScene scene = scenario.scene();
            assertEquals(4, scene.clusters().size(), name + " should have four CELLs");
            int tags = 0;
            for (TagCluster cluster : scene.clusters()) {
                tags += cluster.tags().size();
            }
            assertEquals(16, tags, name + " should have sixteen tags");
        }
    }

    @Test
    void tippingBothHivesBackSwapsEveryCellsHeight() {
        // The point of that scenario: the arrangement the CAD did not capture.
        double officialRedAudience = heightOf(BioBuzzField.official(), BioBuzzField.RED_AUDIENCE);
        double tippedRedAudience = heightOf(
                SimConfigFiles.scenario("hives-tipped-back").scene(), BioBuzzField.RED_AUDIENCE);

        assertTrue(tippedRedAudience < officialRedAudience,
                "tipped back, the red audience CELL should now be the low one: "
                        + tippedRedAudience + " vs " + officialRedAudience);
    }

    @Test
    void thePracticeScenarioPlacesBallsOfEachKind() {
        List<GameElement> elements = SimConfigFiles.scenario("practice-balls").elements();

        assertEquals(6, elements.size());
        int pollen = 0;
        int nectar = 0;
        for (GameElement element : elements) {
            if ("POLLEN".equals(element.name())) {
                pollen++;
                // A scenario chooses where a ball is, never how big it is: that is the season's.
                assertEquals(GameElement.POLLEN_DIAMETER_METRES,
                        element.radiusMetres() * 2.0, 1e-9);
            } else {
                nectar++;
                assertEquals(GameElement.NECTAR_DIAMETER_METRES,
                        element.radiusMetres() * 2.0, 1e-9);
            }
        }
        assertEquals(3, pollen);
        assertEquals(3, nectar);
    }

    @Test
    void anElementThatIsNotAScoringElementIsRefused(@TempDir Path directory) throws IOException {
        // Strict loading earns its place here: a "GOLD_NECTAR" that silently became a grey ball
        // would render a field the season does not have, and every colour assertion after it
        // would be measuring fiction.
        Path file = directory.resolve("broken.json");
        Files.write(file, ("{\"version\": 1, \"name\": \"broken\", \"elements\": {"
                + "\"mystery\": {\"kind\": \"GOLD_NECTAR\", \"xMetres\": 0, \"yMetres\": 0}}}")
                .getBytes(StandardCharsets.UTF_8));

        IllegalArgumentException failure =
                assertThrows(IllegalArgumentException.class, () -> ScenarioConfig.load(file));
        assertTrue(failure.getMessage().contains("GOLD_NECTAR"), failure.getMessage());
        assertTrue(failure.getMessage().contains("POLLEN"),
                "the failure should say what is allowed: " + failure.getMessage());
    }

    @Test
    void aMisspelledScenarioNameSaysWhereItLooked() {
        // Quietly falling back to the official field would let a test pass against an arrangement
        // nobody asked for.
        IllegalArgumentException failure = assertThrows(IllegalArgumentException.class,
                () -> SimConfigFiles.scenario("practise-balls"));

        assertTrue(failure.getMessage().contains("practise-balls"), failure.getMessage());
        assertTrue(failure.getMessage().contains("scenarios"), failure.getMessage());
    }

    private static double heightOf(SimulatedScene scene, String clusterName) {
        for (TagCluster cluster : scene.clusters()) {
            if (cluster.name().equals(clusterName)) {
                List<FieldTag> tags = cluster.tags();
                return tags.get(0).pose().position().z();
            }
        }
        throw new AssertionError("no cluster named " + clusterName);
    }

    private static List<Path> scenarioFiles() throws IOException {
        try (java.util.stream.Stream<Path> files =
                     Files.list(SimConfigFiles.scenarioDirectory())) {
            return files.filter(path -> path.toString().endsWith(".json"))
                    .sorted()
                    .collect(java.util.stream.Collectors.toList());
        }
    }
}
