package com.gnss.rtklib.io;

import com.gnss.rtklib.core.SatelliteUtil;
import com.gnss.rtklib.model.Navigation;
import com.gnss.rtklib.model.ObsData;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIf;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import static com.gnss.rtklib.core.Constants.*;
import static org.junit.jupiter.api.Assertions.*;

class RinexReaderTest {

    private static Path testDataDir;

    @BeforeAll
    static void findTestData() {
        // rtklib-java/ is the project root; test data is at ../../test/data/rinex/
        Path projectRoot = Path.of("").toAbsolutePath();
        testDataDir = projectRoot.resolve("../test/data/rinex").normalize();
    }

    static boolean testDataExists() {
        return Files.isDirectory(testDataDir);
    }

    @Test
    @EnabledIf("testDataExists")
    void readNavFile30400920() throws IOException {
        Path navFile = testDataDir.resolve("30400920.05n");
        assertTrue(Files.exists(navFile), "Nav file should exist: " + navFile);

        Navigation nav = RinexReader.readNav(navFile.toString(), null);
        assertNotNull(nav);
        assertTrue(nav.eph.size() > 0, "Should have ephemerides, got " + nav.eph.size());
    }

    @Test
    @EnabledIf("testDataExists")
    void readObsFile30400920() throws IOException {
        Path obsFile = testDataDir.resolve("30400920.05o");
        assertTrue(Files.exists(obsFile), "Obs file should exist: " + obsFile);

        Navigation nav = new Navigation();
        List<List<ObsData>> obs = RinexReader.readObs(obsFile.toString(), nav);
        assertNotNull(obs);
        assertTrue(obs.size() > 0, "Should have observation epochs");

        // First epoch should have at least one satellite
        List<ObsData> firstEpoch = obs.get(0);
        assertTrue(firstEpoch.size() > 0, "First epoch should have observations");

        // Check that at least one satellite is GPS
        boolean hasGps = false;
        for (ObsData od : firstEpoch) {
            int[] sp = SatelliteUtil.satsys(od.sat);
            if (sp[0] == SYS_GPS) {
                hasGps = true;
                break;
            }
        }
        assertTrue(hasGps, "First epoch should contain GPS satellites");
    }

    @Test
    @EnabledIf("testDataExists")
    void readNavFile07590920() throws IOException {
        Path navFile = testDataDir.resolve("07590920.05n");
        assertTrue(Files.exists(navFile), "Nav file should exist: " + navFile);

        Navigation nav = RinexReader.readNav(navFile.toString(), null);
        assertNotNull(nav);
        assertTrue(nav.eph.size() > 0, "Should have ephemerides");
    }

    @Test
    @EnabledIf("testDataExists")
    void readObsFile07590920() throws IOException {
        Path obsFile = testDataDir.resolve("07590920.05o");
        assertTrue(Files.exists(obsFile), "Obs file should exist: " + obsFile);

        Navigation nav = new Navigation();
        List<List<ObsData>> obs = RinexReader.readObs(obsFile.toString(), nav);
        assertNotNull(obs);
        assertTrue(obs.size() > 0, "Should have observation epochs");

        List<ObsData> firstEpoch = obs.get(0);
        assertTrue(firstEpoch.size() > 0, "First epoch should have observations");
    }
}
