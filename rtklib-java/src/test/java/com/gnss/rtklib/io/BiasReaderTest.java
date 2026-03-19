package com.gnss.rtklib.io;

import com.gnss.rtklib.core.SatelliteUtil;
import com.gnss.rtklib.core.SignalUtil;
import com.gnss.rtklib.model.Navigation;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.PrintWriter;
import java.nio.file.Path;

import static com.gnss.rtklib.core.Constants.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for SINEX BIA (phase bias) reader.
 */
class BiasReaderTest {

    @TempDir
    Path tempDir;

    private static final String SAMPLE_BIA =
        "%=BIA 1.00 WHU 26:059:00000 WHU 26:059:00000 26:060:00000 R\n" +
        "+BIAS/SOLUTION\n" +
        "*BIAS SVN_ PRN STATION__ OBS1 OBS2 BIAS_START____ BIAS_END______ UNIT __ESTIMATED_VALUE____ _STD_DEV___\n" +
        " OSB  G063 G01           L1W       2026:059:00000 2026:060:00300 ns       -2.1234              0.000000\n" +
        " OSB  G063 G01           L5Q       2026:059:00000 2026:060:00300 ns        0.0707              0.000000\n" +
        " OSB  G063 G01           C1W       2026:059:00000 2026:060:00300 ns       -1.5000              0.000000\n" +
        " OSB  E201 E05           L1X       2026:059:00000 2026:060:00300 ns        1.2345              0.000000\n" +
        " OSB  E201 E05           L5X       2026:059:00000 2026:060:00300 ns       -0.5678              0.000000\n" +
        " OSB  C201 C19           L1P       2026:059:00000 2026:060:00300 ns        0.9876              0.000000\n" +
        " OSB  C201 C19           L5P       2026:059:00000 2026:060:00300 ns       -0.1234              0.000000\n" +
        " OSB  G063 G01  STATION1 L1W       2026:059:00000 2026:060:00300 ns        9.9999              0.000000\n" +
        "-BIAS/SOLUTION\n";

    @Test
    void testParseGpsL1W() throws Exception {
        Path biaFile = tempDir.resolve("test.BIA");
        try (PrintWriter w = new PrintWriter(biaFile.toFile())) {
            w.print(SAMPLE_BIA);
        }

        Navigation nav = new Navigation();
        BiasReader.readBias(biaFile.toString(), nav);

        int sat = SatelliteUtil.satno(SYS_GPS, 1);
        int code = SignalUtil.obs2code("1W");
        assertTrue(code > 0, "CODE_L1W should be valid");

        double expected = -2.1234 * 1e-9 * CLIGHT;
        assertEquals(expected, nav.pbias[sat - 1][code], 1e-6,
                "GPS G01 L1W phase bias should be -2.1234 ns converted to meters");
    }

    @Test
    void testParseGpsL5Q() throws Exception {
        Path biaFile = tempDir.resolve("test.BIA");
        try (PrintWriter w = new PrintWriter(biaFile.toFile())) {
            w.print(SAMPLE_BIA);
        }

        Navigation nav = new Navigation();
        BiasReader.readBias(biaFile.toString(), nav);

        int sat = SatelliteUtil.satno(SYS_GPS, 1);
        int code = SignalUtil.obs2code("5Q");

        double expected = 0.0707 * 1e-9 * CLIGHT;
        assertEquals(expected, nav.pbias[sat - 1][code], 1e-6);
    }

    @Test
    void testParseGalileoL1X_L5X() throws Exception {
        Path biaFile = tempDir.resolve("test.BIA");
        try (PrintWriter w = new PrintWriter(biaFile.toFile())) {
            w.print(SAMPLE_BIA);
        }

        Navigation nav = new Navigation();
        BiasReader.readBias(biaFile.toString(), nav);

        int sat = SatelliteUtil.satno(SYS_GAL, 5);
        int codeL1 = SignalUtil.obs2code("1X");
        int codeL5 = SignalUtil.obs2code("5X");

        assertEquals(1.2345 * 1e-9 * CLIGHT, nav.pbias[sat - 1][codeL1], 1e-6);
        assertEquals(-0.5678 * 1e-9 * CLIGHT, nav.pbias[sat - 1][codeL5], 1e-6);
    }

    @Test
    void testParseBdsL1P_L5P() throws Exception {
        Path biaFile = tempDir.resolve("test.BIA");
        try (PrintWriter w = new PrintWriter(biaFile.toFile())) {
            w.print(SAMPLE_BIA);
        }

        Navigation nav = new Navigation();
        BiasReader.readBias(biaFile.toString(), nav);

        int sat = SatelliteUtil.satno(SYS_CMP, 19);
        int codeL1 = SignalUtil.obs2code("1P");
        int codeL5 = SignalUtil.obs2code("5P");

        assertEquals(0.9876 * 1e-9 * CLIGHT, nav.pbias[sat - 1][codeL1], 1e-6);
        assertEquals(-0.1234 * 1e-9 * CLIGHT, nav.pbias[sat - 1][codeL5], 1e-6);
    }

    @Test
    void testCodeBiasLinesSkipped() throws Exception {
        Path biaFile = tempDir.resolve("test.BIA");
        try (PrintWriter w = new PrintWriter(biaFile.toFile())) {
            w.print(SAMPLE_BIA);
        }

        Navigation nav = new Navigation();
        BiasReader.readBias(biaFile.toString(), nav);

        // C1W line starts with 'C', should be skipped by the 'L' check
        int sat = SatelliteUtil.satno(SYS_GPS, 1);
        int codeC1W = SignalUtil.obs2code("1W");
        // Only L1W should be stored, not C1W
        // The L1W phase bias should be the -2.1234 ns value, not the C1W line
        assertEquals(-2.1234 * 1e-9 * CLIGHT, nav.pbias[sat - 1][codeC1W], 1e-6);
    }

    @Test
    void testStationBiasSkipped() throws Exception {
        Path biaFile = tempDir.resolve("test.BIA");
        try (PrintWriter w = new PrintWriter(biaFile.toFile())) {
            w.print(SAMPLE_BIA);
        }

        Navigation nav = new Navigation();
        BiasReader.readBias(biaFile.toString(), nav);

        // The station-specific line (STATION1) should not override satellite bias
        int sat = SatelliteUtil.satno(SYS_GPS, 1);
        int code = SignalUtil.obs2code("1W");
        // Should still be the satellite value, not the station value (9.9999)
        assertEquals(-2.1234 * 1e-9 * CLIGHT, nav.pbias[sat - 1][code], 1e-6);
    }

    @Test
    void testNsToMetersConversion() throws Exception {
        Path biaFile = tempDir.resolve("test.BIA");
        try (PrintWriter w = new PrintWriter(biaFile.toFile())) {
            w.print(SAMPLE_BIA);
        }

        Navigation nav = new Navigation();
        BiasReader.readBias(biaFile.toString(), nav);

        // 1 nanosecond = 1e-9 * CLIGHT ≈ 0.2998 m
        int sat = SatelliteUtil.satno(SYS_GPS, 1);
        int code = SignalUtil.obs2code("5Q");
        double bias_m = nav.pbias[sat - 1][code];
        double expected_m = 0.0707 * 1e-9 * CLIGHT;
        assertEquals(expected_m, bias_m, 1e-8);
        // Verify magnitude is reasonable (0.0707 ns ≈ 0.021 m)
        assertTrue(Math.abs(bias_m) < 1.0, "Phase bias should be < 1 meter");
        assertTrue(Math.abs(bias_m) > 0.001, "Phase bias should be > 1 mm");
    }

}
