package com.gnss.rtklib.io;

import com.gnss.rtklib.core.SatelliteUtil;
import com.gnss.rtklib.model.Navigation;
import org.junit.jupiter.api.Test;

import java.io.File;

import static com.gnss.rtklib.core.Constants.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for SP3 and CLK file readers.
 */
class Sp3ReaderTest {

    private static final String SP3_FILE = "../test/data/sp3/igs15904.sp3";
    private static final String CLK_FILE = "../test/data/sp3/igs15904.clk";

    @Test
    void testReadSp3() throws Exception {
        File f = new File(SP3_FILE);
        if (!f.exists()) {
            System.out.println("SP3 file not found, skipping: " + f.getAbsolutePath());
            return;
        }

        Navigation nav = new Navigation();
        Sp3Reader.readSp3(SP3_FILE, nav);

        // Should have loaded multiple epochs
        assertTrue(nav.peph.size() > 0, "Should have loaded SP3 epochs");
        System.out.printf("SP3: loaded %d epochs%n", nav.peph.size());

        // First epoch should have some valid satellite positions
        int validSats = 0;
        for (int i = 0; i < MAXSAT; i++) {
            if (nav.peph.get(0).pos[i][0] != 0.0 ||
                nav.peph.get(0).pos[i][1] != 0.0 ||
                nav.peph.get(0).pos[i][2] != 0.0) {
                validSats++;
            }
        }
        assertTrue(validSats > 0, "Should have valid satellite positions");
        System.out.printf("SP3: first epoch has %d satellites%n", validSats);

        // Check GPS G01 position is in reasonable range (Earth orbit ~26000 km)
        int g01 = SatelliteUtil.satno(SYS_GPS, 1);
        double[] pos = nav.peph.get(0).pos[g01 - 1];
        double r = Math.sqrt(pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]);
        assertTrue(r > 20000e3 && r < 30000e3,
                   String.format("G01 radius %.0f m should be ~26000km", r));
        System.out.printf("SP3: G01 radius = %.0f km%n", r / 1e3);
    }

    @Test
    void testReadClk() throws Exception {
        File f = new File(CLK_FILE);
        if (!f.exists()) {
            System.out.println("CLK file not found, skipping: " + f.getAbsolutePath());
            return;
        }

        Navigation nav = new Navigation();
        ClkReader.readClk(CLK_FILE, nav);

        assertTrue(nav.pclk.size() > 0, "Should have loaded CLK epochs");
        System.out.printf("CLK: loaded %d epochs%n", nav.pclk.size());

        // Check that some satellites have non-zero clock values
        int validClks = 0;
        for (int i = 0; i < MAXSAT; i++) {
            if (nav.pclk.get(0).clk[i] != 0.0) validClks++;
        }
        assertTrue(validClks > 0, "Should have valid clock values");
        System.out.printf("CLK: first epoch has %d satellite clocks%n", validClks);

        // GPS clock values should be small (order of ~10^-4 to 10^-3 seconds)
        int g01 = SatelliteUtil.satno(SYS_GPS, 1);
        double clk = nav.pclk.get(0).clk[g01 - 1];
        if (clk != 0.0) {
            assertTrue(Math.abs(clk) < 1.0, "Clock value should be < 1 second");
            System.out.printf("CLK: G01 clock = %.12f s%n", clk);
        }
    }

    @Test
    void testSp3ClkCombined() throws Exception {
        File sp3f = new File(SP3_FILE);
        File clkf = new File(CLK_FILE);
        if (!sp3f.exists() || !clkf.exists()) {
            System.out.println("SP3/CLK files not found, skipping");
            return;
        }

        Navigation nav = new Navigation();
        Sp3Reader.readSp3(SP3_FILE, nav);
        ClkReader.readClk(CLK_FILE, nav);

        assertTrue(nav.peph.size() > 0);
        assertTrue(nav.pclk.size() > 0);

        System.out.printf("SP3 epochs: %d, CLK epochs: %d%n", nav.peph.size(), nav.pclk.size());
        // Both should have loaded successfully
        assertTrue(nav.peph.size() > 10, "SP3 should have many epochs");
        assertTrue(nav.pclk.size() > 10, "CLK should have many epochs");
    }
}
