package com.gnss.rtklib.positioning;

import com.gnss.rtklib.core.GTime;
import com.gnss.rtklib.core.SatelliteUtil;
import com.gnss.rtklib.io.ClkReader;
import com.gnss.rtklib.io.Sp3Reader;
import com.gnss.rtklib.model.Navigation;
import org.junit.jupiter.api.Test;

import java.io.File;

import static com.gnss.rtklib.core.Constants.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for precise ephemeris interpolation.
 */
class PreciseEphemerisTest {

    private static final String SP3_FILE = "../test/data/sp3/igs15904.sp3";
    private static final String CLK_FILE = "../test/data/sp3/igs15904.clk";

    @Test
    void testNevilleInterpolation() {
        // Simple test: interpolate y = x^2 at x=0 from points (-2,4),(-1,1),(0,0),(1,1),(2,4)
        double[] x = {-2, -1, 0, 1, 2};
        double[] y = { 4,  1, 0, 1, 4};
        double result = PreciseEphemeris.interppol(x, y, 5);
        assertEquals(0.0, result, 1E-10, "Neville interpolation of x^2 at 0");
    }

    @Test
    void testNevilleInterpolationMidpoint() {
        // Interpolate y = x^2 at x=0.5 from points 0,1,2,3,4
        double[] x = {-0.5, 0.5, 1.5, 2.5, 3.5};
        double[] y = new double[5];
        for (int i = 0; i < 5; i++) y[i] = (x[i] + 0.5) * (x[i] + 0.5); // y = (x+0.5)^2
        double result = PreciseEphemeris.interppol(x, y, 5);
        // At x=0, y should be 0.5^2 = 0.25
        assertEquals(0.25, result, 1E-10);
    }

    @Test
    void testPeph2pos() throws Exception {
        File sp3f = new File(SP3_FILE);
        File clkf = new File(CLK_FILE);
        if (!sp3f.exists()) {
            System.out.println("SP3 file not found, skipping: " + sp3f.getAbsolutePath());
            return;
        }

        Navigation nav = new Navigation();
        Sp3Reader.readSp3(SP3_FILE, nav);
        if (clkf.exists()) ClkReader.readClk(CLK_FILE, nav);

        assertTrue(nav.peph.size() >= 11, "Need at least NMAX+1 epochs");

        // Use a time in the middle of the SP3 data
        GTime midTime = nav.peph.get(nav.peph.size() / 2).time;

        // Test GPS G01
        int g01 = SatelliteUtil.satno(SYS_GPS, 1);
        double[] rs = new double[6];
        double[] dts = new double[2];
        double[] var = new double[1];

        int ret = PreciseEphemeris.peph2pos(midTime, g01, nav, 0, rs, dts, var);
        assertEquals(1, ret, "peph2pos should succeed for G01");

        // Position should be at GPS orbit altitude (~26000 km)
        double r = Math.sqrt(rs[0] * rs[0] + rs[1] * rs[1] + rs[2] * rs[2]);
        assertTrue(r > 20000e3 && r < 30000e3,
                   String.format("G01 orbit radius %.0f m", r));

        // Velocity should be reasonable (GPS orbit ~3.9 km/s)
        double v = Math.sqrt(rs[3] * rs[3] + rs[4] * rs[4] + rs[5] * rs[5]);
        assertTrue(v > 1000 && v < 5000,
                   String.format("G01 velocity %.1f m/s", v));

        // Clock should be reasonable
        assertTrue(Math.abs(dts[0]) < 0.1, "Clock bias should be < 0.1 s");

        System.out.printf("G01 at mid-epoch: r=%.0f km, v=%.1f m/s, clk=%.9f s%n",
                          r / 1e3, v, dts[0]);
    }

    @Test
    void testPeph2posInterpolationAccuracy() throws Exception {
        File sp3f = new File(SP3_FILE);
        if (!sp3f.exists()) return;

        Navigation nav = new Navigation();
        Sp3Reader.readSp3(SP3_FILE, nav);

        // Interpolate at an SP3 epoch — should match the SP3 data closely
        int epochIdx = nav.peph.size() / 2;
        GTime exactTime = nav.peph.get(epochIdx).time;

        int g05 = SatelliteUtil.satno(SYS_GPS, 5);
        double[] sp3Pos = nav.peph.get(epochIdx).pos[g05 - 1];
        if (sp3Pos[0] == 0.0 && sp3Pos[1] == 0.0) {
            System.out.println("G05 not available at test epoch, skipping");
            return;
        }

        double[] rs = new double[6];
        double[] dts = new double[2];
        int ret = PreciseEphemeris.peph2pos(exactTime, g05, nav, 0, rs, dts, null);
        assertEquals(1, ret);

        // At exact SP3 epoch, interpolation error should be very small (<1 mm)
        for (int i = 0; i < 3; i++) {
            assertEquals(sp3Pos[i], rs[i], 1.0,
                         String.format("Position component %d diff: %.6f m",
                                       i, Math.abs(sp3Pos[i] - rs[i])));
        }

        System.out.printf("Interpolation at exact epoch: dX=%.6f dY=%.6f dZ=%.6f m%n",
                          rs[0] - sp3Pos[0], rs[1] - sp3Pos[1], rs[2] - sp3Pos[2]);
    }
}
