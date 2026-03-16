package com.gnss.rtklib.positioning;

import com.gnss.rtklib.core.GTime;
import com.gnss.rtklib.core.MatrixUtil;
import com.gnss.rtklib.core.SatelliteUtil;
import com.gnss.rtklib.model.Ephemeris;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
import static com.gnss.rtklib.core.Constants.*;

class EphemerisCalcTest {

    @Test
    void eph2clkAtTocEqualsF0() {
        Ephemeris eph = new Ephemeris();
        GTime toc = GTime.epoch2time(new double[]{2024, 1, 1, 0, 0, 0});
        eph.toc = toc;
        eph.f0 = 1.5e-4;
        eph.f1 = 0.0;
        eph.f2 = 0.0;

        double clk = EphemerisCalc.eph2clk(toc, eph);
        assertEquals(1.5e-4, clk, 1e-15, "clock bias at toc should equal f0");
    }

    @Test
    void eph2clkWithDrift() {
        Ephemeris eph = new Ephemeris();
        GTime toc = GTime.epoch2time(new double[]{2024, 1, 1, 0, 0, 0});
        eph.toc = toc;
        eph.f0 = 1.0e-4;
        eph.f1 = 2.0e-12;
        eph.f2 = 0.0;

        // At toc + 100s, clock = f0 + f1*100 (after iteration convergence)
        GTime time = toc.add(100.0);
        double clk = EphemerisCalc.eph2clk(time, eph);
        // Expected: f0 + f1*t where t is approximately 100 - f0 (but f0 is tiny)
        double expected = 1.0e-4 + 2.0e-12 * 100.0;
        assertEquals(expected, clk, 1e-12, "clock with drift");
    }

    @Test
    void eph2posCircularOrbit() {
        // Construct a GPS ephemeris with circular orbit (e=0)
        Ephemeris eph = new Ephemeris();
        eph.sat = SatelliteUtil.satno(SYS_GPS, 1); // G01

        double sqrtA = 26560000.0; // semi-major axis ~26560 km
        eph.A = sqrtA * sqrtA;     // A stored as a^2 in RTKLIB? No -- A is the semi-major axis

        // Wait: looking at eph2pos, it uses eph.A directly as semi-major axis
        // sqrt(mu / (A * A * A)) -> needs A in meters
        eph.A = 26560000.0; // 26560 km semi-major axis

        GTime toe = GTime.epoch2time(new double[]{2024, 1, 1, 0, 0, 0});
        eph.toe = toe;
        eph.toc = toe;
        eph.toes = 0.0;
        eph.e = 0.0;
        eph.M0 = 0.0;
        eph.i0 = 55.0 * D2R;
        eph.OMG0 = 0.0;
        eph.omg = 0.0;
        eph.deln = 0.0;
        eph.OMGd = 0.0;
        eph.idot = 0.0;
        eph.crc = 0.0; eph.crs = 0.0;
        eph.cuc = 0.0; eph.cus = 0.0;
        eph.cic = 0.0; eph.cis = 0.0;
        eph.f0 = 0.0; eph.f1 = 0.0; eph.f2 = 0.0;
        eph.sva = 0;

        double[] rs = new double[6];
        double[] dts = new double[2];
        double[] var = new double[1];

        EphemerisCalc.eph2pos(toe, eph, rs, dts, var);

        // Position magnitude should be approximately sqrt(A) ... wait, A is 26560000 m
        // so the position magnitude should be ~26560000 m (circular orbit, r = A)
        double r = MatrixUtil.norm(rs, 3);
        assertEquals(26560000.0, r, 100.0,
                "position magnitude should be approximately the semi-major axis");
    }

    @Test
    void eph2posZeroSemiMajorAxisReturnsZero() {
        Ephemeris eph = new Ephemeris();
        eph.A = 0.0;

        double[] rs = new double[6];
        double[] dts = new double[2];
        double[] var = new double[1];

        EphemerisCalc.eph2pos(GTime.gpst2time(2295, 0), eph, rs, dts, var);
        assertEquals(0.0, rs[0]);
        assertEquals(0.0, rs[1]);
        assertEquals(0.0, rs[2]);
    }
}
