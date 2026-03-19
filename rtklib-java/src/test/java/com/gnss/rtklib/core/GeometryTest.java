package com.gnss.rtklib.core;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
import static com.gnss.rtklib.core.Constants.*;

class GeometryTest {

    @Test
    void geodistKnownPositions() {
        // Satellite at GPS orbit altitude (~26560 km) directly above equator/prime meridian
        double[] rs = {26560000.0, 0.0, 0.0};
        // Receiver at equator/prime meridian on surface
        double[] rr = {RE_WGS84, 0.0, 0.0};
        double[] e = new double[3];

        double dist = Geometry.geodist(rs, rr, e);
        // Expected: ~26560000 - RE_WGS84 = ~20181863 m (plus tiny Sagnac)
        double expectedApprox = 26560000.0 - RE_WGS84;
        assertEquals(expectedApprox, dist, 1.0, "geometric distance");

        // Line-of-sight should be approximately along X axis
        assertEquals(1.0, e[0], 0.01);
        assertEquals(0.0, e[1], 0.01);
        assertEquals(0.0, e[2], 0.01);
    }

    @Test
    void geodistRejectsSatBelowEarthRadius() {
        double[] rs = {1000.0, 0.0, 0.0}; // inside Earth
        double[] rr = {RE_WGS84, 0.0, 0.0};
        double[] e = new double[3];

        double dist = Geometry.geodist(rs, rr, e);
        assertTrue(dist < 0, "should reject satellite below Earth surface");
    }

    @Test
    void satazelZenithSatellite() {
        // Receiver at equator/prime meridian
        double[] pos = {0.0, 0.0, 0.0};
        // Unit vector pointing straight up (radial direction at equator = +X in ECEF)
        double[] e = {1.0, 0.0, 0.0};
        double[] azel = new double[2];

        double el = Geometry.satazel(pos, e, azel);
        assertEquals(PI / 2.0, el, 1e-6, "elevation at zenith");
        assertEquals(PI / 2.0, azel[1], 1e-6);
    }

    @Test
    void satazelHorizonSatellite() {
        // Receiver at equator/prime meridian, sat along the equator (East direction)
        double[] pos = {0.0, 0.0, 0.0};
        // East direction at equator/prime meridian is ECEF +Y
        double[] e = {0.0, 1.0, 0.0};
        double[] azel = new double[2];

        double el = Geometry.satazel(pos, e, azel);
        assertEquals(0.0, el, 1e-6, "elevation at horizon");
        assertEquals(PI / 2.0, azel[0], 1e-6, "azimuth should be 90 deg (East)");
    }

    @Test
    void dops4SatsReasonableValues() {
        // 4 satellites at known azimuths with DIFFERENT elevations
        // (same elevation makes H matrix rank-deficient: vertical and clock are aliased)
        double[] azel = new double[8];
        // Sat 0: az=0 (N), el=60 deg
        azel[0] = 0.0;           azel[1] = 60.0 * D2R;
        // Sat 1: az=90 (E), el=30 deg
        azel[2] = 90.0 * D2R;    azel[3] = 30.0 * D2R;
        // Sat 2: az=180 (S), el=45 deg
        azel[4] = 180.0 * D2R;   azel[5] = 45.0 * D2R;
        // Sat 3: az=270 (W), el=20 deg
        azel[6] = 270.0 * D2R;   azel[7] = 20.0 * D2R;

        double[] dop = new double[4];
        Geometry.dops(4, azel, 0.0, dop);

        // With 4 well-distributed sats at varied elevations, DOPs should be reasonable
        assertTrue(dop[0] > 0 && dop[0] < 20, "GDOP=" + dop[0]);
        assertTrue(dop[1] > 0 && dop[1] < 20, "PDOP=" + dop[1]);
        assertTrue(dop[2] > 0 && dop[2] < 20, "HDOP=" + dop[2]);
        assertTrue(dop[3] > 0 && dop[3] < 20, "VDOP=" + dop[3]);

        // GDOP >= PDOP >= HDOP (by definition)
        assertTrue(dop[0] >= dop[1], "GDOP >= PDOP");
        assertTrue(dop[1] >= dop[2], "PDOP >= HDOP");
    }

    @Test
    void dopsWithFewSatsReturnsZero() {
        // Only 3 sats -- not enough for DOP computation
        double[] azel = new double[6];
        azel[0] = 0.0;         azel[1] = 45.0 * D2R;
        azel[2] = 90.0 * D2R;  azel[3] = 45.0 * D2R;
        azel[4] = 180.0 * D2R; azel[5] = 45.0 * D2R;

        double[] dop = new double[4];
        Geometry.dops(3, azel, 0.0, dop);

        assertEquals(0.0, dop[0], "GDOP should be 0 with <4 sats");
    }
}
