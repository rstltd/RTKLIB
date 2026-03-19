package com.gnss.rtklib.core;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
import static com.gnss.rtklib.core.Constants.*;

class CoordTest {

    @Test
    void pos2ecefAtOrigin() {
        // {lat=0, lon=0, h=0} -> should be on the equator at prime meridian
        double[] r = Coord.pos2ecef(new double[]{0, 0, 0});
        assertEquals(RE_WGS84, r[0], 1e-3, "x should be RE_WGS84");
        assertEquals(0.0, r[1], 1e-3, "y should be 0");
        assertEquals(0.0, r[2], 1e-3, "z should be 0");
    }

    @Test
    void roundTripEcef2pos() {
        double[][] positions = {
            {0.0, 0.0, 0.0},
            {45.0 * D2R, 90.0 * D2R, 100.0},
            {-30.0 * D2R, -120.0 * D2R, 5000.0},
            {80.0 * D2R, 170.0 * D2R, 0.0},
        };

        for (double[] pos : positions) {
            double[] ecef = Coord.pos2ecef(pos);
            double[] back = Coord.ecef2pos(ecef);
            assertEquals(pos[0], back[0], 1e-9, "lat round-trip");
            assertEquals(pos[1], back[1], 1e-9, "lon round-trip");
            assertEquals(pos[2], back[2], 1e-3, "height round-trip");
        }
    }

    @Test
    void pos2ecefNorthPole() {
        // North pole: lat=PI/2, lon=0, h=0
        double[] r = Coord.pos2ecef(new double[]{PI / 2.0, 0, 0});
        assertEquals(0.0, r[0], 1.0, "x at pole");
        assertEquals(0.0, r[1], 1.0, "y at pole");
        // Semi-minor axis b = RE_WGS84 * (1 - FE_WGS84) = ~6356752.3
        double b = RE_WGS84 * (1.0 - FE_WGS84);
        assertEquals(b, r[2], 1.0, "z at pole should be ~semi-minor axis");
    }

    @Test
    void ecef2enuKnownGeometry() {
        // At the equator/prime meridian, ECEF +X is Up, +Y is East, +Z is North
        double[] pos = {0.0, 0.0, 0.0}; // lat=0, lon=0
        double[] dx = {0, 0, 1};  // unit vector in ECEF Z direction -> should be North
        double[] enu = Coord.ecef2enu(pos, dx);
        assertEquals(0.0, enu[0], 1e-10, "east component");
        assertEquals(1.0, enu[1], 1e-10, "north component");
        assertEquals(0.0, enu[2], 1e-10, "up component");
    }

    @Test
    void ecef2enuUpAtEquator() {
        // At equator/prime meridian, ECEF +X is Up
        double[] pos = {0.0, 0.0, 0.0};
        double[] dx = {1, 0, 0};  // ECEF X -> should be Up
        double[] enu = Coord.ecef2enu(pos, dx);
        assertEquals(0.0, enu[0], 1e-10, "east");
        assertEquals(0.0, enu[1], 1e-10, "north");
        assertEquals(1.0, enu[2], 1e-10, "up");
    }
}
