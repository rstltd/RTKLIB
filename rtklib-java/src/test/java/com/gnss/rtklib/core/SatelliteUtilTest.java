package com.gnss.rtklib.core;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
import static com.gnss.rtklib.core.Constants.*;

class SatelliteUtilTest {

    @Test
    void satnoGpsFirst() {
        assertEquals(1, SatelliteUtil.satno(SYS_GPS, 1));
    }

    @Test
    void satnoGpsLast() {
        assertEquals(32, SatelliteUtil.satno(SYS_GPS, 32));
    }

    @Test
    void satnoGloFirst() {
        assertEquals(33, SatelliteUtil.satno(SYS_GLO, 1));
    }

    @Test
    void satnoGloLast() {
        assertEquals(59, SatelliteUtil.satno(SYS_GLO, 27));
    }

    @Test
    void satnoGalFirst() {
        assertEquals(60, SatelliteUtil.satno(SYS_GAL, 1));
    }

    @Test
    void satsysGps1() {
        int[] result = SatelliteUtil.satsys(1);
        assertEquals(SYS_GPS, result[0]);
        assertEquals(1, result[1]);
    }

    @Test
    void satsysGlo1() {
        int[] result = SatelliteUtil.satsys(33);
        assertEquals(SYS_GLO, result[0]);
        assertEquals(1, result[1]);
    }

    @Test
    void satno2idGps() {
        assertEquals("G01", SatelliteUtil.satno2id(1));
    }

    @Test
    void satno2idGlo() {
        assertEquals("R01", SatelliteUtil.satno2id(33));
    }

    @Test
    void satno2idGal() {
        assertEquals("E01", SatelliteUtil.satno2id(60));
    }

    @Test
    void satid2noGps() {
        assertEquals(1, SatelliteUtil.satid2no("G01"));
    }

    @Test
    void satid2noGlo() {
        assertEquals(33, SatelliteUtil.satid2no("R01"));
    }

    @Test
    void satid2noGal() {
        assertEquals(60, SatelliteUtil.satid2no("E01"));
    }

    @Test
    void roundTripAllSatellites() {
        for (int sat = 1; sat <= MAXSAT; sat++) {
            // Skip LEO range since NSATLEO==0, those sats don't exist
            int[] sp = SatelliteUtil.satsys(sat);
            if (sp[0] == SYS_NONE) continue;

            String id = SatelliteUtil.satno2id(sat);
            assertFalse(id.isEmpty(), "satno2id returned empty for sat=" + sat);
            int back = SatelliteUtil.satid2no(id);
            assertEquals(sat, back, "Round-trip failed for sat=" + sat + " id=" + id);
        }
    }

    @Test
    void satexcludeNegativeSvhReturnsTrue() {
        assertTrue(SatelliteUtil.satexclude(1, 0.0, -1, null));
    }

    @Test
    void satexcludeZeroSvhReturnsFalse() {
        assertFalse(SatelliteUtil.satexclude(1, 0.0, 0, null));
    }
}
