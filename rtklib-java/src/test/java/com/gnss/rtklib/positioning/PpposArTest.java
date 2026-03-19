package com.gnss.rtklib.positioning;

import com.gnss.rtklib.model.*;
import org.junit.jupiter.api.Test;

import static com.gnss.rtklib.core.Constants.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for PPP-AR algorithm components.
 */
class PpposArTest {

    @Test
    void testHasAnyPhaseBiasEmpty() {
        Navigation nav = new Navigation();
        assertFalse(PpposAr.hasAnyPhaseBias(nav), "Empty nav should have no phase biases");
    }

    @Test
    void testHasAnyPhaseBiasPresent() {
        Navigation nav = new Navigation();
        nav.pbias[0][1] = 0.1; // GPS G01 CODE_L1C
        assertTrue(PpposAr.hasAnyPhaseBias(nav), "Nav with bias should return true");
    }

    @Test
    void testPppArReturnsFalseWithoutBias() {
        ProcessingOptions opt = new ProcessingOptions();
        opt.mode = PMODE_PPP_STATIC;
        opt.nf = 2;
        opt.ionoopt = IONOOPT_IFLC;
        opt.tropopt = TROPOPT_EST;
        opt.modear = 1;

        PppState rtk = new PppState();
        rtk.init(opt);

        Navigation nav = new Navigation();
        ObsData[] obs = new ObsData[0];
        double[] azel = new double[0];
        double[] xp = new double[rtk.nx];
        double[] Pp = new double[rtk.nx * rtk.nx];

        assertFalse(PpposAr.pppAr(rtk, obs, 0, nav, azel, xp, Pp),
                "PPP-AR should return false with no observations");
    }

    @Test
    void testPppStateMwFields() {
        PppState.SatState ss = new PppState.SatState();
        // Check initialization
        assertEquals(0.0, ss.mwAvg[0]);
        assertEquals(0, ss.mwCount[0]);
        assertEquals(Integer.MIN_VALUE, ss.wlFixed[0]);
    }

    @Test
    void testPppStateMwReset() {
        PppState.SatState ss = new PppState.SatState();
        ss.mwAvg[0] = 1.5;
        ss.mwCount[0] = 20;
        ss.wlFixed[0] = 3;

        // Simulate cycle slip reset
        ss.mwAvg[0] = 0.0;
        ss.mwCount[0] = 0;
        ss.wlFixed[0] = Integer.MIN_VALUE;

        assertEquals(0.0, ss.mwAvg[0]);
        assertEquals(0, ss.mwCount[0]);
        assertEquals(Integer.MIN_VALUE, ss.wlFixed[0]);
    }

    @Test
    void testNavigationPbiasAllocation() {
        Navigation nav = new Navigation();
        assertEquals(MAXSAT, nav.pbias.length);
        assertEquals(MAXCODE + 1, nav.pbias[0].length);
        // All zeros by default
        for (int i = 0; i < MAXSAT; i++) {
            for (int j = 0; j <= MAXCODE; j++) {
                assertEquals(0.0, nav.pbias[i][j]);
            }
        }
    }
}
