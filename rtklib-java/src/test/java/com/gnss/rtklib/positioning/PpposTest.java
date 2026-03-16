package com.gnss.rtklib.positioning;

import com.gnss.rtklib.model.ProcessingOptions;
import com.gnss.rtklib.model.PppState;
import org.junit.jupiter.api.Test;

import static com.gnss.rtklib.core.Constants.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for PPP state management and varerr model.
 */
class PpposTest {

    @Test
    void testPppStateDimensionsIflcStatic() {
        ProcessingOptions opt = new ProcessingOptions();
        opt.mode = PMODE_PPP_STATIC;
        opt.nf = 2;
        opt.ionoopt = IONOOPT_IFLC;
        opt.tropopt = TROPOPT_EST;
        opt.dynamics = 0;

        // NF = 1 (IFLC)
        assertEquals(1, PppState.NF(opt));
        // NP = 3 (no dynamics)
        assertEquals(3, PppState.NP(opt));
        // NC = 7 (NSYS)
        assertEquals(7, PppState.NC());
        // NT = 1 (EST)
        assertEquals(1, PppState.NT(opt));
        // NI = 0 (IFLC)
        assertEquals(0, PppState.NI(opt));
        // ND = 0 (nf < 3)
        assertEquals(0, PppState.ND(opt));
        // NR = 3 + 7 + 1 + 0 + 0 = 11
        assertEquals(11, PppState.NR(opt));
        // NB = 1 * MAXSAT = 204
        assertEquals(MAXSAT, PppState.NB(opt));
        // NX = 11 + 204 = 215
        assertEquals(11 + MAXSAT, PppState.NX(opt));
    }

    @Test
    void testPppStateDimensionsEstgKinema() {
        ProcessingOptions opt = new ProcessingOptions();
        opt.mode = PMODE_PPP_KINEMA;
        opt.nf = 2;
        opt.ionoopt = IONOOPT_IFLC;
        opt.tropopt = TROPOPT_ESTG;
        opt.dynamics = 1;

        assertEquals(1, PppState.NF(opt));
        assertEquals(9, PppState.NP(opt)); // dynamics
        assertEquals(3, PppState.NT(opt)); // ESTG
        // NR = 9 + 7 + 3 + 0 + 0 = 19
        assertEquals(19, PppState.NR(opt));
    }

    @Test
    void testPppStateIndices() {
        ProcessingOptions opt = new ProcessingOptions();
        opt.mode = PMODE_PPP_STATIC;
        opt.nf = 2;
        opt.ionoopt = IONOOPT_IFLC;
        opt.tropopt = TROPOPT_EST;
        opt.dynamics = 0;

        // IC(0) = NP = 3
        assertEquals(3, PppState.IC(0, opt));
        // IC(1) = NP + 1 = 4
        assertEquals(4, PppState.IC(1, opt));
        // IT = NP + NC = 3 + 7 = 10
        assertEquals(10, PppState.IT(opt));
        // IB(1, 0) = NR + 0 * MAXSAT + 0 = 11
        assertEquals(11, PppState.IB(1, 0, opt));
        // IB(2, 0) = NR + 0 * MAXSAT + 1 = 12
        assertEquals(12, PppState.IB(2, 0, opt));
    }

    @Test
    void testPppStateInit() {
        ProcessingOptions opt = new ProcessingOptions();
        opt.mode = PMODE_PPP_STATIC;
        opt.nf = 2;
        opt.ionoopt = IONOOPT_IFLC;
        opt.tropopt = TROPOPT_EST;
        opt.dynamics = 0;

        PppState state = new PppState();
        state.init(opt);

        assertEquals(11 + MAXSAT, state.nx);
        assertEquals(11 + MAXSAT, state.x.length);
        assertEquals((11 + MAXSAT) * (11 + MAXSAT), state.P.length);
        assertEquals(0, state.epoch);
    }

    @Test
    void testInitx() {
        ProcessingOptions opt = new ProcessingOptions();
        opt.mode = PMODE_PPP_STATIC;
        opt.nf = 2;
        opt.ionoopt = IONOOPT_IFLC;
        opt.tropopt = TROPOPT_EST;
        opt.dynamics = 0;

        PppState state = new PppState();
        state.init(opt);

        state.initx(1000.0, 100.0, 0);
        assertEquals(1000.0, state.x[0]);
        assertEquals(100.0, state.P[0]);
        assertEquals(0.0, state.P[1]); // off-diagonal
    }

    @Test
    void testVarerr() {
        ProcessingOptions opt = new ProcessingOptions();
        opt.ionoopt = IONOOPT_IFLC;
        opt.eratio = new double[]{300.0, 300.0, 300.0, 300.0, 300.0, 300.0};
        opt.err = new double[]{100.0, 0.003, 0.003, 0.0, 1.0, 52.0, 0.0, 0.0};

        double el = 30.0 * Math.PI / 180.0;

        // Phase (f=0: frq=0, code=0)
        double varPhase = Pppos.varerr(SYS_GPS, el, 0.0, 0, opt);
        assertTrue(varPhase > 0.0);

        // Code (f=1: frq=0, code=1)
        double varCode = Pppos.varerr(SYS_GPS, el, 0.0, 1, opt);
        assertTrue(varCode > varPhase); // code should have higher variance

        // IFLC scaling: both should include 3^2=9 factor
        ProcessingOptions opt2 = new ProcessingOptions();
        opt2.ionoopt = IONOOPT_BRDC; // no IFLC
        opt2.eratio = opt.eratio.clone();
        opt2.err = opt.err.clone();

        double varNoIflc = Pppos.varerr(SYS_GPS, el, 0.0, 0, opt2);
        assertEquals(varPhase, varNoIflc * 9.0, varNoIflc * 1E-10);
    }
}
