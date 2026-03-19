/*------------------------------------------------------------------------------
 * RtkState.java : RTK positioning state (rtk_t equivalent)
 *
 *          Ported from RTKLIB rtklib.h rtk_t / ssat_t
 *          Java port Copyright (C) 2026
 *
 * Licensed under BSD 2-clause license.
 *-----------------------------------------------------------------------------*/
package com.gnss.rtklib.model;

import com.gnss.rtklib.core.GTime;

import static com.gnss.rtklib.core.Constants.*;

/**
 * RTK positioning state, matching C RTKLIB's rtk_t.
 * Extends {@link FilterState} for shared filter state management.
 */
public class RtkState extends FilterState {

    /** Base station position ECEF {x,y,z,vx,vy,vz} */
    public double[] rb = new double[6];

    /** Number of AR-eligible satellite pairs */
    public int nb_ar;

    /** Excluded satellite number for dropout */
    public int excsat;

    /** Hold ambiguity flag (1: hold has occurred) */
    public int holdamb;

    /** Satellite states */
    public SatState[] ssat = new SatState[MAXSAT];

    public RtkState() {
        for (int i = 0; i < MAXSAT; i++) {
            ssat[i] = new SatState();
        }
    }

    /**
     * Initialize RTK state from processing options.
     * Matches rtkinit() in rtkpos.c.
     */
    public void init(ProcessingOptions opt) {
        this.opt = opt;
        this.nx = NX(opt);
        this.na = NR(opt);
        this.tt = 0.0;
        this.epoch = 0;
        this.nfix = 0;
        this.nb_ar = 0;
        this.excsat = 0;
        this.holdamb = 0;

        x = new double[nx];
        P = new double[nx * nx];
        xa = new double[na];
        Pa = new double[na * na];

        sol = new Solution();
        sol.thres = (float) opt.thresar[0];

        for (int i = 0; i < 6; i++) rb[i] = 0.0;
        for (int i = 0; i < MAXSAT; i++) {
            ssat[i] = new SatState();
        }
    }

    // ---------------------------------------------------------------
    // State dimension functions (matching C macros)
    // ---------------------------------------------------------------

    /** Number of ionosphere states (NI) */
    public static int NI(ProcessingOptions opt) {
        return opt.ionoopt != IONOOPT_EST ? 0 : MAXSAT;
    }

    /** Number of troposphere states (NT) */
    public static int NT(ProcessingOptions opt) {
        return opt.tropopt < TROPOPT_EST ? 0 :
               (opt.tropopt < TROPOPT_ESTG ? 2 : 6);
    }

    /** Number of GLO receiver h/w bias states (NL) */
    public static int NL(ProcessingOptions opt) {
        return opt.glomodear != 2 ? 0 : 2; // GLO_ARMODE_AUTOCAL=2, NFREQGLO=2
    }

    /** Number of phase bias states (NB) */
    public static int NB(ProcessingOptions opt) {
        return opt.mode <= PMODE_DGPS ? 0 : MAXSAT * NF(opt);
    }

    /** Number of real states (NR = NP+NI+NT+NL) */
    public static int NR(ProcessingOptions opt) {
        return NP(opt) + NI(opt) + NT(opt) + NL(opt);
    }

    /** Total number of states (NX = NR+NB) */
    public static int NX(ProcessingOptions opt) {
        return NR(opt) + NB(opt);
    }

    // ---------------------------------------------------------------
    // State index functions
    // ---------------------------------------------------------------

    /** Ionosphere state index (II) */
    public static int II(int sat, ProcessingOptions opt) {
        return NP(opt) + sat - 1;
    }

    /** Troposphere state index (IT) */
    public static int IT(int rcv, ProcessingOptions opt) {
        return NP(opt) + NI(opt) + NT(opt) / 2 * rcv;
    }

    /** Receiver h/w bias state index (IL) */
    public static int IL(int f, ProcessingOptions opt) {
        return NP(opt) + NI(opt) + NT(opt) + f;
    }

    /** Phase bias state index (IB) */
    public static int IB(int sat, int f, ProcessingOptions opt) {
        return NR(opt) + MAXSAT * f + sat - 1;
    }

    // ---------------------------------------------------------------
    // Satellite state (ssat_t equivalent)
    // ---------------------------------------------------------------

    /**
     * Per-satellite state, matching C RTKLIB's ssat_t.
     */
    public static class SatState {
        /** Satellite system */
        public int sys;

        /** Valid satellite flags per frequency */
        public int[] vsat = new int[NFREQ];

        /** Signal-to-noise: rover, per freq */
        public double[] snr_rover = new double[NFREQ];

        /** Signal-to-noise: base, per freq */
        public double[] snr_base = new double[NFREQ];

        /** Azimuth/elevation {az, el} (rad) */
        public double[] azel = new double[2];

        /** Pseudorange residual per freq (m) */
        public double[] resp = new double[NFREQ];

        /** Carrier-phase residual per freq (m) */
        public double[] resc = new double[NFREQ];

        /** Ambiguity fix flag per freq (0:no data, 1:float, 2:fix, 3:hold) */
        public int[] fix = new int[NFREQ];

        /** Cycle-slip flag per freq */
        public int[] slip = new int[NFREQ];

        /** Half-cycle valid flag per freq */
        public int[] half = new int[NFREQ];

        /** Carrier lock count per freq */
        public int[] lock = new int[NFREQ];

        /** Data outage count per freq */
        public int[] outc = new int[NFREQ];

        /** Cycle-slip count per freq */
        public int[] slipc = new int[NFREQ];

        /** Data reject count per freq */
        public int[] rejc = new int[NFREQ];

        /** Geometry-free phase (L1-L2/L5) per freq-1 (m) */
        public double[] gf = new double[NFREQ - 1];

        /** Inter-channel bias per freq */
        public double[] icbias = new double[NFREQ];

        /** Previous phase measurement {rcv0,rcv1}[freq] */
        public double[][] ph = new double[2][NFREQ];

        /** Previous phase measurement time {rcv0,rcv1}[freq] */
        public GTime[][] pt = new GTime[2][NFREQ];

        /** Observation code per freq per rcv: [freq][rcv] */
        public int[][] code = new int[NFREQ][2];

        public SatState() {
            for (int i = 0; i < 2; i++) {
                for (int j = 0; j < NFREQ; j++) {
                    pt[i][j] = new GTime(0, 0.0);
                }
            }
        }
    }
}
