package com.gnss.rtklib.model;

import static com.gnss.rtklib.core.Constants.*;

/**
 * PPP positioning state, analogous to rtk_t but with PPP-specific state layout.
 * Extends {@link FilterState} for shared filter state management.
 * <p>
 * State vector layout: [position(NP)] [clocks(NC=NSYS)] [trop(NT)] [iono(NI)] [dcb(ND)] [bias(NB)]
 * <p>
 * This differs from RTK state layout where iono comes before trop and there are no clock states.
 */
public class PppState extends FilterState {

    /** Number of systems (GPS+GLO+GAL+QZS+BDS+IRN+LEO) */
    public static final int NSYS = 7;

    /** Per-satellite states */
    public SatState[] ssat = new SatState[MAXSAT];

    public PppState() {
        for (int i = 0; i < MAXSAT; i++) {
            ssat[i] = new SatState();
        }
    }

    /**
     * Initialize PPP state from processing options.
     */
    public void init(ProcessingOptions opt) {
        this.opt = opt;
        this.nx = NX(opt);
        this.na = NR(opt);
        this.tt = 0.0;
        this.epoch = 0;
        this.nfix = 0;

        x = new double[nx];
        P = new double[nx * nx];
        xa = new double[na];
        Pa = new double[na * na];

        sol = new Solution();
        for (int i = 0; i < MAXSAT; i++) {
            ssat[i] = new SatState();
        }
    }

    // ---------------------------------------------------------------
    // State dimension functions (matching C ppp.c macros)
    // ---------------------------------------------------------------

    /** Number of clock states (one per system) */
    public static int NC() {
        return NSYS;
    }

    /** Number of troposphere states (single receiver) */
    public static int NT(ProcessingOptions opt) {
        return opt.tropopt < TROPOPT_EST ? 0 :
               (opt.tropopt == TROPOPT_EST ? 1 : 3);
    }

    /** Number of ionosphere states */
    public static int NI(ProcessingOptions opt) {
        return opt.ionoopt == IONOOPT_EST ? MAXSAT : 0;
    }

    /** Number of DCB states */
    public static int ND(ProcessingOptions opt) {
        return opt.nf >= 3 ? 1 : 0;
    }

    /** Number of real (non-ambiguity) states */
    public static int NR(ProcessingOptions opt) {
        return NP(opt) + NC() + NT(opt) + NI(opt) + ND(opt);
    }

    /** Number of phase bias states */
    public static int NB(ProcessingOptions opt) {
        return NF(opt) * MAXSAT;
    }

    /** Total number of states */
    public static int NX(ProcessingOptions opt) {
        return NR(opt) + NB(opt);
    }

    // ---------------------------------------------------------------
    // State index functions
    // ---------------------------------------------------------------

    /** Clock state index for system s (0=GPS,1=GLO,...) */
    public static int IC(int s, ProcessingOptions opt) {
        return NP(opt) + s;
    }

    /** Troposphere state index (single receiver) */
    public static int IT(ProcessingOptions opt) {
        return NP(opt) + NC();
    }

    /** Ionosphere state index */
    public static int II(int sat, ProcessingOptions opt) {
        return NP(opt) + NC() + NT(opt) + sat - 1;
    }

    /** DCB state index */
    public static int ID(ProcessingOptions opt) {
        return NP(opt) + NC() + NT(opt) + NI(opt);
    }

    /** Phase bias state index */
    public static int IB(int sat, int f, ProcessingOptions opt) {
        return NR(opt) + MAXSAT * f + sat - 1;
    }

    // ---------------------------------------------------------------
    // Per-satellite state for PPP
    // ---------------------------------------------------------------

    /**
     * Per-satellite state for PPP processing.
     */
    public static class SatState {
        /** Valid satellite flags per frequency */
        public int[] vsat = new int[NFREQ];

        /** Signal-to-noise: rover, per freq */
        public double[] snr_rover = new double[NFREQ];

        /** Azimuth/elevation {az, el} (rad) */
        public double[] azel = new double[2];

        /** Pseudorange residual per freq (m) */
        public double[] resp = new double[NFREQ];

        /** Carrier-phase residual per freq (m) */
        public double[] resc = new double[NFREQ];

        /** Ambiguity fix flag per freq */
        public int[] fix = new int[NFREQ];

        /** Cycle-slip flag per freq */
        public int[] slip = new int[NFREQ];

        /** Carrier lock count per freq */
        public int[] lock = new int[NFREQ];

        /** Data outage count per freq */
        public int[] outc = new int[NFREQ];

        /** Cycle-slip count per freq */
        public int[] slipc = new int[NFREQ];

        /** Data reject count per freq */
        public int[] rejc = new int[NFREQ];

        /** Geometry-free phase (L1-L2) (m) */
        public double[] gf = new double[NFREQ - 1];

        /** Melbourne-Wubbena combination (m) */
        public double[] mw = new double[NFREQ - 1];

        /** Phase windup (cycle) */
        public double phw;

        /** Visible flag */
        public int vs;

        // --- PPP-AR wide-lane tracking ---

        /** MW running average per frequency pair (cycles, OSB-corrected) */
        public double[] mwAvg = new double[NFREQ - 1];

        /** MW epoch count per frequency pair */
        public int[] mwCount = new int[NFREQ - 1];

        /** Fixed WL integer per frequency pair (Integer.MIN_VALUE = unfixed) */
        public int[] wlFixed = new int[NFREQ - 1];

        {
            java.util.Arrays.fill(wlFixed, Integer.MIN_VALUE);
        }
    }
}
