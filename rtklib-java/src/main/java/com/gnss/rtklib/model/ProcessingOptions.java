package com.gnss.rtklib.model;

import com.gnss.rtklib.core.Constants;

/**
 * Processing options, matching C RTKLIB's prcopt_t.
 * Default values from prcopt_default in rtkcmn.c.
 */
public class ProcessingOptions {

    /** Positioning mode (PMODE_xxx) */
    public int mode = Constants.PMODE_SINGLE;

    /** Solution type (0:forward, 1:backward, 2:combined) */
    public int soltype = Constants.SOLTYPE_FORWARD;

    /** Number of frequencies (1:L1, 2:L1+L2, 3:L1+L2+L5) */
    public int nf = 2;

    /** Navigation system bitmask */
    public int navsys = Constants.SYS_GPS | Constants.SYS_GLO | Constants.SYS_GAL;

    /** Elevation mask angle (rad) */
    public double elmin = 15.0 * Constants.D2R;

    /** SNR mask */
    public SnrMask snrmask = new SnrMask();

    /** Satellite ephemeris/clock (EPHOPT_xxx, 0=broadcast) */
    public int sateph = 0;

    /** AR mode (0:off, 1:continuous, 2:instantaneous, 3:fix-and-hold, 4:ppp-ar) */
    public int modear = 3;

    /** GLONASS AR mode (0:off, 1:on, 2:auto cal, 3:ext cal) */
    public int glomodear = 3;

    /** GPS AR mode (0:off, 1:on) */
    public int gpsmodear = 1;

    /** BeiDou AR mode (0:off, 1:on) */
    public int bdsmodear = 0;

    /** AR filtering to reject bad sats (0:off, 1:on) */
    public int arfilter = 1;

    /** Obs outage count to reset bias */
    public int maxout = 20;

    /** Min lock count to fix ambiguity */
    public int minlock = 0;

    /** Min sats to fix integer ambiguities */
    public int minfixsats = 4;

    /** Min sats to hold integer ambiguities */
    public int minholdsats = 5;

    /** Min sats to enable sat dropout in AR */
    public int mindropsats = 10;

    /** Min fix count to hold ambiguity */
    public int minfix = 20;

    /** Max iteration to resolve ambiguity */
    public int armaxiter = 1;

    /** Ionosphere option (IONOOPT_xxx) */
    public int ionoopt = Constants.IONOOPT_BRDC;

    /** Troposphere option (TROPOPT_xxx) */
    public int tropopt = Constants.TROPOPT_SAAS;

    /** Dynamics model (0:none, 1:velocity, 2:accel) */
    public int dynamics = 1;

    /** Earth tide correction (0:off, +1:solid, +2:otl, +4:spole) */
    public int tidecorr = 0;

    /** Number of filter iterations */
    public int niter = 1;

    /** Code smoothing window size (0:none) */
    public int codesmooth = 0;

    /** Interpolate reference obs (for post mission) */
    public int intpref = 0;

    /** SBAS correction options */
    public int sbascorr = 0;

    /** SBAS satellite selection (0:all) */
    public int sbassatsel = 0;

    /** Rover position for fixed mode */
    public int rovpos = 0;

    /** Base position for relative mode */
    public int refpos = 0;

    /** Code/phase error ratio per frequency */
    public double[] eratio = {300.0, 300.0, 300.0, 300.0, 300.0, 300.0};

    /**
     * Observation error terms.
     * [reserved, constant, elevation, baseline, doppler, snr_max, snr, rcv_std]
     */
    public double[] err = {100.0, 0.003, 0.003, 0.0, 1.0, 52.0, 0.0, 0.0};

    /** Initial-state std: [bias, iono, trop] */
    public double[] std = {30.0, 0.03, 0.3};

    /** Process-noise std: [bias, iono, trop, acch, accv, pos] */
    public double[] prn = {1E-4, 1E-3, 1E-4, 1E-1, 1E-2, 0.0};

    /** Satellite clock stability (sec/sec) */
    public double sclkstab = 5E-12;

    /** AR validation threshold */
    public double[] thresar = {3.0, 0.25, 0.0, 1E-9, 1E-5, 3.0, 3.0, 0.0};

    /** Elevation mask of AR for rising satellite (deg) */
    public double elmaskar = 0.0;

    /** Elevation mask to hold ambiguity (deg) */
    public double elmaskhold = 0.0;

    /** Slip threshold of geometry-free phase (m) */
    public double thresslip = 0.05;

    /** Slip threshold of doppler (m) */
    public double thresdop = 0.0;

    /** Variance for fix-and-hold pseudo measurements (cycle^2) */
    public double varholdamb = 0.1;

    /** Gain for GLO/SBAS sats to adjust ambiguity */
    public double gainholdamb = 0.01;

    /** Max difference of time (s) */
    public double maxtdiff = 30.0;

    /** Reject threshold of innovation {phase, code} (m) */
    public double[] maxinno = {5.0, 30.0};

    /** Baseline length constraint {const, sigma} (m) */
    public double[] baseline = new double[2];

    /** Rover position for fixed mode {x,y,z} ECEF (m) */
    public double[] ru = new double[3];

    /** Base position for relative mode {x,y,z} ECEF (m) */
    public double[] rb = new double[3];

    /** Antenna types {rover, base} */
    public String[] anttype = {"", ""};

    /** Antenna delta {{rov_e,rov_n,rov_u},{ref_e,ref_n,ref_u}} */
    public double[][] antdel = new double[2][3];

    /** Excluded satellites (1:excluded, 2:included) */
    public byte[] exsats = new byte[Constants.MAXSAT];

    /** Max averaging epochs */
    public int maxaveep = 1;

    /** Initialize by restart */
    public int initrst = 1;

    /** Output single by dgps/float/fix/ppp outage */
    public int outsingle = 0;

    /** RINEX options {rover, base} */
    public String[] rnxopt = {"", ""};

    /** Positioning options */
    public int[] posopt = new int[6];

    /** Solution sync mode (0:off, 1:on) */
    public int syncsol = 0;

    /** Disable L2-AR */
    public int freqopt = 0;

    /** PPP option string */
    public String pppopt = "";

    /** Solver type (0:EKF, 1:Batch Least Squares) */
    public int solver = 0;

    public static final int SOLVER_EKF = 0;
    public static final int SOLVER_BATCH = 1;

    public ProcessingOptions() {
    }

    /**
     * SNR mask, matching C RTKLIB's snrmask_t.
     */
    public static class SnrMask {
        /** Enable flag {rover, base} */
        public int[] ena = new int[2];

        /** Mask (dBHz) at 5,10,...,85 deg elevation: [MAXFREQ][9] */
        public double[][] mask = new double[Constants.MAXFREQ][9];

        public SnrMask() {
        }
    }
}
