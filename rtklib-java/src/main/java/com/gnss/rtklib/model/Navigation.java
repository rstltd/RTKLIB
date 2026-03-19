package com.gnss.rtklib.model;

import com.gnss.rtklib.core.Constants;
import com.gnss.rtklib.core.GTime;

import java.util.ArrayList;
import java.util.List;

/**
 * Navigation data, matching C RTKLIB's nav_t (simplified for Phase 1 SPP).
 */
public class Navigation {

    /** GPS/QZS/GAL/BDS/IRN broadcast ephemerides */
    public List<Ephemeris> eph = new ArrayList<>();

    /** GLONASS broadcast ephemerides */
    public List<GloEphemeris> geph = new ArrayList<>();

    // Ionosphere model parameters
    /** GPS ionosphere model {a0,a1,a2,a3,b0,b1,b2,b3} */
    public double[] ionGps = new double[8];

    /** Galileo ionosphere model {ai0,ai1,ai2,0} */
    public double[] ionGal = new double[4];

    /** QZSS ionosphere model */
    public double[] ionQzs = new double[8];

    /** BeiDou ionosphere model */
    public double[] ionCmp = new double[8];

    /** IRNSS ionosphere model */
    public double[] ionIrn = new double[8];

    // UTC parameters
    /** GPS delta-UTC {A0,A1,Tot,WNt,dt_LS,WN_LSF,DN,dt_LSF} */
    public double[] utcGps = new double[8];

    /** GLONASS UTC parameters {tau_C,tau_GPS} */
    public double[] utcGlo = new double[8];

    /** Galileo UTC parameters */
    public double[] utcGal = new double[8];

    /** QZSS UTC parameters */
    public double[] utcQzs = new double[8];

    /** BeiDou UTC parameters */
    public double[] utcCmp = new double[8];

    /** IRNSS UTC parameters {A0,A1,Tot,...,dt_LSF,A2} */
    public double[] utcIrn = new double[9];

    /** GLONASS frequency channel number + 8 */
    public int[] gloFcn = new int[32];

    /**
     * Satellite code biases (m).
     * Dimensions: [MAXSAT][MAX_CODE_BIAS_FREQS][MAX_CODE_BIASES].
     * Index: [sat][freq][bias].
     */
    public double[][][] cbias;

    /** Precise ephemerides (SP3) */
    public List<PrecEph> peph = new ArrayList<>();

    /** Precise clocks (CLK) */
    public List<PrecClk> pclk = new ArrayList<>();

    /** Ephemeris selection for Galileo: 0=any, 1=I/NAV, 2=F/NAV */
    public int[] ephSel = new int[Constants.MAXSAT];

    /** Antenna phase center parameters (ANTEX), indexed by satellite-1 */
    public List<AntennaModel> pcvs = new ArrayList<>();

    /** Earth rotation parameters: {xp, yp, ut1_utc, lod} (rad, rad, s, s/d) */
    public double[] erpv = new double[5];

    public Navigation() {
        cbias = new double[Constants.MAXSAT][Constants.MAX_CODE_BIAS_FREQS][Constants.MAX_CODE_BIASES];
    }
}
