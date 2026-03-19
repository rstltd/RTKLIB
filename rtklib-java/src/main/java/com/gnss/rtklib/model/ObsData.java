package com.gnss.rtklib.model;

import com.gnss.rtklib.core.Constants;
import com.gnss.rtklib.core.GTime;

/**
 * Observation data record, matching C RTKLIB's obsd_t.
 * <p>
 * Array dimensions use NFREQ + NEXOBS = 6.
 */
public class ObsData {

    private static final int NOBS = Constants.NFREQ + Constants.NEXOBS;

    /** Receiver sampling time (GPST) */
    public GTime time = new GTime(0, 0.0);

    /** Satellite number (1-MAXSAT) */
    public int sat;

    /** Receiver number */
    public int rcv;

    /** GLONASS frequency channel (0-13) */
    public int freq;

    /** Loss of lock indicator */
    public int[] LLI = new int[NOBS];

    /** Code indicator (CODE_xxx) */
    public int[] code = new int[NOBS];

    /** Carrier-phase observation (cycle) */
    public double[] L = new double[NOBS];

    /** Pseudorange observation (m) */
    public double[] P = new double[NOBS];

    /** Doppler frequency (Hz) */
    public double[] D = new double[NOBS];

    /** Signal strength (dBHz) */
    public double[] SNR = new double[NOBS];

    /** Carrier-phase standard deviation (cycle) */
    public double[] Lstd = new double[NOBS];

    /** Pseudorange standard deviation (m) */
    public double[] Pstd = new double[NOBS];

    /** Time-valid flag (valid GNSS fix) for time mark */
    public int timevalid;

    /** Time of event (GPST) */
    public GTime eventime = new GTime(0, 0.0);

    public ObsData() {
    }
}
