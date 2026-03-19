package com.gnss.rtklib.model;

import com.gnss.rtklib.core.Constants;
import com.gnss.rtklib.core.GTime;

/**
 * Precise clock data, matching C RTKLIB's pclk_t.
 * Each instance represents one epoch of precise satellite clocks.
 */
public class PrecClk {

    /** Epoch time (GPST) */
    public GTime time = new GTime(0, 0.0);

    /** File index */
    public int index;

    /** Satellite clock bias (s) per satellite [MAXSAT] */
    public double[] clk;

    /** Satellite clock std (s) per satellite [MAXSAT] */
    public float[] std;

    public PrecClk() {
        clk = new double[Constants.MAXSAT];
        std = new float[Constants.MAXSAT];
    }
}
