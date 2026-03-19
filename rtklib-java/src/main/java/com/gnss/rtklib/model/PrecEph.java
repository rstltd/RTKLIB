package com.gnss.rtklib.model;

import com.gnss.rtklib.core.Constants;
import com.gnss.rtklib.core.GTime;

/**
 * Precise ephemeris data, matching C RTKLIB's peph_t.
 * Each instance represents one epoch of precise satellite positions/clocks.
 */
public class PrecEph {

    /** Epoch time (GPST) */
    public GTime time = new GTime(0, 0.0);

    /** File index (for multi-file merge) */
    public int index;

    /** Satellite position {x,y,z,clk} (m, s) per satellite [MAXSAT][4] */
    public double[][] pos;

    /** Satellite position std {x,y,z,clk} per satellite [MAXSAT][4] */
    public float[][] std;

    /** Satellite velocity {vx,vy,vz,clk_rate} (m/s, s/s) per satellite [MAXSAT][4] */
    public double[][] vel;

    /** Satellite velocity std [MAXSAT][4] */
    public float[][] vst;

    public PrecEph() {
        pos = new double[Constants.MAXSAT][4];
        std = new float[Constants.MAXSAT][4];
        vel = new double[Constants.MAXSAT][4];
        vst = new float[Constants.MAXSAT][4];
    }
}
