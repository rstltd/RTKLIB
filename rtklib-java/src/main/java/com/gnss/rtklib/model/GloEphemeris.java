package com.gnss.rtklib.model;

import com.gnss.rtklib.core.GTime;

/**
 * GLONASS broadcast ephemeris, matching C RTKLIB's geph_t.
 */
public class GloEphemeris {

    /** Satellite number */
    public int sat;

    /** IODE (0-6 bit of tb field) */
    public int iode;

    /** Satellite frequency number (-7 to 13) */
    public int frq;

    /** Extended SVH (bit 3:ln, bit 2:Cn_a, bit 1:Cn, bit 0:Bn) */
    public int svh;

    /** Status flags (bits 7-8:M, bit 6:P4, bit 5:P3, bit 4:P2, bits 2-3:P1, bits 0-1:P) */
    public int flags;

    /** SV accuracy */
    public int sva;

    /** Age of operation */
    public int age;

    /** Epoch of ephemeris (GPST) */
    public GTime toe = new GTime(0, 0.0);

    /** Message frame time (GPST) */
    public GTime tof = new GTime(0, 0.0);

    /** Satellite position ECEF (m) */
    public double[] pos = new double[3];

    /** Satellite velocity ECEF (m/s) */
    public double[] vel = new double[3];

    /** Satellite acceleration ECEF (m/s^2) */
    public double[] acc = new double[3];

    /** SV clock bias (s) */
    public double taun;

    /** Relative frequency bias */
    public double gamn;

    /** Delay between L1 and L2 (s) */
    public double dtaun;

    public GloEphemeris() {
    }
}
