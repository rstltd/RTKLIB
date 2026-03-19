package com.gnss.rtklib.model;

import com.gnss.rtklib.core.GTime;

/**
 * GPS/QZS/GAL/BDS/IRN broadcast ephemeris, matching C RTKLIB's eph_t.
 */
public class Ephemeris {

    /** Satellite number */
    public int sat;

    /** IODE, IODC */
    public int iode, iodc;

    /** SV accuracy (URA index) */
    public int sva;

    /** SV health (0: ok) */
    public int svh;

    /** GPS/QZS: GPS week, GAL: Galileo week */
    public int week;

    /** GPS/QZS: code on L2; GAL: data source; BDS: data source */
    public int code;

    /** GPS/QZS: L2 P data flag; BDS: nav type */
    public int flag;

    /** Epoch of ephemeris (Toe) */
    public GTime toe = new GTime(0, 0.0);

    /** Clock reference epoch (Toc) */
    public GTime toc = new GTime(0, 0.0);

    /** Transmission time */
    public GTime ttr = new GTime(0, 0.0);

    // Orbit parameters
    public double A, e, i0, OMG0, omg, M0, deln, OMGd, idot;
    public double crc, crs, cuc, cus, cic, cis;

    /** Toe (s) in week */
    public double toes;

    /** Fit interval (h) */
    public double fit;

    /** SV clock parameters (af0, af1, af2) */
    public double f0, f1, f2;

    /**
     * Group delay parameters.
     * GPS/QZS: tgd[0]=TGD;
     * GAL: tgd[0]=BGD_E1E5a, tgd[1]=BGD_E1E5b;
     * BDS: tgd[0..5] per signal.
     */
    public double[] tgd = new double[6];

    /** Adot, ndot for CNAV */
    public double Adot, ndot;

    public Ephemeris() {
    }
}
