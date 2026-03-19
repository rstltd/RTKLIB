package com.gnss.rtklib.core;

import static com.gnss.rtklib.core.Constants.*;

/**
 * Geometric computation functions ported from RTKLIB rtkcmn.c.
 */
public final class Geometry {

    private Geometry() {}

    /**
     * Compute geometric distance between satellite and receiver with Sagnac correction.
     * <p>
     * Also computes the unit line-of-sight vector from receiver to satellite.
     *
     * @param rs satellite position in ECEF {x, y, z} (m)
     * @param rr receiver position in ECEF {x, y, z} (m)
     * @param e  output: unit line-of-sight vector (3 elements)
     * @return geometric distance (m), or negative value on error
     */
    public static double geodist(double[] rs, double[] rr, double[] e) {
        if (MatrixUtil.norm(rs, 3) < RE_WGS84) {
            return -1.0;
        }

        for (int i = 0; i < 3; i++) {
            e[i] = rs[i] - rr[i];
        }
        double r = MatrixUtil.norm(e, 3);
        for (int i = 0; i < 3; i++) {
            e[i] /= r;
        }

        // Sagnac correction
        return r + OMGE * (rs[0] * rr[1] - rs[1] * rr[0]) / CLIGHT;
    }

    /**
     * Compute geometric distance with offset into rs array.
     *
     * @param rs     satellite positions array
     * @param rsOff  offset into rs (satellite index * 6)
     * @param rr     receiver position ECEF {x,y,z}
     * @param e      output: unit line-of-sight vector (3 elements)
     * @return geometric distance (m), or negative on error
     */
    public static double geodist(double[] rs, int rsOff, double[] rr, double[] e) {
        double[] rsSat = new double[3];
        rsSat[0] = rs[rsOff]; rsSat[1] = rs[rsOff + 1]; rsSat[2] = rs[rsOff + 2];
        return geodist(rsSat, rr, e);
    }

    /**
     * Compute satellite azimuth and elevation with offset into azel array.
     *
     * @param pos     geodetic position {lat, lon, h} (rad, m)
     * @param e       receiver-to-satellite unit vector in ECEF
     * @param azel    output azimuth/elevation array
     * @param azelOff offset into azel array
     * @return elevation angle (rad)
     */
    public static double satazel(double[] pos, double[] e, double[] azel, int azelOff) {
        double[] az_el = new double[2];
        double el = satazel(pos, e, az_el);
        if (azel != null) {
            azel[azelOff] = az_el[0];
            azel[azelOff + 1] = az_el[1];
        }
        return el;
    }

    /**
     * Compute satellite azimuth and elevation angle.
     *
     * @param pos  geodetic position {lat, lon, h} (rad, m)
     * @param e    receiver-to-satellite unit vector in ECEF (3 elements)
     * @param azel output: {azimuth, elevation} in radians (may be null for no output)
     * @return elevation angle (rad)
     */
    public static double satazel(double[] pos, double[] e, double[] azel) {
        double az = 0.0;
        double el = PI / 2.0;

        if (pos[2] > -RE_WGS84) {
            double[] enu = Coord.ecef2enu(pos, e);
            double enuHoriz2 = enu[0] * enu[0] + enu[1] * enu[1]; // dot2(enu, enu)
            az = enuHoriz2 < 1E-12 ? 0.0 : Math.atan2(enu[0], enu[1]);
            if (az < 0.0) az += 2.0 * PI;
            el = Math.asin(enu[2]);
        }

        if (azel != null) {
            azel[0] = az;
            azel[1] = el;
        }
        return el;
    }

    /**
     * Compute dilution of precision (DOP) values.
     *
     * @param ns    number of satellites
     * @param azel  satellite azimuth/elevation pairs: azel[i*2]=az, azel[i*2+1]=el (rad)
     * @param elmin elevation cutoff angle (rad)
     * @param dop   output: {GDOP, PDOP, HDOP, VDOP}
     */
    public static void dops(int ns, double[] azel, double elmin, double[] dop) {
        dop[0] = dop[1] = dop[2] = dop[3] = 0.0;

        int maxSat = Math.min(ns, MAXSAT);

        // Build H matrix (4 x n, column-major)
        double[] H = new double[4 * maxSat];
        int n = 0;
        for (int i = 0; i < maxSat; i++) {
            double el = azel[1 + i * 2];
            if (el < elmin || el <= 0.0) continue;

            double cosel = Math.cos(el);
            double sinel = Math.sin(el);
            H[    4 * n] = cosel * Math.sin(azel[i * 2]);
            H[1 + 4 * n] = cosel * Math.cos(azel[i * 2]);
            H[2 + 4 * n] = sinel;
            H[3 + 4 * n] = 1.0;
            n++;
        }

        if (n < 4) return;

        // Q = (H * H')^-1
        double[] Q = new double[16];
        MatrixUtil.matmul("NT", 4, 4, n, H, H, Q);
        if (MatrixUtil.matinv(Q, 4) == 0) {
            dop[0] = safeSqrt(Q[0] + Q[5] + Q[10] + Q[15]); // GDOP
            dop[1] = safeSqrt(Q[0] + Q[5] + Q[10]);          // PDOP
            dop[2] = safeSqrt(Q[0] + Q[5]);                   // HDOP
            dop[3] = safeSqrt(Q[10]);                          // VDOP
        }
    }

    /**
     * Safe square root: returns 0 for negative or NaN input.
     */
    private static double safeSqrt(double x) {
        return (x < 0.0 || x != x) ? 0.0 : Math.sqrt(x);
    }
}
