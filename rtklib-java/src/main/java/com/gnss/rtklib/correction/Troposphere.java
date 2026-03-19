/*------------------------------------------------------------------------------
 * Troposphere.java : tropospheric correction models ported from rtkcmn.c/pntpos.c
 *
 *          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
 *          Java port Copyright (C) 2026
 *
 * Licensed under BSD 2-clause license.
 *
 * references:
 *     [1] Saastamoinen, J., Atmospheric Correction for the Troposphere and
 *         Stratosphere in Radio Ranging of Satellites, Geophysical Monograph 15,
 *         AGU, 1972
 *-----------------------------------------------------------------------------*/
package com.gnss.rtklib.correction;

import com.gnss.rtklib.core.GTime;

import static com.gnss.rtklib.core.Constants.*;

/**
 * Tropospheric delay correction models.
 * <p>
 * Provides the Saastamoinen standard-atmosphere troposphere model and a
 * dispatch function for selecting the correction method.
 */
public final class Troposphere {

    private Troposphere() {} // utility class

    /** Tropospheric delay standard deviation when correction is off (m). */
    private static final double ERR_TROP = 3.0;

    /** Saastamoinen model error standard deviation (m). */
    private static final double ERR_SAAS = 0.3;

    /** Relative humidity for Saastamoinen model. */
    private static final double REL_HUMI = 0.7;

    // -----------------------------------------------------------------------
    // Saastamoinen troposphere model
    // -----------------------------------------------------------------------

    /**
     * Compute tropospheric delay using the Saastamoinen model with
     * standard atmosphere.
     * <p>
     * Returns the total (hydrostatic + wet) zenith delay mapped to the
     * satellite elevation.
     *
     * @param time GPS time (unused, reserved for seasonal models)
     * @param pos  receiver position {lat, lon, h} in radians and meters
     * @param azel azimuth/elevation {az, el} in radians
     * @param humi relative humidity (0 to 1, e.g. 0.7)
     * @return tropospheric delay (m), or 0.0 if conditions are invalid
     */
    public static double saastamoinen(GTime time, double[] pos, double[] azel,
                                      double humi) {
        final double TEMP0 = 15.0; // temperature at sea level (C)

        if (pos[2] < -100.0 || 1E4 < pos[2] || azel[1] <= 0) return 0.0;

        // standard atmosphere
        double hgt = pos[2] < 0.0 ? 0.0 : pos[2];

        double pres = 1013.25 * Math.pow(1.0 - 2.2557E-5 * hgt, 5.2568);
        double temp = TEMP0 - 6.5E-3 * hgt + 273.16;
        double e = 6.108 * humi * Math.exp((17.15 * temp - 4684.0) / (temp - 38.45));

        // zenith angle
        double z = PI / 2.0 - azel[1];

        // hydrostatic delay
        double trph = 0.0022768 * pres
                / (1.0 - 0.00266 * Math.cos(2.0 * pos[0]) - 0.00028 * hgt / 1E3)
                / Math.cos(z);

        // wet delay
        double trpw = 0.002277 * (1255.0 / temp + 0.05) * e / Math.cos(z);

        return trph + trpw;
    }

    // -----------------------------------------------------------------------
    // tropcorr dispatch
    // -----------------------------------------------------------------------

    /**
     * Compute tropospheric correction by the selected model.
     * <p>
     * Currently supports TROPOPT_OFF, TROPOPT_SAAS, TROPOPT_EST, and TROPOPT_ESTG.
     * TROPOPT_EST and TROPOPT_ESTG use Saastamoinen as the a-priori model
     * (the estimation is done externally in the filter). TROPOPT_SBAS will be
     * added in a later phase.
     *
     * @param time    GPS time
     * @param pos     receiver position {lat, lon, h} (rad, m)
     * @param azel    azimuth/elevation {az, el} (rad)
     * @param tropopt troposphere option (TROPOPT_xxx)
     * @return double[2]: {delay_m, variance_m2}, or null if correction failed
     */
    public static double[] tropcorr(GTime time, double[] pos, double[] azel,
                                    int tropopt) {
        // Saastamoinen model (also used as a-priori for estimation modes)
        if (tropopt == TROPOPT_SAAS || tropopt == TROPOPT_EST
                || tropopt == TROPOPT_ESTG) {
            double trp = saastamoinen(time, pos, azel, REL_HUMI);
            double sinEl = Math.sin(azel[1]);
            double var = ERR_SAAS / (sinEl + 0.1);
            return new double[]{trp, var * var};
        }
        // no correction
        double var = (tropopt == TROPOPT_OFF) ? ERR_TROP : 0.0;
        return new double[]{0.0, var * var};
    }

    // -----------------------------------------------------------------------
    // Troposphere zenith model (no elevation mapping)
    // -----------------------------------------------------------------------

    /**
     * Compute tropospheric delay using Saastamoinen model at zenith (el=90deg).
     * Equivalent to RTKLIB tropmodel() with zazel={0, PI/2}.
     *
     * @param time GPS time
     * @param pos  receiver position {lat, lon, h} (rad, m)
     * @param azel azimuth/elevation {az, el} (rad)
     * @param humi relative humidity (0 to 1)
     * @return tropospheric delay (m)
     */
    public static double tropmodel(GTime time, double[] pos, double[] azel,
                                    double humi) {
        return saastamoinen(time, pos, azel, humi);
    }

    // -----------------------------------------------------------------------
    // SBAS MOPS troposphere model (for ZTD initialization)
    // -----------------------------------------------------------------------

    /** Meteorological parameters at latitudes 15,30,45,60,75 degrees.
     * Columns: P0,T0,e0,beta0,lambda0, dP,dT,de,dbeta,dlambda */
    private static final double[][] METPRM = {
        {1013.25, 299.65, 26.31, 6.30E-3, 2.77,   0.00,  0.00, 0.00, 0.00E-3, 0.00},
        {1017.25, 294.15, 21.79, 6.05E-3, 3.15,  -3.75,  7.00, 8.85, 0.25E-3, 0.33},
        {1015.75, 283.15, 11.66, 5.58E-3, 2.57,  -2.25, 11.00, 7.24, 0.32E-3, 0.46},
        {1011.75, 272.15,  6.78, 5.39E-3, 1.81,  -1.75, 15.00, 5.36, 0.81E-3, 0.74},
        {1013.00, 263.65,  4.11, 4.53E-3, 1.55,  -0.50, 14.50, 3.39, 0.62E-3, 0.30}
    };

    /** Interpolate meteorological parameters by latitude (degrees). */
    private static void getmet(double lat, double[] met) {
        lat = Math.abs(lat);
        if (lat <= 15.0) {
            System.arraycopy(METPRM[0], 0, met, 0, 10);
        } else if (lat >= 75.0) {
            System.arraycopy(METPRM[4], 0, met, 0, 10);
        } else {
            int j = (int) (lat / 15.0);
            double a = (lat - j * 15.0) / 15.0;
            for (int i = 0; i < 10; i++) {
                met[i] = (1.0 - a) * METPRM[j - 1][i] + a * METPRM[j][i];
            }
        }
    }

    /**
     * SBAS MOPS tropospheric delay correction (ported from sbas.c sbstropcorr).
     *
     * @param time GPS time
     * @param pos  receiver position {lat, lon, h} (rad, m)
     * @param azel azimuth/elevation {az, el} (rad)
     * @param varOut 1-element array to receive variance (m^2), or null
     * @return slant tropospheric delay (m)
     */
    public static double sbstropcorr(GTime time, double[] pos, double[] azel,
                                      double[] varOut) {
        final double k1 = 77.604, k2 = 382000.0, rd = 287.054, gm = 9.784, g = 9.80665;

        if (pos[2] < -100.0 || 10000.0 < pos[2] || azel[1] <= 0) {
            if (varOut != null) varOut[0] = 0.0;
            return 0.0;
        }

        double[] met = new double[10];
        getmet(pos[0] * R2D, met);
        double c = Math.cos(2.0 * PI * (time.time2doy() - (pos[0] >= 0.0 ? 28.0 : 211.0)) / 365.25);
        for (int i = 0; i < 5; i++) met[i] -= met[i + 5] * c;

        double zh = 1E-6 * k1 * rd * met[0] / gm;
        double zw = 1E-6 * k2 * rd / (gm * (met[4] + 1.0) - met[3] * rd) * met[2] / met[1];
        double h = pos[2];
        zh *= Math.pow(1.0 - met[3] * h / met[1], g / (rd * met[3]));
        zw *= Math.pow(1.0 - met[3] * h / met[1], (met[4] + 1.0) * g / (rd * met[3]) - 1.0);

        double sinel = Math.sin(azel[1]);
        double m = 1.001 / Math.sqrt(0.002001 + sinel * sinel);
        if (varOut != null) varOut[0] = 0.12 * 0.12 * m * m;
        return (zh + zw) * m;
    }

    // -----------------------------------------------------------------------
    // NMF troposphere mapping function
    // -----------------------------------------------------------------------

    /** NMF coefficients: [9][5] — hydro-ave-a,b,c; hydro-amp-a,b,c; wet-a,b,c
     * at latitudes 15,30,45,60,75 degrees */
    private static final double[][] NMF_COEF = {
        { 1.2769934E-3, 1.2683230E-3, 1.2465397E-3, 1.2196049E-3, 1.2045996E-3},
        { 2.9153695E-3, 2.9152299E-3, 2.9288445E-3, 2.9022565E-3, 2.9024912E-3},
        { 62.610505E-3, 62.837393E-3, 63.721774E-3, 63.824265E-3, 64.258455E-3},
        { 0.0000000E-0, 1.2709626E-5, 2.6523662E-5, 3.4000452E-5, 4.1202191E-5},
        { 0.0000000E-0, 2.1414979E-5, 3.0160779E-5, 7.2562722E-5, 11.723375E-5},
        { 0.0000000E-0, 9.0128400E-5, 4.3497037E-5, 84.795348E-5, 170.37206E-5},
        { 5.8021897E-4, 5.6794847E-4, 5.8118019E-4, 5.9727542E-4, 6.1641693E-4},
        { 1.4275268E-3, 1.5138625E-3, 1.4572752E-3, 1.5007428E-3, 1.7599082E-3},
        { 4.3472961E-2, 4.6729510E-2, 4.3908931E-2, 4.4626982E-2, 5.4736038E-2}
    };
    private static final double[] NMF_AHT = { 2.53E-5, 5.49E-3, 1.14E-3 };

    private static double interpc(double[] coef, double lat) {
        int i = (int) (lat / 15.0);
        if (i < 1) return coef[0];
        if (i > 4) return coef[4];
        return coef[i - 1] * (1.0 - lat / 15.0 + i) + coef[i] * (lat / 15.0 - i);
    }

    private static double mapf(double el, double a, double b, double c) {
        double sinel = Math.sin(el);
        return (1.0 + a / (1.0 + b / (1.0 + c))) /
               (sinel + (a / (sinel + b / (sinel + c))));
    }

    /**
     * Troposphere mapping function (NMF).
     * <p>
     * Computes the hydrostatic mapping function and optionally the wet
     * mapping function.
     *
     * @param time GPS time
     * @param pos  receiver position {lat, lon, h} (rad, m)
     * @param azel azimuth/elevation {az, el} (rad)
     * @param mapfw output: wet mapping function (1-element array), or null
     * @return dry (hydrostatic) mapping function
     */
    public static double tropmapf(GTime time, double[] pos, double[] azel,
                                   double[] mapfw) {
        double el = azel[1];
        if (el <= 0.0) {
            if (mapfw != null) mapfw[0] = 0.0;
            return 0.0;
        }
        if (pos[2] < -1000.0 || pos[2] > 20000.0) {
            if (mapfw != null) mapfw[0] = 0.0;
            return 0.0;
        }

        // Day of year from doy 28, add half year for southern latitudes
        double doy = time.time2doy();
        double y = (doy - 28.0) / 365.25 + (pos[0] < 0.0 ? 0.5 : 0.0);
        double cosy = Math.cos(2.0 * PI * y);
        double lat = Math.abs(pos[0] * R2D);
        double hgt = pos[2];

        double[] ah = new double[3], aw = new double[3];
        for (int i = 0; i < 3; i++) {
            ah[i] = interpc(NMF_COEF[i], lat) - interpc(NMF_COEF[i + 3], lat) * cosy;
            aw[i] = interpc(NMF_COEF[i + 6], lat);
        }

        // Height correction
        double dm = (1.0 / Math.sin(el) - mapf(el, NMF_AHT[0], NMF_AHT[1], NMF_AHT[2])) * hgt / 1E3;

        if (mapfw != null) mapfw[0] = mapf(el, aw[0], aw[1], aw[2]);

        return mapf(el, ah[0], ah[1], ah[2]) + dm;
    }
}
