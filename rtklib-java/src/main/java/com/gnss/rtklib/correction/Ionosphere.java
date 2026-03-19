/*------------------------------------------------------------------------------
 * Ionosphere.java : ionospheric correction models ported from rtkcmn.c/pntpos.c
 *
 *          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
 *          Java port Copyright (C) 2026
 *
 * Licensed under BSD 2-clause license.
 *
 * references:
 *     [1] IS-GPS-200K, 20.3.3.5.2.5 Ionospheric Model
 *-----------------------------------------------------------------------------*/
package com.gnss.rtklib.correction;

import com.gnss.rtklib.core.Constants;
import com.gnss.rtklib.core.GTime;
import com.gnss.rtklib.model.Navigation;

import static com.gnss.rtklib.core.Constants.*;

/**
 * Ionospheric delay correction models.
 * <p>
 * Provides the Klobuchar broadcast ionosphere model (IS-GPS-200) and a
 * dispatch function for selecting the correction method.
 */
public final class Ionosphere {

    private Ionosphere() {} // utility class

    /** Ionospheric delay standard deviation when correction is off (m). */
    private static final double ERR_ION = 5.0;

    /** Broadcast ionosphere model error factor. */
    private static final double ERR_BRDCI = 0.5;

    /** Default Klobuchar parameters (2004/1/1). */
    private static final double[] ION_DEFAULT = {
         0.1118E-07, -0.7451E-08, -0.5961E-07,  0.1192E-06,
         0.1167E+06, -0.2294E+06, -0.1311E+06,  0.1049E+07
    };

    // -----------------------------------------------------------------------
    // Klobuchar ionosphere model
    // -----------------------------------------------------------------------

    /**
     * Compute ionospheric delay by the Klobuchar broadcast model (IS-GPS-200).
     * <p>
     * The returned value already includes the speed of light factor (meters of
     * delay on L1).
     *
     * @param time GPS time
     * @param ion  Klobuchar parameters {a0,a1,a2,a3,b0,b1,b2,b3} from nav header
     * @param pos  receiver position {lat, lon, h} in radians and meters
     * @param azel azimuth/elevation {az, el} in radians
     * @return ionospheric delay on L1 (m), or 0.0 if below -1 km or el &le; 0
     */
    public static double klobuchar(GTime time, double[] ion, double[] pos,
                                   double[] azel) {
        if (pos[2] < -1E3 || azel[1] <= 0) return 0.0;

        // use defaults if ion parameters are all zero
        if (norm8(ion) <= 0.0) ion = ION_DEFAULT;

        // earth-centered angle (semi-circle)
        double psi = 0.0137 / (azel[1] / PI + 0.11) - 0.022;

        // sub-ionospheric latitude (semi-circle)
        double phi = pos[0] / PI + psi * Math.cos(azel[0]);
        if (phi > 0.416) phi = 0.416;
        else if (phi < -0.416) phi = -0.416;

        // sub-ionospheric longitude (semi-circle)
        double lam = pos[1] / PI + psi * Math.sin(azel[0]) / Math.cos(phi * PI);

        // geomagnetic latitude (semi-circle)
        phi += 0.064 * Math.cos((lam - 1.617) * PI);

        // local time (s)
        double[] wt = time.time2gpst(); // {week, tow}
        double tt = 43200.0 * lam + wt[1];
        tt -= Math.floor(tt / 86400.0) * 86400.0; // 0 <= tt < 86400

        // slant factor
        double f = 1.0 + 16.0 * Math.pow(0.53 - azel[1] / PI, 3.0);

        // ionospheric delay
        double amp = ion[0] + phi * (ion[1] + phi * (ion[2] + phi * ion[3]));
        double per = ion[4] + phi * (ion[5] + phi * (ion[6] + phi * ion[7]));
        if (amp < 0.0) amp = 0.0;
        if (per < 72000.0) per = 72000.0;

        double x = 2.0 * PI * (tt - 50400.0) / per;

        return CLIGHT * f * (Math.abs(x) < 1.57
                ? 5E-9 + amp * (1.0 + x * x * (-0.5 + x * x / 24.0))
                : 5E-9);
    }

    // -----------------------------------------------------------------------
    // ionocorr dispatch
    // -----------------------------------------------------------------------

    /**
     * Compute ionospheric correction by the selected model.
     * <p>
     * Currently supports IONOOPT_OFF and IONOOPT_BRDC. Other options (SBAS, TEC,
     * QZS, estimation, iono-free LC) will be added in later phases.
     *
     * @param time    GPS time
     * @param nav     navigation data (contains iono parameters)
     * @param sat     satellite number (unused for Klobuchar, reserved for future)
     * @param pos     receiver position {lat, lon, h} (rad, m)
     * @param azel    azimuth/elevation {az, el} (rad)
     * @param ionoopt ionosphere option (IONOOPT_xxx)
     * @return double[2]: {delay_m, variance_m2}, or null if correction failed
     */
    public static double[] ionocorr(GTime time, Navigation nav, int sat,
                                    double[] pos, double[] azel, int ionoopt) {
        if (ionoopt == IONOOPT_OFF) {
            return new double[]{0.0, ERR_ION * ERR_ION};
        }
        if (ionoopt == IONOOPT_BRDC) {
            double delay = klobuchar(time, nav.ionGps, pos, azel);
            return new double[]{delay, delay * ERR_BRDCI * delay * ERR_BRDCI};
        }
        // fallback: broadcast model
        double delay = klobuchar(time, nav.ionGps, pos, azel);
        return new double[]{delay, delay * ERR_BRDCI * delay * ERR_BRDCI};
    }

    // -----------------------------------------------------------------------
    // helper: 8-element vector norm
    // -----------------------------------------------------------------------

    /**
     * Compute the Euclidean norm of an 8-element array.
     */
    private static double norm8(double[] v) {
        double sum = 0.0;
        for (int i = 0; i < 8 && i < v.length; i++) {
            sum += v[i] * v[i];
        }
        return Math.sqrt(sum);
    }
}
