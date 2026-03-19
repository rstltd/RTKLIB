/*------------------------------------------------------------------------------
 * Spp.java : standard point positioning (SPP) engine
 *
 *          Ported from RTKLIB pntpos.c by T.TAKASU
 *          Java port Copyright (C) 2026
 *
 * Licensed under BSD 2-clause license.
 *
 * history : 2026/03/12 1.0  initial Java port from pntpos.c
 *-----------------------------------------------------------------------------*/
package com.gnss.rtklib.positioning;

import com.gnss.rtklib.core.*;
import com.gnss.rtklib.model.*;

import static com.gnss.rtklib.core.Constants.*;

import java.util.Arrays;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Standard Point Positioning (SPP) engine ported from RTKLIB pntpos.c.
 * <p>
 * Computes receiver position, velocity, and clock bias using
 * pseudorange and Doppler observables via weighted least squares.
 */
public final class Spp {

    private static final Logger trace = Logger.getLogger(Spp.class.getName());

    private Spp() {} // utility class

    // ---------------------------------------------------------------
    // Constants
    // ---------------------------------------------------------------

    /** Number of estimated parameters: 4 position + 5 system clock offsets
     *  (dGLO, dGAL, dBDS, dIRN, dQZS). */
    static final int NX = 4 + 5;

    /** Max number of iterations for point position */
    private static final int MAXITR = 10;

    /** Ionospheric delay std (m) */
    private static final double ERR_ION = 5.0;

    /** Tropospheric delay std (m) */
    private static final double ERR_TROP = 3.0;

    /** Saastamoinen model error std (m) */
    private static final double ERR_SAAS = 0.3;

    /** Broadcast ionosphere model error factor */
    private static final double ERR_BRDCI = 0.5;

    /** Code bias error std (m) */
    private static final double ERR_CBIAS = 0.3;

    /** Relative humidity for Saastamoinen model */
    private static final double REL_HUMI = 0.7;

    /** Min elevation for measurement error (rad) */
    private static final double MIN_EL = 5.0 * D2R;

    /** Max GDOP for valid solution */
    private static final double MAX_GDOP = 30.0;

    /** Max observations in an epoch */
    private static final int MAXOBS = 96;

    /** Error factor by system */
    private static final double EFACT_GPS = 1.0;
    private static final double EFACT_GLO = 1.5;
    private static final double EFACT_SBS = 3.0;
    private static final double EFACT_CMP = 1.0;
    private static final double EFACT_QZS = 1.0;
    private static final double EFACT_IRN = 1.5;

    /** Solution status codes */
    public static final int SOLQ_NONE   = 0;
    public static final int SOLQ_SBAS   = 3;
    public static final int SOLQ_SINGLE = 5;

    /** Ephemeris option: broadcast + SBAS */
    public static final int EPHOPT_SBAS = 2;

    /** Chi-squared table (alpha=0.001), index = degrees of freedom - 1 */
    private static final double[] CHISQR = {
        10.8, 13.8, 16.3, 18.5, 20.5, 22.5, 24.3, 26.1, 27.9, 29.6,
        31.3, 32.9, 34.5, 36.1, 37.7, 39.3, 40.8, 42.3, 43.8, 45.3,
        46.8, 48.3, 49.7, 51.2, 52.6, 54.1, 55.5, 56.9, 58.3, 59.7,
        61.1, 62.5, 63.9, 65.2, 66.6, 68.0, 69.3, 70.7, 72.1, 73.4,
        74.7, 76.0, 77.3, 78.6, 80.0, 81.3, 82.6, 84.0, 85.4, 86.7,
        88.0, 89.3, 90.6, 91.9, 93.3, 94.7, 96.0, 97.4, 98.7, 100,
        101,  102,  103,  104,  105,  107,  108,  109,  110,  112,
        113,  114,  115,  116,  118,  119,  120,  122,  123,  125,
        126,  127,  128,  129,  131,  132,  133,  134,  135,  137,
        138,  139,  140,  142,  143,  144,  145,  147,  148,  149
    };

    // ---------------------------------------------------------------
    // Measurement error variance
    // ---------------------------------------------------------------

    /**
     * Compute pseudorange measurement error variance.
     * <p>
     * Ported from pntpos.c varerr() lines 51-76.
     *
     * @param opt processing options
     * @param obs observation data
     * @param el  elevation angle (rad)
     * @param sys satellite system (SYS_xxx)
     * @return measurement error variance (m^2)
     */
    static double varerr(ProcessingOptions opt, ObsData obs, double el, int sys) {
        double fact = 1.0;

        switch (sys) {
            case SYS_GPS: fact *= EFACT_GPS; break;
            case SYS_GLO: fact *= EFACT_GLO; break;
            case SYS_SBS: fact *= EFACT_SBS; break;
            case SYS_CMP: fact *= EFACT_CMP; break;
            case SYS_QZS: fact *= EFACT_QZS; break;
            case SYS_IRN: fact *= EFACT_IRN; break;
            default:      fact *= EFACT_GPS; break;
        }

        if (el < MIN_EL) el = MIN_EL;

        /* var = R^2*(a^2 + b^2/sin(el) + c^2*(10^(0.1*(snr_max-snr)))) + (d*rcv_std)^2 */
        double varr = sq(opt.err[1]) + sq(opt.err[2]) / Math.sin(el);

        if (opt.err[6] > 0.0) { /* SNR-dependent term */
            varr += sq(opt.err[6]) * Math.pow(10.0, 0.1 * Math.max(opt.err[5] - obs.SNR[0], 0.0));
        }

        varr *= sq(opt.eratio[0]);

        if (opt.err[7] > 0.0) { /* receiver-reported std term */
            varr += sq(opt.err[7] * obs.Pstd[0]);
        }

        if (opt.ionoopt == IONOOPT_IFLC) { /* iono-free */
            varr *= sq(3.0);
        }

        return sq(fact) * varr;
    }

    // ---------------------------------------------------------------
    // Group delay parameter
    // ---------------------------------------------------------------

    /**
     * Get group delay parameter (TGD) in meters.
     * <p>
     * Ported from pntpos.c gettgd() lines 78-94.
     *
     * @param sat  satellite number
     * @param nav  navigation data
     * @param type TGD type index (0-5)
     * @return group delay (m)
     */
    static double gettgd(int sat, Navigation nav, int type) {
        int sys = SatelliteUtil.satsys(sat)[0];

        if (sys == SYS_GLO) {
            for (int i = 0; i < nav.geph.size(); i++) {
                if (nav.geph.get(i).sat == sat) {
                    return -nav.geph.get(i).dtaun * CLIGHT;
                }
            }
            return 0.0;
        } else {
            for (int i = 0; i < nav.eph.size(); i++) {
                if (nav.eph.get(i).sat == sat) {
                    return nav.eph.get(i).tgd[type] * CLIGHT;
                }
            }
            return 0.0;
        }
    }

    // ---------------------------------------------------------------
    // SNR mask test
    // ---------------------------------------------------------------

    /**
     * Test SNR mask. Returns true if observation passes the mask.
     * <p>
     * Ported from pntpos.c snrmask() lines 96-108.
     *
     * @param obs  observation data
     * @param azel azimuth/elevation {az, el} (rad) at offset in array
     * @param azelOff offset into azel array
     * @param opt  processing options
     * @return true if SNR passes mask (satellite should be used)
     */
    static boolean snrmask(ObsData obs, double[] azel, int azelOff, ProcessingOptions opt) {
        if (testsnr(0, 0, azel[azelOff + 1], obs.SNR[0], opt.snrmask)) {
            return false;
        }
        if (opt.ionoopt == IONOOPT_IFLC) {
            int f2 = seliflc(opt.nf, SatelliteUtil.satsys(obs.sat)[0], obs);
            if (testsnr(0, f2, azel[azelOff + 1], obs.SNR[f2], opt.snrmask)) {
                return false;
            }
        }
        return true;
    }

    /**
     * Test SNR against mask threshold.
     * <p>
     * Ported from rtkcmn.c testsnr() lines 567-582.
     *
     * @param base base/rover flag (0=rover)
     * @param idx  frequency index
     * @param el   elevation (rad)
     * @param snr  signal-to-noise ratio (dBHz)
     * @param mask SNR mask settings
     * @return true if SNR is below the mask (satellite should be rejected)
     */
    public static boolean testsnr(int base, int idx, double el, double snr,
                                  ProcessingOptions.SnrMask mask) {
        if (mask.ena[base] == 0 || idx < 0 || idx >= NFREQ) return false;

        double a = (el * R2D + 5.0) / 10.0;
        int i = (int) Math.floor(a);
        a -= i;

        double minsnr;
        if (i < 1) {
            minsnr = mask.mask[idx][0];
        } else if (i > 8) {
            minsnr = mask.mask[idx][8];
        } else {
            minsnr = (1.0 - a) * mask.mask[idx][i - 1] + a * mask.mask[idx][i];
        }

        return snr < minsnr;
    }

    // ---------------------------------------------------------------
    // Iono-free frequency selection
    // ---------------------------------------------------------------

    /**
     * Select second frequency index for iono-free combination.
     * <p>
     * Ported from rtkcmn.c seliflc() lines 3741-3745.
     * Extended: falls back to L5 (f=2) when L2 (f=1) is unavailable,
     * supporting GPS L1+L5 receivers.
     *
     * @param optnf number of frequencies option
     * @param sys   satellite system
     * @return second frequency index (1 or 2)
     */
    public static int seliflc(int optnf, int sys) {
        // Galileo with L5 enabled: always use E5a (f=2)
        if (optnf >= 3 && sys == SYS_GAL) return 2;
        // Default: L2 (f=1)
        return 1;
    }

    /**
     * Select second frequency index with observation-aware fallback.
     * When L2 has no data but L5 does, falls back to L5 for IFLC.
     *
     * @param optnf number of frequencies option
     * @param sys   satellite system
     * @param obs   observation data (to check L/P availability)
     * @return second frequency index (1 or 2)
     */
    public static int seliflc(int optnf, int sys, ObsData obs) {
        int f2 = seliflc(optnf, sys);
        // Fallback: if L2 has no phase/code but L5 does, use L5
        if (f2 == 1 && optnf >= 3 && obs != null) {
            boolean l2empty = (obs.L[1] == 0.0 && obs.P[1] == 0.0);
            boolean l5avail = (obs.L[2] != 0.0 || obs.P[2] != 0.0);
            if (l2empty && l5avail) return 2;
        }
        return f2;
    }

    /**
     * Get ephemeris selection for a system.
     * <p>
     * Ported from ephemeris.c getseleph() lines 869-881.
     * Returns the ephemeris selection for the system from the navigation data.
     *
     * @param sys satellite system
     * @param nav navigation data
     * @return ephemeris selection (0=any, 1=I/NAV, 2=F/NAV)
     */
    static int getseleph(int sys, Navigation nav) {
        /* In the C code this uses a global eph_sel[] array.
         * Here we use the per-system navigation data ephSel array.
         * For simplicity, return 0 (use any) as default. The nav.ephSel
         * can be populated by the caller to control I/NAV vs F/NAV. */
        switch (sys) {
            case SYS_GPS: return 0;
            case SYS_GLO: return 0;
            case SYS_GAL: return nav.ephSel.length > 0 ? nav.ephSel[0] : 0;
            default:      return 0;
        }
    }

    // ---------------------------------------------------------------
    // Pseudorange with code bias and TGD correction
    // ---------------------------------------------------------------

    /**
     * Compute corrected pseudorange with code bias and TGD correction.
     * <p>
     * Ported from pntpos.c prange() lines 110-198.
     *
     * @param obs observation data
     * @param nav navigation data
     * @param opt processing options
     * @param var output: measurement variance (single-element array)
     * @return corrected pseudorange (m), or 0 if error
     */
    static double prange(ObsData obs, Navigation nav, ProcessingOptions opt,
                         double[] var) {
        int sat = obs.sat;
        int sys = SatelliteUtil.satsys(sat)[0];
        double P1 = obs.P[0];
        int f2 = seliflc(opt.nf, sys, obs);
        double P2 = obs.P[f2];
        var[0] = 0.0;

        if (P1 == 0.0 || (opt.ionoopt == IONOOPT_IFLC && P2 == 0.0)) return 0.0;

        /* L1 code bias correction */
        int biasIx = code2biasIx(sys, obs.code[0]);
        if (biasIx > 0) { /* 0 = reference code */
            P1 += nav.cbias[sat - 1][0][biasIx - 1];
        }

        /* L2/L5 code bias correction */
        if (sys == SYS_GAL && f2 == 1) {
            /* skip code bias: no GAL L2 bias available */
        } else {
            biasIx = code2biasIx(sys, obs.code[f2]);
            if (biasIx > 0) {
                P2 += nav.cbias[sat - 1][1][biasIx - 1];
            }
        }

        if (opt.ionoopt == IONOOPT_IFLC) { /* dual-frequency iono-free */
            double gamma, b1, b2;

            if (sys == SYS_GPS || sys == SYS_QZS) { /* L1-L2 or L1-L5 */
                gamma = f2 == 1 ? sq(FREQL1 / FREQL2) : sq(FREQL1 / FREQL5);
                return (P2 - gamma * P1) / (1.0 - gamma);
            } else if (sys == SYS_GLO) { /* G1-G2 or G1-G3 */
                gamma = f2 == 1 ? sq(FREQ1_GLO / FREQ2_GLO) : sq(FREQ1_GLO / FREQ3_GLO);
                return (P2 - gamma * P1) / (1.0 - gamma);
            } else if (sys == SYS_GAL) { /* E1-E5b, E1-E5a */
                gamma = f2 == 1 ? sq(FREQL1 / FREQE5b) : sq(FREQL1 / FREQL5);
                if (f2 == 1 && getseleph(SYS_GAL, nav) != 0) { /* F/NAV */
                    P2 -= gettgd(sat, nav, 0) - gettgd(sat, nav, 1); /* BGD_E5aE5b */
                }
                return (P2 - gamma * P1) / (1.0 - gamma);
            } else if (sys == SYS_CMP) { /* B1-B2 */
                gamma = sq((obs.code[0] == CODE_L2I ? FREQ1_CMP : FREQL1) / FREQ2_CMP);
                if (obs.code[0] == CODE_L2I) {
                    b1 = gettgd(sat, nav, 0); /* TGD_B1I */
                } else if (obs.code[0] == CODE_L1P) {
                    b1 = gettgd(sat, nav, 2); /* TGD_B1Cp */
                } else {
                    b1 = gettgd(sat, nav, 2) + gettgd(sat, nav, 4); /* TGD_B1Cp+ISC_B1Cd */
                }
                b2 = gettgd(sat, nav, 1); /* TGD_B2I/B2bI (m) */
                return ((P2 - gamma * P1) - (b2 - gamma * b1)) / (1.0 - gamma);
            } else if (sys == SYS_IRN) { /* L5-S */
                gamma = sq(FREQL5 / FREQs);
                return (P2 - gamma * P1) / (1.0 - gamma);
            }
        } else { /* single-frequency */
            var[0] = sq(ERR_CBIAS);

            if (sys == SYS_GPS || sys == SYS_QZS) { /* L1 */
                return P1 - gettgd(sat, nav, 0);
            } else if (sys == SYS_GLO) { /* G1 */
                double gamma = sq(FREQ1_GLO / FREQ2_GLO);
                double b1 = gettgd(sat, nav, 0); /* -dtaun (m) */
                return P1 - b1 / (gamma - 1.0);
            } else if (sys == SYS_GAL) { /* E1 */
                double b1;
                if (getseleph(SYS_GAL, nav) != 0) {
                    b1 = gettgd(sat, nav, 0); /* BGD_E1E5a */
                } else {
                    b1 = gettgd(sat, nav, 1); /* BGD_E1E5b */
                }
                return P1 - b1;
            } else if (sys == SYS_CMP) { /* B1I/B1Cp/B1Cd */
                double b1;
                if (obs.code[0] == CODE_L2I) {
                    b1 = gettgd(sat, nav, 0); /* TGD_B1I */
                } else if (obs.code[0] == CODE_L1P) {
                    b1 = gettgd(sat, nav, 2); /* TGD_B1Cp */
                } else {
                    b1 = gettgd(sat, nav, 2) + gettgd(sat, nav, 4); /* TGD_B1Cp+ISC_B1Cd */
                }
                return P1 - b1;
            } else if (sys == SYS_IRN) { /* L5 */
                double gamma = sq(FREQs / FREQL5);
                double b1 = gettgd(sat, nav, 0); /* TGD (m) */
                return P1 - gamma * b1;
            }
        }
        return P1;
    }

    /**
     * Map observation code to code bias index.
     * <p>
     * Simplified port of code2bias_ix() from preceph.c.
     * Returns 0 for the reference code (no bias needed), or a positive
     * index into nav.cbias[sat][freq][index-1].
     * <p>
     * The full implementation requires the code_bias_ix lookup table.
     * This stub returns 0 (no bias) to allow SPP to function without
     * the full bias table. Override or extend when bias support is needed.
     *
     * @param sys  satellite system
     * @param code observation code
     * @return bias index (0 = reference code / no bias)
     */
    static int code2biasIx(int sys, int code) {
        if (code <= 0 || code > MAXCODE) return -1;
        int sysIx;
        switch (sys) {
            case SYS_GPS: case SYS_SBS: sysIx = 0; break;
            case SYS_GLO: sysIx = 1; break;
            case SYS_GAL: sysIx = 2; break;
            case SYS_CMP: sysIx = 3; break;
            default: return 0;
        }
        return CODE_BIAS_IX[sysIx][code - 1];
    }

    /** Number of bias systems supported in lookup table */
    private static final int MAX_BIAS_SYS = 4;

    /** Code bias index lookup table [sys][code-1], -1=not supported, 0=ref, 1-3=bias index */
    private static final int[][] CODE_BIAS_IX = initCodeBiasIx();

    private static int[][] initCodeBiasIx() {
        int[][] ix = new int[MAX_BIAS_SYS][MAXCODE];
        for (int i = 0; i < MAX_BIAS_SYS; i++)
            java.util.Arrays.fill(ix[i], -1);

        // GPS: L1W/L2W=ref(0), L1C/L2L=1, L1L/L2S=2, L1X/L2X=3
        ix[0][Constants.CODE_L1W - 1] = 0;
        ix[0][Constants.CODE_L1C - 1] = 1;
        ix[0][Constants.CODE_L1L - 1] = 2;
        ix[0][Constants.CODE_L1X - 1] = 3;
        ix[0][Constants.CODE_L2W - 1] = 0;
        ix[0][Constants.CODE_L2L - 1] = 1;
        ix[0][Constants.CODE_L2S - 1] = 2;
        ix[0][Constants.CODE_L2X - 1] = 3;
        // GLONASS: L1P/L2P=ref(0), L1C/L2C=1
        ix[1][Constants.CODE_L1P - 1] = 0;
        ix[1][Constants.CODE_L1C - 1] = 1;
        ix[1][Constants.CODE_L2P - 1] = 0;
        ix[1][Constants.CODE_L2C - 1] = 1;
        // Galileo: L1C/L5Q=ref(0), L1X/L5I=1, L5X=2
        ix[2][Constants.CODE_L1C - 1] = 0;
        ix[2][Constants.CODE_L1X - 1] = 1;
        ix[2][Constants.CODE_L5Q - 1] = 0;
        ix[2][Constants.CODE_L5I - 1] = 1;
        ix[2][Constants.CODE_L5X - 1] = 2;
        // BeiDou: L2I/L6I=ref(0)
        ix[3][Constants.CODE_L2I - 1] = 0;
        ix[3][Constants.CODE_L6I - 1] = 0;
        return ix;
    }

    // ---------------------------------------------------------------
    // Observation code constants needed for BDS TGD handling
    // ---------------------------------------------------------------

    /** Obs code: L1P (GPS, GLO, BDS) */
    private static final int CODE_L1P = 2;

    /** Obs code: B1_2I (BDS) */
    private static final int CODE_L2I = 40;

    // ---------------------------------------------------------------
    // Ionospheric correction
    // ---------------------------------------------------------------

    /**
     * Compute ionospheric correction.
     * <p>
     * Ported from pntpos.c ionocorr() lines 211-246.
     *
     * @param time    observation time
     * @param nav     navigation data
     * @param sat     satellite number
     * @param pos     receiver position {lat, lon, h} (rad, m)
     * @param azel    azimuth/elevation {az, el} (rad) at offset
     * @param azelOff offset into azel array
     * @param ionoopt ionosphere correction option
     * @param ion     output: ionospheric delay L1 (m) [1]
     * @param vion    output: ionospheric delay variance (m^2) [1]
     * @return true on success
     */
    static boolean ionocorr(GTime time, Navigation nav, int sat, double[] pos,
                            double[] azel, int azelOff, int ionoopt,
                            double[] ion, double[] vion) {
        boolean err = false;

        /* SBAS ionosphere model */
        if (ionoopt == IONOOPT_SBAS) {
            /* SBAS iono correction not yet implemented in Java port */
            err = true;
        }
        /* IONEX TEC model */
        if (ionoopt == IONOOPT_TEC) {
            /* IONEX TEC not yet implemented in Java port */
            err = true;
        }
        /* QZSS broadcast ionosphere model */
        if (ionoopt == IONOOPT_QZS && MatrixUtil.norm(nav.ionQzs, 8) > 0.0) {
            ion[0] = ionmodel(time, nav.ionQzs, pos, azel, azelOff);
            vion[0] = sq(ion[0] * ERR_BRDCI);
            return true;
        }
        /* GPS broadcast ionosphere model (also fallback for SBAS/TEC errors) */
        if (ionoopt == IONOOPT_BRDC || err) {
            ion[0] = ionmodel(time, nav.ionGps, pos, azel, azelOff);
            vion[0] = sq(ion[0] * ERR_BRDCI);
            return true;
        }
        /* no correction */
        ion[0] = 0.0;
        vion[0] = (ionoopt == IONOOPT_OFF) ? sq(ERR_ION) : 0.0;
        return true;
    }

    /**
     * Compute ionospheric delay by broadcast ionosphere model (Klobuchar).
     * <p>
     * Simplified from rtkcmn.c ionmodel(). This is a placeholder that
     * delegates to the Ionosphere correction class when available.
     *
     * @param time    time (GPST)
     * @param ionPar  ionosphere parameters {a0,a1,a2,a3,b0,b1,b2,b3}
     * @param pos     receiver position {lat, lon, h} (rad, m)
     * @param azel    azimuth/elevation at offset
     * @param azelOff offset into azel array
     * @return ionospheric delay (L1) (m)
     */
    static double ionmodel(GTime time, double[] ionPar, double[] pos,
                           double[] azel, int azelOff) {
        /* Klobuchar model implementation */
        final double[] ION_DEFAULT = {
            0.1118E-07, -0.7451E-08, -0.5961E-07, 0.1192E-06,
            0.1167E+06, -0.2294E+06, -0.1311E+06, 0.1049E+07
        };

        double[] ion = (MatrixUtil.norm(ionPar, 8) > 0.0) ? ionPar : ION_DEFAULT;

        double el = azel[azelOff + 1];
        double az = azel[azelOff];

        if (pos[2] < -1E3 || el <= 0.0) return 0.0;

        /* earth-centered angle (semi-circle) */
        double psi = 0.0137 / (el / PI + 0.11) - 0.022;

        /* subionospheric point latitude (semi-circle) */
        double phi = pos[0] / PI + psi * Math.cos(az);
        if (phi > 0.416) phi = 0.416;
        else if (phi < -0.416) phi = -0.416;

        /* subionospheric point longitude (semi-circle) */
        double lam = pos[1] / PI + psi * Math.sin(az) / Math.cos(phi * PI);

        /* geomagnetic latitude (semi-circle) */
        phi += 0.064 * Math.cos((lam - 1.617) * PI);

        /* local time (s) */
        double[] wt = time.time2gpst();
        double tt = 43200.0 * lam + wt[1];
        tt -= Math.floor(tt / 86400.0) * 86400.0; /* 0 <= tt < 86400 */

        /* slant factor */
        double f = 1.0 + 16.0 * Math.pow(0.53 - el / PI, 3.0);

        /* ionospheric delay */
        double amp = ion[0] + phi * (ion[1] + phi * (ion[2] + phi * ion[3]));
        double per = ion[4] + phi * (ion[5] + phi * (ion[6] + phi * ion[7]));
        if (amp < 0.0) amp = 0.0;
        if (per < 72000.0) per = 72000.0;

        double x = 2.0 * PI * (tt - 50400.0) / per;

        return CLIGHT * f * (Math.abs(x) < 1.57 ? 5E-9 + amp * (1.0 + x * x * (-0.5 + x * x / 24.0)) : 5E-9);
    }

    // ---------------------------------------------------------------
    // Tropospheric correction
    // ---------------------------------------------------------------

    /**
     * Compute tropospheric correction.
     * <p>
     * Ported from pntpos.c tropcorr() lines 258-282.
     *
     * @param time    observation time
     * @param nav     navigation data (unused for Saastamoinen)
     * @param pos     receiver position {lat, lon, h} (rad, m)
     * @param azel    azimuth/elevation at offset
     * @param azelOff offset into azel array
     * @param tropopt troposphere correction option
     * @param trp     output: tropospheric delay (m) [1]
     * @param vtrp    output: tropospheric delay variance (m^2) [1]
     * @return true on success
     */
    static boolean tropcorr(GTime time, Navigation nav, double[] pos,
                            double[] azel, int azelOff, int tropopt,
                            double[] trp, double[] vtrp) {
        /* Saastamoinen model */
        if (tropopt == TROPOPT_SAAS || tropopt == TROPOPT_EST || tropopt == TROPOPT_ESTG) {
            trp[0] = tropmodel(time, pos, azel, azelOff, REL_HUMI);
            vtrp[0] = sq(ERR_SAAS / (Math.sin(azel[azelOff + 1]) + 0.1));
            return true;
        }
        /* SBAS (MOPS) troposphere model */
        if (tropopt == TROPOPT_SBAS) {
            /* SBAS trop not yet implemented in Java port */
            trp[0] = tropmodel(time, pos, azel, azelOff, REL_HUMI);
            vtrp[0] = sq(ERR_SAAS / (Math.sin(azel[azelOff + 1]) + 0.1));
            return true;
        }
        /* no correction */
        trp[0] = 0.0;
        vtrp[0] = (tropopt == TROPOPT_OFF) ? sq(ERR_TROP) : 0.0;
        return true;
    }

    /**
     * Compute tropospheric delay by standard atmosphere and Saastamoinen model.
     * <p>
     * Ported from rtkcmn.c tropmodel() lines 3754-3790.
     *
     * @param time observation time (unused)
     * @param pos  receiver position {lat, lon, h} (rad, m)
     * @param azel azimuth/elevation at offset
     * @param azelOff offset into azel array
     * @param humi relative humidity
     * @return tropospheric delay (m)
     */
    static double tropmodel(GTime time, double[] pos, double[] azel,
                            int azelOff, double humi) {
        final double TEMP0 = 15.0; /* temperature at sea level (C) */

        if (pos[2] < -100.0 || 1E4 < pos[2] || azel[azelOff + 1] <= 0) return 0.0;

        /* standard atmosphere */
        double hgt = pos[2] < 0.0 ? 0.0 : pos[2];

        double pres = 1013.25 * Math.pow(1.0 - 2.2557E-5 * hgt, 5.2568);
        double temp = TEMP0 - 6.5E-3 * hgt + 273.16;
        double e = 6.108 * humi * Math.exp((17.15 * temp - 4684.0) / (temp - 38.45));

        /* Saastamoinen model */
        double z = PI / 2.0 - azel[azelOff + 1];
        double trph = 0.0022768 * pres / (1.0 - 0.00266 * Math.cos(2.0 * pos[0]) - 0.00028 * hgt / 1E3)
                      / Math.cos(z);
        double trpw = 0.002277 * (1255.0 / temp + 0.05) * e / Math.cos(z);

        return trph + trpw;
    }

    // ---------------------------------------------------------------
    // Satellite frequency
    // ---------------------------------------------------------------

    /**
     * Get carrier frequency for a satellite and observation code.
     * <p>
     * Simplified port of sat2freq() from rtkcmn.c.
     *
     * @param sat  satellite number
     * @param code observation code
     * @param nav  navigation data
     * @return carrier frequency (Hz), or 0 if unknown
     */
    static double sat2freq(int sat, int code, Navigation nav) {
        int[] sp = SatelliteUtil.satsys(sat);
        int sys = sp[0];

        /* determine frequency band from code (simplified) */
        /* Code naming: L1x = 1xxx band, L2x = 2xxx band, L5x = 5xxx band etc. */
        switch (sys) {
            case SYS_GPS:
            case SYS_QZS:
                /* codes 1-19 are L1, 20-39 are L2, 24-29 are L5, etc. */
                if (code <= 19) return FREQL1;
                if (code >= 24 && code <= 29) return FREQL5;
                if (code >= 20 && code <= 23) return FREQL2;
                return FREQL1;
            case SYS_GLO: {
                int prn = sp[1];
                int fcn = (prn >= 1 && prn <= 32) ? nav.gloFcn[prn - 1] - 8 : 0;
                if (code <= 19) return FREQ1_GLO + DFRQ1_GLO * fcn;
                if (code >= 20 && code <= 39) return FREQ2_GLO + DFRQ2_GLO * fcn;
                return FREQ1_GLO + DFRQ1_GLO * fcn;
            }
            case SYS_GAL:
                if (code <= 19) return FREQL1;
                if (code >= 30 && code <= 39) return FREQE5b;
                if (code >= 24 && code <= 29) return FREQL5;
                return FREQL1;
            case SYS_CMP:
                if (code == CODE_L2I || code == CODE_L2I + 1 || code == CODE_L2I + 2)
                    return FREQ1_CMP;
                if (code <= 19) return FREQL1;
                if (code >= 30 && code <= 39) return FREQ2_CMP;
                return FREQ1_CMP;
            case SYS_IRN:
                if (code <= 29) return FREQL5;
                return FREQs;
            case SYS_SBS:
                if (code <= 19) return FREQL1;
                return FREQL5;
            default:
                return 0.0;
        }
    }

    // ---------------------------------------------------------------
    // Pseudorange residuals
    // ---------------------------------------------------------------

    /**
     * Build design matrix and residual vector for pseudorange positioning.
     * <p>
     * Ported from pntpos.c rescode() lines 284-376.
     *
     * @param iter iteration number (0=first)
     * @param obs  observation data array
     * @param n    number of observations
     * @param rs   satellite positions/velocities [6*n] (ECEF, m, m/s)
     * @param dts  satellite clock biases/drifts [2*n] (s, s/s)
     * @param vare satellite position variance [n] (m^2)
     * @param svh  satellite health flags [n]
     * @param nav  navigation data
     * @param x    current state estimate [NX]
     * @param opt  processing options
     * @param v    output: residual vector [n+NX-3]
     * @param H    output: design matrix [NX*(n+NX-3)], column-major
     * @param var  output: measurement variance [n+NX-3]
     * @param azel output: azimuth/elevation [2*n] (rad)
     * @param vsat output: valid satellite flags [n]
     * @param resp output: pseudorange residuals [n]
     * @param ns   output: number of valid satellites [1]
     * @return number of valid measurements
     */
    static int rescode(int iter, ObsData[] obs, int n, double[] rs,
                       double[] dts, double[] vare, int[] svh, Navigation nav,
                       double[] x, ProcessingOptions opt, double[] v, double[] H,
                       double[] var, double[] azel, int[] vsat, double[] resp,
                       int[] ns) {
        double[] rr = new double[3];
        double[] pos;
        double[] e = new double[3];
        int nv = 0;
        int[] mask = new int[NX - 3];

        for (int i = 0; i < 3; i++) rr[i] = x[i];
        double dtr = x[3];

        pos = Coord.ecef2pos(rr);

        trace.log(Level.FINE, () -> String.format("rescode: rr=%.3f %.3f %.3f", rr[0], rr[1], rr[2]));

        ns[0] = 0;

        for (int i = 0; i < n && i < MAXOBS; i++) {
            vsat[i] = 0;
            azel[i * 2] = azel[1 + i * 2] = resp[i] = 0.0;

            GTime time = obs[i].time;
            int sat = obs[i].sat;
            int[] sp = SatelliteUtil.satsys(sat);
            int sys = sp[0];
            if (sys == SYS_NONE) continue;

            /* reject duplicated observation data */
            if (i < n - 1 && i < MAXOBS - 1 && sat == obs[i + 1].sat) {
                trace.log(Level.WARNING, () -> String.format("duplicated obs data sat=%d", sat));
                i++;
                continue;
            }

            /* excluded satellite? */
            if (satexclude(sat, vare[i], svh[i], opt)) continue;

            /* geometric distance and elevation mask */
            double[] rsI = rsSlice(rs, i);
            double r = Geometry.geodist(rsI, rr, e);
            if (r <= 0.0) continue;
            double[] azelTmp = new double[2];
            double el = Geometry.satazel(pos, e, azelTmp);
            if (el < opt.elmin) continue;
            azel[i * 2] = azelTmp[0];
            azel[1 + i * 2] = azelTmp[1];

            double dion = 0.0, dtrp = 0.0, vion = 0.0, vtrp = 0.0;

            if (iter > 0) {
                /* test SNR mask */
                if (!snrmask(obs[i], azel, i * 2, opt)) continue;

                /* ionospheric correction */
                double[] ionOut = {0.0}, vionOut = {0.0};
                if (!ionocorr(time, nav, sat, pos, azel, i * 2, opt.ionoopt, ionOut, vionOut)) continue;
                double freq = sat2freq(sat, obs[i].code[0], nav);
                if (freq == 0.0) continue;

                /* convert from FREQL1 to actual frequency */
                dion = ionOut[0] * sq(FREQL1 / freq);
                vion = vionOut[0] * sq(sq(FREQL1 / freq));

                /* tropospheric correction */
                double[] trpOut = {0.0}, vtrpOut = {0.0};
                if (!tropcorr(time, nav, pos, azel, i * 2, opt.tropopt, trpOut, vtrpOut)) continue;
                dtrp = trpOut[0];
                vtrp = vtrpOut[0];
            }

            /* pseudorange with code bias correction */
            double[] vmeas = {0.0};
            double P = prange(obs[i], nav, opt, vmeas);
            if (P == 0.0) continue;

            /* pseudorange residual */
            v[nv] = P - (r + dtr - CLIGHT * dts[i * 2] + dion + dtrp);

            final int satF = sat;
            final double vF = v[nv], PF = P, rF = r, dtrF = dtr;
            final double dionF = dion, dtrpF = dtrp;
            final double dtsF = dts[i * 2];
            trace.log(Level.FINER, () -> String.format(
                "sat=%d: v=%.3f P=%.3f r=%.3f dtr=%.6f dts=%.6f dion=%.3f dtrp=%.3f",
                satF, vF, PF, rF, dtrF, dtsF, dionF, dtrpF));

            /* design matrix */
            for (int j = 0; j < NX; j++) {
                H[j + nv * NX] = j < 3 ? -e[j] : (j == 3 ? 1.0 : 0.0);
            }

            /* time system offset and receiver bias correction */
            if (sys == SYS_GLO)      { v[nv] -= x[4]; H[4 + nv * NX] = 1.0; mask[1] = 1; }
            else if (sys == SYS_GAL) { v[nv] -= x[5]; H[5 + nv * NX] = 1.0; mask[2] = 1; }
            else if (sys == SYS_CMP) { v[nv] -= x[6]; H[6 + nv * NX] = 1.0; mask[3] = 1; }
            else if (sys == SYS_IRN) { v[nv] -= x[7]; H[7 + nv * NX] = 1.0; mask[4] = 1; }
            else if (sys == SYS_QZS) { v[nv] -= x[8]; H[8 + nv * NX] = 1.0; mask[5] = 1; }
            else mask[0] = 1; /* GPS */

            vsat[i] = 1;
            resp[i] = v[nv];
            ns[0]++;

            /* variance of pseudorange error */
            var[nv] = vare[i] + vmeas[0] + vion + vtrp;
            var[nv] += varerr(opt, obs[i], azel[1 + i * 2], sys);

            final int satF2 = obs[i].sat;
            final double azF = azel[i * 2], elF = azel[1 + i * 2];
            final double respF = resp[i], sigF = Math.sqrt(var[nv]);
            trace.log(Level.FINER, () -> String.format(
                "sat=%2d azel=%5.1f %4.1f res=%7.3f sig=%5.3f",
                satF2, azF * R2D, elF * R2D, respF, sigF));

            nv++;
        }

        /* constraint to avoid rank-deficient */
        for (int i = 0; i < NX - 3; i++) {
            if (mask[i] != 0) continue;
            v[nv] = 0.0;
            for (int j = 0; j < NX; j++) {
                H[j + nv * NX] = (j == i + 3) ? 1.0 : 0.0;
            }
            var[nv] = 0.01;
            nv++;
        }

        return nv;
    }

    // ---------------------------------------------------------------
    // Validate solution
    // ---------------------------------------------------------------

    /**
     * Validate SPP solution by chi-squared test and GDOP check.
     * <p>
     * Ported from pntpos.c valsol() lines 377-406.
     *
     * @param azel azimuth/elevation [2*n] (rad)
     * @param vsat valid satellite flags [n]
     * @param n    number of observations
     * @param opt  processing options
     * @param v    residual vector [nv]
     * @param nv   number of valid measurements
     * @param nx   number of estimated parameters
     * @param msg  output: error/warning message
     * @return true if solution is valid
     */
    static boolean valsol(double[] azel, int[] vsat, int n,
                          ProcessingOptions opt, double[] v, int nv, int nx,
                          StringBuilder msg) {
        trace.log(Level.FINE, () -> String.format("valsol: n=%d nv=%d", n, nv));

        /* chi-square validation of residuals */
        double vv = MatrixUtil.dot(v, v, nv);
        if (nv > nx && nv - nx - 1 < CHISQR.length && vv > CHISQR[nv - nx - 1]) {
            msg.append(String.format("Warning: large chi-square error nv=%d vv=%.1f cs=%.1f",
                                     nv, vv, CHISQR[nv - nx - 1]));
            /* threshold too strict for all use cases, continue on */
        }

        /* large GDOP check */
        double[] azels = new double[n * 2];
        int nsv = 0;
        for (int i = 0; i < n; i++) {
            if (vsat[i] == 0) continue;
            azels[nsv * 2] = azel[i * 2];
            azels[1 + nsv * 2] = azel[1 + i * 2];
            nsv++;
        }

        double[] dop = new double[4];
        Geometry.dops(nsv, azels, opt.elmin, dop);

        if (dop[0] <= 0.0 || dop[0] > MAX_GDOP) {
            msg.append(String.format("gdop error nv=%d gdop=%.1f", nv, dop[0]));
            return false;
        }
        return true;
    }

    // ---------------------------------------------------------------
    // Estimate receiver position (weighted least squares)
    // ---------------------------------------------------------------

    /**
     * Estimate receiver position by weighted least-squares iteration.
     * <p>
     * Ported from pntpos.c estpos() lines 407-477.
     *
     * @param obs  observation data array
     * @param n    number of observations
     * @param rs   satellite positions/velocities [6*n]
     * @param dts  satellite clock biases/drifts [2*n]
     * @param vare satellite position variance [n]
     * @param svh  satellite health flags [n]
     * @param nav  navigation data
     * @param opt  processing options
     * @param sol  solution (input: initial position in rr[]; output: solution)
     * @param azel output: azimuth/elevation [2*n]
     * @param vsat output: valid satellite flags [n]
     * @param resp output: pseudorange residuals [n]
     * @param msg  output: error message
     * @return 1 on success, 0 on failure
     */
    static int estpos(ObsData[] obs, int n, double[] rs, double[] dts,
                      double[] vare, int[] svh, Navigation nav,
                      ProcessingOptions opt, Solution sol, double[] azel,
                      int[] vsat, double[] resp, StringBuilder msg) {
        int maxSize = n + NX - 3;
        double[] v = new double[maxSize];
        double[] H = new double[NX * maxSize];
        double[] var = new double[maxSize];
        double[] x = new double[NX];
        double[] dx = new double[NX];
        double[] Q = new double[NX * NX];

        trace.log(Level.FINE, () -> String.format("estpos: n=%d", n));

        for (int i = 0; i < 3; i++) x[i] = sol.rr[i];

        for (int iter = 0; iter < MAXITR; iter++) {
            /* pseudorange residuals (m) */
            int[] ns = {0};
            int nv = rescode(iter, obs, n, rs, dts, vare, svh, nav, x, opt,
                             v, H, var, azel, vsat, resp, ns);

            if (nv < NX) {
                msg.append(String.format("lack of valid sats ns=%d", nv));
                break;
            }

            /* weight by variance (lsq uses sqrt of weight) */
            for (int j = 0; j < nv; j++) {
                double sig = Math.sqrt(var[j]);
                v[j] /= sig;
                for (int k = 0; k < NX; k++) {
                    H[k + j * NX] /= sig;
                }
            }

            /* least square estimation */
            if (MatrixUtil.lsq(H, v, NX, nv, dx, Q) != 0) {
                msg.append(String.format("lsq error"));
                break;
            }

            for (int j = 0; j < NX; j++) x[j] += dx[j];

            if (MatrixUtil.norm(dx, NX) < 1E-4) {
                sol.type = 0;
                sol.time = obs[0].time.add(-x[3] / CLIGHT);
                sol.dtr[0] = x[3] / CLIGHT; /* receiver clock bias (s) */
                sol.dtr[1] = x[4] / CLIGHT; /* GLO-GPS time offset (s) */
                sol.dtr[2] = x[5] / CLIGHT; /* GAL-GPS time offset (s) */
                sol.dtr[3] = x[6] / CLIGHT; /* BDS-GPS time offset (s) */
                sol.dtr[4] = x[7] / CLIGHT; /* IRN-GPS time offset (s) */
                sol.dtr[5] = x[8] / CLIGHT; /* QZS-GPS time offset (s) */

                for (int j = 0; j < 6; j++) sol.rr[j] = j < 3 ? x[j] : 0.0;
                sol.qr[0] = (float) Q[0];            /* var xx */
                sol.qr[1] = (float) Q[1 + NX];       /* var yy */
                sol.qr[2] = (float) Q[2 + 2 * NX];   /* var zz */
                sol.qr[3] = (float) Q[1];             /* cov xy */
                sol.qr[4] = (float) Q[2 + NX];        /* cov yz */
                sol.qr[5] = (float) Q[2];             /* cov zx */
                sol.ns = ns[0];
                sol.age = 0.0f;
                sol.ratio = 0.0f;

                /* validate solution */
                boolean valid = valsol(azel, vsat, n, opt, v, nv, NX, msg);
                if (valid) {
                    sol.stat = (opt.sateph == EPHOPT_SBAS) ? SOLQ_SBAS : SOLQ_SINGLE;
                }
                return valid ? 1 : 0;
            }
        }

        if (msg.length() == 0) {
            msg.append("iteration divergent");
        }
        return 0;
    }

    // ---------------------------------------------------------------
    // RAIM FDE (Failure Detection and Exclusion)
    // ---------------------------------------------------------------

    /**
     * RAIM FDE: if at least 6 satellites available, try excluding each
     * one and re-estimating to detect/exclude a faulty satellite.
     * <p>
     * Ported from pntpos.c raim_fde() lines 478-554.
     *
     * @param obs  observation data array
     * @param n    number of observations
     * @param rs   satellite positions/velocities [6*n]
     * @param dts  satellite clock biases/drifts [2*n]
     * @param vare satellite position variance [n]
     * @param svh  satellite health flags [n]
     * @param nav  navigation data
     * @param opt  processing options
     * @param sol  solution (output)
     * @param azel output: azimuth/elevation [2*n]
     * @param vsat output: valid satellite flags [n]
     * @param resp output: pseudorange residuals [n]
     * @param msg  output: error message
     * @return 1 on success, 0 on failure
     */
    static int raimFde(ObsData[] obs, int n, double[] rs, double[] dts,
                       double[] vare, int[] svh, Navigation nav,
                       ProcessingOptions opt, Solution sol, double[] azel,
                       int[] vsat, double[] resp, StringBuilder msg) {
        trace.log(Level.FINE, () -> String.format("raim_fde: n=%d", n));

        ObsData[] obsE = new ObsData[n - 1];
        double[] rsE = new double[6 * n];
        double[] dtsE = new double[2 * n];
        double[] vareE = new double[n];
        double[] azelE = new double[2 * n];
        double[] respE = new double[n];
        int[] svhE = new int[n];
        int[] vsatE = new int[n];

        int stat = 0;
        int exSat = 0;
        double rms = 100.0;

        for (int i = 0; i < n; i++) {
            /* build observation set excluding satellite i */
            int k = 0;
            for (int j = 0; j < n; j++) {
                if (j == i) continue;
                obsE[k] = obs[j];
                System.arraycopy(rs,   6 * j, rsE,   6 * k, 6);
                System.arraycopy(dts,  2 * j, dtsE,  2 * k, 2);
                vareE[k] = vare[j];
                svhE[k] = svh[j];
                k++;
            }

            /* estimate receiver position without satellite i */
            Solution solE = new Solution();
            StringBuilder msgE = new StringBuilder();
            Arrays.fill(azelE, 0.0);
            Arrays.fill(vsatE, 0);

            if (estpos(obsE, n - 1, rsE, dtsE, vareE, svhE, nav, opt,
                       solE, azelE, vsatE, respE, msgE) == 0) {
                final int exSatL = obs[i].sat;
                trace.log(Level.FINE, () -> String.format("raim_fde: exsat=%d (%s)", exSatL, msgE));
                continue;
            }

            /* compute RMS of residuals */
            int nvsat = 0;
            double rmsE = 0.0;
            for (int j = 0; j < n - 1; j++) {
                if (vsatE[j] == 0) continue;
                rmsE += sq(respE[j]);
                nvsat++;
            }

            if (nvsat < 5) {
                final int satF = obs[i].sat;
                final int nvsatF = nvsat;
                trace.log(Level.FINE, () -> String.format(
                    "raim_fde: exsat=%d lack of satellites nvsat=%d", satF, nvsatF));
                continue;
            }
            rmsE = Math.sqrt(rmsE / nvsat);

            final int satF = obs[i].sat;
            final double rmsF = rmsE;
            trace.log(Level.FINE, () -> String.format("raim_fde: exsat=%d rms=%.3f", satF, rmsF));

            if (rmsE > rms) continue;

            /* save best result */
            k = 0;
            for (int j = 0; j < n; j++) {
                if (j == i) continue;
                System.arraycopy(azelE, 2 * k, azel, 2 * j, 2);
                vsat[j] = vsatE[k];
                resp[j] = respE[k];
                k++;
            }
            stat = 1;
            solE.eventime = sol.eventime;
            copySolution(solE, sol);
            exSat = obs[i].sat;
            rms = rmsE;
            vsat[i] = 0;
            msg.setLength(0);
            msg.append(msgE);
        }

        if (stat != 0) {
            final String name = SatelliteUtil.satno2id(exSat);
            trace.log(Level.WARNING, () -> String.format("%s excluded by raim", name));
        }

        return stat;
    }

    // ---------------------------------------------------------------
    // Range rate residuals (for Doppler velocity)
    // ---------------------------------------------------------------

    /**
     * Compute range-rate residuals from Doppler observations.
     * <p>
     * Ported from pntpos.c resdop() lines 556-603.
     *
     * @param obs  observation data
     * @param n    number of observations
     * @param rs   satellite positions/velocities [6*n]
     * @param dts  satellite clock biases/drifts [2*n]
     * @param nav  navigation data
     * @param rr   receiver position (ECEF, m) [3]
     * @param x    velocity state [4]: {vx, vy, vz, dtr_dot}
     * @param azel azimuth/elevation [2*n]
     * @param vsat valid satellite flags [n]
     * @param err  Doppler error (Hz)
     * @param v    output: residuals [n]
     * @param H    output: design matrix [4*n], column-major
     * @return number of valid Doppler measurements
     */
    private static int resdop(ObsData[] obs, int n, double[] rs, double[] dts,
                              Navigation nav, double[] rr, double[] x,
                              double[] azel, int[] vsat, double err,
                              double[] v, double[] H) {
        trace.log(Level.FINE, () -> String.format("resdop: n=%d", n));

        double[] pos = Coord.ecef2pos(rr);
        double[][] E = Coord.xyz2enu(pos);
        int nv = 0;

        for (int i = 0; i < n && i < MAXOBS; i++) {
            double freq = sat2freq(obs[i].sat, obs[i].code[0], nav);

            if (obs[i].D[0] == 0.0 || freq == 0.0 || vsat[i] == 0) continue;

            /* check satellite velocity is available */
            double velNorm = Math.sqrt(rs[3 + i * 6] * rs[3 + i * 6]
                                     + rs[4 + i * 6] * rs[4 + i * 6]
                                     + rs[5 + i * 6] * rs[5 + i * 6]);
            if (velNorm <= 0.0) continue;

            /* LOS vector in ECEF from ENU */
            double cosel = Math.cos(azel[1 + i * 2]);
            double[] a = {
                Math.sin(azel[i * 2]) * cosel,
                Math.cos(azel[i * 2]) * cosel,
                Math.sin(azel[1 + i * 2])
            };

            /* transform ENU LOS to ECEF */
            double[] e = new double[3];
            /* E is row-major [3][3], need E' * a (transpose since E goes ECEF->ENU) */
            for (int j = 0; j < 3; j++) {
                e[j] = E[0][j] * a[0] + E[1][j] * a[1] + E[2][j] * a[2];
            }

            /* satellite velocity relative to receiver in ECEF */
            double[] vs = new double[3];
            for (int j = 0; j < 3; j++) {
                vs[j] = rs[j + 3 + i * 6] - x[j];
            }

            /* range rate with earth rotation correction */
            double rate = MatrixUtil.dot3(vs, e)
                + OMGE / CLIGHT * (rs[4 + i * 6] * rr[0] + rs[1 + i * 6] * x[0]
                                 - rs[3 + i * 6] * rr[1] - rs[i * 6] * x[1]);

            /* std of range rate error (m/s) */
            double sig = (err <= 0.0) ? 1.0 : err * CLIGHT / freq;

            /* range rate residual (m/s) */
            v[nv] = (-obs[i].D[0] * CLIGHT / freq - (rate + x[3] - CLIGHT * dts[1 + i * 2])) / sig;

            /* design matrix */
            for (int j = 0; j < 4; j++) {
                H[j + nv * 4] = (j < 3 ? -e[j] : 1.0) / sig;
            }
            nv++;
        }
        return nv;
    }

    // ---------------------------------------------------------------
    // Estimate receiver velocity
    // ---------------------------------------------------------------

    /**
     * Estimate receiver velocity from Doppler observations.
     * <p>
     * Ported from pntpos.c estvel() lines 604-639.
     *
     * @param obs  observation data
     * @param n    number of observations
     * @param rs   satellite positions/velocities [6*n]
     * @param dts  satellite clock biases/drifts [2*n]
     * @param nav  navigation data
     * @param opt  processing options
     * @param sol  solution (input: position in rr[]; output: velocity in rr[3..5])
     * @param azel azimuth/elevation [2*n]
     * @param vsat valid satellite flags [n]
     */
    static void estvel(ObsData[] obs, int n, double[] rs, double[] dts,
                       Navigation nav, ProcessingOptions opt, Solution sol,
                       double[] azel, int[] vsat) {
        double[] x = new double[4];
        double[] dx = new double[4];
        double[] Q = new double[16];
        double[] v = new double[n];
        double[] H = new double[4 * n];
        double err = opt.err[4]; /* Doppler error (Hz) */

        for (int iter = 0; iter < MAXITR; iter++) {
            /* range rate residuals (m/s) */
            int nv = resdop(obs, n, rs, dts, nav, sol.rr, x, azel, vsat, err, v, H);
            if (nv < 4) break;

            /* least square estimation */
            if (MatrixUtil.lsq(H, v, 4, nv, dx, Q) != 0) break;

            for (int j = 0; j < 4; j++) x[j] += dx[j];

            if (MatrixUtil.norm(dx, 4) < 1E-6) {
                trace.log(Level.FINE, () -> String.format(
                    "estvel: vx=%.3f vy=%.3f vz=%.3f n=%d", x[0], x[1], x[2], n));

                sol.rr[3] = x[0];
                sol.rr[4] = x[1];
                sol.rr[5] = x[2];
                sol.qv[0] = (float) Q[0];   /* xx */
                sol.qv[1] = (float) Q[5];   /* yy */
                sol.qv[2] = (float) Q[10];  /* zz */
                sol.qv[3] = (float) Q[1];   /* xy */
                sol.qv[4] = (float) Q[6];   /* yz */
                sol.qv[5] = (float) Q[2];   /* zx */
                break;
            }
        }
    }

    // ---------------------------------------------------------------
    // Main entry point
    // ---------------------------------------------------------------

    /**
     * Compute receiver position, velocity, and clock bias by single-point
     * positioning with pseudorange and Doppler observables.
     * <p>
     * Ported from pntpos.c pntpos() lines 653-722.
     *
     * @param obs  observation data array
     * @param n    number of observations
     * @param nav  navigation data
     * @param opt  processing options
     * @param sol  solution (output)
     * @param azel output: azimuth/elevation [2*n] (rad), or null
     * @param ssat output: per-satellite status [MAXSAT], or null
     * @param msg  output: error message
     * @return 1 on success, 0 on failure
     */
    public static int pntpos(ObsData[] obs, int n, Navigation nav,
                             ProcessingOptions opt, Solution sol,
                             double[] azel, SatStatus[] ssat,
                             StringBuilder msg) {
        trace.log(Level.FINE, () -> String.format("pntpos: tobs=%s n=%d", obs[0].time.format(3), n));

        sol.stat = SOLQ_NONE;

        if (n <= 0) {
            msg.append("no observation data");
            return 0;
        }
        sol.time = obs[0].time;
        msg.setLength(0);
        sol.eventime = obs[0].eventime;

        /* make a working copy of options */
        ProcessingOptions optW = copyOptions(opt);

        double[] rs = new double[6 * n];
        double[] dts = new double[2 * n];
        double[] var = new double[n];
        double[] azelW = new double[2 * n];
        double[] resp = new double[n];
        int[] vsat = new int[n];
        int[] svh = new int[n];

        if (ssat != null) {
            for (int i = 0; i < MAXSAT && i < ssat.length; i++) {
                ssat[i].snrRover[0] = 0;
                ssat[i].snrBase[0] = 0;
            }
            for (int i = 0; i < n; i++) {
                if (obs[i].sat - 1 < ssat.length) {
                    ssat[obs[i].sat - 1].snrRover[0] = obs[i].SNR[0];
                }
            }
        }

        if (optW.mode != PMODE_SINGLE) { /* for precise positioning */
            optW.ionoopt = IONOOPT_BRDC;
            optW.tropopt = TROPOPT_SAAS;
        }

        /* satellite positions, velocities and clocks */
        satposs(sol.time, obs, n, nav, optW.sateph, rs, dts, var, svh);

        /* estimate receiver position and time with pseudorange */
        int stat = estpos(obs, n, rs, dts, var, svh, nav, optW, sol, azelW, vsat, resp, msg);

        /* RAIM FDE */
        if (stat == 0 && n >= 6 && opt.posopt[4] != 0) {
            stat = raimFde(obs, n, rs, dts, var, svh, nav, optW, sol, azelW, vsat, resp, msg);
        }

        /* estimate receiver velocity with Doppler */
        if (stat != 0) {
            estvel(obs, n, rs, dts, nav, optW, sol, azelW, vsat);
        }

        if (azel != null) {
            System.arraycopy(azelW, 0, azel, 0, Math.min(n * 2, azel.length));
        }

        if (ssat != null) {
            for (int i = 0; i < MAXSAT && i < ssat.length; i++) {
                ssat[i].vs = 0;
                ssat[i].azel[0] = ssat[i].azel[1] = 0.0;
                ssat[i].resp[0] = ssat[i].resc[0] = 0.0;
            }
            for (int i = 0; i < n; i++) {
                int idx = obs[i].sat - 1;
                if (idx >= ssat.length) continue;
                ssat[idx].azel[0] = azelW[i * 2];
                ssat[idx].azel[1] = azelW[1 + i * 2];
                if (vsat[i] == 0) continue;
                ssat[idx].vs = 1;
                ssat[idx].resp[0] = resp[i];
            }
        }

        return stat;
    }

    // ---------------------------------------------------------------
    // Per-satellite status (simplified ssat_t)
    // ---------------------------------------------------------------

    /**
     * Per-satellite status information, simplified from C ssat_t.
     */
    public static class SatStatus {
        /** Valid satellite flag */
        public int vs;

        /** Azimuth/elevation (rad) */
        public double[] azel = new double[2];

        /** Pseudorange residual (m) */
        public double[] resp = new double[NFREQ];

        /** Carrier-phase residual (m) */
        public double[] resc = new double[NFREQ];

        /** Signal strength - rover (dBHz) */
        public double[] snrRover = new double[NFREQ];

        /** Signal strength - base (dBHz) */
        public double[] snrBase = new double[NFREQ];
    }

    // ---------------------------------------------------------------
    // Satellite positions (stub for Phase 1)
    // ---------------------------------------------------------------

    /**
     * Compute satellite positions, velocities, and clocks.
     * <p>
     * This is a delegation stub. The full implementation resides in
     * the ephemeris module. For Phase 1, this must be provided externally
     * or via the Ephemeris computation class.
     *
     * @param time   observation time
     * @param obs    observation data
     * @param n      number of observations
     * @param nav    navigation data
     * @param sateph ephemeris option
     * @param rs     output: satellite positions/velocities [6*n]
     * @param dts    output: satellite clock biases/drifts [2*n]
     * @param var    output: satellite position variances [n]
     * @param svh    output: satellite health flags [n]
     */
    static void satposs(GTime time, ObsData[] obs, int n, Navigation nav,
                        int sateph, double[] rs, double[] dts, double[] var,
                        int[] svh) {
        EphemerisCalc.satposs(time, obs, n, nav, sateph, rs, dts, var, svh);
    }

    // ---------------------------------------------------------------
    // Helper methods
    // ---------------------------------------------------------------

    /** Square of x. */
    private static double sq(double x) {
        return x * x;
    }

    /**
     * Test whether a satellite should be excluded.
     */
    private static boolean satexclude(int sat, double var, int svh,
                                      ProcessingOptions opt) {
        return SatelliteUtil.satexclude(sat, var, svh, opt);
    }

    /**
     * Extract a 6-element slice from the satellite position/velocity array
     * for satellite index i.
     */
    private static double[] rsSlice(double[] rs, int i) {
        double[] s = new double[6];
        System.arraycopy(rs, i * 6, s, 0, 6);
        return s;
    }

    /**
     * Shallow-copy solution fields.
     */
    private static void copySolution(Solution src, Solution dst) {
        dst.time = src.time;
        dst.type = src.type;
        dst.stat = src.stat;
        dst.ns = src.ns;
        dst.age = src.age;
        dst.ratio = src.ratio;
        System.arraycopy(src.rr, 0, dst.rr, 0, 6);
        System.arraycopy(src.qr, 0, dst.qr, 0, 6);
        System.arraycopy(src.qv, 0, dst.qv, 0, 6);
        System.arraycopy(src.dtr, 0, dst.dtr, 0, 6);
    }

    /**
     * Shallow-copy processing options.
     */
    private static ProcessingOptions copyOptions(ProcessingOptions src) {
        ProcessingOptions dst = new ProcessingOptions();
        dst.mode = src.mode;
        dst.soltype = src.soltype;
        dst.nf = src.nf;
        dst.navsys = src.navsys;
        dst.elmin = src.elmin;
        dst.snrmask = src.snrmask;
        dst.sateph = src.sateph;
        dst.ionoopt = src.ionoopt;
        dst.tropopt = src.tropopt;
        System.arraycopy(src.eratio, 0, dst.eratio, 0, src.eratio.length);
        System.arraycopy(src.err, 0, dst.err, 0, src.err.length);
        System.arraycopy(src.posopt, 0, dst.posopt, 0, src.posopt.length);
        System.arraycopy(src.exsats, 0, dst.exsats, 0, src.exsats.length);
        return dst;
    }
}
