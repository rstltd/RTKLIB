/*------------------------------------------------------------------------------
 * Rtkpos.java : precise relative positioning (RTK)
 *
 *          Ported from RTKLIB rtkpos.c by T.TAKASU
 *          Java port Copyright (C) 2026
 *
 * Licensed under BSD 2-clause license.
 *-----------------------------------------------------------------------------*/
package com.gnss.rtklib.positioning;

import com.gnss.rtklib.core.*;
import com.gnss.rtklib.correction.Troposphere;
import com.gnss.rtklib.model.*;

import static com.gnss.rtklib.core.Constants.*;

/**
 * RTK (Real-Time Kinematic) relative positioning engine.
 * Ported from RTKLIB rtkpos.c, targeting static baseline mode.
 */
public final class Rtkpos {

    private Rtkpos() {}

    // ---------------------------------------------------------------
    // Constants
    // ---------------------------------------------------------------

    private static final double VAR_POS     = 900.0;   // 30^2
    private static final double VAR_POS_FIX = 1E-8;    // 1e-4^2
    private static final double VAR_VEL     = 100.0;   // 10^2
    private static final double VAR_ACC     = 100.0;   // 10^2
    private static final double VAR_GRA     = 1E-6;    // 0.001^2
    private static final double INIT_ZWD    = 0.15;
    private static final int    GAP_RESION  = 120;
    private static final int    ARMODE_OFF       = 0;
    private static final int    ARMODE_CONT      = 1;
    private static final int    ARMODE_INST      = 2;
    private static final int    ARMODE_FIXHOLD   = 3;
    private static final int    GLO_ARMODE_OFF      = 0;
    private static final int    GLO_ARMODE_ON       = 1;
    private static final int    GLO_ARMODE_AUTOCAL  = 2;
    private static final int    GLO_ARMODE_FIXHOLD  = 3;
    private static final int    NFREQGLO = 2;
    private static final int    NSYS     = 6;
    private static final int    POSOPT_RINEX = 4;

    // FFRT polynomial coefficients (Verhagen & Teunissen 2009) — from C RTKLIB ar_poly_coeffs
    private static final double[][] AR_POLY_COEFFS = {
        {-1.94058448e-01, -7.79023476e+00,  1.24231120e+02, -4.03126050e+02,  3.50413202e+02},
        { 6.42237302e-01, -8.39813962e+00,  2.92107285e+01, -2.37577308e+01, -1.14307128e+00},
        {-2.22600390e-02,  3.23169103e-01, -1.39837429e+00,  2.19282996e+00, -5.34583971e-02}
    };

    // ---------------------------------------------------------------
    // Helper: SQR, SQRT
    // ---------------------------------------------------------------

    private static double SQR(double x) { return x * x; }
    private static double SQRT(double x) { return x <= 0.0 || x != x ? 0.0 : Math.sqrt(x); }

    // ---------------------------------------------------------------
    // test_sys: satellite system group index
    // ---------------------------------------------------------------

    /** Test satellite system (m=0:GPS/SBS/QZS, 1:GLO, 2:GAL, 3:BDS, 4:(unused), 5:IRN).
     *  QZS is merged into GPS group: same L1C/A and L5 signal structure,
     *  zero ISB on same-model receivers, avoids wasting a reference satellite
     *  on 2-3 QZS satellites. */
    static boolean testSys(int sys, int m) {
        switch (sys) {
            case SYS_GPS: case SYS_SBS: case SYS_QZS: case SYS_GAL: return m == 0;
            case SYS_GLO: return m == 1;
            case SYS_CMP: return m == 3;
            case SYS_IRN: return m == 5;
        }
        return false;
    }

    /** Error factor for satellite system. */
    private static double sysFact(int sys) {
        switch (sys) {
            case SYS_GPS: return EFACT_GPS;
            case SYS_GLO: return EFACT_GLO;
            case SYS_GAL: return EFACT_GAL;
            case SYS_SBS: return EFACT_SBS;
            case SYS_QZS: return EFACT_QZS;
            case SYS_CMP: return EFACT_CMP;
            case SYS_IRN: return EFACT_IRN;
            default:      return EFACT_GPS;
        }
    }

    // ---------------------------------------------------------------
    // NF helper
    // ---------------------------------------------------------------

    private static int NF(ProcessingOptions opt) {
        return opt.ionoopt == IONOOPT_IFLC ? 1 : opt.nf;
    }

    // ---------------------------------------------------------------
    // baseline length
    // ---------------------------------------------------------------

    static double baseline(double[] ru, double[] rb, double[] dr) {
        for (int i = 0; i < 3; i++) dr[i] = ru[i] - rb[i];
        return MatrixUtil.norm(dr, 3);
    }

    // ---------------------------------------------------------------
    // selsat: select common satellites
    // ---------------------------------------------------------------

    /**
     * Select common satellites between rover and base station.
     *
     * @param obs  observation data (rover 0..nu-1, base nu..nu+nr-1)
     * @param azel azimuth/elevation array (2*n)
     * @param nu   number of rover observations
     * @param nr   number of base observations
     * @param opt  processing options
     * @param sat  output: common satellite numbers
     * @param iu   output: rover indices
     * @param ir   output: base indices
     * @return number of common satellites
     */
    static int selsat(ObsData[] obs, double[] azel, int nu, int nr,
                      ProcessingOptions opt, int[] sat, int[] iu, int[] ir) {
        int i = 0, j = nu, k = 0;
        while (i < nu && j < nu + nr) {
            if (obs[i].sat < obs[j].sat) { i++; }
            else if (obs[i].sat > obs[j].sat) { j++; }
            else {
                if (azel[1 + j * 2] >= opt.elmin) {
                    sat[k] = obs[i].sat;
                    iu[k] = i;
                    ir[k] = j;
                    k++;
                }
                i++; j++;
            }
        }
        return k;
    }

    // ---------------------------------------------------------------
    // sdobs: single-differenced observable
    // ---------------------------------------------------------------

    private static double sdobs(ObsData[] obs, int i, int j, int k) {
        double pi = (k < NFREQ) ? obs[i].L[k] : obs[i].P[k - NFREQ];
        double pj = (k < NFREQ) ? obs[j].L[k] : obs[j].P[k - NFREQ];
        return (pi == 0.0 || pj == 0.0) ? 0.0 : pi - pj;
    }

    // ---------------------------------------------------------------
    // gfobs: geometry-free SD phase combination
    // ---------------------------------------------------------------

    private static double gfobs(ObsData[] obs, int i, int j, int k, Navigation nav) {
        double freq1 = SignalUtil.sat2freq(obs[i].sat, obs[i].code[0], nav);
        double freq2 = SignalUtil.sat2freq(obs[i].sat, obs[i].code[k], nav);
        double L1 = sdobs(obs, i, j, 0);
        double L2 = sdobs(obs, i, j, k);
        if (freq1 == 0.0 || freq2 == 0.0 || L1 == 0.0 || L2 == 0.0) return 0.0;
        return L1 * CLIGHT / freq1 - L2 * CLIGHT / freq2;
    }

    // ---------------------------------------------------------------
    // varerr: single-differenced measurement error variance
    // ---------------------------------------------------------------

    static double varerr(int sat, int sys, double el, double snrRover,
                                  double snrBase, double bl, double dt, int f,
                                  ProcessingOptions opt) {
        int nf = NF(opt);
        int frq = f % nf;
        int code = f < nf ? 0 : 1;
        double fact;

        if (code != 0) fact = opt.eratio[frq];
        else fact = opt.eratio[frq] / opt.eratio[0];

        fact *= sysFact(sys);

        double c = opt.err[3] * bl / 1E4;
        double d = CLIGHT * opt.sclkstab * dt;
        double var;

        if (opt.snrmodel == 1 && snrRover > 0 && snrBase > 0) {
            // SIGMA-epsilon: C/N0-driven stochastic model
            double sigma0 = fact * opt.err[1];
            double cn0ref = opt.err[5];
            double varRov = sigma0 * sigma0 * Math.pow(10, -0.1 * (snrRover - cn0ref));
            double varBas = sigma0 * sigma0 * Math.pow(10, -0.1 * (snrBase - cn0ref));
            var = varRov + varBas + 2.0 * c * c + d * d;
        } else {
            // Elevation-angle model (default)
            double a = fact * opt.err[1];
            double b = fact * opt.err[2];
            double sinel = Math.sin(el);
            var = 2.0 * (a * a + b * b / sinel / sinel + c * c) + d * d;

            if (opt.err[6] > 0 && snrRover > 0 && snrBase > 0) {
                double snrMax = opt.err[5];
                double e = fact * opt.err[6];
                var += e * e * (Math.pow(10, 0.1 * Math.max(snrMax - snrRover, 0))
                              + Math.pow(10, 0.1 * Math.max(snrMax - snrBase, 0)));
            }
        }
        if (opt.ionoopt == IONOOPT_IFLC) var *= 9.0;
        return var;
    }

    // ---------------------------------------------------------------
    // validobs: test valid observation data
    // ---------------------------------------------------------------

    private static boolean validobs(int i, int j, int f, int nf, double[] y) {
        return y[f + i * nf * 2] != 0.0 && y[f + j * nf * 2] != 0.0;
    }

    // ---------------------------------------------------------------
    // ddcov: double-differenced measurement error covariance
    // ---------------------------------------------------------------

    private static void ddcov(int[] nb, int n, double[] Ri, double[] Rj,
                               int nv, double[] R) {
        for (int i = 0; i < nv * nv; i++) R[i] = 0.0;
        int k = 0;
        for (int b = 0; b < n; k += nb[b++]) {
            for (int i = 0; i < nb[b]; i++) {
                for (int j = 0; j < nb[b]; j++) {
                    R[k + i + (k + j) * nv] = Ri[k + i] + (i == j ? Rj[k + i] : 0.0);
                }
            }
        }
    }

    // ---------------------------------------------------------------
    // Cycle-slip detection
    // ---------------------------------------------------------------

    /** Detect cycle slip by LLI flag. */
    private static void detslpLl(RtkState rtk, ObsData[] obs, int i, int rcv) {
        int sat = obs[i].sat;
        for (int f = 0; f < rtk.opt.nf; f++) {
            if ((obs[i].L[f] == 0.0 && obs[i].LLI[f] == 0) ||
                Math.abs(obs[i].time.timediff(rtk.ssat[sat - 1].pt[rcv - 1][f])) < DTTOL) {
                continue;
            }
            int LLI;
            if (rcv == 1) LLI = rtk.ssat[sat - 1].slip[f] & 0x03;
            else          LLI = (rtk.ssat[sat - 1].slip[f] >> 2) & 0x03;

            int slip;
            if (rtk.tt >= 0.0) {
                slip = obs[i].LLI[f];
            } else {
                slip = LLI;
            }
            // half-cycle transition
            if (((LLI & LLI_HALFC) != 0) != ((obs[i].LLI[f] & LLI_HALFC) != 0)) {
                slip |= LLI_SLIP;
            }
            // save current LLI
            if (rcv == 1) {
                rtk.ssat[sat - 1].slip[f] = (rtk.ssat[sat - 1].slip[f] & ~0x03) | (obs[i].LLI[f] & 0x03);
            } else {
                rtk.ssat[sat - 1].slip[f] = (rtk.ssat[sat - 1].slip[f] & ~0x0C) | ((obs[i].LLI[f] & 0x03) << 2);
            }
            rtk.ssat[sat - 1].slip[f] |= slip;
            rtk.ssat[sat - 1].half[f] = ((obs[i].LLI[f] & LLI_HALFC) != 0) ? 0 : 1;
        }
    }

    /** Detect cycle slip by geometry-free phase jump. */
    private static void detslpGf(RtkState rtk, ObsData[] obs, int i, int j,
                                   Navigation nav) {
        int sat = obs[i].sat;
        if (rtk.opt.thresslip == 0) return;
        for (int k = 0; k < rtk.opt.nf; k++) {
            if ((rtk.ssat[sat - 1].slip[k] & LLI_SLIP) != 0) return;
        }
        double freq1 = SignalUtil.sat2freq(obs[i].sat, obs[i].code[0], nav);
        for (int k = 1; k < rtk.opt.nf; k++) {
            double gf1 = gfobs(obs, i, j, k, nav);
            if (gf1 == 0.0) continue;
            double gf0 = rtk.ssat[sat - 1].gf[k - 1];
            rtk.ssat[sat - 1].gf[k - 1] = gf1;

            // Scale threshold by ionospheric factor ratio: (f1²/fk² - 1) / (f1²/f2² - 1)
            // L1-L2 baseline factor = 0.647, L1-L5 factor = 0.793 → threshold ~1.23x larger
            double freqk = SignalUtil.sat2freq(obs[i].sat, obs[i].code[k], nav);
            double thres = rtk.opt.thresslip;
            if (freq1 > 0.0 && freqk > 0.0) {
                double ionFactor = SQR(freq1 / freqk) - 1.0;
                double ionFactorL2 = SQR(FREQL1 / FREQL2) - 1.0;
                thres *= ionFactor / ionFactorL2;
            }

            if (gf0 != 0.0 && Math.abs(gf1 - gf0) > thres) {
                rtk.ssat[sat - 1].slip[0] |= LLI_SLIP;
                rtk.ssat[sat - 1].slip[k] |= LLI_SLIP;
            }
        }
    }

    /** Detect cycle slip by doppler and phase difference. */
    private static void detslpDop(RtkState rtk, ObsData[] obs, int[] ix, int ns,
                                    int rcv, Navigation nav) {
        if (rtk.opt.thresdop <= 0) return;
        int nf = rtk.opt.nf;
        double[][] dopdif = new double[ns][NFREQ];
        double[][] tt = new double[ns][NFREQ];
        double meanDop = 0;
        int ndop = 0;

        for (int i = 0; i < ns; i++) {
            int ii = ix[i];
            int sat = obs[ii].sat;
            for (int f = 0; f < nf; f++) {
                dopdif[i][f] = 0; tt[i][f] = 0;
                if (obs[ii].L[f] == 0.0 || obs[ii].D[f] == 0.0 ||
                    rtk.ssat[sat - 1].ph[rcv - 1][f] == 0.0) continue;
                tt[i][f] = obs[ii].time.timediff(rtk.ssat[sat - 1].pt[rcv - 1][f]);
                if (Math.abs(tt[i][f]) < DTTOL) continue;
                double dph = (obs[ii].L[f] - rtk.ssat[sat - 1].ph[rcv - 1][f]) / tt[i][f];
                double dpt = -obs[ii].D[f];
                dopdif[i][f] = dph - dpt;
                if (Math.abs(dopdif[i][f]) < 3 * rtk.opt.thresdop) {
                    meanDop += dopdif[i][f];
                    ndop++;
                }
            }
        }
        if (ndop == 0) return;
        meanDop /= ndop;

        for (int i = 0; i < ns; i++) {
            int sat = obs[ix[i]].sat;
            for (int f = 0; f < nf; f++) {
                if (dopdif[i][f] == 0.0) continue;
                if (Math.abs(dopdif[i][f] - meanDop) > rtk.opt.thresdop) {
                    rtk.ssat[sat - 1].slip[f] |= LLI_SLIP;
                }
            }
        }
    }

    /** Detect cycle slip by code change. */
    private static void detslpCode(RtkState rtk, ObsData[] obs, int i, int rcv) {
        int sat = obs[i].sat;
        int nf = Math.min(rtk.opt.nf, NFREQ);
        for (int f = 0; f < nf; f++) {
            int code = obs[i].code[f];
            if (code == CODE_NONE) continue;
            int ccode = rtk.ssat[sat - 1].code[f][rcv - 1];
            if (code != ccode) {
                rtk.ssat[sat - 1].code[f][rcv - 1] = code;
                if (ccode != CODE_NONE) {
                    rtk.ssat[sat - 1].slip[f] |= LLI_SLIP;
                }
            }
        }
    }

    // ---------------------------------------------------------------
    // udpos: temporal update of position
    // ---------------------------------------------------------------

    private static void udpos(RtkState rtk, double tt) {
        if (rtk.opt.mode == PMODE_FIXED) {
            for (int i = 0; i < 3; i++) rtk.initx(rtk.opt.ru[i], VAR_POS_FIX, i);
            return;
        }
        if (MatrixUtil.norm(rtk.x, 3) <= RE_WGS84 / 2) {
            for (int i = 0; i < 3; i++) rtk.initx(rtk.sol.rr[i], VAR_POS, i);
            if (rtk.opt.dynamics != 0) {
                for (int i = 3; i < 6; i++) rtk.initx(rtk.sol.rr[i], VAR_VEL, i);
                for (int i = 6; i < 9; i++) rtk.initx(1E-6, VAR_ACC, i);
            }
        }
        // static mode: no time update after initialization
        if (rtk.opt.mode == PMODE_STATIC || rtk.opt.mode == PMODE_STATIC_START) return;

        // kinematic without dynamics
        if (rtk.opt.dynamics == 0) {
            for (int i = 0; i < 3; i++) rtk.initx(rtk.sol.rr[i], VAR_POS, i);
            return;
        }
        // dynamics mode - state transition (not used for static baseline)
        double var = 0;
        for (int i = 0; i < 3; i++) var += rtk.P[i + i * rtk.nx];
        var /= 3.0;
        if (var > VAR_POS) {
            for (int i = 0; i < 3; i++) rtk.initx(rtk.sol.rr[i], VAR_POS, i);
            for (int i = 3; i < 6; i++) rtk.initx(rtk.sol.rr[i], VAR_VEL, i);
            for (int i = 6; i < 9; i++) rtk.initx(1E-6, VAR_ACC, i);
        }
    }

    // ---------------------------------------------------------------
    // udbias: temporal update of phase biases
    // ---------------------------------------------------------------

    private static void udbias(RtkState rtk, double tt, ObsData[] obs,
                                int[] sat, int[] iu, int[] ir, int ns,
                                Navigation nav) {
        int nf = NF(rtk.opt);

        // clear cycle slips
        for (int i = 0; i < ns; i++) {
            for (int k = 0; k < rtk.opt.nf; k++) {
                rtk.ssat[sat[i] - 1].slip[k] &= 0xFC;
            }
        }

        // detect cycle slip by doppler
        detslpDop(rtk, obs, iu, ns, 1, nav);
        detslpDop(rtk, obs, ir, ns, 2, nav);

        for (int i = 0; i < ns; i++) {
            detslpCode(rtk, obs, iu[i], 1);
            detslpCode(rtk, obs, ir[i], 2);
            detslpLl(rtk, obs, iu[i], 1);
            detslpLl(rtk, obs, ir[i], 2);
            detslpGf(rtk, obs, iu[i], ir[i], nav);

            // update half-cycle valid flag
            for (int k = 0; k < nf; k++) {
                rtk.ssat[sat[i] - 1].half[k] =
                    ((obs[iu[i]].LLI[k] & LLI_HALFC) != 0 ||
                     (obs[ir[i]].LLI[k] & LLI_HALFC) != 0) ? 0 : 1;
            }
        }

        for (int k = 0; k < nf; k++) {
            // reset phase-bias if instantaneous AR or expire obs outage counter
            for (int i = 1; i <= MAXSAT; i++) {
                boolean reset = ++rtk.ssat[i - 1].outc[k] > rtk.opt.maxout;
                int j = RtkState.IB(i, k, rtk.opt);

                if (rtk.opt.modear == ARMODE_INST && rtk.x[j] != 0.0) {
                    rtk.initx(0.0, 0.0, j);
                } else if (reset && rtk.x[j] != 0.0) {
                    // Full reset to match C RTKLIB: clear state and covariance
                    // so the satellite re-initializes cleanly on next appearance.
                    rtk.initx(0.0, 0.0, j);
                    rtk.ssat[i - 1].outc[k] = 0;
                }
                if (rtk.opt.modear == ARMODE_INST && reset) {
                    rtk.ssat[i - 1].lock[k] = -rtk.opt.minlock;
                }
            }

            // update phase bias noise and check for cycle slips
            for (int i = 0; i < ns; i++) {
                int j = RtkState.IB(sat[i], k, rtk.opt);
                rtk.P[j + j * rtk.nx] += rtk.opt.prn[0] * rtk.opt.prn[0] * Math.abs(tt);
                int slip = rtk.ssat[sat[i] - 1].slip[k];
                int rejc = rtk.ssat[sat[i] - 1].rejc[k];
                if (rtk.opt.ionoopt == IONOOPT_IFLC) {
                    int f2 = Spp.seliflc(rtk.opt.nf, rtk.ssat[sat[i] - 1].sys, obs[iu[i]]);
                    slip |= rtk.ssat[sat[i] - 1].slip[f2];
                }

                if (rtk.opt.modear == ARMODE_INST || ((slip & LLI_SLIP) == 0 && rejc < 2)) continue;
                // reset phase-bias state
                rtk.x[j] = 0.0;
                rtk.ssat[sat[i] - 1].rejc[k] = 0;
                rtk.ssat[sat[i] - 1].lock[k] = -rtk.opt.minlock;
                if (rtk.ssat[sat[i] - 1].sys != SYS_GLO) rtk.ssat[sat[i] - 1].icbias[k] = 0;
            }

            // estimate approximate phase-bias by delta phase - delta code
            double[] bias = new double[ns];
            double offset = 0.0;
            int cnt = 0;
            for (int i = 0; i < ns; i++) {
                if (rtk.opt.ionoopt != IONOOPT_IFLC) {
                    double cp = sdobs(obs, iu[i], ir[i], k);
                    double pr = sdobs(obs, iu[i], ir[i], k + NFREQ);
                    double freqi = SignalUtil.sat2freq(sat[i], obs[iu[i]].code[k], nav);
                    if (cp == 0.0 || pr == 0.0 || freqi == 0.0) continue;
                    bias[i] = cp - pr * freqi / CLIGHT;
                } else {
                    // IFLC: use iono-free combination of two frequencies
                    int f2 = Spp.seliflc(rtk.opt.nf, rtk.ssat[sat[i] - 1].sys, obs[iu[i]]);
                    double cp1 = sdobs(obs, iu[i], ir[i], 0);
                    double cp2 = sdobs(obs, iu[i], ir[i], f2);
                    double pr1 = sdobs(obs, iu[i], ir[i], NFREQ);
                    double pr2 = sdobs(obs, iu[i], ir[i], NFREQ + f2);
                    double freq1 = SignalUtil.sat2freq(sat[i], obs[iu[i]].code[0], nav);
                    double freq2 = SignalUtil.sat2freq(sat[i], obs[iu[i]].code[f2], nav);
                    if (cp1 == 0.0 || cp2 == 0.0 || pr1 == 0.0 || pr2 == 0.0
                        || freq1 <= 0.0 || freq2 <= 0.0) continue;
                    double C1 =  SQR(freq1) / (SQR(freq1) - SQR(freq2));
                    double C2 = -SQR(freq2) / (SQR(freq1) - SQR(freq2));
                    bias[i] = (C1 * cp1 * CLIGHT / freq1 + C2 * cp2 * CLIGHT / freq2)
                            - (C1 * pr1 + C2 * pr2);
                }

                if (rtk.x[RtkState.IB(sat[i], k, rtk.opt)] != 0.0) {
                    offset += bias[i] - rtk.x[RtkState.IB(sat[i], k, rtk.opt)];
                    cnt++;
                }
            }
            // correct phase-bias offset
            if (cnt > 0) {
                for (int i = 1; i <= MAXSAT; i++) {
                    if (rtk.x[RtkState.IB(i, k, rtk.opt)] != 0.0) {
                        rtk.x[RtkState.IB(i, k, rtk.opt)] += offset / cnt;
                    }
                }
            }
            // set initial states of phase-bias
            for (int i = 0; i < ns; i++) {
                if (bias[i] == 0.0 || rtk.x[RtkState.IB(sat[i], k, rtk.opt)] != 0.0) continue;
                rtk.initx(bias[i], SQR(rtk.opt.std[0]), RtkState.IB(sat[i], k, rtk.opt));
                if (rtk.opt.modear != ARMODE_INST) {
                    rtk.ssat[sat[i] - 1].lock[k] = -rtk.opt.minlock;
                }
            }
        }
    }

    // ---------------------------------------------------------------
    // udstate: temporal update of states
    // ---------------------------------------------------------------

    private static void udstate(RtkState rtk, ObsData[] obs, int[] sat,
                                 int[] iu, int[] ir, int ns, Navigation nav) {
        double tt = rtk.tt;
        udpos(rtk, tt);

        // ionosphere and troposphere estimation (skip for short baseline default)
        if (rtk.opt.ionoopt == IONOOPT_EST || rtk.opt.tropopt >= TROPOPT_EST) {
            double[] dr = new double[3];
            double bl = baseline(rtk.x, rtk.rb, dr);
            if (rtk.opt.ionoopt == IONOOPT_EST) {
                udion(rtk, tt, bl, sat, ns);
            }
            if (rtk.opt.tropopt >= TROPOPT_EST) {
                udtrop(rtk, tt);
            }
        }
        // receiver h/w bias update (GLONASS AUTOCAL)
        if (rtk.opt.glomodear == GLO_ARMODE_AUTOCAL && (rtk.opt.navsys & SYS_GLO) != 0) {
            udrcvbias(rtk, tt);
        }
        // phase bias update
        if (rtk.opt.mode > PMODE_DGPS) {
            udbias(rtk, tt, obs, sat, iu, ir, ns, nav);
        }
    }

    // ---------------------------------------------------------------
    // udion: temporal update of ionospheric parameters
    // ---------------------------------------------------------------

    private static void udion(RtkState rtk, double tt, double bl, int[] sat, int ns) {
        for (int i = 1; i <= MAXSAT; i++) {
            int j = RtkState.II(i, rtk.opt);
            if (rtk.x[j] != 0.0 && rtk.ssat[i - 1].outc[0] > GAP_RESION
                && rtk.ssat[i - 1].outc[1] > GAP_RESION) {
                rtk.x[j] = 0.0;
            }
        }
        for (int i = 0; i < ns; i++) {
            int j = RtkState.II(sat[i], rtk.opt);
            if (rtk.x[j] == 0.0) {
                rtk.initx(1E-6, SQR(rtk.opt.std[1] * bl / 1E4), j);
            } else {
                double el = rtk.ssat[sat[i] - 1].azel[1];
                double fact = Math.cos(el);
                rtk.P[j + j * rtk.nx] += SQR(rtk.opt.prn[1] * bl / 1E4 * fact) * Math.abs(tt);
            }
        }
    }

    // ---------------------------------------------------------------
    // udtrop: temporal update of tropospheric parameters
    // ---------------------------------------------------------------

    private static void udtrop(RtkState rtk, double tt) {
        for (int i = 0; i < 2; i++) {
            int j = RtkState.IT(i, rtk.opt);
            if (rtk.x[j] == 0.0) {
                rtk.initx(INIT_ZWD, SQR(rtk.opt.std[2]), j);
                if (rtk.opt.tropopt >= TROPOPT_ESTG) {
                    for (int k = 0; k < 2; k++) rtk.initx(1E-6, VAR_GRA, ++j);
                }
            } else {
                rtk.P[j + j * rtk.nx] += SQR(rtk.opt.prn[2]) * Math.abs(tt);
                if (rtk.opt.tropopt >= TROPOPT_ESTG) {
                    for (int k = 0; k < 2; k++) {
                        ++j;
                        rtk.P[j + j * rtk.nx] += SQR(rtk.opt.prn[2] * 0.3) * Math.abs(tt);
                    }
                }
            }
        }
    }

    // ---------------------------------------------------------------
    // udrcvbias: temporal update of receiver h/w bias (GLONASS AUTOCAL)
    // ---------------------------------------------------------------

    private static void udrcvbias(RtkState rtk, double tt) {
        for (int i = 0; i < NFREQGLO; i++) {
            int j = RtkState.IL(i, rtk.opt);
            if (rtk.x[j] == 0.0) {
                // initialize with small offset to avoid zero
                rtk.initx(rtk.opt.thresar[2] + 1e-6, rtk.opt.thresar[3], j);
            } else if (rtk.nfix >= rtk.opt.minfix) {
                // hold to fixed solution
                rtk.initx(rtk.xa[j], rtk.Pa[j + j * rtk.na], j);
            } else {
                // process noise
                rtk.P[j + j * rtk.nx] += SQR(rtk.opt.thresar[4]) * Math.abs(tt);
            }
        }
    }

    // ---------------------------------------------------------------
    // zdres_sat: undifferenced residual for one satellite
    // ---------------------------------------------------------------

    private static void zdresSat(int base, double r, ObsData obs, Navigation nav,
                                   double[] azel, ProcessingOptions opt,
                                   double[] y, int yOff, double[] freq, int fOff) {
        int nf = NF(opt);

        if (opt.ionoopt == IONOOPT_IFLC) {
            // Iono-free linear combination (ported from rtkpos.c:969-1005)
            double freq1 = SignalUtil.sat2freq(obs.sat, obs.code[0], nav);
            int f2 = Spp.seliflc(opt.nf, SatelliteUtil.satsys(obs.sat)[0], obs);
            double freq2 = SignalUtil.sat2freq(obs.sat, obs.code[f2], nav);

            if (freq1 == 0.0 || freq2 == 0.0) return;

            // SNR mask check
            if (Spp.testsnr(base, 0, azel[1], obs.SNR[0], opt.snrmask)) return;
            if (Spp.testsnr(base, f2, azel[1], obs.SNR[f2], opt.snrmask)) return;

            double C1 =  SQR(freq1) / (SQR(freq1) - SQR(freq2));
            double C2 = -SQR(freq2) / (SQR(freq1) - SQR(freq2));

            if (obs.L[0] != 0.0 && obs.L[f2] != 0.0) {
                y[yOff] = C1 * obs.L[0] * CLIGHT / freq1
                        + C2 * obs.L[f2] * CLIGHT / freq2 - r;
            }
            if (obs.P[0] != 0.0 && obs.P[f2] != 0.0) {
                y[yOff + nf] = C1 * obs.P[0] + C2 * obs.P[f2] - r;
            }
            freq[fOff] = 1.0;
        } else {
            for (int i = 0; i < nf; i++) {
                freq[fOff + i] = SignalUtil.sat2freq(obs.sat, obs.code[i], nav);
                if (freq[fOff + i] == 0.0) continue;

                // SNR mask check (matching C rtkpos.c:996)
                if (Spp.testsnr(base, i, azel[1], obs.SNR[i], opt.snrmask)) {
                    continue;
                }
                if (obs.L[i] != 0.0) y[yOff + i]      = obs.L[i] * CLIGHT / freq[fOff + i] - r;
                if (obs.P[i] != 0.0) y[yOff + i + nf]  = obs.P[i] - r;
            }
        }
    }

    // ---------------------------------------------------------------
    // zdres: undifferenced phase/code residuals
    // ---------------------------------------------------------------

    /**
     * Compute zero-difference residuals for all satellites.
     *
     * @param base 0=rover, 1=base
     * @param obs  observations
     * @param n    number of observations
     * @param rs   satellite positions (6*n)
     * @param dts  satellite clocks (2*n)
     * @param var  satellite variances (n)
     * @param svh  satellite health (n)
     * @param nav  navigation data
     * @param rr   receiver position ECEF (3)
     * @param opt  processing options
     * @param y    output: residuals (nf*2*n)
     * @param e    output: LOS unit vectors (3*n)
     * @param azel output: azimuth/elevation (2*n)
     * @param freq output: frequencies (nf*n)
     * @return 1 on success, 0 on failure
     */
    static int zdres(int base, ObsData[] obs, int n, double[] rs,
                     double[] dts, double[] var, int[] svh, Navigation nav,
                     double[] rr, ProcessingOptions opt, double[] y,
                     double[] e, double[] azel, double[] freq) {
        int nf = NF(opt);

        // init residuals
        for (int i = 0; i < n * nf * 2; i++) y[i] = 0.0;

        if (MatrixUtil.norm(rr, 3) <= RE_WGS84 / 2) return 0;

        double[] rr_ = new double[3];
        System.arraycopy(rr, 0, rr_, 0, 3);

        double[] pos = Coord.ecef2pos(rr_);

        // pre-compute zenith hydrostatic delay (same for all satellites in epoch)
        double[] zazel = {0.0, PI / 2.0};
        double zhd = Troposphere.tropmodel(obs[0].time, pos, zazel, 0.0);

        // pre-allocate per-satellite work arrays
        double[] ei = new double[3];
        double[] azelI = new double[2];
        double[] mapfw = new double[1];

        for (int i = 0; i < n; i++) {
            double r = Geometry.geodist(new double[]{rs[i * 6], rs[i * 6 + 1], rs[i * 6 + 2]},
                                         rr_, ei);
            if (r <= 0.0) continue;

            e[i * 3] = ei[0]; e[i * 3 + 1] = ei[1]; e[i * 3 + 2] = ei[2];

            if (Geometry.satazel(pos, ei, azelI) < opt.elmin) continue;
            azel[i * 2] = azelI[0]; azel[i * 2 + 1] = azelI[1];

            if (SatelliteUtil.satexclude(obs[i].sat, var[i], svh[i], opt)) continue;

            // satellite clock correction
            r += -CLIGHT * dts[i * 2];

            // troposphere correction (Saastamoinen hydrostatic)
            double mapfh = Troposphere.tropmapf(obs[i].time, pos, azelI, mapfw);
            r += mapfh * zhd;

            // compute undifferenced residuals
            zdresSat(base, r, obs[i], nav, azelI, opt, y, i * nf * 2, freq, i * nf);
        }
        return 1;
    }

    // ---------------------------------------------------------------
    // ddres: double-differenced residuals and partial derivatives
    // ---------------------------------------------------------------

    static int ddres(RtkState rtk, ObsData[] obs, double dt,
                      double[] x, double[] P, int[] sat,
                      double[] y, double[] e, double[] azel,
                      double[] freq, int[] iu, int[] ir, int ns,
                      double[] v, double[] H, double[] R, int[] vflg) {
        ProcessingOptions opt = rtk.opt;
        int nf = NF(opt);
        int nv = 0;
        int[] nb = new int[NFREQ * NSYS * 2 + 2];
        int b = 0;

        double[] dr = new double[3];
        double bl = baseline(x, rtk.rb, dr);

        double[] Ri = new double[ns * nf * 2 + 2];
        double[] Rj = new double[ns * nf * 2 + 2];

        // zero out residuals
        for (int i = 0; i < MAXSAT; i++) {
            for (int j = 0; j < NFREQ; j++) {
                rtk.ssat[i].resp[j] = 0.0;
                rtk.ssat[i].resc[j] = 0.0;
            }
        }

        // troposphere estimation factors and mapping functions (if enabled)
        double[] tropu = new double[ns], tropr = new double[ns];
        double[] mfU = new double[ns], mfR = new double[ns]; // cached mapping functions
        if (opt.tropopt >= TROPOPT_EST) {
            double[] posu = Coord.ecef2pos(x);
            double[] posr = Coord.ecef2pos(rtk.rb);
            double[] mapfw = new double[1];
            for (int i = 0; i < ns; i++) {
                double[] azelU = {azel[iu[i] * 2], azel[iu[i] * 2 + 1]};
                double[] azelR = {azel[ir[i] * 2], azel[ir[i] * 2 + 1]};
                mfU[i] = Troposphere.tropmapf(rtk.sol.time, posu, azelU, mapfw);
                tropu[i] = mfU[i] * x[RtkState.IT(0, opt)];
                mfR[i] = Troposphere.tropmapf(rtk.sol.time, posr, azelR, mapfw);
                tropr[i] = mfR[i] * x[RtkState.IT(1, opt)];
            }
        }

        // step through satellite systems
        for (int m = 0; m < 6; m++) {
            // step through phases/codes
            for (int f = opt.mode > PMODE_DGPS ? 0 : nf; f < nf * 2; f++) {
                int frq = f % nf;
                int code = f < nf ? 0 : 1;

                // find reference satellite with highest elevation
                int refIdx = -1;
                for (int j = 0; j < ns; j++) {
                    int sysi = rtk.ssat[sat[j] - 1].sys;
                    if (!testSys(sysi, m) || sysi == SYS_SBS) continue;
                    if (!validobs(iu[j], ir[j], f, nf, y)) continue;
                    if (refIdx >= 0 && (rtk.ssat[sat[j] - 1].slip[frq] & LLI_SLIP) != 0) continue;
                    if (refIdx < 0 || azel[1 + iu[j] * 2] >= azel[1 + iu[refIdx] * 2]) refIdx = j;
                }
                if (refIdx < 0) { b++; continue; }

                // calculate DD for each satellite
                for (int j = 0; j < ns; j++) {
                    if (refIdx == j) continue;
                    int sysi = rtk.ssat[sat[refIdx] - 1].sys;
                    int sysj = rtk.ssat[sat[j] - 1].sys;
                    double freqi = freq[frq + iu[refIdx] * nf];
                    double freqj = freq[frq + iu[j] * nf];
                    if (freqi <= 0.0 || freqj <= 0.0) continue;
                    if (!testSys(sysj, m)) continue;
                    if (!validobs(iu[j], ir[j], f, nf, y)) continue;

                    double[] Hi = null;
                    if (H != null) {
                        Hi = rtk.wsHi;
                        java.util.Arrays.fill(Hi, 0, rtk.nx, 0.0);
                    }

                    // DD measurement
                    v[nv] = (y[f + iu[refIdx] * nf * 2] - y[f + ir[refIdx] * nf * 2])
                          - (y[f + iu[j] * nf * 2] - y[f + ir[j] * nf * 2]);

                    // partial derivatives by rover position
                    if (Hi != null) {
                        for (int k = 0; k < 3; k++) {
                            Hi[k] = -e[k + iu[refIdx] * 3] + e[k + iu[j] * 3];
                        }
                    }

                    // ionosphere estimation
                    if (opt.ionoopt == IONOOPT_EST) {
                        double didxi = (code != 0 ? -1.0 : 1.0) * SQR(FREQL1 / freqi);
                        double didxj = (code != 0 ? -1.0 : 1.0) * SQR(FREQL1 / freqj);
                        v[nv] -= didxi * x[RtkState.II(sat[refIdx], opt)]
                                - didxj * x[RtkState.II(sat[j], opt)];
                        if (Hi != null) {
                            Hi[RtkState.II(sat[refIdx], opt)] = didxi;
                            Hi[RtkState.II(sat[j], opt)] = -didxj;
                        }
                    }

                    // troposphere estimation (uses pre-computed mapping functions)
                    if (opt.tropopt >= TROPOPT_EST) {
                        v[nv] -= (tropu[refIdx] - tropu[j]) - (tropr[refIdx] - tropr[j]);
                        if (Hi != null) {
                            Hi[RtkState.IT(0, opt)] = mfU[refIdx] - mfU[j];
                            Hi[RtkState.IT(1, opt)] = -(mfR[refIdx] - mfR[j]);
                        }
                    }

                    // phase bias
                    if (opt.mode > PMODE_DGPS && code == 0) {
                        int ii = RtkState.IB(sat[refIdx], frq, opt);
                        int jj = RtkState.IB(sat[j], frq, opt);
                        v[nv] -= CLIGHT / freqi * x[ii] - CLIGHT / freqj * x[jj];
                        if (Hi != null) {
                            Hi[ii] = CLIGHT / freqi;
                            Hi[jj] = -CLIGHT / freqj;
                        }
                    }

                    // GLONASS inter-channel bias
                    if (sysi == SYS_GLO && sysj == SYS_GLO) {
                        if (rtk.opt.glomodear == GLO_ARMODE_AUTOCAL && frq < NFREQGLO) {
                            // auto-cal: estimate h/w bias (m/MHz) in EKF state
                            double df = (freqi - freqj) / (frq == 0 ? DFRQ1_GLO : DFRQ2_GLO);
                            v[nv] -= df * x[RtkState.IL(frq, opt)];
                            if (Hi != null) Hi[RtkState.IL(frq, opt)] = df;
                        } else if (rtk.opt.glomodear == GLO_ARMODE_FIXHOLD && frq < NFREQGLO) {
                            // fix-and-hold: apply accumulated ICB correction
                            double icb = rtk.ssat[sat[refIdx] - 1].icbias[frq] * CLIGHT / freqi
                                       - rtk.ssat[sat[j] - 1].icbias[frq] * CLIGHT / freqj;
                            v[nv] -= icb;
                        }
                    }

                    // save residuals
                    if (code != 0) rtk.ssat[sat[j] - 1].resp[frq] = v[nv];
                    else           rtk.ssat[sat[j] - 1].resc[frq] = v[nv];

                    // open up outlier threshold for newly initialized biases
                    double threshadj = 1;
                    if (opt.mode > PMODE_DGPS && P != null) {
                        int ii = RtkState.IB(sat[refIdx], frq, opt);
                        int jj = RtkState.IB(sat[j], frq, opt);
                        if (P[ii + rtk.nx * ii] == SQR(rtk.opt.std[0]) ||
                            P[jj + rtk.nx * jj] == SQR(rtk.opt.std[0]))
                            threshadj = 10;
                    }

                    // outlier rejection
                    if (Math.abs(v[nv]) > opt.maxinno[code] * threshadj) {
                        rtk.ssat[sat[j] - 1].vsat[frq] = 0;
                        rtk.ssat[sat[j] - 1].rejc[frq]++;
                        continue;
                    }

                    // SD measurement error variances
                    Ri[nv] = varerr(sat[refIdx], sysi, azel[1 + iu[refIdx] * 2],
                                     rtk.ssat[sat[refIdx] - 1].snr_rover[frq],
                                     rtk.ssat[sat[refIdx] - 1].snr_base[frq],
                                     bl, dt, f, opt);
                    Rj[nv] = varerr(sat[j], sysj, azel[1 + iu[j] * 2],
                                     rtk.ssat[sat[j] - 1].snr_rover[frq],
                                     rtk.ssat[sat[j] - 1].snr_base[frq],
                                     bl, dt, f, opt);

                    // increase variance if half cycle flags set
                    if (code == 0 && (obs[iu[refIdx]].LLI[frq] & LLI_HALFC) != 0) Ri[nv] += 0.01;
                    if (code == 0 && (obs[iu[j]].LLI[frq] & LLI_HALFC) != 0)      Rj[nv] += 0.01;

                    // set valid data flags
                    if (opt.mode > PMODE_DGPS) {
                        if (code == 0) {
                            rtk.ssat[sat[refIdx] - 1].vsat[frq] = 1;
                            rtk.ssat[sat[j] - 1].vsat[frq] = 1;
                        }
                    } else {
                        rtk.ssat[sat[refIdx] - 1].vsat[frq] = 1;
                        rtk.ssat[sat[j] - 1].vsat[frq] = 1;
                    }

                    // copy Hi row into H matrix
                    if (H != null && Hi != null) {
                        System.arraycopy(Hi, 0, H, nv * rtk.nx, rtk.nx);
                    }

                    vflg[nv] = (sat[refIdx] << 16) | (sat[j] << 8) | ((code != 0 ? 1 : 0) << 4) | frq;
                    nv++;
                    nb[b]++;
                }
                b++;
            }
        }

        // DD measurement error covariance
        ddcov(nb, b, Ri, Rj, nv, R);

        return nv;
    }

    // ---------------------------------------------------------------
    // ddidx: index for SD to DD transformation
    // ---------------------------------------------------------------

    private static int ddidx(RtkState rtk, int[] ix, int gps, int glo, int sbs) {
        int nb = 0, na = rtk.na, nf = NF(rtk.opt);

        // clear fix flags
        for (int i = 0; i < MAXSAT; i++) {
            for (int j = 0; j < NFREQ; j++) rtk.ssat[i].fix[j] = 0;
        }

        for (int m = 0; m < 6; m++) {
            boolean nofix = (m == 0 && gps == 0) || (m == 1 && glo == 0) ||
                            (m == 3 && rtk.opt.bdsmodear == 0);

            for (int f = 0, k = na; f < nf; f++, k += MAXSAT) {
                // find reference satellite (first valid by state index for stability)
                int refI = -1;
                for (int i = k; i < k + MAXSAT; i++) {
                    if (rtk.x[i] == 0.0 || !testSys(rtk.ssat[i - k].sys, m) ||
                        rtk.ssat[i - k].vsat[f] == 0) continue;
                    if (rtk.ssat[i - k].lock[f] >= 0 &&
                        (rtk.ssat[i - k].slip[f] & LLI_HALFC) == 0 &&
                        rtk.ssat[i - k].azel[1] >= rtk.opt.elmaskar && !nofix) {
                        rtk.ssat[i - k].fix[f] = 2;
                        refI = i;
                        break;
                    } else {
                        rtk.ssat[i - k].fix[f] = 1;
                    }
                }
                if (refI < 0 || rtk.ssat[refI - k].fix[f] != 2) continue;

                int n = 0;
                for (int j = k; j < k + MAXSAT; j++) {
                    if (refI == j || rtk.x[j] == 0.0 || !testSys(rtk.ssat[j - k].sys, m) ||
                        rtk.ssat[j - k].vsat[f] == 0) continue;
                    if (sbs == 0 && SatelliteUtil.satsys(j - k + 1)[0] == SYS_SBS) continue;
                    if (rtk.ssat[j - k].lock[f] >= 0 &&
                        (rtk.ssat[j - k].slip[f] & LLI_HALFC) == 0 &&
                        rtk.ssat[j - k].vsat[f] != 0 &&
                        rtk.ssat[j - k].azel[1] >= rtk.opt.elmaskar && !nofix) {
                        ix[nb * 2] = refI;
                        ix[nb * 2 + 1] = j;
                        rtk.ssat[j - k].fix[f] = 2;
                        nb++;
                        n++;
                    } else {
                        rtk.ssat[j - k].fix[f] = 1;
                    }
                }
                if (n == 0) rtk.ssat[refI - k].fix[f] = 1;
            }
        }
        return nb;
    }

    // ---------------------------------------------------------------
    // restamb: translate DD fixed phase-bias to SD
    // ---------------------------------------------------------------

    private static void restamb(RtkState rtk, double[] bias, int nb, double[] xa) {
        int nf = NF(rtk.opt);
        System.arraycopy(rtk.x, 0, xa, 0, rtk.nx);
        System.arraycopy(rtk.xa, 0, xa, 0, rtk.na);

        int nv = 0;
        for (int m = 0; m < 6; m++) {
            for (int f = 0; f < nf; f++) {
                int[] index = new int[MAXSAT];
                int n = 0;
                for (int i = 0; i < MAXSAT; i++) {
                    if (!testSys(rtk.ssat[i].sys, m) || rtk.ssat[i].fix[f] != 2) continue;
                    index[n++] = RtkState.IB(i + 1, f, rtk.opt);
                }
                if (n < 2) continue;
                xa[index[0]] = rtk.x[index[0]];
                for (int i = 1; i < n; i++) {
                    xa[index[i]] = xa[index[0]] - bias[nv++];
                }
            }
        }
    }

    // ---------------------------------------------------------------
    // holdamb: hold integer ambiguity
    // ---------------------------------------------------------------

    private static void holdamb(RtkState rtk, double[] xa) {
        int nf = NF(rtk.opt);
        int nbMax = rtk.nx - rtk.na;
        double[] v = new double[nbMax];
        double[] H = new double[nbMax * rtk.nx];
        int nv = 0;

        for (int m = 0; m < 6; m++) {
            for (int f = 0; f < nf; f++) {
                int[] index = new int[MAXSAT];
                int n = 0;
                for (int i = 0; i < MAXSAT; i++) {
                    if (!testSys(rtk.ssat[i].sys, m) || rtk.ssat[i].fix[f] != 2 ||
                        rtk.ssat[i].azel[1] < rtk.opt.elmaskhold) continue;
                    index[n++] = RtkState.IB(i + 1, f, rtk.opt);
                    rtk.ssat[i].fix[f] = 3; // hold
                }
                for (int i = 1; i < n; i++) {
                    v[nv] = (xa[index[0]] - xa[index[i]])
                           - (rtk.x[index[0]] - rtk.x[index[i]]);
                    H[index[0] + nv * rtk.nx] = 1.0;
                    H[index[i] + nv * rtk.nx] = -1.0;
                    nv++;
                }
            }
        }

        if (rtk.opt.modear == ARMODE_FIXHOLD && nv < rtk.opt.minholdsats) return;

        rtk.holdamb = 1;
        double[] Rv = new double[nv * nv];
        for (int i = 0; i < nv; i++) Rv[i + i * nv] = rtk.opt.varholdamb;

        // trim H to nv rows
        double[] Ht = new double[rtk.nx * nv];
        System.arraycopy(H, 0, Ht, 0, rtk.nx * nv);

        MatrixUtil.filter(rtk.x, rtk.P, Ht, v, Rv, rtk.nx, nv, rtk.filterWs);

        // GLONASS ICB update (fix-and-hold)
        if (rtk.opt.glomodear != GLO_ARMODE_FIXHOLD) return;
        for (int f = 0; f < nf; f++) {
            int refI = -1;
            for (int j = 0; j < MAXSAT; j++) {
                if (testSys(rtk.ssat[j].sys, 1) && rtk.ssat[j].vsat[f] != 0 &&
                    rtk.ssat[j].lock[f] >= 0) {
                    if (refI < 0) {
                        refI = j;
                    } else {
                        double dd = rtk.x[RtkState.IB(j + 1, f, rtk.opt)]
                                  - rtk.x[RtkState.IB(refI + 1, f, rtk.opt)];
                        dd = rtk.opt.gainholdamb * (dd - Math.floor(dd + 0.5));
                        rtk.x[RtkState.IB(j + 1, f, rtk.opt)] -= dd;
                        rtk.ssat[j].icbias[f] += dd;
                    }
                }
            }
        }
    }

    // ---------------------------------------------------------------
    // FFRT adaptive AR threshold: lower threshold when more DDs available
    // ---------------------------------------------------------------

    static double computeAdaptiveArThreshold(int nb, double nominalThreshold) {
        int nb1 = Math.min(nb, 50);
        double[] coeff = new double[3];
        for (int i = 0; i < 3; i++) {
            coeff[i] = AR_POLY_COEFFS[i][0];
            for (int j = 1; j < 5; j++)
                coeff[i] = coeff[i] * nominalThreshold + AR_POLY_COEFFS[i][j];
        }
        double thres = coeff[0];
        for (int i = 1; i < 3; i++)
            thres = thres / (nb1 + 1.0) + coeff[i];
        return Math.max(1.0, Math.min(thres, nominalThreshold));
    }

    // ---------------------------------------------------------------
    // findNextDropSat: round-robin satellite dropout (matches C RTKLIB)
    // ---------------------------------------------------------------

    /**
     * Find the next AR-eligible satellite to exclude using round-robin.
     * Starts scanning from the position after the last excluded satellite
     * ({@code rtk.excsat}), wrapping to the beginning if needed.
     * Matches C RTKLIB's manage_amb_LAMBDA dropout logic.
     *
     * @return satellite number to exclude, or 0 if none found
     */
    private static int findNextDropSat(RtkState rtk, int[] sat, int ns, int nf) {
        // Find position of last excluded sat
        int start = 0;
        if (rtk.excsat != 0) {
            for (int i = 0; i < ns; i++) {
                if (rtk.excsat == sat[i]) {
                    start = i + 1;
                    break;
                }
            }
            // If not found, restart from beginning
            if (start >= ns) start = 0;
        }
        // Find next AR-eligible sat from start position
        for (int i = start; i < ns; i++) {
            for (int f = 0; f < nf; f++) {
                if (rtk.ssat[sat[i] - 1].vsat[f] != 0 &&
                    rtk.ssat[sat[i] - 1].lock[f] >= 0 &&
                    rtk.ssat[sat[i] - 1].azel[1] >= rtk.opt.elmin) {
                    return sat[i];
                }
            }
        }
        return 0;
    }

    // ---------------------------------------------------------------
    // resamb_LAMBDA: resolve integer ambiguity
    // ---------------------------------------------------------------

    private static int resambLAMBDA(RtkState rtk, double[] bias, double[] xa,
                                      int gps, int glo, int sbs) {
        int nx = rtk.nx, na = rtk.na;
        rtk.sol.ratio = 0.0f;
        rtk.nb_ar = 0;

        int[] ix = new int[nx * 2];
        int nb = ddidx(rtk, ix, gps, glo, sbs);
        if (nb < rtk.opt.minfixsats - 1) return -1;
        rtk.nb_ar = nb;

        double[] y = new double[nb];
        double[] DP = new double[nb * (nx - na)];
        double[] b = new double[nb * 2];
        double[] db = new double[nb];
        double[] Qb = new double[nb * nb];
        double[] Qab = new double[na * nb];
        double[] QQ = new double[na * nb];
        double[] s = new double[2];

        // y=D*xc, Qb=D*Qc*D', Qab=Qac*D'
        for (int i = 0; i < nb; i++) {
            y[i] = rtk.x[ix[i * 2]] - rtk.x[ix[i * 2 + 1]];
        }
        for (int j = 0; j < nx - na; j++) {
            for (int i = 0; i < nb; i++) {
                DP[i + j * nb] = rtk.P[ix[i * 2] + (na + j) * nx]
                               - rtk.P[ix[i * 2 + 1] + (na + j) * nx];
            }
        }
        for (int j = 0; j < nb; j++) {
            for (int i = 0; i < nb; i++) {
                Qb[i + j * nb] = DP[i + (ix[j * 2] - na) * nb]
                               - DP[i + (ix[j * 2 + 1] - na) * nb];
            }
        }
        for (int j = 0; j < nb; j++) {
            for (int i = 0; i < na; i++) {
                Qab[i + j * na] = rtk.P[i + ix[j * 2] * nx]
                                - rtk.P[i + ix[j * 2 + 1] * nx];
            }
        }

        // LAMBDA
        int info = Lambda.lambda(nb, 2, y, Qb, b, s);
        if (info != 0) return 0;

        rtk.sol.ratio = s[0] > 0 ? (float)(s[1] / s[0]) : 0.0f;
        if (rtk.sol.ratio > 999.9f) rtk.sol.ratio = 999.9f;

        rtk.sol.thres = (float) computeAdaptiveArThreshold(nb, rtk.opt.thresar[0]);

        // ratio test
        if (s[0] <= 0.0 || s[1] / s[0] >= rtk.sol.thres) {
            // init xa/Pa with float solution
            for (int i = 0; i < na; i++) {
                rtk.xa[i] = rtk.x[i];
                for (int j = 0; j < na; j++) rtk.Pa[i + j * na] = rtk.P[i + j * nx];
            }
            for (int i = 0; i < nb; i++) {
                bias[i] = b[i];
                y[i] = (rtk.x[ix[i * 2]] - rtk.x[ix[i * 2 + 1]]) - b[i];
            }
            // conditional covariance update: xa = x - Qab*Qb^-1*(b0-b)
            if (MatrixUtil.matinv(Qb, nb) == 0) {
                MatrixUtil.matmul("NN", nb, 1, nb, Qb, y, db);
                // xa = x - Qab*db
                System.arraycopy(rtk.x, 0, rtk.xa, 0, na);
                for (int i = 0; i < na; i++) rtk.xa[i] = rtk.x[i];
                MatrixUtil.matmul("NN", na, 1, nb, -1.0, Qab, db, 1.0, rtk.xa);
                // Pa = P - Qab*Qb^-1*Qab'
                MatrixUtil.matmul("NN", na, nb, nb, Qab, Qb, QQ);
                for (int i = 0; i < na; i++) {
                    for (int j = 0; j < na; j++) rtk.Pa[i + j * na] = rtk.P[i + j * nx];
                }
                MatrixUtil.matmul("NT", na, na, nb, -1.0, QQ, Qab, 1.0, rtk.Pa);

                restamb(rtk, bias, nb, xa);
            } else {
                nb = 0;
            }
        } else {
            nb = 0;
        }
        return nb;
    }

    // ---------------------------------------------------------------
    // manage_amb_LAMBDA: AR with partial fix and dropout
    // ---------------------------------------------------------------

    private static int manageAmbLAMBDA(RtkState rtk, double[] bias, double[] xa,
                                         int[] sat, int nf, int ns) {
        // position variance check
        double posvar = 0;
        for (int i = 0; i < 3; i++) posvar += rtk.P[i + i * rtk.nx];
        posvar /= 3.0;

        if (rtk.opt.mode <= PMODE_DGPS || rtk.opt.modear == ARMODE_OFF ||
            rtk.opt.thresar[0] < 1.0 || posvar > rtk.opt.thresar[1]) {
            rtk.sol.ratio = 0.0f;
            rtk.sol.prevRatio1 = rtk.sol.prevRatio2 = 0.0f;
            rtk.nb_ar = 0;
            return 0;
        }

        // satellite dropout: round-robin exclude next AR-eligible satellite
        int excsat = 0;
        int[] lockc = new int[NFREQ];
        if (rtk.sol.prevRatio2 < rtk.sol.thres && rtk.nb_ar >= rtk.opt.mindropsats) {
            excsat = findNextDropSat(rtk, sat, ns, nf);
            if (excsat != 0) {
                for (int f = 0; f < nf; f++) {
                    lockc[f] = rtk.ssat[excsat - 1].lock[f];
                    rtk.ssat[excsat - 1].lock[f] = -rtk.nb_ar;
                }
            }
            rtk.excsat = excsat;
        }

        // initial AR
        int gps1 = 1;
        int glo1 = (rtk.opt.navsys & SYS_GLO) != 0 ?
                   ((rtk.opt.glomodear == GLO_ARMODE_FIXHOLD && rtk.holdamb == 0) ? 0 : 1) : 0;
        int sbas1 = (rtk.opt.navsys & SYS_GLO) != 0 ? glo1 :
                    ((rtk.opt.navsys & SYS_SBS) != 0 ? 1 : 0);

        int nb = resambLAMBDA(rtk, bias, xa, gps1, glo1, sbas1);
        float ratio1 = rtk.sol.ratio;

        // AR filter: reject new satellites if ratio degraded
        if (rtk.opt.arfilter != 0) {
            boolean rerun = false;
            if (nb >= 0 && rtk.sol.prevRatio2 >= rtk.sol.thres &&
                (rtk.sol.ratio < rtk.sol.thres ||
                 (rtk.sol.ratio < rtk.opt.thresar[0] * 1.1 &&
                  rtk.sol.ratio < rtk.sol.prevRatio1 / 2.0))) {
                int dly = 2;
                for (int i = 0; i < ns; i++) {
                    for (int f = 0; f < nf; f++) {
                        if (rtk.ssat[sat[i] - 1].fix[f] != 2) continue;
                        if (rtk.ssat[sat[i] - 1].lock[f] == 0) {
                            rtk.ssat[sat[i] - 1].lock[f] = -rtk.opt.minlock - dly;
                            dly += 2;
                            rerun = true;
                        }
                    }
                }
            }
            if (rerun) {
                nb = resambLAMBDA(rtk, bias, xa, gps1, glo1, sbas1);
            }
        }
        rtk.sol.prevRatio1 = ratio1;

        // re-run with different GLO settings if needed
        if ((rtk.opt.navsys & SYS_GLO) != 0 && rtk.opt.glomodear == GLO_ARMODE_FIXHOLD
            && rtk.sol.ratio < rtk.sol.thres) {
            int gps2 = (rtk.opt.gpsmodear == 0 && rtk.sol.ratio >= rtk.sol.thres) ? 0 : 1;
            if (glo1 != 0 || gps1 != gps2) {
                nb = resambLAMBDA(rtk, bias, xa, gps2, 0, 0);
            }
        }

        // restore excluded satellite if no improvement
        if (excsat != 0 && rtk.sol.ratio < rtk.sol.thres &&
            rtk.sol.ratio < 1.5 * rtk.sol.prevRatio2) {
            for (int f = 0; f < nf; f++) rtk.ssat[excsat - 1].lock[f] = lockc[f];
        }

        rtk.sol.prevRatio1 = ratio1 > 0 ? ratio1 : rtk.sol.ratio;
        rtk.sol.prevRatio2 = rtk.sol.ratio;

        return nb;
    }

    // ---------------------------------------------------------------
    // valFixedPos: fixed vs float position consistency check
    // ---------------------------------------------------------------

    private static boolean valFixedPos(RtkState rtk, double[] xa) {
        double thres = rtk.opt.arposthres;
        if (thres <= 0.0) return true;
        // 3D distance between fixed and float solutions
        double dist = 0.0;
        for (int i = 0; i < 3; i++) {
            double d = xa[i] - rtk.x[i];
            dist += d * d;
        }
        return Math.sqrt(dist) <= thres;
    }

    // ---------------------------------------------------------------
    // valChiSquare: post-fit residual chi-square test
    // ---------------------------------------------------------------

    private static boolean valChiSquare(double[] v, double[] R, int nv,
                                         double alpha) {
        if (alpha <= 0.0 || nv < 1) return true;
        double chi2 = 0.0;
        for (int i = 0; i < nv; i++) {
            if (R[i + i * nv] > 0.0) chi2 += v[i] * v[i] / R[i + i * nv];
        }
        // Wilson-Hilferty approximation for chi-square critical value
        double k = nv;
        double z = invNormCdf(1.0 - alpha);
        double cb = 1.0 - 2.0 / (9.0 * k);
        double sb = Math.sqrt(2.0 / (9.0 * k));
        double x = cb + z * sb;
        double thres = k * x * x * x;
        return chi2 <= thres;
    }

    /** Inverse normal CDF approximation (Abramowitz & Stegun 26.2.23). */
    private static double invNormCdf(double p) {
        if (p <= 0.0) return -8.0;
        if (p >= 1.0) return 8.0;
        double t = Math.sqrt(-2.0 * Math.log(p < 0.5 ? p : 1.0 - p));
        double c0 = 2.515517, c1 = 0.802853, c2 = 0.010328;
        double d1 = 1.432788, d2 = 0.189269, d3 = 0.001308;
        double x = t - (c0 + c1 * t + c2 * t * t) / (1.0 + d1 * t + d2 * t * t + d3 * t * t * t);
        return p < 0.5 ? -x : x;
    }

    // ---------------------------------------------------------------
    // valpos: validation of solution
    // ---------------------------------------------------------------

    private static boolean valpos(RtkState rtk, double[] v, double[] R,
                                   int[] vflg, int nv, double thres) {
        double fact = thres * thres;
        for (int i = 0; i < nv; i++) {
            if (v[i] * v[i] > fact * R[i + i * nv]) {
                // large residual (logged in C version)
            }
        }
        return true; // always returns valid (same as C)
    }

    // ---------------------------------------------------------------
    // relpos: relative positioning
    // ---------------------------------------------------------------

    /**
     * Relative positioning (RTK).
     *
     * @param rtk RTK state
     * @param obs merged observations (rover first, then base)
     * @param nu  number of rover observations
     * @param nr  number of base observations
     * @param nav navigation data
     * @return 1 on success, 0 on failure
     */
    static int relpos(RtkState rtk, ObsData[] obs, int nu, int nr, Navigation nav) {
        ProcessingOptions opt = rtk.opt;
        int n = nu + nr;
        int nf = NF(opt);
        int stat = opt.mode <= PMODE_DGPS ? SOLQ_DGPS : SOLQ_FLOAT;

        double[] rs = new double[6 * n];
        double[] dts = new double[2 * n];
        double[] var = new double[n];
        double[] y = new double[nf * 2 * n];
        double[] e = new double[3 * n];
        double[] azel = new double[2 * n];
        double[] freq = new double[nf * n];
        int[] svh = new int[n];

        // init satellite status
        for (int i = 0; i < MAXSAT; i++) {
            rtk.ssat[i].sys = SatelliteUtil.satsys(i + 1)[0];
            for (int j = 0; j < NFREQ; j++) {
                rtk.ssat[i].vsat[j] = 0;
                rtk.ssat[i].snr_rover[j] = 0;
                rtk.ssat[i].snr_base[j] = 0;
            }
        }

        // satellite positions and clocks
        EphemerisCalc.satposs(obs[0].time, obs, n, nav, opt.sateph, rs, dts, var, svh);

        // base station zero-diff residuals
        ObsData[] obsBase = new ObsData[nr];
        System.arraycopy(obs, nu, obsBase, 0, nr);
        double[] rsBase = new double[6 * nr];
        System.arraycopy(rs, nu * 6, rsBase, 0, 6 * nr);
        double[] dtsBase = new double[2 * nr];
        System.arraycopy(dts, nu * 2, dtsBase, 0, 2 * nr);
        double[] varBase = new double[nr];
        System.arraycopy(var, nu, varBase, 0, nr);
        int[] svhBase = new int[nr];
        System.arraycopy(svh, nu, svhBase, 0, nr);
        double[] yBase = new double[nf * 2 * nr];
        double[] eBase = new double[3 * nr];
        double[] azelBase = new double[2 * nr];
        double[] freqBase = new double[nf * nr];

        if (zdres(1, obsBase, nr, rsBase, dtsBase, varBase, svhBase, nav,
                  rtk.rb, opt, yBase, eBase, azelBase, freqBase) == 0) {
            return 0;
        }
        // copy base results back into full arrays
        System.arraycopy(yBase, 0, y, nu * nf * 2, nf * 2 * nr);
        System.arraycopy(eBase, 0, e, nu * 3, 3 * nr);
        System.arraycopy(azelBase, 0, azel, nu * 2, 2 * nr);
        System.arraycopy(freqBase, 0, freq, nu * nf, nf * nr);

        // time diff
        double dt = obs[0].time.timediff(obs[nu].time);
        rtk.sol.age = (float) dt;
        if (Math.abs(rtk.sol.age) > opt.maxtdiff) return 1;

        // select common satellites
        int[] satArr = new int[MAXSAT], iuArr = new int[MAXSAT], irArr = new int[MAXSAT];
        int ns = selsat(obs, azel, nu, nr, opt, satArr, iuArr, irArr);
        if (ns <= 0) return 0;

        // temporal state update
        udstate(rtk, obs, satArr, iuArr, irArr, ns, nav);

        // save SNR
        for (int i = 0; i < ns; i++) {
            for (int j = 0; j < nf; j++) {
                rtk.ssat[satArr[i] - 1].snr_rover[j] = obs[iuArr[i]].SNR[j];
                rtk.ssat[satArr[i] - 1].snr_base[j] = obs[irArr[i]].SNR[j];
            }
        }

        // initialize xp, Pp
        double[] xp = new double[rtk.nx];
        double[] Pp = new double[rtk.nx * rtk.nx];
        double[] xa = new double[rtk.nx];
        System.arraycopy(rtk.x, 0, xp, 0, rtk.nx);
        System.arraycopy(rtk.P, 0, Pp, 0, rtk.nx * rtk.nx);

        int ny = ns * nf * 2 + 2;
        double[] v = new double[ny];
        double[] H = new double[rtk.nx * ny];
        double[] Rm = new double[ny * ny];
        double[] bias = new double[rtk.nx];
        int[] vflg = new int[ny];

        // pre-allocate rover zdres arrays (shared across iterations + post-fit + fix verify)
        double[] yRov = new double[nf * 2 * nu];
        double[] eRov = new double[3 * nu];
        double[] azelRov = new double[2 * nu];
        double[] freqRov = new double[nf * nu];

        // iterative filter
        for (int i = 0; i < opt.niter; i++) {
            // rover zero-diff residuals
            if (zdres(0, obs, nu, rs, dts, var, svh, nav, xp, opt,
                      yRov, eRov, azelRov, freqRov) == 0) {
                stat = SOLQ_NONE;
                break;
            }
            // copy rover results into full arrays
            System.arraycopy(yRov, 0, y, 0, nf * 2 * nu);
            System.arraycopy(eRov, 0, e, 0, 3 * nu);
            System.arraycopy(azelRov, 0, azel, 0, 2 * nu);
            System.arraycopy(freqRov, 0, freq, 0, nf * nu);

            int nv = ddres(rtk, obs, dt, xp, Pp, satArr, y, e, azel, freq,
                           iuArr, irArr, ns, v, H, Rm, vflg);
            if (nv < 4) {
                stat = SOLQ_NONE;
                break;
            }

            // EKF measurement update
            if (MatrixUtil.filter(xp, Pp, H, v, Rm, rtk.nx, nv, rtk.filterWs) != 0) {
                stat = SOLQ_NONE;
                break;
            }
        }

        // post-fit residuals for float solution
        if (stat != SOLQ_NONE) {
            if (zdres(0, obs, nu, rs, dts, var, svh, nav, xp, opt,
                      yRov, eRov, azelRov, freqRov) != 0) {
                System.arraycopy(yRov, 0, y, 0, nf * 2 * nu);
                System.arraycopy(eRov, 0, e, 0, 3 * nu);
                System.arraycopy(azelRov, 0, azel, 0, 2 * nu);
                System.arraycopy(freqRov, 0, freq, 0, nf * nu);

                int nv = ddres(rtk, obs, dt, xp, Pp, satArr, y, e, azel, freq,
                               iuArr, irArr, ns, v, null, Rm, vflg);
                if (valpos(rtk, v, Rm, vflg, nv, 4.0)) {
                    System.arraycopy(xp, 0, rtk.x, 0, rtk.nx);
                    System.arraycopy(Pp, 0, rtk.P, 0, rtk.nx * rtk.nx);

                    rtk.sol.ns = 0;
                    for (int i = 0; i < ns; i++) {
                        for (int f = 0; f < nf; f++) {
                            if (rtk.ssat[satArr[i] - 1].vsat[f] == 0) continue;
                            rtk.ssat[satArr[i] - 1].outc[f] = 0;
                            if (f == 0) rtk.sol.ns++;
                        }
                    }
                    if (rtk.sol.ns < 4) stat = SOLQ_DGPS;
                } else {
                    stat = SOLQ_NONE;
                }
            }
        }

        // resolve integer ambiguity by LAMBDA
        if (stat == SOLQ_FLOAT) {
            if (manageAmbLAMBDA(rtk, bias, xa, satArr, nf, ns) > 1) {
                // Check A: fixed vs float position consistency
                if (valFixedPos(rtk, xa)) {
                    // verify fixed solution residuals
                    if (zdres(0, obs, nu, rs, dts, var, svh, nav, xa, opt,
                              yRov, eRov, azelRov, freqRov) != 0) {
                        System.arraycopy(yRov, 0, y, 0, nf * 2 * nu);
                        System.arraycopy(eRov, 0, e, 0, 3 * nu);
                        System.arraycopy(azelRov, 0, azel, 0, 2 * nu);
                        System.arraycopy(freqRov, 0, freq, 0, nf * nu);

                        int nv = ddres(rtk, obs, dt, xa, Pp, satArr, y, e, azel, freq,
                                       iuArr, irArr, ns, v, null, Rm, vflg);
                        // Check B: chi-square test on post-fit residuals
                        if (valChiSquare(v, Rm, nv, opt.archisqthres) &&
                            valpos(rtk, v, Rm, vflg, nv, 4.0)) {
                            if (++rtk.nfix >= rtk.opt.minfix) {
                                if (rtk.opt.modear == ARMODE_FIXHOLD) {
                                    holdamb(rtk, xa);
                                }
                            }
                            stat = SOLQ_FIX;
                        }
                    }
                }
            }
        }

        // save solution
        if (stat == SOLQ_FIX) {
            for (int i = 0; i < 3; i++) {
                rtk.sol.rr[i] = rtk.xa[i];
                rtk.sol.qr[i] = (float) rtk.Pa[i + i * rtk.na];
            }
            rtk.sol.qr[3] = (float) rtk.Pa[1];
            rtk.sol.qr[4] = (float) rtk.Pa[1 + 2 * rtk.na];
            rtk.sol.qr[5] = (float) rtk.Pa[2];
        } else {
            for (int i = 0; i < 3; i++) {
                rtk.sol.rr[i] = rtk.x[i];
                rtk.sol.qr[i] = (float) rtk.P[i + i * rtk.nx];
            }
            rtk.sol.qr[3] = (float) rtk.P[1];
            rtk.sol.qr[4] = (float) rtk.P[1 + 2 * rtk.nx];
            rtk.sol.qr[5] = (float) rtk.P[2];
            rtk.nfix = 0;
        }

        // save phase measurements and update lock/slip counters
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < nf; j++) {
                if (obs[i].L[j] == 0.0) continue;
                rtk.ssat[obs[i].sat - 1].pt[obs[i].rcv - 1][j] = obs[i].time;
                rtk.ssat[obs[i].sat - 1].ph[obs[i].rcv - 1][j] = obs[i].L[j];
            }
        }
        for (int i = 0; i < MAXSAT; i++) {
            for (int j = 0; j < nf; j++) {
                if ((rtk.ssat[i].slip[j] & LLI_SLIP) != 0) rtk.ssat[i].slipc[j]++;
                if (rtk.ssat[i].vsat[j] == 0) continue;
                if (rtk.ssat[i].lock[j] < 0 || (rtk.nfix > 0 && rtk.ssat[i].fix[j] >= 2)) {
                    rtk.ssat[i].lock[j]++;
                }
            }
        }

        if (stat != SOLQ_NONE) rtk.sol.stat = stat;
        return stat != SOLQ_NONE ? 1 : 0;
    }

    // ---------------------------------------------------------------
    // rtkpos: main entry point
    // ---------------------------------------------------------------

    /**
     * RTK positioning main entry point.
     * <p>
     * Matches C RTKLIB's rtkpos(). Input observations should have
     * rcv=1 for rover and rcv=2 for base, sorted by receiver then satellite.
     *
     * @param rtk RTK state (initialized with {@link RtkState#init})
     * @param obs observation data array (rover first, then base)
     * @param n   total number of observations
     * @param nav navigation data
     * @return 1 on success, 0 on failure
     */
    public static int rtkpos(RtkState rtk, ObsData[] obs, int n, Navigation nav) {
        ProcessingOptions opt = rtk.opt;

        // set base station position
        if (opt.refpos <= POSOPT_RINEX && opt.mode != PMODE_SINGLE &&
            opt.mode != PMODE_MOVEB) {
            for (int i = 0; i < 6; i++) rtk.rb[i] = i < 3 ? opt.rb[i] : 0.0;
        }

        // count rover/base observations
        int nu = 0;
        while (nu < n && obs[nu].rcv == 1) nu++;
        int nr = 0;
        while (nu + nr < n && obs[nu + nr].rcv == 2) nr++;

        GTime timePrev = rtk.sol.time;

        // SPP for rover initial position
        StringBuilder msg = new StringBuilder();
        Spp.SatStatus[] sppSsat = new Spp.SatStatus[MAXSAT];
        for (int i = 0; i < MAXSAT; i++) sppSsat[i] = new Spp.SatStatus();
        if (Spp.pntpos(obs, nu, nav, opt, rtk.sol, null, sppSsat, msg) == 0) {
            if (opt.dynamics == 0) return 0;
        }
        // copy SPP results (azel) to rtk state
        for (int i = 0; i < MAXSAT; i++) {
            rtk.ssat[i].azel[0] = sppSsat[i].azel[0];
            rtk.ssat[i].azel[1] = sppSsat[i].azel[1];
        }
        if (timePrev.time != 0) rtk.tt = rtk.sol.time.timediff(timePrev);

        // single point positioning mode
        if (opt.mode == PMODE_SINGLE) return 1;

        // suppress output of single solution
        if (opt.outsingle == 0) rtk.sol.stat = SOLQ_NONE;

        // check base station data
        if (nr == 0) return 1;

        // relative positioning
        relpos(rtk, obs, nu, nr, nav);
        rtk.epoch++;

        return 1;
    }
}
