package com.gnss.rtklib.positioning;

import com.gnss.rtklib.core.MatrixUtil;
import com.gnss.rtklib.core.SatelliteUtil;
import com.gnss.rtklib.core.SignalUtil;
import com.gnss.rtklib.model.*;

import static com.gnss.rtklib.core.Constants.*;
import static com.gnss.rtklib.model.PppState.*;

/**
 * PPP Ambiguity Resolution — SD (single-difference) WL/NL two-step integer fixing.
 * <p>
 * Uses satellite-pair single-differencing to cancel receiver biases, following
 * the approach in heiwa0519/PPP_AR (Chen Chao, based on RTKLIB).
 * <p>
 * Algorithm:
 * <ol>
 *   <li>Select reference satellite per system (highest elevation)</li>
 *   <li>SD WL fixing: MW_sat - MW_ref → round to integer (receiver bias cancels)</li>
 *   <li>SD NL extraction: (B_IF_sat - B_IF_ref - gamma * WL_sd) / λ_NL</li>
 *   <li>LAMBDA search on SD NL ambiguities with ratio test</li>
 *   <li>Conditional covariance update for fixed solution</li>
 * </ol>
 */
public final class PpposAr {

    private PpposAr() {}

    private static final double WL_FRAC_THRES = 0.25;
    private static final int MIN_SD_AMB = 3;
    /** Max position correction from AR fix (m) — reject if exceeded */
    private static final double MAX_POS_DIFF_FIX = 0.5;

    // Diagnostic counters
    public static int diagAttempt, diagWlOk, diagNlOk, diagFixed;
    public static int diagWlPeak, diagNlPeak, diagEligPeak;
    public static int diagMwAccum; // total MW accumulations from Pppos

    /**
     * Check if navigation data has any satellite phase biases loaded.
     */
    public static boolean hasAnyPhaseBias(Navigation nav) {
        for (int i = 0; i < MAXSAT; i++)
            for (int j = 1; j <= MAXCODE; j++)
                if (nav.pbias[i][j] != 0.0) return true;
        return false;
    }

    /**
     * PPP ambiguity resolution for one epoch (SD approach).
     */
    public static boolean pppAr(PppState rtk, ObsData[] obs, int n,
                                 Navigation nav, double[] azel,
                                 double[] xp, double[] Pp) {
        ProcessingOptions opt = rtk.opt;
        int nx = rtk.nx, na = rtk.na;
        diagAttempt++;

        // --- Step 1: Collect eligible satellites per system ---
        int[] eligSat = new int[n];
        int[] eligF2 = new int[n];
        int[] eligObs = new int[n];
        int ne = 0;

        for (int i = 0; i < n && i < 96; i++) {
            int sat = obs[i].sat;
            int sys = SatelliteUtil.satsys(sat)[0];
            if (sys == SYS_GLO) continue;

            PppState.SatState ss = rtk.ssat[sat - 1];
            if (ss.azel[1] < opt.elmaskar) continue;
            int f2 = Spp.seliflc(opt.nf, sys, obs[i]);
            if (f2 >= NFREQ || obs[i].L[0] == 0.0 || obs[i].L[f2] == 0.0 ||
                obs[i].P[0] == 0.0 || obs[i].P[f2] == 0.0) continue;

            int lockIdx = (opt.ionoopt == IONOOPT_IFLC) ? 0 : f2;
            if (ss.lock[0] < opt.minlock || ss.lock[lockIdx] < opt.minlock) continue;
            int slipVal = (opt.ionoopt == IONOOPT_IFLC) ? ss.slip[0] : (ss.slip[0] | ss.slip[f2]);
            if (slipVal != 0) continue;

            double freq1 = SignalUtil.sat2freq(sat, obs[i].code[0], nav);
            double freq2 = SignalUtil.sat2freq(sat, obs[i].code[f2], nav);
            if (freq1 == 0.0 || freq2 == 0.0) continue;

            double pb1 = lookupPhaseBias(nav, sat, obs[i].code[0]);
            double pb2 = lookupPhaseBias(nav, sat, obs[i].code[f2]);
            if (pb1 == 0.0 && pb2 == 0.0) continue;

            int ib = IB(sat, 0, opt);
            if (xp[ib] == 0.0) continue;

            eligSat[ne] = sat;
            eligF2[ne] = f2;
            eligObs[ne] = i;
            ne++;
        }
        if (ne > diagEligPeak) diagEligPeak = ne;
        if (ne < MIN_SD_AMB + 1) return false; // need ref + MIN_SD_AMB rovers

        // MW averages are now accumulated independently in Pppos.udbiasPpp()

        // --- Step 2: Generate SD pairs per system (ref = highest elevation) ---
        int[] sdSat = new int[ne];     // rover satellite
        int[] sdRef = new int[ne];     // reference satellite
        int[] sdF2 = new int[ne];      // f2 index
        int[] sdOiRov = new int[ne];   // rover obs index
        int[] sdOiRef = new int[ne];   // ref obs index
        int nsd = 0;

        int[] SYS_LIST = {SYS_GPS, SYS_GAL, SYS_CMP, SYS_QZS};
        for (int si = 0; si < SYS_LIST.length; si++) {
            int refSys = SYS_LIST[si];

            // Find reference: highest elevation in this system
            int refIdx = -1;
            double refEl = -1;
            for (int i = 0; i < ne; i++) {
                int sys = SatelliteUtil.satsys(eligSat[i])[0];
                if (sys != refSys) continue;
                double el = rtk.ssat[eligSat[i] - 1].azel[1];
                if (el > refEl) { refEl = el; refIdx = i; }
            }
            if (refIdx < 0) continue;

            // Generate SD pairs: each rover vs reference
            for (int i = 0; i < ne; i++) {
                if (i == refIdx) continue;
                int sys = SatelliteUtil.satsys(eligSat[i])[0];
                if (sys != refSys) continue;

                sdSat[nsd] = eligSat[i];
                sdRef[nsd] = eligSat[refIdx];
                sdF2[nsd] = eligF2[i];
                sdOiRov[nsd] = eligObs[i];
                sdOiRef[nsd] = eligObs[refIdx];
                nsd++;
            }
        }
        if (nsd < MIN_SD_AMB) return false;

        // --- Step 4: SD WL fixing ---
        int[] sdWl = new int[nsd];
        int nwl = 0;

        double interval = Math.abs(rtk.tt);
        if (interval <= 0.0) interval = 30.0;
        int minCount = Math.max(5, (int) Math.min(120.0 / interval, 120));

        for (int i = 0; i < nsd; i++) {
            sdWl[i] = Integer.MIN_VALUE;
            PppState.SatState ssRov = rtk.ssat[sdSat[i] - 1];
            PppState.SatState ssRef = rtk.ssat[sdRef[i] - 1];
            if (ssRov.mwCount[0] < minCount || ssRef.mwCount[0] < minCount) continue;

            // SD MW = MW_rover - MW_ref (receiver bias cancels!)
            double sdMw = ssRov.mwAvg[0] - ssRef.mwAvg[0];
            int wlInt = (int) Math.round(sdMw);
            double frac = Math.abs(sdMw - wlInt);
            if (frac > WL_FRAC_THRES) continue;

            sdWl[i] = wlInt;
            nwl++;
        }

        diagWlOk = nwl;
        if (nwl > diagWlPeak) diagWlPeak = nwl;
        if (nwl < MIN_SD_AMB) return false;

        // --- Step 5: SD NL extraction ---
        // Compact to WL-fixed pairs only
        int[] fSat = new int[nwl], fRef = new int[nwl], fWl = new int[nwl];
        int[] fF2 = new int[nwl], fOiR = new int[nwl], fOiB = new int[nwl];
        int nb = 0;
        for (int i = 0; i < nsd; i++) {
            if (sdWl[i] == Integer.MIN_VALUE) continue;
            fSat[nb] = sdSat[i]; fRef[nb] = sdRef[i]; fWl[nb] = sdWl[i];
            fF2[nb] = sdF2[i]; fOiR[nb] = sdOiRov[i]; fOiB[nb] = sdOiRef[i];
            nb++;
        }

        double[] nlFloat = new double[nb];
        int[] ibRov = new int[nb], ibRef = new int[nb];

        for (int i = 0; i < nb; i++) {
            int sat = fSat[i], ref = fRef[i], wl = fWl[i];
            int oi = fOiR[i], oiRef = fOiB[i];
            double freq1 = SignalUtil.sat2freq(sat, obs[oi].code[0], nav);
            double freq2 = SignalUtil.sat2freq(sat, obs[oi].code[fF2[i]], nav);
            if (freq1 == 0 || freq2 == 0) { nlFloat[i] = 0; continue; }

            double lamNL = CLIGHT / (freq1 + freq2);
            double gamma = CLIGHT * freq2 / (sq(freq1) - sq(freq2));

            // SD IFLC bias
            int ibS = IB(sat, 0, opt);
            int ibR = IB(ref, 0, opt);
            double sdIF = xp[ibS] - xp[ibR];

            // SD NL float = (sd_IF - gamma * WL_sd) / λ_NL
            nlFloat[i] = (sdIF - gamma * wl) / lamNL;
            ibRov[i] = ibS;
            ibRef[i] = ibR;
        }

        diagNlOk = nb;
        if (nb > diagNlPeak) diagNlPeak = nb;
        if (nb < MIN_SD_AMB) return false;

        // --- Step 6: Build SD NL covariance ---
        // H_nl: nb × nx matrix, H_nl[ibS] = 1/lamNL, H_nl[ibR] = -1/lamNL per row
        // Qnl = H_nl * P * H_nl'
        double[] Qnl = new double[nb * nb];
        for (int i = 0; i < nb; i++) {
            int oi = fOiR[i];
            double freq1_i = SignalUtil.sat2freq(fSat[i], obs[oi].code[0], nav);
            double freq2_i = SignalUtil.sat2freq(fSat[i], obs[oi].code[fF2[i]], nav);
            double lamNL_i = (freq1_i > 0 && freq2_i > 0) ? CLIGHT / (freq1_i + freq2_i) : 0.109;

            for (int j = 0; j < nb; j++) {
                int oj = fOiR[j];
                double freq1_j = SignalUtil.sat2freq(fSat[j], obs[oj].code[0], nav);
                double freq2_j = SignalUtil.sat2freq(fSat[j], obs[oj].code[fF2[j]], nav);
                double lamNL_j = (freq1_j > 0 && freq2_j > 0) ? CLIGHT / (freq1_j + freq2_j) : 0.109;

                // SD covariance: P[ibS_i,ibS_j] - P[ibS_i,ibR_j] - P[ibR_i,ibS_j] + P[ibR_i,ibR_j]
                double cov = Pp[ibRov[i] + ibRov[j] * nx]
                           - Pp[ibRov[i] + ibRef[j] * nx]
                           - Pp[ibRef[i] + ibRov[j] * nx]
                           + Pp[ibRef[i] + ibRef[j] * nx];
                Qnl[i + j * nb] = cov / (lamNL_i * lamNL_j);
            }
        }

        // --- Step 7: LAMBDA search ---
        double[] F = new double[nb * 2];
        double[] s = new double[2];
        int info = Lambda.lambda(nb, 2, nlFloat, Qnl, F, s);
        if (info != 0) return false;

        double ratio = s[0] > 0 ? s[1] / s[0] : 0.0;
        double thres = Rtkpos.computeAdaptiveArThreshold(nb, opt.thresar[0]);
        rtk.sol.ratio = ratio > 999.9 ? 999.9f : (float) ratio;
        rtk.sol.thres = (float) thres;

        if (s[0] <= 0.0 || ratio < thres) return false;

        // --- Step 8: Conditional covariance update ---
        // Build SD IFLC differencing matrix H_if (ny × nx) where ny = na + nb
        // H_if[0..na-1, 0..na-1] = I(na)
        // H_if[na+i, ibRov[i]] = 1, H_if[na+i, ibRef[i]] = -1
        int ny = na + nb;
        double[] y = new double[ny]; // SD IFLC values
        for (int i = 0; i < na; i++) y[i] = xp[i];
        for (int i = 0; i < nb; i++) y[na + i] = xp[ibRov[i]] - xp[ibRef[i]];

        // Build Qy = H_if * P * H_if' for the bias block
        double[] QbIF = new double[nb * nb];
        double[] Qab = new double[na * nb];
        for (int i = 0; i < nb; i++) {
            for (int j = 0; j < nb; j++) {
                QbIF[i + j * nb] = Pp[ibRov[i] + ibRov[j] * nx]
                                 - Pp[ibRov[i] + ibRef[j] * nx]
                                 - Pp[ibRef[i] + ibRov[j] * nx]
                                 + Pp[ibRef[i] + ibRef[j] * nx];
            }
            for (int j = 0; j < na; j++) {
                Qab[j + i * na] = Pp[j + ibRov[i] * nx] - Pp[j + ibRef[i] * nx];
            }
        }

        // Save original position for outlier check after conditional update
        double[] xpOrig = new double[]{xp[0], xp[1], xp[2]};

        // Reconstruct fixed SD IFLC biases
        double[] db = new double[nb];
        for (int i = 0; i < nb; i++) {
            int sat = fSat[i];
            int oi = fOiR[i];
            double freq1 = SignalUtil.sat2freq(sat, obs[oi].code[0], nav);
            double freq2 = SignalUtil.sat2freq(sat, obs[oi].code[fF2[i]], nav);
            if (freq1 == 0 || freq2 == 0) continue;

            double lamNL = CLIGHT / (freq1 + freq2);
            double gamma = CLIGHT * freq2 / (sq(freq1) - sq(freq2));

            double sdIF_fixed = lamNL * F[i] + gamma * fWl[i];
            db[i] = y[na + i] - sdIF_fixed;
        }

        // Invert QbIF
        if (MatrixUtil.matinv(QbIF, nb) != 0) return false;

        // xp[0..na-1] -= Qab * QbIF^-1 * db
        double[] QbDb = new double[nb];
        MatrixUtil.matmul("NN", nb, 1, nb, QbIF, db, QbDb);
        for (int i = 0; i < na; i++) {
            double corr = 0;
            for (int j = 0; j < nb; j++) corr += Qab[i + j * na] * QbDb[j];
            xp[i] -= corr;
        }

        // Reject if position correction is too large (outlier protection)
        double posDiff = 0;
        for (int i = 0; i < 3; i++) {
            double d = xp[i] - xpOrig[i];
            posDiff += d * d;
        }
        if (Math.sqrt(posDiff) > MAX_POS_DIFF_FIX) return false;

        // Pp[0..na-1, 0..na-1] -= Qab * QbIF^-1 * Qab'
        double[] QQ = new double[na * nb];
        MatrixUtil.matmul("NN", na, nb, nb, Qab, QbIF, QQ);
        for (int i = 0; i < na; i++) {
            for (int j = 0; j < na; j++) {
                double sum = 0;
                for (int k = 0; k < nb; k++) sum += QQ[i + k * na] * Qab[j + k * na];
                Pp[i + j * nx] -= sum;
            }
        }

        // Mark fixed satellites (for fix-and-hold feedback)
        for (int i = 0; i < nb; i++) {
            rtk.ssat[fSat[i] - 1].fix[0] = 2;
            rtk.ssat[fRef[i] - 1].fix[0] = 2;
        }

        diagFixed++;
        return true;
    }

    /**
     * Update MW running average for one satellite.
     * Applies phase + code OSB correction. Result stored per-satellite (undifferenced).
     * SD is formed later by subtracting reference satellite's average.
     */
    static void updateMwAverage(PppState.SatState ssat, ObsData obs, int f2,
                                         double freq1, double freq2,
                                         Navigation nav, double tt) {
        double lamWL = CLIGHT / (freq1 - freq2);

        // Melbourne-Wübbena combination (meters)
        double mwRaw = (obs.L[0] - obs.L[f2]) * CLIGHT / (freq1 - freq2)
                     - (freq1 * obs.P[0] + freq2 * obs.P[f2]) / (freq1 + freq2);

        // Phase OSB correction
        double pbias1 = lookupPhaseBias(nav, obs.sat, obs.code[0]);
        double pbias2 = lookupPhaseBias(nav, obs.sat, obs.code[f2]);
        double phCorr = pbias1 * freq1 / CLIGHT - pbias2 * freq2 / CLIGHT;

        // Code OSB correction
        double cbias1 = lookupCodeBias(nav, obs.sat, obs.code[0]);
        double cbias2 = lookupCodeBias(nav, obs.sat, obs.code[f2]);
        double codeCorr = (freq1 * cbias1 + freq2 * cbias2) / ((freq1 + freq2) * lamWL);

        double mwCorr = mwRaw / lamWL - phCorr + codeCorr;

        ssat.mwCount[0]++;
        ssat.mwAvg[0] += (mwCorr - ssat.mwAvg[0]) / ssat.mwCount[0];
    }

    private static int findObs(ObsData[] obs, int n, int sat) {
        for (int i = 0; i < n; i++) if (obs[i].sat == sat) return i;
        return -1;
    }

    public static double lookupPhaseBias(Navigation nav, int sat, int code) {
        if (code <= 0 || code > MAXCODE) return 0.0;
        double b = nav.pbias[sat - 1][code];
        if (b != 0.0) return b;
        int alt = altCode(code);
        if (alt > 0 && alt <= MAXCODE) b = nav.pbias[sat - 1][alt];
        return b;
    }

    public static double lookupCodeBias(Navigation nav, int sat, int code) {
        if (code <= 0 || code > MAXCODE) return 0.0;
        double b = nav.cbias_osb[sat - 1][code];
        if (b != 0.0) return b;
        int alt = altCode(code);
        if (alt > 0 && alt <= MAXCODE) b = nav.cbias_osb[sat - 1][alt];
        return b;
    }

    private static int altCode(int code) {
        switch (code) {
            case CODE_L5X: return CODE_L5Q;
            case CODE_L5Q: return CODE_L5X;
            case CODE_L5I: return CODE_L5Q;
            case CODE_L1X: return CODE_L1C;
            case CODE_L1C: return CODE_L1X;
            case CODE_L1P: return CODE_L1X;
            case CODE_L5P: return CODE_L5X;
            // L2 fallbacks
            case CODE_L2X: return CODE_L2W;
            case CODE_L2W: return CODE_L2X;
            case CODE_L2L: return CODE_L2W;
            case CODE_L2S: return CODE_L2W;
            // E5b (L7) fallbacks
            case CODE_L7X: return CODE_L7Q;
            case CODE_L7Q: return CODE_L7X;
            case CODE_L7I: return CODE_L7Q;
            // GPS L2 additional fallbacks
            case CODE_L2P: return CODE_L2W;
            case CODE_L2C: return CODE_L2S;
            // GPS L1 additional fallback
            case CODE_L1W: return CODE_L1C;
            // GAL E5ab (L8) fallbacks
            case CODE_L8X: return CODE_L8Q;
            case CODE_L8Q: return CODE_L8X;
            // GAL E6 (L6) fallbacks
            case CODE_L6X: return CODE_L6C;
            case CODE_L6C: return CODE_L6X;
            default: return 0;
        }
    }

    private static double sq(double x) { return x * x; }
}
