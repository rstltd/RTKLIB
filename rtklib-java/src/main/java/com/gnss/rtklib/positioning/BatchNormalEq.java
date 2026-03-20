package com.gnss.rtklib.positioning;

import com.gnss.rtklib.core.*;
import com.gnss.rtklib.model.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static com.gnss.rtklib.core.Constants.*;

/**
 * BLS DD observation construction, normal equation accumulation,
 * and Schur complement solve.
 */
final class BatchNormalEq {

    private BatchNormalEq() {}

    /** DD observation for one measurement (phase or code). */
    static class DdObs {
        double v;
        double[] hPos;
        double lambda;
        int ddAmbIdx;
        double var;       // total DD variance = varRef + varNonRef
        double varRef;    // ref sat SD variance (for DD correlation correction)
        int refSat;       // ref sat number (for grouping)
        int sat;          // non-ref sat number (for outlier identification)
        int freq;         // frequency index (for grouping)
        int epoch;        // epoch index (for outlier identification)
        boolean isPhase;
    }

    // ---------------------------------------------------------------
    // makeDdObs
    // ---------------------------------------------------------------

    /** Build index for fast DD ambiguity lookup. Call once, reuse across epochs. */
    static Map<Long, Integer> buildAmbIndex(List<BatchPreprocess.AmbParam> ambParams) {
        Map<Long, Integer> idx = new HashMap<>();
        for (int i = 0; i < ambParams.size(); i++) {
            BatchPreprocess.AmbParam ap = ambParams.get(i);
            // Key encodes (refSat, sat, freq, startEpoch) to handle multiple segments
            long key = ((long) ap.refSat << 40) | ((long) ap.sat << 20)
                     | ((long) ap.freq << 16) | ap.startEpoch;
            idx.put(key, i);
        }
        return idx;
    }

    /** Fast DD ambiguity index lookup using pre-built index. */
    static int findDdAmbIdxFast(Map<Long, Integer> ambIndex,
                                 List<BatchPreprocess.AmbParam> ambParams,
                                 int refSat, int sat, int freq, int epochIdx) {
        // Try exact segment match by scanning possible start epochs
        // Since segments don't overlap for same (refSat, sat, freq), iterate candidates
        for (Map.Entry<Long, Integer> e : ambIndex.entrySet()) {
            long key = e.getKey();
            int rs = (int) (key >> 40);
            int s  = (int) ((key >> 20) & 0xFFFFF);
            int f  = (int) ((key >> 16) & 0xF);
            if (rs != refSat || s != sat || f != freq) continue;
            BatchPreprocess.AmbParam ap = ambParams.get(e.getValue());
            if (epochIdx >= ap.startEpoch && epochIdx <= ap.endEpoch) {
                return e.getValue();
            }
        }
        return -1;
    }

    static List<DdObs> makeDdObs(BatchPreprocess.EpochData ed, double[] pos,
                                  ProcessingOptions opt, Navigation nav, int nf,
                                  double[] yRov, double[] eRov, double[] azelRov,
                                  double[] freqRov,
                                  double[] yBase, double[] azelBase, double[] freqBase,
                                  List<BatchPreprocess.AmbParam> ambParams,
                                  Map<Long, Integer> ambIndex,
                                  double[] ambValues, int epochIdx, double bl,
                                  int[][] refSatMap) {
        int[] systems = {SYS_GPS, SYS_GLO, SYS_GAL, SYS_QZS, SYS_CMP, SYS_IRN, SYS_SBS};
        List<DdObs> result = new ArrayList<>();

        for (int si = 0; si < systems.length; si++) {
            int sys = systems[si];
            if ((sys & opt.navsys) == 0) continue;

            for (int f = 0; f < nf; f++) {
                int refSat = refSatMap[si][f];
                if (refSat == 0) continue;

                int refIdx = -1;
                for (int i = 0; i < ed.ns; i++) {
                    if (ed.sat[i] == refSat) { refIdx = i; break; }
                }
                if (refIdx < 0) continue;

                int iuRef = ed.iu[refIdx];
                int irRef = ed.ir[refIdx] - ed.nu;

                double yPhRefRov = yRov[f + iuRef * nf * 2];
                double yPhRefBase = yBase[f + irRef * nf * 2];
                if (yPhRefRov == 0 || yPhRefBase == 0) continue;

                double elRef = azelRov[1 + iuRef * 2];
                if (elRef < opt.elmin) continue;
                if (Spp.testsnr(0, f, elRef, ed.obs[iuRef].SNR[f], opt.snrmask)) continue;
                if (Spp.testsnr(1, f, azelBase[1 + irRef * 2],
                        ed.obs[ed.ir[refIdx]].SNR[f], opt.snrmask)) continue;

                double varRef = Rtkpos.varerr(refSat, sys, elRef,
                        ed.obs[iuRef].SNR[f], ed.obs[ed.ir[refIdx]].SNR[f],
                        bl, 0, f, opt);

                double yCdRefRov = yRov[f + nf + iuRef * nf * 2];
                double yCdRefBase = yBase[f + nf + irRef * nf * 2];

                for (int j = 0; j < ed.ns; j++) {
                    if (j == refIdx) continue;
                    int sat = ed.sat[j];
                    if (SatelliteUtil.satsys(sat)[0] != sys) continue;

                    int iuJ = ed.iu[j];
                    int irJ = ed.ir[j] - ed.nu;

                    double elJ = azelRov[1 + iuJ * 2];
                    if (elJ < opt.elmin) continue;
                    if (Spp.testsnr(0, f, elJ, ed.obs[iuJ].SNR[f], opt.snrmask)) continue;
                    if (Spp.testsnr(1, f, azelBase[1 + irJ * 2],
                            ed.obs[ed.ir[j]].SNR[f], opt.snrmask)) continue;

                    double varJ = Rtkpos.varerr(sat, sys, elJ,
                            ed.obs[iuJ].SNR[f], ed.obs[ed.ir[j]].SNR[f],
                            bl, 0, f, opt);

                    double[] hPos = new double[3];
                    for (int k = 0; k < 3; k++) {
                        hPos[k] = -eRov[k + iuRef * 3] + eRov[k + iuJ * 3];
                    }

                    double freq = freqRov[f + iuJ * nf];
                    double lambda = (freq > 0) ? CLIGHT / freq : 0;

                    // Phase DD
                    double yPhJRov = yRov[f + iuJ * nf * 2];
                    double yPhJBase = yBase[f + irJ * nf * 2];
                    if (yPhJRov != 0 && yPhJBase != 0 && lambda > 0) {
                        double ddPhase = (yPhRefRov - yPhRefBase) - (yPhJRov - yPhJBase);
                        int ddAmbIdx = ambIndex != null
                                ? findDdAmbIdxFast(ambIndex, ambParams, refSat, sat, f, epochIdx)
                                : findDdAmbIdx(ambParams, refSat, sat, f, epochIdx);

                        if (ddAmbIdx >= 0) {
                            DdObs obs = new DdObs();
                            obs.v = ddPhase - lambda * ambValues[ddAmbIdx];
                            obs.hPos = hPos;
                            obs.lambda = lambda;
                            obs.ddAmbIdx = ddAmbIdx;
                            obs.var = varRef + varJ;
                            obs.varRef = varRef;
                            obs.refSat = refSat;
                            obs.sat = sat;
                            obs.freq = f;
                            obs.epoch = epochIdx;
                            obs.isPhase = true;
                            result.add(obs);
                        }
                    }

                    // Code DD
                    double yCdJRov = yRov[f + nf + iuJ * nf * 2];
                    double yCdJBase = yBase[f + nf + irJ * nf * 2];
                    if (yCdRefRov != 0 && yCdRefBase != 0 && yCdJRov != 0 && yCdJBase != 0) {
                        double ddCode = (yCdRefRov - yCdRefBase) - (yCdJRov - yCdJBase);

                        double varRefCode = Rtkpos.varerr(refSat, sys, elRef,
                                ed.obs[iuRef].SNR[f], ed.obs[ed.ir[refIdx]].SNR[f],
                                bl, 0, f + nf, opt);
                        double varJCode = Rtkpos.varerr(sat, sys, elJ,
                                ed.obs[iuJ].SNR[f], ed.obs[ed.ir[j]].SNR[f],
                                bl, 0, f + nf, opt);

                        DdObs obs = new DdObs();
                        obs.v = ddCode;
                        obs.hPos = hPos;
                        obs.lambda = 0;
                        obs.ddAmbIdx = -1;
                        obs.var = varRefCode + varJCode;
                        obs.varRef = varRefCode;
                        obs.refSat = refSat;
                        obs.sat = sat;
                        obs.freq = f;
                        obs.epoch = epochIdx;
                        obs.isPhase = false;
                        result.add(obs);
                    }
                }
            }
        }

        return result;
    }

    static int findDdAmbIdx(List<BatchPreprocess.AmbParam> ambParams,
                             int refSat, int sat, int freq, int epochIdx) {
        for (int i = 0; i < ambParams.size(); i++) {
            BatchPreprocess.AmbParam ap = ambParams.get(i);
            if (ap.refSat == refSat && ap.sat == sat && ap.freq == freq
                && epochIdx >= ap.startEpoch && epochIdx <= ap.endEpoch) {
                return i;
            }
        }
        return -1;
    }

    // ---------------------------------------------------------------
    // initDdAmbFromZdres
    // ---------------------------------------------------------------

    static double[] initDdAmbFromZdres(List<BatchPreprocess.EpochData> epochs,
                                        List<BatchPreprocess.AmbParam> ambParams,
                                        Navigation nav, ProcessingOptions opt,
                                        double[] pos, int nf) {
        double[] ambValues = new double[ambParams.size()];

        for (int j = 0; j < ambParams.size(); j++) {
            BatchPreprocess.AmbParam ap = ambParams.get(j);
            int f = ap.freq;
            double sumBias = 0;
            int count = 0;

            int maxEp = Math.min(ap.endEpoch + 1, epochs.size());
            for (int ep = ap.startEpoch; ep < maxEp; ep++) {
                BatchPreprocess.EpochData ed = epochs.get(ep);

                int refIdx = -1, satIdx = -1;
                for (int i = 0; i < ed.ns; i++) {
                    if (ed.sat[i] == ap.refSat) refIdx = i;
                    if (ed.sat[i] == ap.sat) satIdx = i;
                }
                if (refIdx < 0 || satIdx < 0) continue;

                double[] yRov = new double[nf * 2 * ed.nu];
                double[] eRov = new double[3 * ed.nu];
                double[] azelRov = new double[2 * ed.nu];
                double[] freqRov = new double[nf * ed.nu];
                ObsData[] obsRov = new ObsData[ed.nu];
                System.arraycopy(ed.obs, 0, obsRov, 0, ed.nu);

                if (Rtkpos.zdres(0, obsRov, ed.nu, ed.rs, ed.dts, ed.var, ed.svh,
                                  nav, pos, opt, yRov, eRov, azelRov, freqRov) == 0) continue;

                double[] yBase = new double[nf * 2 * ed.nr];
                double[] eBase = new double[3 * ed.nr];
                double[] azelBase = new double[2 * ed.nr];
                double[] freqBase = new double[nf * ed.nr];
                ObsData[] obsBase = new ObsData[ed.nr];
                System.arraycopy(ed.obs, ed.nu, obsBase, 0, ed.nr);
                double[] rsBase = new double[6 * ed.nr];
                System.arraycopy(ed.rs, ed.nu * 6, rsBase, 0, 6 * ed.nr);
                double[] dtsBase = new double[2 * ed.nr];
                System.arraycopy(ed.dts, ed.nu * 2, dtsBase, 0, 2 * ed.nr);
                double[] varBase = new double[ed.nr];
                System.arraycopy(ed.var, ed.nu, varBase, 0, ed.nr);
                int[] svhBase = new int[ed.nr];
                System.arraycopy(ed.svh, ed.nu, svhBase, 0, ed.nr);

                if (Rtkpos.zdres(1, obsBase, ed.nr, rsBase, dtsBase, varBase, svhBase,
                                  nav, opt.rb, opt, yBase, eBase, azelBase, freqBase) == 0) continue;

                int iuRef = ed.iu[refIdx], irRef = ed.ir[refIdx] - ed.nu;
                int iuJ = ed.iu[satIdx], irJ = ed.ir[satIdx] - ed.nu;

                if (iuRef >= ed.nu || irRef < 0 || irRef >= ed.nr) continue;
                if (iuJ >= ed.nu || irJ < 0 || irJ >= ed.nr) continue;

                double phRefRov = yRov[f + iuRef * nf * 2];
                double phRefBase = yBase[f + irRef * nf * 2];
                double phJRov = yRov[f + iuJ * nf * 2];
                double phJBase = yBase[f + irJ * nf * 2];

                double cdRefRov = yRov[f + nf + iuRef * nf * 2];
                double cdRefBase = yBase[f + nf + irRef * nf * 2];
                double cdJRov = yRov[f + nf + iuJ * nf * 2];
                double cdJBase = yBase[f + nf + irJ * nf * 2];

                if (phRefRov == 0 || phRefBase == 0 || phJRov == 0 || phJBase == 0) continue;
                if (cdRefRov == 0 || cdRefBase == 0 || cdJRov == 0 || cdJBase == 0) continue;

                double ddPhase = (phRefRov - phRefBase) - (phJRov - phJBase);
                double ddCode = (cdRefRov - cdRefBase) - (cdJRov - cdJBase);

                double freq = freqRov[f + iuJ * nf];
                if (freq <= 0) continue;
                double lambda = CLIGHT / freq;

                sumBias += (ddPhase - ddCode) / lambda;
                count++;

                if (count >= 10) break;
            }

            if (count > 0) {
                ambValues[j] = sumBias / count;
            }
        }
        return ambValues;
    }

    // ---------------------------------------------------------------
    // accumulateEpoch: add one epoch's DD observations to normal eq
    // ---------------------------------------------------------------

    /**
     * Accumulate DD observations into normal equations with Sherman-Morrison
     * correction for DD correlation (DDs sharing the same ref sat are correlated).
     *
     * For a group of DDs with the same ref sat:
     *   R = diag(varNonRef_i) + varRef * 11'
     *   R^-1 = diag(1/varNonRef_i) - alpha * w_i * w_j
     *   where w_i = 1/varNonRef_i, alpha = varRef / (1 + varRef * sum(w_i))
     */
    static int accumulateEpoch(List<DdObs> ddObs, int nx, int nAmb,
                                double[] N, double[] b) {
        // Group by (refSat, freq, isPhase)
        Map<Long, List<DdObs>> groups = new HashMap<>();
        for (DdObs dd : ddObs) {
            long key = ((long) dd.refSat << 16) | ((long) dd.freq << 1) | (dd.isPhase ? 1 : 0);
            groups.computeIfAbsent(key, k -> new ArrayList<>()).add(dd);
        }

        int count = 0;
        double[] Hsum = new double[nx]; // reuse across groups

        for (List<DdObs> group : groups.values()) {
            double varRefGrp = group.get(0).varRef;

            if (group.size() <= 1 || varRefGrp <= 0) {
                // Single DD or no ref variance: use diagonal weight (no correlation)
                for (DdObs dd : group) {
                    accumulateOne(dd, 1.0 / dd.var, nx, N, b);
                    count++;
                }
                continue;
            }

            // Sherman-Morrison: w_i = 1/varNonRef_i, alpha = varRef / (1 + varRef * sum(w_i))
            double sumW = 0;
            for (DdObs dd : group) {
                double varNonRef = dd.var - varRefGrp;
                if (varNonRef <= 0) varNonRef = dd.var; // safety
                sumW += 1.0 / varNonRef;
            }
            double alpha = varRefGrp / (1.0 + varRefGrp * sumW);

            // Diagonal part: accumulate each DD with w_i = 1/varNonRef_i
            Arrays.fill(Hsum, 0, nx, 0.0);
            double bwSum = 0;
            for (DdObs dd : group) {
                double varNonRef = dd.var - varRefGrp;
                if (varNonRef <= 0) varNonRef = dd.var;
                double wi = 1.0 / varNonRef;

                accumulateOne(dd, wi, nx, N, b);

                // Accumulate weighted H sum for rank-1 correction
                for (int k = 0; k < 3; k++) Hsum[k] += wi * dd.hPos[k];
                if (dd.isPhase && dd.ddAmbIdx >= 0) {
                    Hsum[3 + dd.ddAmbIdx] += wi * dd.lambda;
                }
                bwSum += wi * dd.v;
                count++;
            }

            // Subtract rank-1 correction: N -= alpha * Hsum*Hsum', b -= alpha * Hsum*bwSum
            for (int k = 0; k < nx; k++) {
                if (Hsum[k] == 0.0) continue;
                b[k] -= alpha * Hsum[k] * bwSum;
                for (int l = k; l < nx; l++) {
                    if (Hsum[l] == 0.0) continue;
                    double val = alpha * Hsum[k] * Hsum[l];
                    N[k + l * nx] -= val;
                    if (k != l) N[l + k * nx] -= val;
                }
            }
        }
        return count;
    }

    private static void accumulateOne(DdObs dd, double w, int nx,
                                       double[] N, double[] b) {
        for (int k = 0; k < 3; k++) {
            b[k] += dd.hPos[k] * w * dd.v;
        }
        for (int k = 0; k < 3; k++) {
            for (int l = k; l < 3; l++) {
                double val = dd.hPos[k] * w * dd.hPos[l];
                N[k + l * nx] += val;
                if (k != l) N[l + k * nx] += val;
            }
        }
        if (dd.isPhase && dd.ddAmbIdx >= 0) {
            int ai = 3 + dd.ddAmbIdx;
            b[ai] += dd.lambda * w * dd.v;
            for (int k = 0; k < 3; k++) {
                double val = dd.hPos[k] * w * dd.lambda;
                N[k + ai * nx] += val;
                N[ai + k * nx] += val;
            }
            N[ai + ai * nx] += dd.lambda * w * dd.lambda;
        }
    }

    // ---------------------------------------------------------------
    // Schur complement solve
    // ---------------------------------------------------------------

    /** Solve via Schur complement. Returns false if singular. */
    static boolean schurSolve(double[] N, double[] b, int nx, int nAmb,
                               List<List<Integer>> ambComponents,
                               List<BatchPreprocess.AmbParam> ambParams,
                               double[] dxp, double[] dxa) {
        double[] Npp = new double[9], Npa = new double[3 * nAmb];
        double[] Naa = new double[nAmb * nAmb];
        double[] bp = new double[3], ba = new double[nAmb];

        for (int i = 0; i < 3; i++) {
            bp[i] = b[i];
            for (int j = 0; j < 3; j++) Npp[i + j * 3] = N[i + j * nx];
            for (int j = 0; j < nAmb; j++) Npa[i + j * 3] = N[i + (3 + j) * nx];
        }
        for (int i = 0; i < nAmb; i++) {
            ba[i] = b[3 + i];
            for (int j = 0; j < nAmb; j++) {
                Naa[i + j * nAmb] = N[(3 + i) + (3 + j) * nx];
            }
        }

        // Block-diagonal Naa inverse
        double[] NaaInv = new double[nAmb * nAmb];
        for (List<Integer> comp : ambComponents) {
            int cn = comp.size();
            double[] Nsub = new double[cn * cn];
            for (int ci = 0; ci < cn; ci++)
                for (int cj = 0; cj < cn; cj++)
                    Nsub[ci + cj * cn] = Naa[comp.get(ci) + comp.get(cj) * nAmb];
            if (MatrixUtil.matinv(Nsub, cn) != 0) return false;
            for (int ci = 0; ci < cn; ci++)
                for (int cj = 0; cj < cn; cj++)
                    NaaInv[comp.get(ci) + comp.get(cj) * nAmb] = Nsub[ci + cj * cn];
        }

        double[] tmp = new double[3 * nAmb];
        MatrixUtil.matmul("NN", 3, nAmb, nAmb, Npa, NaaInv, tmp);

        double[] Nred = new double[9];
        System.arraycopy(Npp, 0, Nred, 0, 9);
        MatrixUtil.matmul("NT", 3, 3, nAmb, -1.0, tmp, Npa, 1.0, Nred);

        double[] bred = new double[3];
        System.arraycopy(bp, 0, bred, 0, 3);
        MatrixUtil.matmul("NN", 3, 1, nAmb, -1.0, tmp, ba, 1.0, bred);

        double[] NredInv = new double[9];
        System.arraycopy(Nred, 0, NredInv, 0, 9);
        if (MatrixUtil.matinv(NredInv, 3) != 0) return false;

        MatrixUtil.matmul("NN", 3, 1, 3, NredInv, bred, dxp);

        double[] baRed = new double[nAmb];
        System.arraycopy(ba, 0, baRed, 0, nAmb);
        MatrixUtil.matmul("TN", nAmb, 1, 3, -1.0, Npa, dxp, 1.0, baRed);

        MatrixUtil.matmul("NN", nAmb, 1, nAmb, NaaInv, baRed, dxa);
        return true;
    }

    /** Regularize weakly observed ambiguities. */
    static void regularizeNaa(double[] N, int nx, int nAmb) {
        double maxNaaDiag = 0;
        for (int j = 0; j < nAmb; j++) {
            double d = N[(3 + j) + (3 + j) * nx];
            if (d > maxNaaDiag) maxNaaDiag = d;
        }
        if (maxNaaDiag > 0) {
            double minDiag = maxNaaDiag * 1e-6;
            for (int j = 0; j < nAmb; j++) {
                int p = 3 + j;
                if (N[p + p * nx] < minDiag) {
                    N[p + p * nx] += minDiag;
                }
            }
        }
    }

    // ---------------------------------------------------------------
    // Covariance computation
    // ---------------------------------------------------------------

    /** Compute Qpp, Qpa, Qaa from final normal equation. Returns false if singular. */
    static boolean computeCovariance(double[] N, int nx, int nAmb,
                                      List<List<Integer>> ambComponents,
                                      double[] Qpp, double[] Qpa, double[] Qaa) {
        double[] Npp = new double[9], Npa = new double[3 * nAmb];
        double[] Naa = new double[nAmb * nAmb];

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) Npp[i + j * 3] = N[i + j * nx];
            for (int j = 0; j < nAmb; j++) Npa[i + j * 3] = N[i + (3 + j) * nx];
        }
        for (int i = 0; i < nAmb; i++)
            for (int j = 0; j < nAmb; j++)
                Naa[i + j * nAmb] = N[(3 + i) + (3 + j) * nx];

        // Block-diagonal NaaInv
        double[] NaaInv = new double[nAmb * nAmb];
        for (List<Integer> comp : ambComponents) {
            int cn = comp.size();
            double[] Nsub = new double[cn * cn];
            for (int ci = 0; ci < cn; ci++)
                for (int cj = 0; cj < cn; cj++)
                    Nsub[ci + cj * cn] = Naa[comp.get(ci) + comp.get(cj) * nAmb];
            if (MatrixUtil.matinv(Nsub, cn) != 0) return false;
            for (int ci = 0; ci < cn; ci++)
                for (int cj = 0; cj < cn; cj++)
                    NaaInv[comp.get(ci) + comp.get(cj) * nAmb] = Nsub[ci + cj * cn];
        }

        // tmp3a = Npa * NaaInv (3 x nAmb)
        double[] tmp3a = new double[3 * nAmb];
        MatrixUtil.matmul("NN", 3, nAmb, nAmb, Npa, NaaInv, tmp3a);

        // Nred = Npp - tmp3a * Npa'
        double[] Nred = new double[9];
        System.arraycopy(Npp, 0, Nred, 0, 9);
        MatrixUtil.matmul("NT", 3, 3, nAmb, -1.0, tmp3a, Npa, 1.0, Nred);

        // Qpp = Nred^-1
        System.arraycopy(Nred, 0, Qpp, 0, 9);
        if (MatrixUtil.matinv(Qpp, 3) != 0) return false;

        // Qpa = -Qpp * Npa * NaaInv = -Qpp * tmp3a
        MatrixUtil.matmul("NN", 3, nAmb, 3, Qpp, tmp3a, Qpa);
        for (int i = 0; i < 3 * nAmb; i++) Qpa[i] = -Qpa[i];

        // Qaa = NaaInv + tmp3a' * Qpp * tmp3a
        System.arraycopy(NaaInv, 0, Qaa, 0, nAmb * nAmb);
        double[] QppU = new double[3 * nAmb];
        MatrixUtil.matmul("NN", 3, nAmb, 3, Qpp, tmp3a, QppU);
        MatrixUtil.matmul("TN", nAmb, nAmb, 3, 1.0, tmp3a, QppU, 1.0, Qaa);

        return true;
    }
}
