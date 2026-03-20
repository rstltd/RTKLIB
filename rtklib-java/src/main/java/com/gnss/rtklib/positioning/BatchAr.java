/*------------------------------------------------------------------------------
 * BatchAr.java : Ambiguity resolution for Batch Least Squares solver
 *
 *          Copyright (C) 2026
 *
 * Licensed under BSD 2-clause license.
 *-----------------------------------------------------------------------------*/
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
 * Ambiguity resolution helpers for the Batch Least Squares solver.
 * <p>
 * Contains union-find grouping, connected-component AR, WL/NL two-step
 * integer ambiguity resolution with PAR fallback, and post-fix validation.
 */
final class BatchAr {

    private BatchAr() {}

    private static final double ADOP_THRESHOLD = 0.15; // cycles

    /** Get post-fix phase RMS threshold from options (thresar[7]), default 0.015m. */
    private static double getPfThreshold(ProcessingOptions opt, double epochInterval) {
        double thres1hz = opt.thresar[7] > 0 ? opt.thresar[7] : 0.015;
        return epochInterval <= 1.5 ? thres1hz : 0.05;
    }

    /**
     * Compute ADOP (Ambiguity Dilution of Precision) for a covariance matrix.
     * ADOP = exp(sum(log(D[i])) / (2*n)) where D[] from LD decomposition.
     * High ADOP (> 0.15 cycles) indicates poor AR geometry.
     */
    static double computeAdop(double[] Q, int n) {
        double[] L = new double[n * n];
        double[] D = new double[n];
        double[] Qclone = Q.clone();
        if (Lambda.LD(n, Qclone, L, D) != 0) return Double.MAX_VALUE;
        double sumLogD = 0;
        for (int i = 0; i < n; i++) {
            if (D[i] <= 0) return Double.MAX_VALUE;
            sumLogD += Math.log(D[i]);
        }
        return Math.exp(sumLogD / (2.0 * n));
    }

    // ---------------------------------------------------------------
    // ADOP-minimized PAR search
    // ---------------------------------------------------------------

    /** Result of PAR search. */
    static class ParResult {
        final double[] fixedValues; // LAMBDA best candidates (length = active.size)
        final List<Integer> activeIdx; // indices into original a[]/Q[] that were fixed
        final float ratio;

        ParResult(double[] f, List<Integer> idx, float r) {
            fixedValues = f;
            activeIdx = idx;
            ratio = r;
        }

        boolean success() { return fixedValues != null; }
    }

    /**
     * PAR subset selection with ADOP-minimized greedy dropout.
     * <p>
     * Instead of LD-sorting and peeling the worst element, at each reduction
     * step this method tries removing each candidate ambiguity and picks the
     * removal that yields the lowest ADOP for the remaining subset. This
     * accounts for the full correlation structure rather than ranking each
     * ambiguity independently.
     * <p>
     * Algorithm:
     * <ol>
     *   <li>Start with full set, try LAMBDA → accept if ratio passes</li>
     *   <li>For each element i, compute ADOP of subset without i</li>
     *   <li>Remove element with lowest resulting ADOP</li>
     *   <li>Try LAMBDA on reduced set → accept if ratio passes</li>
     *   <li>Repeat until minSize reached</li>
     * </ol>
     *
     * @param a float ambiguity values (length n)
     * @param Q covariance matrix (n x n, column-major)
     * @param n number of ambiguities
     * @param minRatio base ratio threshold (opt.thresar[0])
     * @param minSize minimum subset size
     * @return ParResult with fixed values and indices, or failed (fixedValues=null)
     */
    static ParResult adopParSearch(double[] a, double[] Q, int n,
                                    double minRatio, int minSize) {
        List<Integer> active = new ArrayList<>(n);
        for (int i = 0; i < n; i++) active.add(i);

        float bestRatio = 0;

        while (active.size() >= minSize) {
            int m = active.size();

            // Extract sub-arrays for active set
            double[] aSub = new double[m];
            double[] QSub = new double[m * m];
            for (int i = 0; i < m; i++) {
                aSub[i] = a[active.get(i)];
                for (int j = 0; j < m; j++)
                    QSub[i + j * m] = Q[active.get(i) + active.get(j) * n];
            }

            // Try LAMBDA
            double[] F = new double[m * 2];
            double[] s = new double[2];
            int info = Lambda.lambda(m, 2, aSub, QSub, F, s);
            if (info == 0 && s[0] > 0) {
                float r = (float) (s[1] / s[0]);
                if (r > 999.9f) r = 999.9f;

                float thres = (float) Math.max(minRatio,
                        Rtkpos.computeAdaptiveArThreshold(m, minRatio));
                if (r >= thres) {
                    return new ParResult(F, new ArrayList<>(active), r);
                }
                if (r > bestRatio) bestRatio = r;
            }

            if (m <= minSize) break;

            // Greedy ADOP-minimized removal:
            // try removing each element, pick the one yielding lowest ADOP
            int bestRemove = -1;
            double bestAdop = Double.MAX_VALUE;

            for (int rem = 0; rem < m; rem++) {
                int m1 = m - 1;
                double[] QRem = new double[m1 * m1];
                int ri = 0;
                for (int i = 0; i < m; i++) {
                    if (i == rem) continue;
                    int rj = 0;
                    for (int j = 0; j < m; j++) {
                        if (j == rem) continue;
                        QRem[ri + rj * m1] = QSub[i + j * m];
                        rj++;
                    }
                    ri++;
                }
                double adop = computeAdop(QRem, m1);
                if (adop < bestAdop) {
                    bestAdop = adop;
                    bestRemove = rem;
                }
            }

            if (bestRemove < 0) break; // shouldn't happen
            active.remove(bestRemove);
        }

        return new ParResult(null, null, bestRatio);
    }

    // ---------------------------------------------------------------
    // Union-Find helpers
    // ---------------------------------------------------------------

    /** Union-Find: find root with path compression. */
    static int ufFind(int[] parent, int i) {
        while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; }
        return i;
    }

    /** Union-Find: union two elements. */
    static void ufUnion(int[] parent, int a, int b) {
        int ra = ufFind(parent, a), rb = ufFind(parent, b);
        if (ra != rb) parent[ra] = rb;
    }

    // ---------------------------------------------------------------
    // Connected Component grouping
    // ---------------------------------------------------------------

    /**
     * Group ambiguities into connected components.
     * Two ambiguities are connected if they can appear in the same DD
     * observation: same ref satellite, same frequency, AND overlapping epochs.
     * This prevents chain-merging across systems or across non-overlapping
     * ref-sat periods in long sessions.
     */
    static List<List<Integer>> findAmbiguityComponents(List<BatchPreprocess.AmbParam> ambParams) {
        int n = ambParams.size();
        int[] parent = new int[n];
        for (int i = 0; i < n; i++) parent[i] = i;

        for (int i = 0; i < n; i++) {
            BatchPreprocess.AmbParam a = ambParams.get(i);
            for (int j = i + 1; j < n; j++) {
                BatchPreprocess.AmbParam b = ambParams.get(j);
                // Must share same ref sat and same freq to be in the same DD group
                if (a.refSat != b.refSat || a.freq != b.freq) continue;
                // Must have overlapping epoch ranges
                if (a.startEpoch <= b.endEpoch && b.startEpoch <= a.endEpoch) {
                    ufUnion(parent, i, j);
                }
            }
        }

        Map<Integer, List<Integer>> groups = new HashMap<>();
        for (int i = 0; i < n; i++) {
            groups.computeIfAbsent(ufFind(parent, i), k -> new ArrayList<>()).add(i);
        }
        return new ArrayList<>(groups.values());
    }

    /**
     * Group AR-eligible indices into connected components.
     * Uses the same temporal overlap criterion on the underlying AmbParam.
     */
    static List<List<Integer>> findArComponents(List<Integer> arIdx,
                                                 List<BatchPreprocess.AmbParam> ambParams) {
        int n = arIdx.size();
        int[] parent = new int[n];
        for (int i = 0; i < n; i++) parent[i] = i;

        for (int i = 0; i < n; i++) {
            BatchPreprocess.AmbParam a = ambParams.get(arIdx.get(i));
            for (int j = i + 1; j < n; j++) {
                BatchPreprocess.AmbParam b = ambParams.get(arIdx.get(j));
                // Same DD group: same ref sat and same freq
                if (a.refSat != b.refSat || a.freq != b.freq) continue;
                if (a.startEpoch <= b.endEpoch && b.startEpoch <= a.endEpoch) {
                    ufUnion(parent, i, j);
                }
            }
        }

        Map<Integer, List<Integer>> groups = new HashMap<>();
        for (int i = 0; i < n; i++) {
            groups.computeIfAbsent(ufFind(parent, i), k -> new ArrayList<>()).add(i);
        }
        return new ArrayList<>(groups.values());
    }

    // ---------------------------------------------------------------
    // General multi-frequency WL/NL two-step AR with PAR fallback
    // ---------------------------------------------------------------

    /**
     * Run WL/NL AR on a component subset of ambiguities.
     * <p>
     * Supports any dual-frequency (L1+L2, L1+L5) or triple-frequency (L1+L2+L5)
     * combination. Groups ambiguities by (refSat, sat), pairs primary frequency
     * (freq=0 / L1) with each available secondary frequency.
     * <ul>
     *   <li>WL step: fixes N_primary - N_secondary for each pair</li>
     *   <li>NL step: fixes N_primary (conditional on WL constraints)</li>
     *   <li>Recovery: N_secondary = N_primary - WL_fix</li>
     * </ul>
     *
     * @param aComp float ambiguity values for this component
     * @param QComp covariance matrix for this component (cn x cn)
     * @param cn    size of component
     * @param comp  indices into arIdx for this component
     * @param ambParams all ambiguity parameters
     * @param arIdx AR-eligible indices into ambParams
     * @param opt   processing options
     * @param ratioOut if non-null, ratioOut[0] is set to the AR ratio achieved
     * @return fixed integers for each element in comp (Integer.MIN_VALUE = unfixed),
     *         or null if AR failed completely
     */
    static int[] runComponentAr(double[] aComp, double[] QComp, int cn,
                                 List<Integer> comp, List<BatchPreprocess.AmbParam> ambParams,
                                 List<Integer> arIdx, ProcessingOptions opt,
                                 float[] ratioOut, boolean shortWindow) {

        // Quality gate: reject component if float ambiguity quality is poor
        double fracAvgThres = shortWindow ? 0.30 : 0.25;
        double sumAbsFrac = 0;
        int badFracCount = 0;
        for (int i = 0; i < cn; i++) {
            double frac = Math.abs(aComp[i] - Math.round(aComp[i]));
            sumAbsFrac += frac;
            if (frac > 0.35) badFracCount++;
        }
        if (sumAbsFrac / cn > fracAvgThres || badFracCount > cn * 0.3) {
            return null;
        }

        // Step 1: Group ambiguities by (refSat, sat) and find WL pairs
        // Key: (refSat << 20) | sat → Map<freq, compIdx>
        Map<Long, Map<Integer, Integer>> satGroups = new java.util.LinkedHashMap<>();
        for (int i = 0; i < cn; i++) {
            BatchPreprocess.AmbParam ap = ambParams.get(arIdx.get(comp.get(i)));
            long key = ((long) ap.refSat << 20) | ap.sat;
            satGroups.computeIfAbsent(key, k -> new HashMap<>()).put(ap.freq, i);
        }

        // Build WL pairs: pair primary (freq=0) with each secondary frequency
        // wlPairs[w] = {primaryCompIdx, secondaryCompIdx}
        // wlPairOwner[w] = index into nlPrimaries (which satellite this WL belongs to)
        List<int[]> wlPairs = new ArrayList<>();
        List<Integer> wlPairOwner = new ArrayList<>(); // which NL primary owns this WL pair
        List<Integer> nlPrimaries = new ArrayList<>();  // comp indices of primaries with WL pairs
        List<Integer> singleIdx = new ArrayList<>();    // comp indices of unpaired primaries
        boolean[] used = new boolean[cn];

        int satGroupIdx = 0;
        for (Map<Integer, Integer> freqMap : satGroups.values()) {
            Integer primaryIdx = freqMap.get(0); // freq=0 is primary
            if (primaryIdx == null) { satGroupIdx++; continue; }

            BatchPreprocess.AmbParam apPrimary = ambParams.get(arIdx.get(comp.get(primaryIdx)));
            boolean hasPair = false;

            // Try all secondary frequencies (1, 2, ...) in order
            for (Map.Entry<Integer, Integer> e : freqMap.entrySet()) {
                int secFreq = e.getKey();
                if (secFreq == 0) continue; // skip primary
                int secIdx = e.getValue();

                // Check temporal overlap
                BatchPreprocess.AmbParam apSec = ambParams.get(arIdx.get(comp.get(secIdx)));
                if (apPrimary.startEpoch > apSec.endEpoch || apPrimary.endEpoch < apSec.startEpoch) continue;

                wlPairs.add(new int[]{primaryIdx, secIdx});
                wlPairOwner.add(nlPrimaries.size()); // will be set after adding primary
                used[secIdx] = true;
                hasPair = true;
            }

            if (hasPair) {
                // Fix owner indices — they all point to this primary's NL index
                int nlIdx = nlPrimaries.size();
                for (int w = wlPairOwner.size() - 1; w >= 0 && wlPairOwner.get(w) == nlIdx; w--) {
                    // already correct
                }
                nlPrimaries.add(primaryIdx);
                used[primaryIdx] = true;
            } else {
                singleIdx.add(primaryIdx);
                used[primaryIdx] = true;
            }
            satGroupIdx++;
        }

        int nWL = wlPairs.size();

        if (nWL >= 4) {
            // Step 2: WL fix — each WL = N_primary - N_secondary
            double[] aWL = new double[nWL];
            double[] QWL = new double[nWL * nWL];

            for (int i = 0; i < nWL; i++) {
                int pi = wlPairs.get(i)[0], si = wlPairs.get(i)[1];
                aWL[i] = aComp[pi] - aComp[si];
            }

            for (int i = 0; i < nWL; i++) {
                int pi = wlPairs.get(i)[0], si = wlPairs.get(i)[1];
                for (int j = 0; j < nWL; j++) {
                    int pj = wlPairs.get(j)[0], sj = wlPairs.get(j)[1];
                    QWL[i + j * nWL] = QComp[pi + pj * cn] + QComp[si + sj * cn]
                                     - QComp[pi + sj * cn] - QComp[si + pj * cn];
                }
            }

            double[] FWL = new double[nWL * 2];
            double[] sWL = new double[2];
            int infoWL = Lambda.lambda(nWL, 2, aWL, QWL, FWL, sWL);

            float ratioWL = 0;
            if (infoWL == 0 && sWL[0] > 0) {
                ratioWL = (float)(sWL[1] / sWL[0]);
                if (ratioWL > 999.9f) ratioWL = 999.9f;
            }

            if (ratioWL >= 2.0) {
                // Apply WL constraint to get conditional covariance
                double[] QWLinv = QWL.clone();
                boolean wlConstraintOk = (MatrixUtil.matinv(QWLinv, nWL) == 0);

                double[] QaaC = QComp.clone();
                if (wlConstraintOk) {
                    // T matrix: WL = T * a, where T[w,a] = +1 for primary, -1 for secondary
                    double[] TQaa = new double[nWL * cn];
                    for (int w = 0; w < nWL; w++) {
                        int pw = wlPairs.get(w)[0], sw = wlPairs.get(w)[1];
                        for (int a = 0; a < cn; a++) {
                            TQaa[w + a * nWL] = QComp[pw + a * cn] - QComp[sw + a * cn];
                        }
                    }
                    double[] QWLinvTQaa = new double[nWL * cn];
                    MatrixUtil.matmul("NN", nWL, cn, nWL, QWLinv, TQaa, QWLinvTQaa);
                    MatrixUtil.matmul("TN", cn, cn, nWL, -1.0, TQaa, QWLinvTQaa, 1.0, QaaC);
                }

                // Step 3: NL fix — solve for primary ambiguities + unpaired singles
                int nNL = nlPrimaries.size() + singleIdx.size();
                double[] aNL = new double[nNL];
                double[] QNL = new double[nNL * nNL];

                // NL values: just the primary ambiguity (not sum)
                int nP = nlPrimaries.size();
                for (int i = 0; i < nP; i++) {
                    aNL[i] = aComp[nlPrimaries.get(i)];
                }
                for (int i = 0; i < singleIdx.size(); i++) {
                    aNL[nP + i] = aComp[singleIdx.get(i)];
                }

                // NL covariance from conditional covariance QaaC
                for (int i = 0; i < nNL; i++) {
                    int ci = i < nP ? nlPrimaries.get(i) : singleIdx.get(i - nP);
                    for (int j = 0; j < nNL; j++) {
                        int cj = j < nP ? nlPrimaries.get(j) : singleIdx.get(j - nP);
                        QNL[i + j * nNL] = QaaC[ci + cj * cn];
                    }
                }

                // PAR with ADOP-minimized greedy dropout
                int minNL = Math.max(4, opt.minfixsats - 1);
                ParResult nlResult = adopParSearch(aNL, QNL, nNL,
                        opt.thresar[0], minNL);

                if (nlResult.success()) {
                    int bestNfix = nlResult.activeIdx.size();
                    int[] result = new int[cn];
                    Arrays.fill(result, Integer.MIN_VALUE);

                    // Build set of fixed NL indices for recovery
                    Map<Integer, Integer> nlFixMap = new HashMap<>(); // nlIdx → fixedValue
                    for (int i = 0; i < bestNfix; i++) {
                        int nlIdx = nlResult.activeIdx.get(i);
                        int nlFix = (int) Math.round(nlResult.fixedValues[i]);
                        nlFixMap.put(nlIdx, nlFix);

                        // Set primary ambiguity directly
                        if (nlIdx < nP) {
                            result[nlPrimaries.get(nlIdx)] = nlFix;
                        } else {
                            result[singleIdx.get(nlIdx - nP)] = nlFix;
                        }
                    }

                    // Recover secondary ambiguities: N_sec = N_primary - WL_fix
                    for (int w = 0; w < nWL; w++) {
                        int ownerNlIdx = wlPairOwner.get(w);
                        Integer primaryFix = nlFixMap.get(ownerNlIdx);
                        if (primaryFix == null) continue; // primary not fixed

                        int wlFix = (int) Math.round(FWL[w]);
                        int secCompIdx = wlPairs.get(w)[1];
                        result[secCompIdx] = primaryFix - wlFix;
                    }

                    if (ratioOut != null) ratioOut[0] = nlResult.ratio;
                    return result;
                }
            }
        }

        return null; // WL/NL failed — fall through to global PAR
    }

    // ---------------------------------------------------------------
    // Post-fix validation
    // ---------------------------------------------------------------

    private static final double WTEST_THRESHOLD = 4.0;
    private static final double WTEST_BAD_RATIO = 0.05; // >5% bad → reject

    /**
     * Post-fix validation using both RMS and per-observation w-test.
     * <p>
     * RMS check: correct fix ~0.005m, wrong fix >0.05m.
     * W-test: normalized residual |v_i/sqrt(var_i)| > 4.0 is "bad".
     * If >5% observations are bad, reject the fix.
     * Either check failing causes rejection.
     *
     * @return true if fix passes validation, false to reject
     */
    static boolean validatePostFix(
            List<BatchPreprocess.EpochData> epochs, List<BatchPreprocess.AmbParam> ambParams,
            Navigation nav, ProcessingOptions opt,
            double[] pos, double[] fixedAmb, int nf,
            int[][] refSatMap, java.util.Set<Integer> fixedIndices,
            double pfThreshold, double sigma0sq) {
        double sumSq = 0;
        int count = 0;
        int badCount = 0;
        double[] dr = new double[3];
        double bl = Rtkpos.baseline(pos, opt.rb, dr);

        for (int ep = 0; ep < epochs.size(); ep++) {
            BatchPreprocess.EpochData ed = epochs.get(ep);
            if (ed.ns <= 0) continue;

            double[] yRov = new double[nf * 2 * ed.nu];
            double[] eRov = new double[3 * ed.nu];
            double[] azelRov = new double[2 * ed.nu];
            double[] freqRov = new double[nf * ed.nu];
            ObsData[] obsRov = new ObsData[ed.nu];
            System.arraycopy(ed.obs, 0, obsRov, 0, ed.nu);
            if (Rtkpos.zdres(0, obsRov, ed.nu, ed.rs, ed.dts, ed.var, ed.svh,
                              nav, pos, opt, yRov, eRov, azelRov, freqRov) == 0) continue;

            ObsData[] obsBase = new ObsData[ed.nr];
            System.arraycopy(ed.obs, ed.nu, obsBase, 0, ed.nr);
            double[] rsB = new double[6 * ed.nr]; System.arraycopy(ed.rs, ed.nu * 6, rsB, 0, 6 * ed.nr);
            double[] dtB = new double[2 * ed.nr]; System.arraycopy(ed.dts, ed.nu * 2, dtB, 0, 2 * ed.nr);
            double[] vrB = new double[ed.nr]; System.arraycopy(ed.var, ed.nu, vrB, 0, ed.nr);
            int[] svB = new int[ed.nr]; System.arraycopy(ed.svh, ed.nu, svB, 0, ed.nr);
            double[] yBase = new double[nf * 2 * ed.nr];
            double[] eBase = new double[3 * ed.nr];
            double[] azelBase = new double[2 * ed.nr];
            double[] freqBase = new double[nf * ed.nr];
            if (Rtkpos.zdres(1, obsBase, ed.nr, rsB, dtB, vrB, svB,
                              nav, opt.rb, opt, yBase, eBase, azelBase, freqBase) == 0) continue;

            List<BatchNormalEq.DdObs> ddObs = BatchNormalEq.makeDdObs(ed, pos, opt, nav, nf,
                    yRov, eRov, azelRov, freqRov,
                    yBase, azelBase, freqBase,
                    ambParams, null, fixedAmb, ep, bl, refSatMap);

            for (BatchNormalEq.DdObs dd : ddObs) {
                if (dd.isPhase && dd.ddAmbIdx >= 0
                        && fixedIndices.contains(dd.ddAmbIdx)) {
                    sumSq += dd.v * dd.v;
                    count++;
                    // Per-obs w-test: normalized residual (scaled by σ₀² when > 1)
                    if (dd.var > 0) {
                        double s0 = Math.max(1.0, sigma0sq);
                        double wtest = Math.abs(dd.v) / Math.sqrt(s0 * dd.var);
                        if (wtest > WTEST_THRESHOLD) badCount++;
                    }
                }
            }
        }

        if (count == 0) return false;

        double rms = Math.sqrt(sumSq / count);
        boolean rmsOk = rms <= pfThreshold;
        boolean wtestOk = (double) badCount / count <= WTEST_BAD_RATIO;

        return rmsOk && wtestOk;
    }

    /**
     * Group AR-eligible ambiguities into connected components,
     * merging QZS/SBS into GPS. QZS and GPS share the same L1/L5 signal
     * structure with zero ISB, so their DD ambiguities are integer-valued
     * when using a GPS ref sat. This boosts AR geometry for short windows.
     */
    static List<List<Integer>> findArComponentsMerged(List<Integer> arIdx,
                                                       List<BatchPreprocess.AmbParam> ambParams) {
        int n = arIdx.size();
        int[] parent = new int[n];
        for (int i = 0; i < n; i++) parent[i] = i;

        for (int i = 0; i < n; i++) {
            BatchPreprocess.AmbParam a = ambParams.get(arIdx.get(i));
            int sysA = SatelliteUtil.satsys(a.sat)[0];
            int sysRefA = SatelliteUtil.satsys(a.refSat)[0];
            for (int j = i + 1; j < n; j++) {
                BatchPreprocess.AmbParam b = ambParams.get(arIdx.get(j));
                // Must share same freq
                if (a.freq != b.freq) continue;
                // Must have overlapping epochs
                if (a.startEpoch > b.endEpoch || b.startEpoch > a.endEpoch) continue;
                // Same ref sat → same component (standard case)
                if (a.refSat == b.refSat) {
                    ufUnion(parent, i, j);
                    continue;
                }
                // Merge QZS/SBS with GPS: if both ref sats are GPS (or one is GPS
                // and the sat is QZS/SBS), they share integer ambiguity space
                int sysRefB = SatelliteUtil.satsys(b.refSat)[0];
                int sysB = SatelliteUtil.satsys(b.sat)[0];
                boolean gpsGroupA = (sysRefA == SYS_GPS) &&
                        (sysA == SYS_GPS || sysA == SYS_QZS || sysA == SYS_SBS);
                boolean gpsGroupB = (sysRefB == SYS_GPS) &&
                        (sysB == SYS_GPS || sysB == SYS_QZS || sysB == SYS_SBS);
                if (gpsGroupA && gpsGroupB) {
                    ufUnion(parent, i, j);
                }
            }
        }

        Map<Integer, List<Integer>> groups = new HashMap<>();
        for (int i = 0; i < n; i++) {
            groups.computeIfAbsent(ufFind(parent, i), k -> new ArrayList<>()).add(i);
        }
        return new ArrayList<>(groups.values());
    }

    // ---------------------------------------------------------------
    // resolveAmbiguities: top-level AR orchestration
    // ---------------------------------------------------------------

    static BatchSolver.BatchResult resolveAmbiguities(
            double[] pos, float[] qr, double[] ambValues,
            List<BatchPreprocess.AmbParam> ambParams,
            double[] Qpp, double[] Qpa, double[] Qaa,
            List<BatchPreprocess.EpochData> epochs,
            Navigation nav, ProcessingOptions opt, int nf,
            int[][] refSatMap, int ns, double epochInterval,
            double sigma0sq) {

        int nAmb = ambParams.size();
        boolean shortWindow = epochs.size() < 3600;

        // Build AR-eligible list (skip GLO FDMA + poorly determined)
        double[] diagArr = new double[nAmb];
        for (int j = 0; j < nAmb; j++) {
            diagArr[j] = Qaa[j + j * nAmb];
        }
        double[] sorted = diagArr.clone();
        java.util.Arrays.sort(sorted);
        double medianDiag = sorted[Math.max(0, nAmb / 2)];

        List<Integer> arIdx = new ArrayList<>();
        for (int j = 0; j < nAmb; j++) {
            BatchPreprocess.AmbParam ap = ambParams.get(j);
            int sys = SatelliteUtil.satsys(ap.sat)[0];
            if (sys == SYS_GLO) continue;
            if (medianDiag > 0 && diagArr[j] > medianDiag * 100) continue;
            arIdx.add(j);
        }

        int nAR = arIdx.size();
        if (nAR < 1) {
            return new BatchSolver.BatchResult(pos, qr, SOLQ_FLOAT, 0, ns,
                    epochs.size(), nAmb, ambValues, ambParams, sigma0sq);
        }

        // Extract AR-eligible sub-blocks
        double[] aAR = new double[nAR];
        double[] QaaAR = new double[nAR * nAR];
        for (int i = 0; i < nAR; i++) {
            aAR[i] = ambValues[arIdx.get(i)];
            for (int j = 0; j < nAR; j++) {
                QaaAR[i + j * nAR] = Qaa[arIdx.get(i) + arIdx.get(j) * nAmb];
            }
        }

        // Merge QZS/SBS into GPS component for AR:
        // QZS and GPS share the same L1/L5 signal structure with zero ISB,
        // so their DD ambiguities are integer-valued relative to each other.
        // Without merging, QZS forms a tiny component (1-2 sats) that gets skipped.
        List<List<Integer>> arComponents = findArComponentsMerged(arIdx, ambParams);
        int[] fixedOriginal = new int[nAR];
        Arrays.fill(fixedOriginal, Integer.MIN_VALUE);
        float ratio = 999.9f;

        // Adaptive minimum component size for short windows
        int minCompSize = shortWindow ? 4 : 8;

        int compFixed = 0;
        for (List<Integer> comp : arComponents) {
            if (comp.size() < minCompSize) continue;

            int cn = comp.size();
            double[] aComp = new double[cn];
            double[] QComp = new double[cn * cn];
            for (int i = 0; i < cn; i++) {
                aComp[i] = aAR[comp.get(i)];
                for (int j = 0; j < cn; j++)
                    QComp[i + j * cn] = QaaAR[comp.get(i) + comp.get(j) * nAR];
            }

            // ADOP pre-screening: skip component if geometry is too poor
            double adop = computeAdop(QComp, cn);
            if (adop > ADOP_THRESHOLD) continue;

            float[] compRatioOut = {0};
            int[] compFix = runComponentAr(aComp, QComp, cn, comp, ambParams,
                    arIdx, opt, compRatioOut, shortWindow);

            if (compFix != null) {
                compFixed++;
                for (int i = 0; i < cn; i++) {
                    if (compFix[i] != Integer.MIN_VALUE) {
                        fixedOriginal[comp.get(i)] = compFix[i];
                    }
                }
                if (compRatioOut[0] > 0 && compRatioOut[0] < ratio) {
                    ratio = compRatioOut[0];
                }
            }
        }

        // Per-component post-fix validation (relaxed for short windows)
        double pfThreshold = getPfThreshold(opt, epochInterval);
        double scaledPfThreshold = pfThreshold * Math.max(1.0, Math.sqrt(sigma0sq));
        {
            double[] valAmb = new double[nAmb];
            System.arraycopy(ambValues, 0, valAmb, 0, nAmb);
            for (int i = 0; i < nAR; i++) {
                if (fixedOriginal[i] != Integer.MIN_VALUE) {
                    valAmb[arIdx.get(i)] = fixedOriginal[i];
                }
            }

            for (List<Integer> comp : arComponents) {
                java.util.Set<Integer> compFixedSet = new java.util.HashSet<>();
                for (int i : comp) {
                    if (fixedOriginal[i] != Integer.MIN_VALUE) {
                        compFixedSet.add(arIdx.get(i));
                    }
                }
                if (compFixedSet.isEmpty()) continue;

                boolean compOk = validatePostFix(epochs, ambParams, nav, opt,
                        pos, valAmb, nf, refSatMap, compFixedSet,
                        scaledPfThreshold, sigma0sq);

                if (!compOk) {
                    for (int i : comp) {
                        if (fixedOriginal[i] != Integer.MIN_VALUE) {
                            fixedOriginal[i] = Integer.MIN_VALUE;
                            valAmb[arIdx.get(i)] = ambValues[arIdx.get(i)];
                        }
                    }
                }
            }
        }

        int totalFixed = 0;
        for (int i = 0; i < nAR; i++) {
            if (fixedOriginal[i] != Integer.MIN_VALUE) totalFixed++;
        }
        if (totalFixed < minCompSize) {
            Arrays.fill(fixedOriginal, Integer.MIN_VALUE);
            totalFixed = 0;
        }
        if (totalFixed == 0) fixedOriginal = null;

        // Compute fixed solution from component AR
        if (fixedOriginal != null) {
            BatchSolver.BatchResult fixResult = computeFixedSolution(
                    pos, qr, ambValues, ambParams, arIdx, fixedOriginal,
                    aAR, QaaAR, Qpp, Qpa, Qaa,
                    epochs, nav, opt, nf, refSatMap, ns, nAmb, nAR,
                    ratio, scaledPfThreshold, epochInterval, sigma0sq);
            if (fixResult != null) return fixResult;
            fixedOriginal = null;
        }

        // Fallback: single-step PAR
        if (fixedOriginal == null) {
            return parFallback(pos, qr, ambValues, ambParams, arIdx,
                    aAR, QaaAR, Qpp, Qpa, Qaa,
                    epochs, nav, opt, nf, refSatMap, ns, nAmb, nAR,
                    ratio, epochInterval, shortWindow, sigma0sq);
        }

        return new BatchSolver.BatchResult(pos, qr, SOLQ_FLOAT, ratio, ns,
                epochs.size(), nAmb, ambValues, ambParams, sigma0sq);
    }

    private static BatchSolver.BatchResult computeFixedSolution(
            double[] pos, float[] qr, double[] ambValues,
            List<BatchPreprocess.AmbParam> ambParams,
            List<Integer> arIdx, int[] fixedOriginal,
            double[] aAR, double[] QaaAR, double[] Qpp, double[] Qpa, double[] Qaa,
            List<BatchPreprocess.EpochData> epochs,
            Navigation nav, ProcessingOptions opt, int nf, int[][] refSatMap,
            int ns, int nAmb, int nAR, float ratio,
            double pfThreshold, double epochInterval, double sigma0sq) {

        List<Integer> fixedArSubset = new ArrayList<>();
        for (int i = 0; i < nAR; i++) {
            if (fixedOriginal[i] != Integer.MIN_VALUE) fixedArSubset.add(i);
        }
        int bestNfix = fixedArSubset.size();

        // Extract cross-covariance for fixed subset
        double[] Qpa_ar = new double[3 * bestNfix];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < bestNfix; j++)
                Qpa_ar[i + j * 3] = Qpa[i + arIdx.get(fixedArSubset.get(j)) * 3];

        double[] QaaFixSub = new double[bestNfix * bestNfix];
        for (int i = 0; i < bestNfix; i++)
            for (int j = 0; j < bestNfix; j++)
                QaaFixSub[i + j * bestNfix] = QaaAR[fixedArSubset.get(i) + fixedArSubset.get(j) * nAR];
        double[] QaaInvFix = QaaFixSub.clone();
        if (MatrixUtil.matinv(QaaInvFix, bestNfix) != 0) return null;

        // Fixed position: pos_fix = pos - Qpa * Qaa^-1 * (float - fixed)
        double[] da = new double[bestNfix];
        for (int i = 0; i < bestNfix; i++)
            da[i] = aAR[fixedArSubset.get(i)] - fixedOriginal[fixedArSubset.get(i)];

        double[] QaaInvDa = new double[bestNfix];
        MatrixUtil.matmul("NN", bestNfix, 1, bestNfix, QaaInvFix, da, QaaInvDa);

        double[] posFix = pos.clone();
        MatrixUtil.matmul("NN", 3, 1, bestNfix, -1.0, Qpa_ar, QaaInvDa, 1.0, posFix);

        // Fixed covariance: Q_fix = Qpp - Qpa * Qaa^-1 * Qpa'
        float[] qrFix = new float[6];
        double[] QpaQaaInv = new double[3 * bestNfix];
        MatrixUtil.matmul("NN", 3, bestNfix, bestNfix, Qpa_ar, QaaInvFix, QpaQaaInv);
        double[] Qfix = new double[9];
        System.arraycopy(Qpp, 0, Qfix, 0, 9);
        MatrixUtil.matmul("NT", 3, 3, bestNfix, -1.0, QpaQaaInv, Qpa_ar, 1.0, Qfix);
        qrFix[0] = (float) Qfix[0]; qrFix[1] = (float) Qfix[4]; qrFix[2] = (float) Qfix[8];
        qrFix[3] = (float) Qfix[1]; qrFix[4] = (float) Qfix[5]; qrFix[5] = (float) Qfix[2];

        // Post-fix validation
        double[] fixedAmb = new double[nAmb];
        System.arraycopy(ambValues, 0, fixedAmb, 0, nAmb);
        java.util.Set<Integer> fixedIndices = new java.util.HashSet<>();
        for (int idx : fixedArSubset) {
            int fullIdx = arIdx.get(idx);
            fixedAmb[fullIdx] = fixedOriginal[idx];
            fixedIndices.add(fullIdx);
        }

        boolean postFixOk = validatePostFix(epochs, ambParams, nav, opt,
                posFix, fixedAmb, nf, refSatMap, fixedIndices,
                pfThreshold, sigma0sq);

        if (postFixOk) {
            return new BatchSolver.BatchResult(posFix, qrFix, SOLQ_FIX, ratio, ns,
                    epochs.size(), nAmb, ambValues, ambParams, sigma0sq);
        }
        return null;
    }

    private static BatchSolver.BatchResult parFallback(
            double[] pos, float[] qr, double[] ambValues,
            List<BatchPreprocess.AmbParam> ambParams,
            List<Integer> arIdx, double[] aAR, double[] QaaAR,
            double[] Qpp, double[] Qpa, double[] Qaa,
            List<BatchPreprocess.EpochData> epochs,
            Navigation nav, ProcessingOptions opt, int nf, int[][] refSatMap,
            int ns, int nAmb, int nAR, float ratio, double epochInterval,
            boolean shortWindow, double sigma0sq) {

        int minAR = Math.max(4, opt.minfixsats - 1);
        ParResult parResult = adopParSearch(aAR, QaaAR, nAR, opt.thresar[0], minAR);

        if (!parResult.success()) {
            return new BatchSolver.BatchResult(pos, qr, SOLQ_FLOAT, parResult.ratio, ns,
                    epochs.size(), nAmb, ambValues, ambParams, sigma0sq);
        }

        ratio = parResult.ratio;
        List<Integer> fixedArIdx = parResult.activeIdx;
        double[] bestF = parResult.fixedValues;
        int bestNfix = fixedArIdx.size();

        double[] Qpa_ar = new double[3 * bestNfix];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < bestNfix; j++)
                Qpa_ar[i + j * 3] = Qpa[i + arIdx.get(fixedArIdx.get(j)) * 3];

        double[] QaaFixSub = new double[bestNfix * bestNfix];
        for (int i = 0; i < bestNfix; i++)
            for (int j = 0; j < bestNfix; j++)
                QaaFixSub[i + j * bestNfix] = QaaAR[fixedArIdx.get(i) + fixedArIdx.get(j) * nAR];
        double[] QaaInvFix = QaaFixSub.clone();
        if (MatrixUtil.matinv(QaaInvFix, bestNfix) != 0) {
            return new BatchSolver.BatchResult(pos, qr, SOLQ_FLOAT, ratio, ns,
                    epochs.size(), nAmb, ambValues, ambParams, sigma0sq);
        }

        double[] da = new double[bestNfix];
        for (int i = 0; i < bestNfix; i++) da[i] = aAR[fixedArIdx.get(i)] - bestF[i];

        double[] QaaInvDa = new double[bestNfix];
        MatrixUtil.matmul("NN", bestNfix, 1, bestNfix, QaaInvFix, da, QaaInvDa);

        double[] posFix = pos.clone();
        MatrixUtil.matmul("NN", 3, 1, bestNfix, -1.0, Qpa_ar, QaaInvDa, 1.0, posFix);

        float[] qrFix = new float[6];
        double[] QpaQaaInv = new double[3 * bestNfix];
        MatrixUtil.matmul("NN", 3, bestNfix, bestNfix, Qpa_ar, QaaInvFix, QpaQaaInv);
        double[] Qfix = new double[9];
        System.arraycopy(Qpp, 0, Qfix, 0, 9);
        MatrixUtil.matmul("NT", 3, 3, bestNfix, -1.0, QpaQaaInv, Qpa_ar, 1.0, Qfix);
        qrFix[0] = (float) Qfix[0]; qrFix[1] = (float) Qfix[4]; qrFix[2] = (float) Qfix[8];
        qrFix[3] = (float) Qfix[1]; qrFix[4] = (float) Qfix[5]; qrFix[5] = (float) Qfix[2];

        int minFixForAccept = shortWindow ? Math.max(4, nAR / 4) : Math.max(8, nAR / 3);
        if (bestNfix < minFixForAccept) {
            return new BatchSolver.BatchResult(pos, qr, SOLQ_FLOAT, ratio, ns,
                    epochs.size(), nAmb, ambValues, ambParams, sigma0sq);
        }

        // Post-fix validation (scaled threshold and w-test)
        double[] fixedAmb = new double[nAmb];
        System.arraycopy(ambValues, 0, fixedAmb, 0, nAmb);
        java.util.Set<Integer> fixedIndices = new java.util.HashSet<>();
        for (int i = 0; i < bestNfix; i++) {
            int fullIdx = arIdx.get(fixedArIdx.get(i));
            fixedAmb[fullIdx] = bestF[i];
            fixedIndices.add(fullIdx);
        }

        double pfThreshold = getPfThreshold(opt, epochInterval) * Math.max(1.0, Math.sqrt(sigma0sq));
        boolean postFixOk = validatePostFix(epochs, ambParams, nav, opt,
                posFix, fixedAmb, nf, refSatMap, fixedIndices,
                pfThreshold, sigma0sq);

        if (!postFixOk) {
            return new BatchSolver.BatchResult(pos, qr, SOLQ_FLOAT, ratio, ns,
                    epochs.size(), nAmb, ambValues, ambParams, sigma0sq);
        }

        return new BatchSolver.BatchResult(posFix, qrFix, SOLQ_FIX, ratio, ns,
                epochs.size(), nAmb, ambValues, ambParams, sigma0sq);
    }
}
