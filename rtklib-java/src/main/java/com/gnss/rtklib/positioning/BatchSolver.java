/*------------------------------------------------------------------------------
 * BatchSolver.java : Batch Least Squares solver for RTK static positioning
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
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static com.gnss.rtklib.core.Constants.*;

/**
 * Batch Least Squares (BLS) solver for RTK static positioning.
 * <p>
 * Solves all epochs simultaneously via normal equations, yielding optimal
 * position and ambiguity estimates without EKF convergence delays.
 * Uses double-differenced (DD) parameterization for inherent full-rank
 * ambiguity states without datum constraints.
 * <p>
 * Pipeline is split across four classes:
 * <ul>
 *   <li>{@code BatchPreprocess} — epoch matching, ref sat, arc scanning</li>
 *   <li>{@code BatchNormalEq} — DD obs, normal equation accumulation, Schur solve</li>
 *   <li>{@code BatchAr} — connected-component WL/NL AR, PAR, post-fix validation</li>
 *   <li>{@code BatchSolver} (this class) — orchestration</li>
 * </ul>
 */
public final class BatchSolver {

    private BatchSolver() {}

    private static final int MAX_ITER = 10;
    private static final double CONV_THRESHOLD = 1e-4;
    private static final int MAX_CLEAN = 2;
    private static final double CLEAN_WTEST_THRESHOLD = 4.0;

    /** Result of BLS solve. */
    public static class BatchResult {
        public final double[] pos;
        public final float[] qr;
        public final int stat;
        public final float ratio;
        public final int ns;
        public final int nEpochs;
        public final int nAmb;
        public final double[] ambValues;
        public final List<BatchPreprocess.AmbParam> ambParams;
        public final double sigma0sq;

        BatchResult(double[] pos, float[] qr, int stat, float ratio, int ns,
                    int nEpochs, int nAmb) {
            this(pos, qr, stat, ratio, ns, nEpochs, nAmb, null, null, 1.0);
        }

        BatchResult(double[] pos, float[] qr, int stat, float ratio, int ns,
                    int nEpochs, int nAmb, double[] ambValues,
                    List<BatchPreprocess.AmbParam> ambParams, double sigma0sq) {
            this.pos = pos;
            this.qr = qr;
            this.stat = stat;
            this.ratio = ratio;
            this.ns = ns;
            this.nEpochs = nEpochs;
            this.nAmb = nAmb;
            this.ambValues = ambValues;
            this.ambParams = ambParams;
            this.sigma0sq = sigma0sq;
        }
    }

    /**
     * Solve RTK static using Batch Least Squares with DD parameterization.
     */
    public static BatchResult solve(List<List<ObsData>> roverEpochs,
                                     List<List<ObsData>> baseEpochs,
                                     Navigation nav, ProcessingOptions opt) {
        // 1. Initial position from SPP
        double[] pos = BatchPreprocess.sppPosition(roverEpochs, nav, opt);
        if (pos == null) {
            return new BatchResult(new double[3], new float[6], SOLQ_NONE, 0, 0, 0, 0);
        }

        // 2. Preprocess epochs
        List<BatchPreprocess.EpochData> epochs =
                BatchPreprocess.preprocessEpochs(roverEpochs, baseEpochs, nav, opt);
        if (epochs.isEmpty()) {
            return new BatchResult(pos, new float[6], SOLQ_NONE, 0, 0, 0, 0);
        }

        int nf = FilterState.NF(opt);

        // 3. Choose stable ref sat per (system, freq)
        int[][] refSatMap = BatchPreprocess.chooseRefSats(epochs, opt, nf);

        // 4. Scan DD ambiguities
        List<BatchPreprocess.AmbParam> ambParams =
                BatchPreprocess.scanDdAmbiguities(epochs, opt, nf, refSatMap);

        // Remove short segments
        double epochInterval = 30.0;
        if (epochs.size() >= 2) {
            double t0 = epochs.get(0).obs[0].time.time + epochs.get(0).obs[0].time.sec;
            double t1 = epochs.get(1).obs[0].time.time + epochs.get(1).obs[0].time.sec;
            epochInterval = Math.max(1.0, t1 - t0);
        }
        final double MIN_SEG_DURATION = 30.0;
        final int MIN_SEG_LEN = Math.max(4, (int)(MIN_SEG_DURATION / epochInterval));
        ambParams.removeIf(ap -> (ap.endEpoch - ap.startEpoch + 1) < MIN_SEG_LEN);

        int nAmb = ambParams.size();
        int nx = 3 + nAmb;

        // 5. Initialize DD ambiguity values
        double[] ambValues = BatchNormalEq.initDdAmbFromZdres(epochs, ambParams, nav, opt, pos, nf);

        // Pre-build ambiguity index for fast lookup (#2 fix)
        java.util.Map<Long, Integer> ambIndex = BatchNormalEq.buildAmbIndex(ambParams);

        // Compute ambiguity components once (#8 fix)
        List<List<Integer>> ambComponents = BatchAr.findAmbiguityComponents(ambParams);

        // Sub-sampling: use every N-th epoch for normal equations
        int subsample = opt.blsSubsample;
        if (subsample <= 0) subsample = 1;
        if (epochs.size() / Math.max(1, subsample) < 20) subsample = 1;

        double[] N = null;
        double[] b = null;

        // 6. Gauss-Newton iterations (sub-sampled)
        for (int iter = 0; iter < MAX_ITER; iter++) {
            N = new double[nx * nx];
            b = new double[nx];

            int totalObs = 0;

            for (int ep = 0; ep < epochs.size(); ep += subsample) {
                List<BatchNormalEq.DdObs> ddObs = buildEpochDdObs(
                        epochs.get(ep), pos, opt, nav, nf, ambParams, ambIndex,
                        ambValues, ep, refSatMap);
                if (ddObs == null) continue;
                totalObs += BatchNormalEq.accumulateEpoch(ddObs, nx, nAmb, N, b);
            }

            if (totalObs < nx) {
                return new BatchResult(pos, new float[6], SOLQ_NONE, 0, 0, epochs.size(), nAmb);
            }

            BatchNormalEq.regularizeNaa(N, nx, nAmb);

            double[] dxp = new double[3];
            double[] dxa = new double[nAmb];
            if (!BatchNormalEq.schurSolve(N, b, nx, nAmb, ambComponents, ambParams, dxp, dxa)) {
                return new BatchResult(pos, new float[6], SOLQ_NONE, 0, 0, epochs.size(), nAmb);
            }

            pos[0] += dxp[0]; pos[1] += dxp[1]; pos[2] += dxp[2];
            for (int j = 0; j < nAmb; j++) ambValues[j] += dxa[j];

            double dpos = Math.sqrt(dxp[0]*dxp[0] + dxp[1]*dxp[1] + dxp[2]*dxp[2]);
            double dambMax = 0;
            for (int j = 0; j < nAmb; j++) dambMax = Math.max(dambMax, Math.abs(dxa[j]));
            if (dpos < CONV_THRESHOLD && dambMax < 0.01) break;
        }

        // 6a. Compute σ₀² (VCE) from sub-sampled residuals after convergence
        double sigma0sq = computeSigma0sq(epochs, pos, opt, nav, nf, ambParams,
                ambIndex, ambValues, refSatMap, nx, subsample, null);

        // 6b. Iterative outlier cleaning via scaled w-test
        Set<Long> excludedObs = new HashSet<>();
        for (int cleanIter = 0; cleanIter < MAX_CLEAN; cleanIter++) {
            double maxWtest = 0;
            long worstKey = -1;

            // Scan ALL epochs for worst normalized residual (scaled by σ₀²)
            for (int ep = 0; ep < epochs.size(); ep++) {
                List<BatchNormalEq.DdObs> ddObs = buildEpochDdObs(
                        epochs.get(ep), pos, opt, nav, nf, ambParams, ambIndex,
                        ambValues, ep, refSatMap);
                if (ddObs == null) continue;

                for (BatchNormalEq.DdObs dd : ddObs) {
                    if (!dd.isPhase || dd.var <= 0) continue;
                    long key = ((long) ep << 32) | ((long) dd.sat << 16) | dd.freq;
                    if (excludedObs.contains(key)) continue;

                    double scaledVar = Math.max(1.0, sigma0sq) * dd.var;
                    double wtest = Math.abs(dd.v) / Math.sqrt(scaledVar);
                    if (wtest > maxWtest) {
                        maxWtest = wtest;
                        worstKey = key;
                    }
                }
            }

            if (maxWtest < CLEAN_WTEST_THRESHOLD || worstKey < 0) break;

            // Exclude the worst observation and re-solve (sub-sampled)
            excludedObs.add(worstKey);

            N = new double[nx * nx];
            b = new double[nx];
            for (int ep = 0; ep < epochs.size(); ep += subsample) {
                List<BatchNormalEq.DdObs> ddObs = buildEpochDdObs(
                        epochs.get(ep), pos, opt, nav, nf, ambParams, ambIndex,
                        ambValues, ep, refSatMap);
                if (ddObs == null) continue;

                ddObs.removeIf(dd -> {
                    long k = ((long) dd.epoch << 32) | ((long) dd.sat << 16) | dd.freq;
                    return excludedObs.contains(k);
                });
                if (ddObs.isEmpty()) continue;

                BatchNormalEq.accumulateEpoch(ddObs, nx, nAmb, N, b);
            }

            BatchNormalEq.regularizeNaa(N, nx, nAmb);

            double[] dxp = new double[3];
            double[] dxa = new double[nAmb];
            if (!BatchNormalEq.schurSolve(N, b, nx, nAmb, ambComponents, ambParams, dxp, dxa)) break;

            pos[0] += dxp[0]; pos[1] += dxp[1]; pos[2] += dxp[2];
            for (int j = 0; j < nAmb; j++) ambValues[j] += dxa[j];

            // Recompute σ₀² after cleaning
            sigma0sq = computeSigma0sq(epochs, pos, opt, nav, nf, ambParams,
                    ambIndex, ambValues, refSatMap, nx, subsample, excludedObs);
        }

        // Rebuild final N for covariance computation (sub-sampled)
        {
            N = new double[nx * nx];
            b = new double[nx];
            for (int ep = 0; ep < epochs.size(); ep += subsample) {
                List<BatchNormalEq.DdObs> ddObs = buildEpochDdObs(
                        epochs.get(ep), pos, opt, nav, nf, ambParams, ambIndex,
                        ambValues, ep, refSatMap);
                if (ddObs == null) continue;
                if (!excludedObs.isEmpty()) {
                    ddObs.removeIf(dd -> {
                        long k = ((long) dd.epoch << 32) | ((long) dd.sat << 16) | dd.freq;
                        return excludedObs.contains(k);
                    });
                    if (ddObs.isEmpty()) continue;
                }
                BatchNormalEq.accumulateEpoch(ddObs, nx, nAmb, N, b);
            }
            BatchNormalEq.regularizeNaa(N, nx, nAmb);
        }

        // 7. Compute covariances (reuse ambComponents — #8 fix)
        int ns = BatchPreprocess.countSatellites(ambParams);

        double[] Qpp = new double[9];
        double[] Qpa = new double[3 * nAmb];
        double[] Qaa = new double[nAmb * nAmb];

        if (!BatchNormalEq.computeCovariance(N, nx, nAmb, ambComponents, Qpp, Qpa, Qaa)) {
            return new BatchResult(pos, new float[6], SOLQ_FLOAT, 0, ns, epochs.size(), nAmb,
                                   ambValues, ambParams, sigma0sq);
        }

        // Scale covariance by σ₀²
        for (int i = 0; i < 9; i++) Qpp[i] *= sigma0sq;
        for (int i = 0; i < 3 * nAmb; i++) Qpa[i] *= sigma0sq;
        for (int i = 0; i < nAmb * nAmb; i++) Qaa[i] *= sigma0sq;

        float[] qr = new float[6];
        qr[0] = (float) Qpp[0]; qr[1] = (float) Qpp[4]; qr[2] = (float) Qpp[8];
        qr[3] = (float) Qpp[1]; qr[4] = (float) Qpp[5]; qr[5] = (float) Qpp[2];

        // 8. LAMBDA AR
        if (nAmb < 1 || opt.modear == 0) {
            return new BatchResult(pos, qr, SOLQ_FLOAT, 0, ns, epochs.size(), nAmb,
                                   ambValues, ambParams, sigma0sq);
        }

        return BatchAr.resolveAmbiguities(pos, qr, ambValues, ambParams, Qpp, Qpa, Qaa,
                epochs, nav, opt, nf, refSatMap, ns, epochInterval, sigma0sq);
    }

    /**
     * Compute a posteriori variance factor σ₀² = v'Wv / (n_obs - n_x).
     * Clamped to [0.1, 10.0] to avoid extreme values.
     */
    private static double computeSigma0sq(
            List<BatchPreprocess.EpochData> epochs, double[] pos,
            ProcessingOptions opt, Navigation nav, int nf,
            List<BatchPreprocess.AmbParam> ambParams,
            java.util.Map<Long, Integer> ambIndex,
            double[] ambValues, int[][] refSatMap,
            int nx, int subsample, Set<Long> excludedObs) {
        double vtWv = 0;
        int vtWvCount = 0;

        for (int ep = 0; ep < epochs.size(); ep += subsample) {
            List<BatchNormalEq.DdObs> ddObs = buildEpochDdObs(
                    epochs.get(ep), pos, opt, nav, nf, ambParams, ambIndex,
                    ambValues, ep, refSatMap);
            if (ddObs == null) continue;

            for (BatchNormalEq.DdObs dd : ddObs) {
                if (dd.var <= 0) continue;
                if (excludedObs != null) {
                    long k = ((long) dd.epoch << 32) | ((long) dd.sat << 16) | dd.freq;
                    if (excludedObs.contains(k)) continue;
                }
                vtWv += dd.v * dd.v / dd.var;
                vtWvCount++;
            }
        }

        int dof = Math.max(1, vtWvCount - nx);
        return Math.max(0.1, Math.min(10.0, vtWv / dof));
    }

    /** Build DD observations for one epoch. */
    private static List<BatchNormalEq.DdObs> buildEpochDdObs(
            BatchPreprocess.EpochData ed, double[] pos,
            ProcessingOptions opt, Navigation nav, int nf,
            List<BatchPreprocess.AmbParam> ambParams,
            java.util.Map<Long, Integer> ambIndex,
            double[] ambValues, int ep, int[][] refSatMap) {
        if (ed.ns <= 0) return null;

        double[] yRov = new double[nf * 2 * ed.nu];
        double[] eRov = new double[3 * ed.nu];
        double[] azelRov = new double[2 * ed.nu];
        double[] freqRov = new double[nf * ed.nu];
        ObsData[] obsRov = new ObsData[ed.nu];
        System.arraycopy(ed.obs, 0, obsRov, 0, ed.nu);
        if (Rtkpos.zdres(0, obsRov, ed.nu, ed.rs, ed.dts, ed.var, ed.svh,
                          nav, pos, opt, yRov, eRov, azelRov, freqRov) == 0) return null;

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
                          nav, opt.rb, opt, yBase, eBase, azelBase, freqBase) == 0) return null;

        double[] dr = new double[3];
        double bl = Rtkpos.baseline(pos, opt.rb, dr);

        List<BatchNormalEq.DdObs> ddObs = BatchNormalEq.makeDdObs(
                ed, pos, opt, nav, nf, yRov, eRov, azelRov, freqRov,
                yBase, azelBase, freqBase, ambParams, ambIndex,
                ambValues, ep, bl, refSatMap);

        return ddObs.isEmpty() ? null : ddObs;
    }
}
