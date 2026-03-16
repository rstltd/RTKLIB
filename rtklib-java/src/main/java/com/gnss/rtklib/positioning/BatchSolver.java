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
import java.util.List;

import static com.gnss.rtklib.core.Constants.*;

/**
 * Batch Least Squares (BLS) solver for RTK static positioning.
 * <p>
 * Solves all epochs simultaneously via normal equations, yielding optimal
 * position and ambiguity estimates without EKF convergence delays.
 * Uses double-differenced (DD) parameterization for inherent full-rank
 * ambiguity states without datum constraints.
 * <p>
 * State vector: [pos(3), DD_N_1, DD_N_2, ..., DD_N_n]
 * where each DD_N corresponds to a {refSat, sat, freq, segment} tuple.
 */
public final class BatchSolver {

    private BatchSolver() {}

    private static final int MAX_ITER = 10;
    private static final double CONV_THRESHOLD = 1e-4; // m

    /**
     * DD ambiguity parameter descriptor for BLS state vector.
     */
    static class AmbParam {
        final int refSat;     // reference satellite number (1-based)
        final int sat;        // non-reference satellite number (1-based)
        final int freq;       // frequency index
        final int segment;    // segment number (incremented on ref change or gap)
        int startEpoch;       // first epoch of this segment
        int endEpoch;         // last epoch (-1 if ongoing)

        AmbParam(int refSat, int sat, int freq, int segment, int startEpoch) {
            this.refSat = refSat;
            this.sat = sat;
            this.freq = freq;
            this.segment = segment;
            this.startEpoch = startEpoch;
            this.endEpoch = -1;
        }
    }

    /**
     * Result of BLS solve.
     */
    public static class BatchResult {
        public final double[] pos;       // ECEF position [3]
        public final float[] qr;         // position covariance {xx,yy,zz,xy,yz,xz}
        public final int stat;           // solution status (SOLQ_xxx)
        public final float ratio;        // AR ratio
        public final int ns;             // number of satellites used
        public final int nEpochs;        // number of epochs processed
        public final int nAmb;           // number of ambiguity parameters
        public final double[] ambValues; // DD float ambiguity values (for diagnostics)
        public final List<AmbParam> ambParams; // ambiguity parameter descriptors

        BatchResult(double[] pos, float[] qr, int stat, float ratio, int ns,
                    int nEpochs, int nAmb) {
            this(pos, qr, stat, ratio, ns, nEpochs, nAmb, null);
        }

        BatchResult(double[] pos, float[] qr, int stat, float ratio, int ns,
                    int nEpochs, int nAmb, double[] ambValues) {
            this(pos, qr, stat, ratio, ns, nEpochs, nAmb, ambValues, null);
        }

        BatchResult(double[] pos, float[] qr, int stat, float ratio, int ns,
                    int nEpochs, int nAmb, double[] ambValues, List<AmbParam> ambParams) {
            this.pos = pos;
            this.qr = qr;
            this.stat = stat;
            this.ratio = ratio;
            this.ns = ns;
            this.nEpochs = nEpochs;
            this.nAmb = nAmb;
            this.ambValues = ambValues;
            this.ambParams = ambParams;
        }
    }

    /**
     * Per-epoch preprocessed data: merged obs, satellite positions, common sats.
     */
    static class EpochData {
        ObsData[] obs;         // merged rover+base
        int nu, nr;            // counts
        double[] rs, dts;      // satellite pos/vel, clocks
        double[] var;          // satellite position variance
        int[] svh;             // satellite health
        int[] sat, iu, ir;     // common satellite numbers, indices
        int ns;                // number of common sats
    }

    /**
     * DD observation for one measurement (phase or code).
     */
    private static class DdObs {
        double v;         // DD residual (meters) — phase includes amb correction
        double[] hPos;    // position partial derivatives [3]
        double lambda;    // wavelength (meters), 0 for code
        int ddAmbIdx;     // index into DD ambiguity parameter list (-1 for code)
        double var;       // DD measurement variance
        boolean isPhase;  // true for phase, false for code
    }

    /**
     * Solve RTK static using Batch Least Squares with DD parameterization.
     *
     * @param roverEpochs rover observation epochs
     * @param baseEpochs  base observation epochs
     * @param nav         navigation data
     * @param opt         processing options
     * @return BLS result with position, covariance, and status
     */
    public static BatchResult solve(List<List<ObsData>> roverEpochs,
                                     List<List<ObsData>> baseEpochs,
                                     Navigation nav, ProcessingOptions opt) {
        // 1. Initial position from SPP
        double[] pos = sppPosition(roverEpochs, nav, opt);
        if (pos == null) {
            return new BatchResult(new double[3], new float[6], SOLQ_NONE, 0, 0, 0, 0);
        }

        // 2. Preprocess: match epochs, compute satellite positions, detect slips
        List<EpochData> epochs = preprocessEpochs(roverEpochs, baseEpochs, nav, opt);
        if (epochs.isEmpty()) {
            return new BatchResult(pos, new float[6], SOLQ_NONE, 0, 0, 0, 0);
        }

        int nf = FilterState.NF(opt);

        // 3. Choose stable ref sat per (system, freq) across all epochs
        int[][] refSatMap = chooseRefSats(epochs, opt, nf);

        // 4. Scan DD ambiguities
        List<AmbParam> ambParams = scanDdAmbiguities(epochs, opt, nf, refSatMap);

        // Remove short segments (< 4 epochs): too few observations to
        // reliably constrain a DD ambiguity parameter
        final int MIN_SEG_LEN = 4;
        ambParams.removeIf(ap -> (ap.endEpoch - ap.startEpoch + 1) < MIN_SEG_LEN);

        int nAmb = ambParams.size();
        int nx = 3 + nAmb;

        // 5. Initialize DD ambiguity values from DD code-phase differences
        double[] ambValues = initDdAmbFromZdres(epochs, ambParams, nav, opt, pos, nf);

        // Relax outlier threshold for BLS
        double[] savedMaxinno = {opt.maxinno[0], opt.maxinno[1]};
        opt.maxinno[0] = 100.0;
        opt.maxinno[1] = 100.0;

        double[] N = null;
        double[] b = null;

        for (int iter = 0; iter < MAX_ITER; iter++) {
            N = new double[nx * nx];
            b = new double[nx];

            int totalObs = 0;

            for (int ep = 0; ep < epochs.size(); ep++) {
                EpochData ed = epochs.get(ep);
                if (ed.ns <= 0) continue;

                // Compute zdres for rover and base
                double[] yRov = new double[nf * 2 * ed.nu];
                double[] eRov = new double[3 * ed.nu];
                double[] azelRov = new double[2 * ed.nu];
                double[] freqRov = new double[nf * ed.nu];

                ObsData[] obsRov = new ObsData[ed.nu];
                System.arraycopy(ed.obs, 0, obsRov, 0, ed.nu);

                if (Rtkpos.zdres(0, obsRov, ed.nu, ed.rs, ed.dts, ed.var, ed.svh,
                                  nav, pos, opt, yRov, eRov, azelRov, freqRov) == 0) {
                    continue;
                }

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
                                  nav, opt.rb, opt, yBase, eBase, azelBase, freqBase) == 0) {
                    continue;
                }

                // Baseline length for varerr
                double[] dr = new double[3];
                double bl = Rtkpos.baseline(pos, opt.rb, dr);

                // Form DD observations and accumulate normal equations
                List<DdObs> ddObs = makeDdObs(ed, pos, opt, nav, nf,
                        yRov, eRov, azelRov, freqRov,
                        yBase, azelBase, freqBase,
                        ambParams, ambValues, ep, bl, refSatMap);

                if (ddObs.isEmpty()) continue;

                // Accumulate N += H'WH, b += H'Wv with Huber robust weighting
                for (DdObs dd : ddObs) {
                    double w = 1.0 / dd.var;

                    // Huber M-estimator: downweight observations with large
                    // normalized residuals. On the first iteration (ambiguities
                    // from code-phase init), residuals may be large — Huber
                    // naturally handles this without a separate reweighting pass.
                    // Apply Huber downweighting only on last iteration
                    // (earlier iterations: let LS converge without weight perturbation;
                    //  last iteration: refine with robust weights)
                    if (iter >= 1) {
                        double sigma = Math.sqrt(dd.var);
                        double r = Math.abs(dd.v) / sigma;
                        double HUBER_K = 4.0; // conservative: only extreme outliers > 4σ
                        if (r > HUBER_K) {
                            w *= HUBER_K / r;
                        }
                    }

                    // b[0:3] += hPos * w * v
                    for (int k = 0; k < 3; k++) {
                        b[k] += dd.hPos[k] * w * dd.v;
                    }

                    // N[pos,pos] += hPos * w * hPos^T
                    for (int k = 0; k < 3; k++) {
                        for (int l = k; l < 3; l++) {
                            double val = dd.hPos[k] * w * dd.hPos[l];
                            N[k + l * nx] += val;
                            if (k != l) N[l + k * nx] += val;
                        }
                    }

                    if (dd.isPhase && dd.ddAmbIdx >= 0) {
                        int ai = 3 + dd.ddAmbIdx;

                        // b[amb] += lambda * w * v
                        b[ai] += dd.lambda * w * dd.v;

                        // N[pos,amb] += hPos * w * lambda
                        for (int k = 0; k < 3; k++) {
                            double val = dd.hPos[k] * w * dd.lambda;
                            N[k + ai * nx] += val;
                            N[ai + k * nx] += val;
                        }

                        // N[amb,amb] += lambda * w * lambda
                        N[ai + ai * nx] += dd.lambda * w * dd.lambda;
                    }

                    totalObs++;
                }
            }

            if (totalObs < nx) {
                return new BatchResult(pos, new float[6], SOLQ_NONE, 0, 0, epochs.size(), nAmb);
            }

            // Check for weakly observed ambiguities (near-zero Naa diagonal).
            // In correct DD parameterization with per-system ref sat, this should
            // not happen. If it does, it indicates a segment/ref-sat bug.
            // Regularize as last resort to prevent singular Naa, but this biases
            // the affected ambiguity toward zero.
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

            // Solve using Schur complement
            double[] Npp = new double[9], Npa = new double[3 * nAmb];
            double[] Naa = new double[nAmb * nAmb];
            double[] bp = new double[3], ba = new double[nAmb];

            for (int ii = 0; ii < 3; ii++) {
                bp[ii] = b[ii];
                for (int jj = 0; jj < 3; jj++) Npp[ii + jj * 3] = N[ii + jj * nx];
                for (int jj = 0; jj < nAmb; jj++) Npa[ii + jj * 3] = N[ii + (3 + jj) * nx];
            }
            for (int ii = 0; ii < nAmb; ii++) {
                ba[ii] = b[3 + ii];
                for (int jj = 0; jj < nAmb; jj++) {
                    Naa[ii + jj * nAmb] = N[(3 + ii) + (3 + jj) * nx];
                }
            }

            double[] NaaInv = new double[nAmb * nAmb];
            System.arraycopy(Naa, 0, NaaInv, 0, nAmb * nAmb);
            if (MatrixUtil.matinv(NaaInv, nAmb) != 0) {
                return new BatchResult(pos, new float[6], SOLQ_NONE, 0, 0, epochs.size(), nAmb);
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
            if (MatrixUtil.matinv(NredInv, 3) != 0) {
                return new BatchResult(pos, new float[6], SOLQ_NONE, 0, 0, epochs.size(), nAmb);
            }

            double[] dxp = new double[3];
            MatrixUtil.matmul("NN", 3, 1, 3, NredInv, bred, dxp);

            double[] baRed = new double[nAmb];
            System.arraycopy(ba, 0, baRed, 0, nAmb);
            MatrixUtil.matmul("TN", nAmb, 1, 3, -1.0, Npa, dxp, 1.0, baRed);

            double[] dxa = new double[nAmb];
            MatrixUtil.matmul("NN", nAmb, 1, nAmb, NaaInv, baRed, dxa);

            pos[0] += dxp[0];
            pos[1] += dxp[1];
            pos[2] += dxp[2];

            for (int j = 0; j < nAmb; j++) {
                ambValues[j] += dxa[j];
            }

            double dpos = Math.sqrt(dxp[0] * dxp[0] + dxp[1] * dxp[1] + dxp[2] * dxp[2]);
            // Check both position AND ambiguity convergence
            double dambMax = 0;
            for (int j = 0; j < nAmb; j++) {
                dambMax = Math.max(dambMax, Math.abs(dxa[j]));
            }
            if (dpos < CONV_THRESHOLD && dambMax < 0.01) break; // both must converge
        } // end Gauss-Newton iterations (with Huber IRLS built-in)

        // Restore outlier threshold
        opt.maxinno[0] = savedMaxinno[0];
        opt.maxinno[1] = savedMaxinno[1];

        // 6. Compute covariances
        int ns = countSatellites(ambParams);

        double[] Npp = new double[9], Npa = new double[3 * nAmb];
        double[] Naa = new double[nAmb * nAmb];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) Npp[i + j * 3] = N[i + j * nx];
            for (int j = 0; j < nAmb; j++) Npa[i + j * 3] = N[i + (3 + j) * nx];
        }
        for (int i = 0; i < nAmb; i++) {
            for (int j = 0; j < nAmb; j++) {
                Naa[i + j * nAmb] = N[(3 + i) + (3 + j) * nx];
            }
        }

        double[] NaaInv = new double[nAmb * nAmb];
        System.arraycopy(Naa, 0, NaaInv, 0, nAmb * nAmb);
        if (MatrixUtil.matinv(NaaInv, nAmb) != 0) {
            return new BatchResult(pos, new float[6], SOLQ_FLOAT, 0, ns, epochs.size(), nAmb);
        }

        double[] tmp3a = new double[3 * nAmb];
        MatrixUtil.matmul("NN", 3, nAmb, nAmb, Npa, NaaInv, tmp3a);
        double[] Nred = new double[9];
        System.arraycopy(Npp, 0, Nred, 0, 9);
        MatrixUtil.matmul("NT", 3, 3, nAmb, -1.0, tmp3a, Npa, 1.0, Nred);

        double[] Qpp = new double[9];
        System.arraycopy(Nred, 0, Qpp, 0, 9);
        if (MatrixUtil.matinv(Qpp, 3) != 0) {
            return new BatchResult(pos, new float[6], SOLQ_FLOAT, 0, ns, epochs.size(), nAmb);
        }

        float[] qr = new float[6];
        qr[0] = (float) Qpp[0]; qr[1] = (float) Qpp[4]; qr[2] = (float) Qpp[8];
        qr[3] = (float) Qpp[1]; qr[4] = (float) Qpp[5]; qr[5] = (float) Qpp[2];

        // 7. LAMBDA AR — DD ambiguities go directly (no extraction needed)
        if (nAmb < 1 || opt.modear == 0) {
            return new BatchResult(pos, qr, SOLQ_FLOAT, 0, ns, epochs.size(), nAmb, ambValues, ambParams);
        }

        // Build AR-eligible list: exclude GLONASS (FDMA) and weakly observed
        double maxNorm = 0;
        double[] diagArr = new double[nAmb];
        for (int j = 0; j < nAmb; j++) {
            diagArr[j] = Naa[j + j * nAmb];
            if (diagArr[j] > maxNorm) maxNorm = diagArr[j];
        }
        double[] sorted = diagArr.clone();
        java.util.Arrays.sort(sorted);
        double medianDiag = sorted[Math.max(0, nAmb / 2)];

        List<Integer> arIdx = new ArrayList<>();
        for (int j = 0; j < nAmb; j++) {
            AmbParam ap = ambParams.get(j);
            int sys = SatelliteUtil.satsys(ap.sat)[0];

            // Skip GLONASS (FDMA, non-integer DD ambiguity)
            if (sys == SYS_GLO) continue;

            // Skip weakly observed (diagonal < 1% of median)
            if (medianDiag > 0 && diagArr[j] < medianDiag * 0.01) continue;

            arIdx.add(j);
        }

        int nAR = arIdx.size();
        if (nAR < 1) {
            return new BatchResult(pos, qr, SOLQ_FLOAT, 0, ns, epochs.size(), nAmb, ambValues, ambParams);
        }

        // Compute marginal ambiguity covariance for LAMBDA:
        // Qaa = NaaInv + NaaInv * Npa^T * Qpp * Npa * NaaInv
        // This accounts for position uncertainty (unlike plain NaaInv).
        // Also compute Qpa = -Qpp * Npa * NaaInv for fixed solution.
        double[] Qpa = new double[3 * nAmb];
        MatrixUtil.matmul("NN", 3, nAmb, 3, Qpp, tmp3a, Qpa);  // Qpp * (Npa * NaaInv)
        for (int i = 0; i < 3 * nAmb; i++) Qpa[i] = -Qpa[i];   // Qpa = -Qpp * Npa * NaaInv

        // Qaa = NaaInv + NaaInv * Npa^T * Qpp * Npa * NaaInv
        //     = NaaInv + (NaaInv * Npa^T) * (-Qpa)^T  [since Qpa = -Qpp*Npa*NaaInv]
        //     = NaaInv - tmp3a^T * Qpa^T
        // where tmp3a = Npa * NaaInv (3 x nAmb), so tmp3a^T = NaaInv^T * Npa^T (nAmb x 3)
        // Actually: Qaa = NaaInv + NaaInv * Nap * Qpp * Npa * NaaInv
        //   Let U = Npa * NaaInv (3 x nAmb) = tmp3a
        //   Qaa = NaaInv + U^T * Qpp * U
        double[] Qaa = new double[nAmb * nAmb];
        System.arraycopy(NaaInv, 0, Qaa, 0, nAmb * nAmb);
        // QppU = Qpp * U = Qpp * tmp3a (3 x nAmb)
        double[] QppU = new double[3 * nAmb];
        MatrixUtil.matmul("NN", 3, nAmb, 3, Qpp, tmp3a, QppU);
        // Qaa += U^T * QppU = tmp3a^T * QppU
        MatrixUtil.matmul("TN", nAmb, nAmb, 3, 1.0, tmp3a, QppU, 1.0, Qaa);

        // Extract AR-eligible sub-blocks
        double[] aAR = new double[nAR];
        double[] QaaAR = new double[nAR * nAR];
        for (int i = 0; i < nAR; i++) {
            aAR[i] = ambValues[arIdx.get(i)];
            for (int j = 0; j < nAR; j++) {
                QaaAR[i + j * nAR] = Qaa[arIdx.get(i) + arIdx.get(j) * nAmb];
            }
        }

        // ========== WL/NL Two-Step AR ==========

        // Step 1: Pair L1/L5 ambiguities
        // For each AR-eligible amb at freq=0 (L1), find matching freq=2 (L5)
        // with same refSat, same sat, overlapping segments.
        List<int[]> dualPairs = new ArrayList<>();  // {arIdx_for_L1, arIdx_for_L5}
        boolean[] paired = new boolean[nAR];

        for (int i = 0; i < nAR; i++) {
            AmbParam api = ambParams.get(arIdx.get(i));
            if (api.freq != 0) continue;  // only start from L1
            for (int j = 0; j < nAR; j++) {
                if (paired[j]) continue;
                AmbParam apj = ambParams.get(arIdx.get(j));
                if (apj.freq != 2) continue;  // L5 is at freq index 2
                if (apj.refSat != api.refSat || apj.sat != api.sat) continue;
                // Check segment overlap
                if (api.startEpoch > apj.endEpoch || api.endEpoch < apj.startEpoch) continue;
                dualPairs.add(new int[]{i, j});
                paired[i] = true;
                paired[j] = true;
                break;
            }
        }

        // Single-freq (L1 only) ambiguities for NL step
        List<Integer> singleIdx = new ArrayList<>();
        for (int i = 0; i < nAR; i++) {
            if (!paired[i] && ambParams.get(arIdx.get(i)).freq == 0) {
                singleIdx.add(i);
            }
        }

        int nWL = dualPairs.size();
        float ratio = 0;
        int[] fixedOriginal = null;  // fixed integer values for all nAR ambs (null = fail)

        if (nWL >= 4) {
            // Step 2: WL fix
            double[] aWL = new double[nWL];
            double[] QWL = new double[nWL * nWL];

            for (int i = 0; i < nWL; i++) {
                int li = dualPairs.get(i)[0], fi = dualPairs.get(i)[1];
                aWL[i] = aAR[li] - aAR[fi];  // N_WL = N_L1 - N_L5 (cycles)
            }

            for (int i = 0; i < nWL; i++) {
                int li = dualPairs.get(i)[0], fi = dualPairs.get(i)[1];
                for (int j = 0; j < nWL; j++) {
                    int lj = dualPairs.get(j)[0], fj = dualPairs.get(j)[1];
                    // Var(N_WL) = Var(N_L1) + Var(N_L5) - 2*Cov(N_L1, N_L5)
                    QWL[i + j * nWL] = QaaAR[li + lj * nAR] + QaaAR[fi + fj * nAR]
                                     - QaaAR[li + fj * nAR] - QaaAR[fi + lj * nAR];
                }
            }

            // LAMBDA on WL (should have very high ratio due to long wavelength)
            double[] FWL = new double[nWL * 2];
            double[] sWL = new double[2];
            int infoWL = Lambda.lambda(nWL, 2, aWL, QWL, FWL, sWL);

            float ratioWL = 0;
            if (infoWL == 0 && sWL[0] > 0) {
                ratioWL = (float)(sWL[1] / sWL[0]);
                if (ratioWL > 999.9f) ratioWL = 999.9f;
            }

            if (ratioWL >= 2.0) {  // WL threshold is low (easy to fix)
                // WL fixed! Apply WL constraint to Qaa before NL step.
                // Constrained: Qaa_c = Qaa - Qaa*T'*(T*Qaa*T')^{-1}*T*Qaa
                // where T is the WL differencing matrix [+1(L1), -1(L5)]
                // T*Qaa*T' = QWL (already computed), need T*Qaa and QWL^{-1}
                double[] QWLinv = QWL.clone();
                boolean wlConstraintOk = (MatrixUtil.matinv(QWLinv, nWL) == 0);

                double[] QaaC = QaaAR.clone(); // constrained Qaa (start from original)
                if (wlConstraintOk) {
                    // TQaa[w,a] = Qaa[L1_w, a] - Qaa[L5_w, a] for each WL pair w
                    double[] TQaa = new double[nWL * nAR];
                    for (int w = 0; w < nWL; w++) {
                        int lw = dualPairs.get(w)[0], fw = dualPairs.get(w)[1];
                        for (int a = 0; a < nAR; a++) {
                            TQaa[w + a * nWL] = QaaAR[lw + a * nAR] - QaaAR[fw + a * nAR];
                        }
                    }
                    // QWLinv * TQaa (nWL x nAR)
                    double[] QWLinvTQaa = new double[nWL * nAR];
                    MatrixUtil.matmul("NN", nWL, nAR, nWL, QWLinv, TQaa, QWLinvTQaa);
                    // QaaC = Qaa - TQaa' * QWLinv * TQaa
                    MatrixUtil.matmul("TN", nAR, nAR, nWL, -1.0, TQaa, QWLinvTQaa, 1.0, QaaC);
                }

                // Step 3: NL fix using WL-constrained covariance
                int nNL = nWL + singleIdx.size();
                double[] aNL = new double[nNL];
                double[] QNL = new double[nNL * nNL];

                // NL float values
                for (int i = 0; i < nWL; i++) {
                    int li = dualPairs.get(i)[0], fi = dualPairs.get(i)[1];
                    aNL[i] = aAR[li] + aAR[fi];  // N_NL = N_L1 + N_L5
                }
                for (int i = 0; i < singleIdx.size(); i++) {
                    aNL[nWL + i] = aAR[singleIdx.get(i)];  // L1 only
                }

                // NL covariance from WL-constrained QaaC
                for (int i = 0; i < nNL; i++) {
                    for (int j = 0; j < nNL; j++) {
                        double cov;
                        if (i < nWL && j < nWL) {
                            int li = dualPairs.get(i)[0], fi = dualPairs.get(i)[1];
                            int lj = dualPairs.get(j)[0], fj = dualPairs.get(j)[1];
                            cov = QaaC[li + lj * nAR] + QaaC[fi + fj * nAR]
                                + QaaC[li + fj * nAR] + QaaC[fi + lj * nAR];
                        } else if (i < nWL) {
                            int li = dualPairs.get(i)[0], fi = dualPairs.get(i)[1];
                            int sj = singleIdx.get(j - nWL);
                            cov = QaaC[li + sj * nAR] + QaaC[fi + sj * nAR];
                        } else if (j < nWL) {
                            int si = singleIdx.get(i - nWL);
                            int lj = dualPairs.get(j)[0], fj = dualPairs.get(j)[1];
                            cov = QaaC[si + lj * nAR] + QaaC[si + fj * nAR];
                        } else {
                            int si = singleIdx.get(i - nWL);
                            int sj = singleIdx.get(j - nWL);
                            cov = QaaC[si + sj * nAR];
                        }
                        QNL[i + j * nNL] = cov;
                    }
                }

                // LAMBDA on NL with PAR (use LD conditional variance sorting)
                double[] ldLnl = new double[nNL * nNL];
                double[] ldDnl = new double[nNL];
                double[] QNLclone = QNL.clone();
                Integer[] sortNL = new Integer[nNL];
                for (int i = 0; i < nNL; i++) sortNL[i] = i;

                if (Lambda.LD(nNL, QNLclone, ldLnl, ldDnl) == 0) {
                    java.util.Arrays.sort(sortNL, (a1, b1) ->
                            Double.compare(ldDnl[a1], ldDnl[b1]));
                } else {
                    java.util.Arrays.sort(sortNL, (a1, b1) ->
                            Double.compare(QNL[a1 + a1 * nNL], QNL[b1 + b1 * nNL]));
                }

                // PAR loop for NL
                int minNL = Math.max(4, opt.minfixsats - 1);
                double[] bestFNL = null;
                List<Integer> bestSubNL = null;
                float bestRatioNL = 0;

                for (int nTry = nNL; nTry >= minNL; nTry--) {
                    double[] aSub = new double[nTry];
                    double[] QSub = new double[nTry * nTry];
                    List<Integer> subIdx = new ArrayList<>();
                    for (int i = 0; i < nTry; i++) {
                        int si = sortNL[i];
                        subIdx.add(si);
                        aSub[i] = aNL[si];
                        for (int j = 0; j < nTry; j++) {
                            QSub[i + j * nTry] = QNL[sortNL[i] + sortNL[j] * nNL];
                        }
                    }

                    double[] Ftry = new double[nTry * 2];
                    double[] stry = new double[2];
                    int info = Lambda.lambda(nTry, 2, aSub, QSub, Ftry, stry);
                    if (info != 0 || stry[0] <= 0) continue;

                    float r = (float)(stry[1] / stry[0]);
                    if (r > 999.9f) r = 999.9f;

                    float thres = (float) Rtkpos.computeAdaptiveArThreshold(nTry, opt.thresar[0]);
                    if (r >= thres) {
                        bestRatioNL = r;
                        bestFNL = Ftry;
                        bestSubNL = subIdx;
                        break;
                    }
                    if (r > bestRatioNL) bestRatioNL = r;
                }

                if (bestSubNL != null && bestFNL != null) {
                    // Step 4: Recover original L1/L5 integers from WL + NL
                    int bestNfix = bestSubNL.size();

                    // Minimum fix count check
                    if (bestNfix >= Math.max(8, nAR / 3)) {
                        fixedOriginal = new int[nAR];
                        java.util.Arrays.fill(fixedOriginal, Integer.MIN_VALUE); // unfixed marker

                        for (int i = 0; i < bestNfix; i++) {
                            int nlIdx = bestSubNL.get(i);
                            int nlFix = (int) Math.round(bestFNL[i]);

                            if (nlIdx < nWL) {
                                // Dual-freq: recover L1 and L5 from WL + NL
                                int wlFix = (int) Math.round(FWL[nlIdx]);
                                // N_WL = N_L1 - N_L5, N_NL = N_L1 + N_L5
                                // N_L1 = (N_WL + N_NL) / 2, N_L5 = (N_NL - N_WL) / 2
                                // Both must be integer, so N_WL and N_NL must have same parity
                                if ((wlFix + nlFix) % 2 != 0) {
                                    // Parity mismatch — adjust NL by 1 to nearest valid value
                                    double nlFloat = aNL[nlIdx];
                                    if (nlFloat - nlFix > 0) nlFix++; else nlFix--;
                                }
                                int nL1 = (wlFix + nlFix) / 2;
                                int nL5 = (nlFix - wlFix) / 2;

                                fixedOriginal[dualPairs.get(nlIdx)[0]] = nL1;
                                fixedOriginal[dualPairs.get(nlIdx)[1]] = nL5;
                            } else {
                                // Single-freq L1
                                int sIdx = singleIdx.get(nlIdx - nWL);
                                fixedOriginal[sIdx] = nlFix;
                            }
                        }

                        ratio = bestRatioNL;
                    }
                }
            }
        }

        // Step 5: If WL/NL succeeded, compute fixed solution
        if (fixedOriginal != null) {
            // Count how many are actually fixed
            List<Integer> fixedArSubset = new ArrayList<>();
            for (int i = 0; i < nAR; i++) {
                if (fixedOriginal[i] != Integer.MIN_VALUE) fixedArSubset.add(i);
            }
            int bestNfix = fixedArSubset.size();

            // Extract fixed subset for position computation
            double[] Qpa_ar = new double[3 * bestNfix];
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < bestNfix; j++) {
                    Qpa_ar[i + j * 3] = Qpa[i + arIdx.get(fixedArSubset.get(j)) * 3];
                }
            }

            // QaaInv for fixed subset
            double[] QaaFixSub = new double[bestNfix * bestNfix];
            for (int i = 0; i < bestNfix; i++) {
                for (int j = 0; j < bestNfix; j++) {
                    QaaFixSub[i + j * bestNfix] = QaaAR[fixedArSubset.get(i) + fixedArSubset.get(j) * nAR];
                }
            }
            double[] QaaInvFix = new double[bestNfix * bestNfix];
            System.arraycopy(QaaFixSub, 0, QaaInvFix, 0, bestNfix * bestNfix);
            if (MatrixUtil.matinv(QaaInvFix, bestNfix) != 0) {
                // Fall through to single-step PAR
                fixedOriginal = null;
            } else {
                double[] da = new double[bestNfix];
                for (int i = 0; i < bestNfix; i++) {
                    da[i] = aAR[fixedArSubset.get(i)] - fixedOriginal[fixedArSubset.get(i)];
                }

                double[] QaaInvDa = new double[bestNfix];
                MatrixUtil.matmul("NN", bestNfix, 1, bestNfix, QaaInvFix, da, QaaInvDa);

                double[] posFix = new double[3];
                System.arraycopy(pos, 0, posFix, 0, 3);
                MatrixUtil.matmul("NN", 3, 1, bestNfix, -1.0, Qpa_ar, QaaInvDa, 1.0, posFix);

                // Fixed covariance
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

                double postFixRms = computePostFixPhaseRms(epochs, ambParams, nav, opt,
                        posFix, fixedAmb, nf, refSatMap, fixedIndices);

                if (postFixRms <= 0.05) {
                    return new BatchResult(posFix, qrFix, SOLQ_FIX, ratio, ns,
                            epochs.size(), nAmb, ambValues, ambParams);
                }
                // else fall through to single-step PAR
                fixedOriginal = null;
            }
        }

        // Step 6: Fallback to single-step PAR (existing logic)
        if (fixedOriginal == null) {
            // Partial Ambiguity Resolution (PAR):
            // Sort by LD conditional variance (D[i] from Q = L'DL decomposition).
            double[] ldL = new double[nAR * nAR];
            double[] ldD = new double[nAR];
            double[] QaaClone = QaaAR.clone();
            Integer[] sortOrder = new Integer[nAR];
            for (int i = 0; i < nAR; i++) sortOrder[i] = i;

            if (Lambda.LD(nAR, QaaClone, ldL, ldD) == 0) {
                java.util.Arrays.sort(sortOrder, (a1, b1) ->
                        Double.compare(ldD[a1], ldD[b1]));
            } else {
                java.util.Arrays.sort(sortOrder, (a1, b1) ->
                        Double.compare(QaaAR[a1 + a1 * nAR], QaaAR[b1 + b1 * nAR]));
            }

            double[] bestF = null;
            List<Integer> bestSubset = null;
            int bestNfix = 0;

            int minAR = Math.max(4, opt.minfixsats - 1);

            for (int nTry = nAR; nTry >= minAR; nTry--) {
                double[] aSub = new double[nTry];
                double[] QSub = new double[nTry * nTry];
                List<Integer> subIdx = new ArrayList<>();
                for (int i = 0; i < nTry; i++) {
                    int si = sortOrder[i];
                    subIdx.add(si);
                    aSub[i] = aAR[si];
                    for (int j = 0; j < nTry; j++) {
                        int sj = sortOrder[j];
                        QSub[i + j * nTry] = QaaAR[si + sj * nAR];
                    }
                }

                double[] Ftry = new double[nTry * 2];
                double[] stry = new double[2];
                int info = Lambda.lambda(nTry, 2, aSub, QSub, Ftry, stry);
                if (info != 0 || stry[0] <= 0) continue;

                float r = (float) (stry[1] / stry[0]);
                if (r > 999.9f) r = 999.9f;

                float thres = (float) Rtkpos.computeAdaptiveArThreshold(nTry, opt.thresar[0]);
                if (r >= thres) {
                    ratio = r;
                    bestF = Ftry;
                    bestSubset = subIdx;
                    bestNfix = nTry;
                    break;
                }

                if (r > ratio) { ratio = r; }
            }

            if (bestSubset == null || bestF == null) {
                return new BatchResult(pos, qr, SOLQ_FLOAT, ratio, ns, epochs.size(), nAmb, ambValues, ambParams);
            }

            List<Integer> fixedArIdx = new ArrayList<>();
            for (int si : bestSubset) fixedArIdx.add(si);

            double[] Qpa_ar = new double[3 * bestNfix];
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < bestNfix; j++) {
                    Qpa_ar[i + j * 3] = Qpa[i + arIdx.get(fixedArIdx.get(j)) * 3];
                }
            }

            double[] QaaFixSub = new double[bestNfix * bestNfix];
            for (int i = 0; i < bestNfix; i++) {
                for (int j = 0; j < bestNfix; j++) {
                    QaaFixSub[i + j * bestNfix] = QaaAR[fixedArIdx.get(i) + fixedArIdx.get(j) * nAR];
                }
            }
            double[] QaaInvFix = new double[bestNfix * bestNfix];
            System.arraycopy(QaaFixSub, 0, QaaInvFix, 0, bestNfix * bestNfix);
            if (MatrixUtil.matinv(QaaInvFix, bestNfix) != 0) {
                return new BatchResult(pos, qr, SOLQ_FLOAT, ratio, ns, epochs.size(), nAmb, ambValues, ambParams);
            }

            double[] da = new double[bestNfix];
            for (int i = 0; i < bestNfix; i++) da[i] = aAR[fixedArIdx.get(i)] - bestF[i];

            double[] QaaInvDa = new double[bestNfix];
            MatrixUtil.matmul("NN", bestNfix, 1, bestNfix, QaaInvFix, da, QaaInvDa);

            double[] posFix = new double[3];
            System.arraycopy(pos, 0, posFix, 0, 3);
            MatrixUtil.matmul("NN", 3, 1, bestNfix, -1.0, Qpa_ar, QaaInvDa, 1.0, posFix);

            float[] qrFix = new float[6];
            double[] QpaQaaInv = new double[3 * bestNfix];
            MatrixUtil.matmul("NN", 3, bestNfix, bestNfix, Qpa_ar, QaaInvFix, QpaQaaInv);
            double[] Qfix = new double[9];
            System.arraycopy(Qpp, 0, Qfix, 0, 9);
            MatrixUtil.matmul("NT", 3, 3, bestNfix, -1.0, QpaQaaInv, Qpa_ar, 1.0, Qfix);
            qrFix[0] = (float) Qfix[0]; qrFix[1] = (float) Qfix[4]; qrFix[2] = (float) Qfix[8];
            qrFix[3] = (float) Qfix[1]; qrFix[4] = (float) Qfix[5]; qrFix[5] = (float) Qfix[2];

            if (bestNfix < Math.max(8, nAR / 3)) {
                return new BatchResult(pos, qr, SOLQ_FLOAT, ratio, ns,
                        epochs.size(), nAmb, ambValues, ambParams);
            }

            double[] fixedAmb = new double[nAmb];
            System.arraycopy(ambValues, 0, fixedAmb, 0, nAmb);
            java.util.Set<Integer> fixedIndices = new java.util.HashSet<>();
            for (int i = 0; i < bestNfix; i++) {
                int fullIdx = arIdx.get(fixedArIdx.get(i));
                fixedAmb[fullIdx] = bestF[i];
                fixedIndices.add(fullIdx);
            }

            double postFixRms = computePostFixPhaseRms(epochs, ambParams, nav, opt,
                    posFix, fixedAmb, nf, refSatMap, fixedIndices);

            if (postFixRms > 0.05) {
                return new BatchResult(pos, qr, SOLQ_FLOAT, ratio, ns,
                        epochs.size(), nAmb, ambValues, ambParams);
            }

            return new BatchResult(posFix, qrFix, SOLQ_FIX, ratio, ns, epochs.size(), nAmb, ambValues, ambParams);
        }

        // Should not reach here (fixedOriginal != null handled above)
        return new BatchResult(pos, qr, SOLQ_FLOAT, ratio, ns, epochs.size(), nAmb, ambValues, ambParams);
    }

    // ---------------------------------------------------------------
    // chooseRefSats: pick stable ref sat per (system, freq) across all epochs
    // ---------------------------------------------------------------

    /**
     * Choose the reference satellite per (system, freq) based on longest
     * continuous visibility with valid phase. Returns refSatMap[sysIdx][freq]
     * where sysIdx maps SYS_GPS=0, SYS_GLO=1, SYS_GAL=2, SYS_QZS=3, SYS_CMP=4, SYS_IRN=5, SYS_SBS=6.
     */
    private static int[][] chooseRefSats(List<EpochData> epochs,
                                          ProcessingOptions opt, int nf) {
        int[] systems = {SYS_GPS, SYS_GLO, SYS_GAL, SYS_QZS, SYS_CMP, SYS_IRN, SYS_SBS};
        // Count visibility per (sat, freq)
        int[][] visCount = new int[MAXSAT][NFREQ];

        for (EpochData ed : epochs) {
            for (int i = 0; i < ed.ns; i++) {
                int sat = ed.sat[i];
                for (int f = 0; f < nf; f++) {
                    if (ed.obs[ed.iu[i]].L[f] != 0.0 && ed.obs[ed.ir[i]].L[f] != 0.0) {
                        visCount[sat - 1][f]++;
                    }
                }
            }
        }

        // refSatMap[sysIdx][freq] = sat number (1-based), 0 if none
        int[][] refSatMap = new int[systems.length][NFREQ];

        for (int si = 0; si < systems.length; si++) {
            int sys = systems[si];
            if ((sys & opt.navsys) == 0) continue;

            // Prefer same ref sat for L1 (freq=0) and L5 (freq=2) to maximize
            // WL ionosphere cancellation. Pick satellite with longest dual-freq
            // visibility: min(visCount[L1], visCount[L5]).
            int bestDualSat = 0;
            int bestDualCount = 0;
            for (int s = 1; s <= MAXSAT; s++) {
                if (SatelliteUtil.satsys(s)[0] != sys) continue;
                int dualMin = Math.min(visCount[s - 1][0],
                        nf > 2 ? visCount[s - 1][2] : 0);
                if (dualMin > bestDualCount) {
                    bestDualCount = dualMin;
                    bestDualSat = s;
                }
            }

            if (bestDualSat != 0) {
                // Use the dual-freq satellite as ref for ALL frequencies
                for (int f = 0; f < nf; f++) {
                    if (visCount[bestDualSat - 1][f] > 0) {
                        refSatMap[si][f] = bestDualSat;
                    } else {
                        // This freq not observed by dual-freq ref → per-freq fallback
                        int bestSat = 0;
                        int bestCount = 0;
                        for (int s = 1; s <= MAXSAT; s++) {
                            if (SatelliteUtil.satsys(s)[0] != sys) continue;
                            if (visCount[s - 1][f] > bestCount) {
                                bestCount = visCount[s - 1][f];
                                bestSat = s;
                            }
                        }
                        refSatMap[si][f] = bestSat;
                    }
                }
            } else {
                // No dual-freq satellite: fall back to per-freq logic
                for (int f = 0; f < nf; f++) {
                    int bestSat = 0;
                    int bestCount = 0;
                    for (int s = 1; s <= MAXSAT; s++) {
                        if (SatelliteUtil.satsys(s)[0] != sys) continue;
                        if (visCount[s - 1][f] > bestCount) {
                            bestCount = visCount[s - 1][f];
                            bestSat = s;
                        }
                    }
                    refSatMap[si][f] = bestSat;
                }
            }
        }

        return refSatMap;
    }

    /**
     * Get system index for refSatMap.
     */
    private static int sysIndex(int sys) {
        switch (sys) {
            case SYS_GPS: return 0;
            case SYS_GLO: return 1;
            case SYS_GAL: return 2;
            case SYS_QZS: return 3;
            case SYS_CMP: return 4;
            case SYS_IRN: return 5;
            case SYS_SBS: return 6;
            default: return -1;
        }
    }

    // ---------------------------------------------------------------
    // scanDdAmbiguities: build DD ambiguity parameter table
    // ---------------------------------------------------------------

    /**
     * Scan epochs and build DD ambiguity parameters. Each parameter represents
     * a DD pair (refSat - sat) for a given freq, with segment tracking for gaps.
     */
    static List<AmbParam> scanDdAmbiguities(List<EpochData> epochs,
                                              ProcessingOptions opt, int nf,
                                              int[][] refSatMap) {
        List<AmbParam> params = new ArrayList<>();
        boolean[][] active = new boolean[MAXSAT][NFREQ];
        int[][] currentSegment = new int[MAXSAT][NFREQ];
        // Previous geometry-free combination per satellite for slip detection
        // GF = L[0]*c/f0 - L[f]*c/ff (meters), rover SD
        double[][] prevGf = new double[MAXSAT][NFREQ];
        boolean[][] hasGf = new boolean[MAXSAT][NFREQ];

        for (int ep = 0; ep < epochs.size(); ep++) {
            EpochData ed = epochs.get(ep);
            boolean[][] seenThisEpoch = new boolean[MAXSAT][NFREQ];

            // Phase 1: GF slip detection for ALL satellites (including potential ref sats)
            boolean[][] slipMap = new boolean[MAXSAT][NFREQ];
            if (nf >= 2) {
                for (int i = 0; i < ed.ns; i++) {
                    int sat = ed.sat[i];
                    int sys = SatelliteUtil.satsys(sat)[0];
                    if ((sys & opt.navsys) == 0) continue;

                    double L0rov = ed.obs[ed.iu[i]].L[0];
                    double L0base = ed.obs[ed.ir[i]].L[0];
                    double f0 = SignalUtil.sat2freq(sat, ed.obs[ed.iu[i]].code[0], null);
                    if (L0rov == 0 || L0base == 0 || f0 <= 0) continue;
                    double sdL0 = (L0rov - L0base) * CLIGHT / f0;

                    for (int f = 1; f < nf; f++) {
                        double Lfrov = ed.obs[ed.iu[i]].L[f];
                        double Lfbase = ed.obs[ed.ir[i]].L[f];
                        double ff = SignalUtil.sat2freq(sat, ed.obs[ed.iu[i]].code[f], null);
                        if (Lfrov == 0 || Lfbase == 0 || ff <= 0) continue;
                        double sdLf = (Lfrov - Lfbase) * CLIGHT / ff;
                        double gf = sdL0 - sdLf;
                        if (hasGf[sat - 1][f] && Math.abs(gf - prevGf[sat - 1][f]) > 0.05) {
                            slipMap[sat - 1][0] = true;
                            slipMap[sat - 1][f] = true;
                        }
                        prevGf[sat - 1][f] = gf;
                        hasGf[sat - 1][f] = true;
                    }
                }
            }

            // Phase 2: Propagate ref sat slips to all DD pairs using that ref
            int[] systems = {SYS_GPS, SYS_GLO, SYS_GAL, SYS_QZS, SYS_CMP, SYS_IRN, SYS_SBS};
            boolean[][] refSlipForce = new boolean[MAXSAT][NFREQ];
            for (int si = 0; si < systems.length; si++) {
                if ((systems[si] & opt.navsys) == 0) continue;
                for (int f = 0; f < nf; f++) {
                    int refSat = refSatMap[si][f];
                    if (refSat == 0) continue;
                    if (slipMap[refSat - 1][f]) {
                        // Ref sat has slip on freq f → force new segment for all
                        // active DD pairs in this system/freq group
                        for (int s = 0; s < MAXSAT; s++) {
                            if (active[s][f] && SatelliteUtil.satsys(s + 1)[0] == systems[si]) {
                                refSlipForce[s][f] = true;
                            }
                        }
                    }
                }
            }

            // Phase 3: Build/update DD segments
            for (int i = 0; i < ed.ns; i++) {
                int sat = ed.sat[i];
                int sys = SatelliteUtil.satsys(sat)[0];
                if ((sys & opt.navsys) == 0) continue;
                int si = sysIndex(sys);
                if (si < 0) continue;

                for (int f = 0; f < nf; f++) {
                    int refSat = refSatMap[si][f];
                    if (refSat == 0 || refSat == sat) continue;

                    if (ed.obs[ed.iu[i]].L[f] == 0.0 || ed.obs[ed.ir[i]].L[f] == 0.0) continue;

                    boolean refVisible = false;
                    for (int r = 0; r < ed.ns; r++) {
                        if (ed.sat[r] == refSat) {
                            if (ed.obs[ed.iu[r]].L[f] != 0.0 && ed.obs[ed.ir[r]].L[f] != 0.0) {
                                refVisible = true;
                            }
                            break;
                        }
                    }
                    if (!refVisible) continue;

                    seenThisEpoch[sat - 1][f] = true;

                    // New segment on: first appearance, own slip, OR ref sat slip
                    boolean needNew = !active[sat - 1][f]
                                   || slipMap[sat - 1][f]
                                   || refSlipForce[sat - 1][f];

                    if (needNew) {
                        if (active[sat - 1][f]) {
                            for (int p = params.size() - 1; p >= 0; p--) {
                                AmbParam ap = params.get(p);
                                if (ap.sat == sat && ap.freq == f && ap.endEpoch < 0) {
                                    ap.endEpoch = ep - 1;
                                    break;
                                }
                            }
                            currentSegment[sat - 1][f]++;
                        }
                        AmbParam ap = new AmbParam(refSat, sat, f,
                                currentSegment[sat - 1][f], ep);
                        params.add(ap);
                        active[sat - 1][f] = true;
                    }
                }
            }

            // End segments for sats that disappeared
            for (int s = 0; s < MAXSAT; s++) {
                for (int f = 0; f < nf; f++) {
                    if (active[s][f] && !seenThisEpoch[s][f]) {
                        for (int p = params.size() - 1; p >= 0; p--) {
                            AmbParam ap = params.get(p);
                            if (ap.sat == s + 1 && ap.freq == f && ap.endEpoch < 0) {
                                ap.endEpoch = ep - 1;
                                break;
                            }
                        }
                        active[s][f] = false;
                        currentSegment[s][f]++;
                    }
                }
            }
        }

        // Close open segments
        for (AmbParam ap : params) {
            if (ap.endEpoch < 0) ap.endEpoch = epochs.size() - 1;
        }

        return params;
    }

    // ---------------------------------------------------------------
    // makeDdObs: form DD observations for one epoch
    // ---------------------------------------------------------------

    /**
     * For one epoch, produce list of DD observations (phase + code).
     */
    private static List<DdObs> makeDdObs(EpochData ed, double[] pos,
                                          ProcessingOptions opt, Navigation nav, int nf,
                                          double[] yRov, double[] eRov, double[] azelRov,
                                          double[] freqRov,
                                          double[] yBase, double[] azelBase, double[] freqBase,
                                          List<AmbParam> ambParams, double[] ambValues,
                                          int epochIdx, double bl,
                                          int[][] refSatMap) {
        int[] systems = {SYS_GPS, SYS_GLO, SYS_GAL, SYS_QZS, SYS_CMP, SYS_IRN, SYS_SBS};
        List<DdObs> result = new ArrayList<>();

        for (int si = 0; si < systems.length; si++) {
            int sys = systems[si];
            if ((sys & opt.navsys) == 0) continue;

            for (int f = 0; f < nf; f++) {
                int refSat = refSatMap[si][f];
                if (refSat == 0) continue;

                // Find ref sat index in this epoch's common sats
                int refIdx = -1;
                for (int i = 0; i < ed.ns; i++) {
                    if (ed.sat[i] == refSat) { refIdx = i; break; }
                }
                if (refIdx < 0) continue;

                int iuRef = ed.iu[refIdx];
                int irRef = ed.ir[refIdx] - ed.nu;

                // Verify ref sat has valid phase
                double yPhRefRov = yRov[f + iuRef * nf * 2];
                double yPhRefBase = yBase[f + irRef * nf * 2];
                if (yPhRefRov == 0 || yPhRefBase == 0) continue;

                // SNR mask check for ref sat
                double elRef = azelRov[1 + iuRef * 2];
                if (elRef < opt.elmin) continue;
                if (Spp.testsnr(0, f, elRef, ed.obs[iuRef].SNR[f],
                        opt.snrmask)) continue;
                if (Spp.testsnr(1, f, azelBase[1 + irRef * 2],
                        ed.obs[ed.ir[refIdx]].SNR[f], opt.snrmask)) continue;

                // Ref sat variance (for DD variance computation)
                double varRef = Rtkpos.varerr(refSat, sys, elRef,
                        ed.obs[iuRef].SNR[f],
                        ed.obs[ed.ir[refIdx]].SNR[f],
                        bl, 0, f, opt);

                // DD code zdres for ref sat
                double yCdRefRov = yRov[f + nf + iuRef * nf * 2];
                double yCdRefBase = yBase[f + nf + irRef * nf * 2];

                for (int j = 0; j < ed.ns; j++) {
                    if (j == refIdx) continue;
                    int sat = ed.sat[j];
                    if (SatelliteUtil.satsys(sat)[0] != sys) continue;

                    int iuJ = ed.iu[j];
                    int irJ = ed.ir[j] - ed.nu;

                    // Elevation and SNR check for non-ref sat
                    double elJ = azelRov[1 + iuJ * 2];
                    if (elJ < opt.elmin) continue;
                    if (Spp.testsnr(0, f, elJ, ed.obs[iuJ].SNR[f],
                            opt.snrmask)) continue;
                    if (Spp.testsnr(1, f, azelBase[1 + irJ * 2],
                            ed.obs[ed.ir[j]].SNR[f], opt.snrmask)) continue;

                    // Non-ref sat variance
                    double varJ = Rtkpos.varerr(sat, sys, elJ,
                            ed.obs[iuJ].SNR[f],
                            ed.obs[ed.ir[j]].SNR[f],
                            bl, 0, f, opt);

                    // Position derivatives: H = -e[ref] + e[j] (same as ddres)
                    double[] hPos = new double[3];
                    for (int k = 0; k < 3; k++) {
                        hPos[k] = -eRov[k + iuRef * 3] + eRov[k + iuJ * 3];
                    }

                    // Wavelength
                    double freq = freqRov[f + iuJ * nf];
                    double lambda = (freq > 0) ? CLIGHT / freq : 0;

                    // Phase DD
                    double yPhJRov = yRov[f + iuJ * nf * 2];
                    double yPhJBase = yBase[f + irJ * nf * 2];
                    if (yPhJRov != 0 && yPhJBase != 0 && lambda > 0) {
                        double ddPhase = (yPhRefRov - yPhRefBase) - (yPhJRov - yPhJBase);

                        // Find DD ambiguity index
                        int ddAmbIdx = findDdAmbIdx(ambParams, refSat, sat, f, epochIdx);

                        if (ddAmbIdx >= 0) {
                            DdObs obs = new DdObs();
                            obs.v = ddPhase - lambda * ambValues[ddAmbIdx];
                            obs.hPos = hPos;
                            obs.lambda = lambda;
                            obs.ddAmbIdx = ddAmbIdx;
                            obs.var = varRef + varJ; // DD phase variance (diagonal approx)
                            obs.isPhase = true;
                            result.add(obs);
                        }
                    }

                    // Code DD
                    double yCdJRov = yRov[f + nf + iuJ * nf * 2];
                    double yCdJBase = yBase[f + nf + irJ * nf * 2];
                    if (yCdRefRov != 0 && yCdRefBase != 0 && yCdJRov != 0 && yCdJBase != 0) {
                        double ddCode = (yCdRefRov - yCdRefBase) - (yCdJRov - yCdJBase);

                        // Code variance from varerr with f+nf (code index)
                        double varRefCode = Rtkpos.varerr(refSat, sys, elRef,
                                ed.obs[iuRef].SNR[f], ed.obs[ed.ir[refIdx]].SNR[f],
                                bl, 0, f + nf, opt);
                        double varJCode = Rtkpos.varerr(sat, sys, elJ,
                                ed.obs[iuJ].SNR[f], ed.obs[ed.ir[j]].SNR[f],
                                bl, 0, f + nf, opt);
                        double varCode = varRefCode + varJCode;

                        DdObs obs = new DdObs();
                        obs.v = ddCode;
                        obs.hPos = hPos;
                        obs.lambda = 0;
                        obs.ddAmbIdx = -1;
                        obs.var = varCode;
                        obs.isPhase = false;
                        result.add(obs);
                    }
                }
            }
        }

        return result;
    }

    /**
     * Find DD ambiguity index for given (refSat, sat, freq) at given epoch.
     */
    private static int findDdAmbIdx(List<AmbParam> ambParams, int refSat, int sat,
                                     int freq, int epochIdx) {
        for (int i = 0; i < ambParams.size(); i++) {
            AmbParam ap = ambParams.get(i);
            if (ap.refSat == refSat && ap.sat == sat && ap.freq == freq
                && epochIdx >= ap.startEpoch && epochIdx <= ap.endEpoch) {
                return i;
            }
        }
        return -1;
    }

    // ---------------------------------------------------------------
    // initDdAmbFromZdres: initialize DD ambiguities from DD code-phase
    // ---------------------------------------------------------------

    /**
     * Initialize DD ambiguity estimates from DD code-phase differences.
     * N_DD_init = (dd_phase - dd_code) / lambda (cycles).
     */
    private static double[] initDdAmbFromZdres(List<EpochData> epochs,
                                                List<AmbParam> ambParams,
                                                Navigation nav, ProcessingOptions opt,
                                                double[] pos, int nf) {
        double[] ambValues = new double[ambParams.size()];

        for (int j = 0; j < ambParams.size(); j++) {
            AmbParam ap = ambParams.get(j);
            int f = ap.freq;
            double sumBias = 0;
            int count = 0;

            int maxEp = Math.min(ap.endEpoch + 1, epochs.size());
            for (int ep = ap.startEpoch; ep < maxEp; ep++) {
                EpochData ed = epochs.get(ep);

                // Find ref sat and non-ref sat indices
                int refIdx = -1, satIdx = -1;
                for (int i = 0; i < ed.ns; i++) {
                    if (ed.sat[i] == ap.refSat) refIdx = i;
                    if (ed.sat[i] == ap.sat) satIdx = i;
                }
                if (refIdx < 0 || satIdx < 0) continue;

                // Compute zdres for rover
                double[] yRov = new double[nf * 2 * ed.nu];
                double[] eRov = new double[3 * ed.nu];
                double[] azelRov = new double[2 * ed.nu];
                double[] freqRov = new double[nf * ed.nu];
                ObsData[] obsRov = new ObsData[ed.nu];
                System.arraycopy(ed.obs, 0, obsRov, 0, ed.nu);

                if (Rtkpos.zdres(0, obsRov, ed.nu, ed.rs, ed.dts, ed.var, ed.svh,
                                  nav, pos, opt, yRov, eRov, azelRov, freqRov) == 0) continue;

                // Compute zdres for base
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

                // DD phase (meters from zdres)
                double phRefRov = yRov[f + iuRef * nf * 2];
                double phRefBase = yBase[f + irRef * nf * 2];
                double phJRov = yRov[f + iuJ * nf * 2];
                double phJBase = yBase[f + irJ * nf * 2];

                // DD code (meters from zdres)
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

                // N_DD = (dd_phase - dd_code) / lambda
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

    /**
     * Compute post-fix DD phase residual RMS (in meters) for FIXED ambiguities only.
     * Only includes DD observations whose ambiguity index is in fixedIndices.
     * This avoids dilution by near-zero float residuals.
     * Correct fix → RMS ≈ 0.005m (noise level).
     * Wrong fix → RMS > 0.05m (cycle-level residuals).
     */
    private static double computePostFixPhaseRms(
            List<EpochData> epochs, List<AmbParam> ambParams,
            Navigation nav, ProcessingOptions opt,
            double[] pos, double[] fixedAmb, int nf,
            int[][] refSatMap, java.util.Set<Integer> fixedIndices) {
        double sumSq = 0;
        int count = 0;
        double[] dr = new double[3];
        double bl = Rtkpos.baseline(pos, opt.rb, dr);

        for (int ep = 0; ep < epochs.size(); ep++) {
            EpochData ed = epochs.get(ep);
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

            // Only check phase DD residuals
            List<DdObs> ddObs = makeDdObs(ed, pos, opt, nav, nf,
                    yRov, eRov, azelRov, freqRov,
                    yBase, azelBase, freqBase,
                    ambParams, fixedAmb, ep, bl, refSatMap);

            for (DdObs dd : ddObs) {
                if (dd.isPhase && dd.ddAmbIdx >= 0
                        && fixedIndices.contains(dd.ddAmbIdx)) {
                    sumSq += dd.v * dd.v;
                    count++;
                }
            }
        }

        return count > 0 ? Math.sqrt(sumSq / count) : 999.0;
    }

    // ---------------------------------------------------------------
    // sppPosition
    // ---------------------------------------------------------------

    /**
     * Get initial position from SPP on the first epoch with valid data.
     */
    private static double[] sppPosition(List<List<ObsData>> roverEpochs,
                                         Navigation nav, ProcessingOptions opt) {
        for (List<ObsData> epoch : roverEpochs) {
            if (epoch == null || epoch.isEmpty()) continue;
            ObsData[] obs = epoch.toArray(new ObsData[0]);
            Solution sol = new Solution();
            Spp.SatStatus[] ssat = new Spp.SatStatus[MAXSAT];
            for (int i = 0; i < MAXSAT; i++) ssat[i] = new Spp.SatStatus();
            StringBuilder msg = new StringBuilder();

            if (Spp.pntpos(obs, obs.length, nav, opt, sol, null, ssat, msg) != 0
                && sol.stat != SOLQ_NONE) {
                return new double[]{sol.rr[0], sol.rr[1], sol.rr[2]};
            }
        }
        return null;
    }

    // ---------------------------------------------------------------
    // preprocessEpochs: unchanged from original
    // ---------------------------------------------------------------

    /**
     * Preprocess all epochs: match rover/base, compute satellite positions.
     */
    private static List<EpochData> preprocessEpochs(List<List<ObsData>> roverEpochs,
                                                      List<List<ObsData>> baseEpochs,
                                                      Navigation nav,
                                                      ProcessingOptions opt) {
        List<EpochData> result = new ArrayList<>();
        int baseIdx = 0;
        int nf = FilterState.NF(opt);

        for (List<ObsData> roverEpoch : roverEpochs) {
            if (roverEpoch == null || roverEpoch.isEmpty()) continue;

            double roverTime = roverEpoch.get(0).time.time + roverEpoch.get(0).time.sec;
            List<ObsData> bestBase = null;
            double bestDt = Double.MAX_VALUE;

            for (int j = Math.max(0, baseIdx - 1); j < baseEpochs.size(); j++) {
                List<ObsData> be = baseEpochs.get(j);
                if (be == null || be.isEmpty()) continue;
                double baseTime = be.get(0).time.time + be.get(0).time.sec;
                double dt = Math.abs(roverTime - baseTime);
                if (dt < bestDt) {
                    bestDt = dt;
                    bestBase = be;
                    baseIdx = j;
                }
                if (baseTime > roverTime + opt.maxtdiff) break;
            }

            if (bestBase == null || bestDt > opt.maxtdiff) continue;

            // Merge observations
            List<ObsData> merged = new ArrayList<>(roverEpoch.size() + bestBase.size());
            merged.addAll(roverEpoch);
            merged.addAll(bestBase);

            EpochData ed = new EpochData();
            ed.obs = merged.toArray(new ObsData[0]);
            ed.nu = roverEpoch.size();
            ed.nr = bestBase.size();
            int n = ed.nu + ed.nr;

            ed.rs = new double[6 * n];
            ed.dts = new double[2 * n];
            ed.var = new double[n];
            ed.svh = new int[n];

            EphemerisCalc.satposs(ed.obs[0].time, ed.obs, n, nav, opt.sateph,
                                   ed.rs, ed.dts, ed.var, ed.svh);

            // Base zdres to get azel for selsat
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

            // Use base zdres for azel
            double[] azelFull = new double[2 * n];
            Rtkpos.zdres(1, obsBase, ed.nr, rsBase, dtsBase, varBase, svhBase,
                          nav, opt.rb, opt, yBase, eBase, azelBase, freqBase);
            System.arraycopy(azelBase, 0, azelFull, ed.nu * 2, 2 * ed.nr);

            // Also compute rover azel for selsat
            double[] yRov = new double[nf * 2 * ed.nu];
            double[] eRov = new double[3 * ed.nu];
            double[] azelRov = new double[2 * ed.nu];
            double[] freqRov = new double[nf * ed.nu];
            ObsData[] obsRov = new ObsData[ed.nu];
            System.arraycopy(ed.obs, 0, obsRov, 0, ed.nu);
            Rtkpos.zdres(0, obsRov, ed.nu, ed.rs, ed.dts, ed.var, ed.svh,
                          nav, opt.rb, opt, yRov, eRov, azelRov, freqRov);
            System.arraycopy(azelRov, 0, azelFull, 0, 2 * ed.nu);

            ed.sat = new int[MAXSAT];
            ed.iu = new int[MAXSAT];
            ed.ir = new int[MAXSAT];
            ed.ns = Rtkpos.selsat(ed.obs, azelFull, ed.nu, ed.nr, opt, ed.sat, ed.iu, ed.ir);

            if (ed.ns > 0) {
                result.add(ed);
            }
        }
        return result;
    }

    // ---------------------------------------------------------------
    // scanAmbiguities: kept for backward compatibility with tests
    // ---------------------------------------------------------------

    /**
     * Scan all epochs for cycle slips and build ambiguity parameter table.
     * Kept for backward compatibility. Returns SD-style params.
     */
    static List<AmbParam> scanAmbiguities(List<EpochData> epochs, RtkState rtk,
                                            Navigation nav, ProcessingOptions opt) {
        int nf = FilterState.NF(opt);
        // Delegate to DD scan with dummy ref sats
        int[][] refSatMap = chooseRefSats(epochs, opt, nf);
        return scanDdAmbiguities(epochs, opt, nf, refSatMap);
    }

    /**
     * Initialize DD ambiguity estimates from zdres.
     * Kept for backward compatibility with tests.
     */
    static double[] initAmbFromZdres(List<EpochData> epochs,
                                              List<AmbParam> ambParams,
                                              RtkState rtk, Navigation nav,
                                              ProcessingOptions opt, double[] pos) {
        int nf = FilterState.NF(opt);
        return initDdAmbFromZdres(epochs, ambParams, nav, opt, pos, nf);
    }

    /**
     * Count unique satellites in ambiguity parameters.
     */
    private static int countSatellites(List<AmbParam> ambParams) {
        boolean[] seen = new boolean[MAXSAT];
        for (AmbParam ap : ambParams) {
            if (ap.sat > 0 && ap.sat <= MAXSAT) seen[ap.sat - 1] = true;
            if (ap.refSat > 0 && ap.refSat <= MAXSAT) seen[ap.refSat - 1] = true;
        }
        int count = 0;
        for (boolean s : seen) if (s) count++;
        return count;
    }
}
