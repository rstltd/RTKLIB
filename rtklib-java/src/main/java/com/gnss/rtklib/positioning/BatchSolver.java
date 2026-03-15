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
 * Reuses Rtkpos.zdres/ddres/selsat for observation processing.
 * <p>
 * State vector: [pos(3), amb_1, amb_2, ..., amb_k]
 * where each amb corresponds to a {sat, freq, segment} triple,
 * with segment incremented on cycle slip.
 */
public final class BatchSolver {

    private BatchSolver() {}

    private static final int MAX_ITER = 4;
    private static final double CONV_THRESHOLD = 1e-4; // m
    private static final int NSYS = 6;

    /**
     * Ambiguity parameter descriptor for BLS state vector.
     */
    static class AmbParam {
        final int sat;        // satellite number (1-based)
        final int freq;       // frequency index
        final int segment;    // segment number (incremented on slip)
        int startEpoch;       // first epoch of this segment
        int endEpoch;         // last epoch (-1 if ongoing)
        int blsIndex;         // index in BLS state vector (3 + j)

        AmbParam(int sat, int freq, int segment, int startEpoch) {
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
        public final double[] ambValues; // SD float ambiguity values (for diagnostics)
        public final List<AmbParam> ambParams; // ambiguity parameter descriptors (for diagnostics)

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
     * Solve RTK static using Batch Least Squares.
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

        // 3. Build minimal RtkState for ddres calls
        RtkState rtk = initMinimalState(opt, pos);

        // 4. Pre-scan: build ambiguity parameter table from slip flags
        List<AmbParam> ambParams = scanAmbiguities(epochs, rtk, nav, opt);

        // Remove short segments (< 5 epochs) that cause rank deficiency
        // and don't contribute useful ambiguity information
        ambParams.removeIf(ap -> (ap.endEpoch - ap.startEpoch + 1) < 20);
        for (int j = 0; j < ambParams.size(); j++) {
            ambParams.get(j).blsIndex = 3 + j;
        }

        // Identify ref sat per system/freq group for zero-constraint.
        // DD observations only constrain (N_ref - N_j), so we fix one SD bias per
        // group to zero by adding a tight pseudo-observation.
        List<Integer> refSatIndices = identifyRefSatIndices(ambParams, opt);

        int nx = 3 + ambParams.size();

        // 5. Iterative BLS
        double[] N = null;
        double[] b = null;

        // Initialize ambiguity estimates from zdres code-phase differences
        // zdres removes geometry (range + trop + sat clock), giving clean bias estimate
        double[] ambValues = initAmbFromZdres(epochs, ambParams, rtk, nav, opt, pos);

        // Relax outlier threshold for BLS (residuals are larger before convergence)
        double[] savedMaxinno = {opt.maxinno[0], opt.maxinno[1]};
        opt.maxinno[0] = 100.0;  // relaxed outlier rejection for BLS
        opt.maxinno[1] = 100.0;

        for (int iter = 0; iter < MAX_ITER; iter++) {
            N = new double[nx * nx];
            b = new double[nx];

            // Update rtk state with current position estimate
            System.arraycopy(pos, 0, rtk.x, 0, 3);

            int nf = FilterState.NF(opt);
            int totalObs = 0;

            for (int ep = 0; ep < epochs.size(); ep++) {
                EpochData ed = epochs.get(ep);
                if (ed.ns <= 0) continue;

                int n = ed.nu + ed.nr;

                // Reset valid satellite flags
                for (int i = 0; i < MAXSAT; i++) {
                    for (int j = 0; j < NFREQ; j++) {
                        rtk.ssat[i].vsat[j] = 0;
                        rtk.ssat[i].resp[j] = 0;
                        rtk.ssat[i].resc[j] = 0;
                    }
                }

                // Rover zdres
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

                // Base zdres
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

                // Assemble full y, e, azel, freq arrays
                double[] y = new double[nf * 2 * n];
                double[] e = new double[3 * n];
                double[] azel = new double[2 * n];
                double[] freq = new double[nf * n];

                System.arraycopy(yRov, 0, y, 0, nf * 2 * ed.nu);
                System.arraycopy(yBase, 0, y, ed.nu * nf * 2, nf * 2 * ed.nr);
                System.arraycopy(eRov, 0, e, 0, 3 * ed.nu);
                System.arraycopy(eBase, 0, e, ed.nu * 3, 3 * ed.nr);
                System.arraycopy(azelRov, 0, azel, 0, 2 * ed.nu);
                System.arraycopy(azelBase, 0, azel, ed.nu * 2, 2 * ed.nr);
                System.arraycopy(freqRov, 0, freq, 0, nf * ed.nu);
                System.arraycopy(freqBase, 0, freq, ed.nu * nf, nf * ed.nr);

                // Store SNR for varerr
                for (int i = 0; i < ed.ns; i++) {
                    for (int j = 0; j < nf; j++) {
                        rtk.ssat[ed.sat[i] - 1].snr_rover[j] = ed.obs[ed.iu[i]].SNR[j];
                        rtk.ssat[ed.sat[i] - 1].snr_base[j] = ed.obs[ed.ir[i]].SNR[j];
                    }
                }

                // Set current ambiguity values in rtk.x for ddres
                setAmbiguityStates(rtk, ambParams, ep, ambValues, iter == 0);

                // ddres: get v, H_ekf, R
                int ny = ed.ns * nf * 2 + 2;
                double[] v = new double[ny];
                double[] Hekf = new double[rtk.nx * ny];
                double[] R = new double[ny * ny];
                int[] vflg = new int[ny];

                double dt = ed.obs[0].time.timediff(ed.obs[ed.nu].time);

                // Set solution time for troposphere mapping
                rtk.sol.time = ed.obs[0].time;

                // Initialize satellite system info
                for (int i = 0; i < ed.ns; i++) {
                    rtk.ssat[ed.sat[i] - 1].sys = SatelliteUtil.satsys(ed.sat[i])[0];
                }

                int nv = Rtkpos.ddres(rtk, ed.obs, dt, rtk.x, null, ed.sat,
                                       y, e, azel, freq, ed.iu, ed.ir, ed.ns,
                                       v, Hekf, R, vflg);
                if (nv <= 0) continue;

                totalObs += nv;

                // Remap H from EKF layout to BLS layout and accumulate normal equations
                accumulateNormal(N, b, Hekf, v, R, nv, rtk.nx, nx, ambParams, ep, opt);
            }

            // Fix rank deficiency in Naa by:
            // 1. Zero-constrain ref sat ambiguities (datum definition)
            // 2. Regularize any remaining near-singular params
            double maxNaaDiag = 0;
            for (int j = 0; j < ambParams.size(); j++) {
                double d = N[(3 + j) + (3 + j) * nx];
                if (d > maxNaaDiag) maxNaaDiag = d;
            }
            double constraintWeight = maxNaaDiag * 1e6;

            // Zero-constrain ALL segments of ref satellites (not just one per freq)
            for (int refIdx : refSatIndices) {
                int refSat = ambParams.get(refIdx).sat;
                int refFreq = ambParams.get(refIdx).freq;
                for (int j = 0; j < ambParams.size(); j++) {
                    AmbParam ap = ambParams.get(j);
                    if (ap.sat == refSat && ap.freq == refFreq) {
                        int p = 3 + j;
                        N[p + p * nx] += constraintWeight;
                        b[p] += constraintWeight * (-ambValues[j]);
                    }
                }
            }

            // Also regularize ambiguities with weak observations (near-zero diagonal)
            // These are segments that don't overlap well with other sats
            double minDiag = maxNaaDiag * 1e-6;
            for (int j = 0; j < ambParams.size(); j++) {
                int p = 3 + j;
                if (N[p + p * nx] < minDiag) {
                    N[p + p * nx] += minDiag;
                }
            }

            if (totalObs < nx) {
                return new BatchResult(pos, new float[6], SOLQ_NONE, 0, 0, epochs.size(), ambParams.size());
            }

            // Solve using Schur complement reduction (eliminate ambiguities first)
            // Partition: N = [Npp Npa; Nap Naa], b = [bp; ba]
            // where p=position(3), a=ambiguity(nAmb)
            int nAmb = ambParams.size();
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

            // Reduced normal: Nred = Npp - Npa * Naa^-1 * Nap
            double[] NaaInv = new double[nAmb * nAmb];
            System.arraycopy(Naa, 0, NaaInv, 0, nAmb * nAmb);
            if (MatrixUtil.matinv(NaaInv, nAmb) != 0) {
                return new BatchResult(pos, new float[6], SOLQ_NONE, 0, 0, epochs.size(), ambParams.size());
            }

            // Npa * NaaInv → tmp[3 x nAmb]
            double[] tmp = new double[3 * nAmb];
            MatrixUtil.matmul("NN", 3, nAmb, nAmb, Npa, NaaInv, tmp);

            // Nred = Npp - tmp * Nap (Nap = Npa^T)
            double[] Nred = new double[9];
            System.arraycopy(Npp, 0, Nred, 0, 9);
            MatrixUtil.matmul("NT", 3, 3, nAmb, -1.0, tmp, Npa, 1.0, Nred);

            // bred = bp - tmp * ba
            double[] bred = new double[3];
            System.arraycopy(bp, 0, bred, 0, 3);
            MatrixUtil.matmul("NN", 3, 1, nAmb, -1.0, tmp, ba, 1.0, bred);

            // Solve for position: dxp = Nred^-1 * bred
            double[] NredInv = new double[9];
            System.arraycopy(Nred, 0, NredInv, 0, 9);
            if (MatrixUtil.matinv(NredInv, 3) != 0) {
                return new BatchResult(pos, new float[6], SOLQ_NONE, 0, 0, epochs.size(), ambParams.size());
            }

            double[] dxp = new double[3];
            MatrixUtil.matmul("NN", 3, 1, 3, NredInv, bred, dxp);

            // Back-substitute: dxa = NaaInv * (ba - Nap * dxp)
            double[] baRed = new double[nAmb];
            System.arraycopy(ba, 0, baRed, 0, nAmb);
            // Nap = Npa^T, so Nap * dxp = Npa^T * dxp
            MatrixUtil.matmul("TN", nAmb, 1, 3, -1.0, Npa, dxp, 1.0, baRed);

            double[] dxa = new double[nAmb];
            MatrixUtil.matmul("NN", nAmb, 1, nAmb, NaaInv, baRed, dxa);

            // Combine result
            double[] result = new double[nx];
            System.arraycopy(dxp, 0, result, 0, 3);
            System.arraycopy(dxa, 0, result, 3, nAmb);

            // Update both position and ambiguities (one-shot solve)
            pos[0] += result[0];
            pos[1] += result[1];
            pos[2] += result[2];

            for (int j = 0; j < ambParams.size(); j++) {
                ambValues[j] += result[3 + j];
            }

            double dpos = Math.sqrt(result[0] * result[0] + result[1] * result[1] + result[2] * result[2]);
            if (dpos < CONV_THRESHOLD) break;
        }

        // Restore outlier threshold
        opt.maxinno[0] = savedMaxinno[0];
        opt.maxinno[1] = savedMaxinno[1];

        // 6. Compute covariances using Schur complement from last iteration's N
        // Extract blocks from the LAST N matrix
        int nAmb = ambParams.size();
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

        // NaaInv for ambiguity covariance
        double[] NaaInv = new double[nAmb * nAmb];
        System.arraycopy(Naa, 0, NaaInv, 0, nAmb * nAmb);
        if (MatrixUtil.matinv(NaaInv, nAmb) != 0) {
            return new BatchResult(pos, new float[6], SOLQ_FLOAT, 0, ns, epochs.size(), nAmb);
        }

        // Reduced position covariance: Qpp = (Npp - Npa * NaaInv * Nap)^-1
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

        // 7. LAMBDA AR
        // Only include well-observed ambiguities (exclude ref sats and regularized params).
        if (nAmb < 1 || opt.modear == 0) {
            return new BatchResult(pos, qr, SOLQ_FLOAT, 0, ns, epochs.size(), nAmb, ambValues, ambParams);
        }

        // Build list of AR-eligible ambiguity indices.
        // Exclude: ref sat params (constrained to zero) and weakly observed params.
        // Detect via Naa diagonal: constrained params have huge diagonal (>> normal),
        // weakly observed have tiny diagonal.
        double maxNorm = 0, medianDiag = 0;
        double[] diagArr = new double[nAmb];
        for (int j = 0; j < nAmb; j++) {
            diagArr[j] = Naa[j + j * nAmb];
            if (diagArr[j] > maxNorm) maxNorm = diagArr[j];
        }
        // Sort to find median (ignoring constrained outliers)
        double[] sorted = diagArr.clone();
        java.util.Arrays.sort(sorted);
        medianDiag = sorted[nAmb / 2];

        List<Integer> arIdx = new ArrayList<>();
        for (int j = 0; j < nAmb; j++) {
            // Skip constrained params (diagonal >> 100x median)
            if (diagArr[j] > medianDiag * 100) continue;
            // Skip weakly observed (diagonal < 1% of median)
            if (diagArr[j] < medianDiag * 0.01) continue;
            arIdx.add(j);
        }

        int nAR = arIdx.size();
        if (nAR < 1) {
            return new BatchResult(pos, qr, SOLQ_FLOAT, 0, ns, epochs.size(), nAmb, ambValues, ambParams);
        }

        // Extract AR-eligible sub-block from ORIGINAL Naa (before constraint),
        // then invert to get proper covariance for LAMBDA.
        double[] aAR = new double[nAR];
        double[] NaaAR = new double[nAR * nAR];
        for (int i = 0; i < nAR; i++) {
            aAR[i] = ambValues[arIdx.get(i)];
            for (int j = 0; j < nAR; j++) {
                NaaAR[i + j * nAR] = Naa[arIdx.get(i) + arIdx.get(j) * nAmb];
            }
        }

        // Remove constraints from the AR sub-block (they shouldn't be there
        // since we excluded ref sats, but regularization might remain)
        double[] QaAR = new double[nAR * nAR];
        System.arraycopy(NaaAR, 0, QaAR, 0, nAR * nAR);
        if (MatrixUtil.matinv(QaAR, nAR) != 0) {
            return new BatchResult(pos, qr, SOLQ_FLOAT, 0, ns, epochs.size(), nAmb, ambValues, ambParams);
        }

        double[] F = new double[nAR * 2];
        double[] s = new double[2];
        int info = Lambda.lambda(nAR, 2, aAR, QaAR, F, s);

        float ratio;
        if (info != 0 || s[0] <= 0) {
            return new BatchResult(pos, qr, SOLQ_FLOAT, 0, ns, epochs.size(), nAmb, ambValues, ambParams);
        }

        ratio = (float) (s[1] / s[0]);
        if (ratio > 999.9f) ratio = 999.9f;

        float thres = (float) Rtkpos.computeAdaptiveArThreshold(nAR, opt.thresar[0]);
        if (ratio < thres) {
            return new BatchResult(pos, qr, SOLQ_FLOAT, ratio, ns, epochs.size(), nAmb, ambValues, ambParams);
        }

        // Apply fixed solution using AR-eligible subset
        // pos_fix = pos_float - Qpa_ar * Qa_ar^-1 * (a_float - a_fix)
        // Qpa_ar: cross-covariance between pos and AR-eligible ambiguities
        double[] Qpa_full = new double[3 * nAmb];
        MatrixUtil.matmul("NN", 3, nAmb, 3, Qpp, tmp3a, Qpa_full);
        for (int i = 0; i < 3 * nAmb; i++) Qpa_full[i] = -Qpa_full[i];

        double[] Qpa_ar = new double[3 * nAR];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < nAR; j++) {
                Qpa_ar[i + j * 3] = Qpa_full[i + arIdx.get(j) * 3];
            }
        }

        double[] da = new double[nAR];
        for (int i = 0; i < nAR; i++) da[i] = aAR[i] - F[i];

        double[] NaaDa = new double[nAR];
        MatrixUtil.matmul("NN", nAR, 1, nAR, NaaAR, da, NaaDa);

        double[] posFix = new double[3];
        System.arraycopy(pos, 0, posFix, 0, 3);
        MatrixUtil.matmul("NN", 3, 1, nAR, -1.0, Qpa_ar, NaaDa, 1.0, posFix);

        // Fixed covariance
        float[] qrFix = new float[6];
        double[] QpaNaa = new double[3 * nAR];
        MatrixUtil.matmul("NN", 3, nAR, nAR, Qpa_ar, NaaAR, QpaNaa);
        double[] Qfix = new double[9];
        System.arraycopy(Qpp, 0, Qfix, 0, 9);
        MatrixUtil.matmul("NT", 3, 3, nAR, -1.0, QpaNaa, Qpa_ar, 1.0, Qfix);
        qrFix[0] = (float) Qfix[0]; qrFix[1] = (float) Qfix[4]; qrFix[2] = (float) Qfix[8];
        qrFix[3] = (float) Qfix[1]; qrFix[4] = (float) Qfix[5]; qrFix[5] = (float) Qfix[2];

        return new BatchResult(posFix, qrFix, SOLQ_FIX, ratio, ns, epochs.size(), nAmb, ambValues, ambParams);
    }

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

    /**
     * Scan all epochs for cycle slips and build the ambiguity parameter table.
     * Uses RINEX LLI flags for slip detection.
     */
    static List<AmbParam> scanAmbiguities(List<EpochData> epochs, RtkState rtk,
                                            Navigation nav, ProcessingOptions opt) {
        int nf = FilterState.NF(opt);
        List<AmbParam> params = new ArrayList<>();
        // Track current segment per (sat, freq)
        int[][] currentSegment = new int[MAXSAT][NFREQ];
        boolean[][] active = new boolean[MAXSAT][NFREQ];

        for (int ep = 0; ep < epochs.size(); ep++) {
            EpochData ed = epochs.get(ep);

            // Mark all as inactive for this epoch
            boolean[][] seenThisEpoch = new boolean[MAXSAT][NFREQ];

            for (int i = 0; i < ed.ns; i++) {
                int sat = ed.sat[i];
                int satIdx = sat - 1;

                for (int f = 0; f < nf; f++) {
                    // Check for carrier phase availability
                    if (ed.obs[ed.iu[i]].L[f] == 0.0 || ed.obs[ed.ir[i]].L[f] == 0.0) {
                        continue;
                    }

                    seenThisEpoch[satIdx][f] = true;

                    // Only create new segment on data gap (satellite reappears after
                    // being absent). LLI flags are ignored because u-blox receivers
                    // often set them spuriously; BLS relies on having continuous
                    // segments and can detect true slips through post-fit residuals.
                    if (!active[satIdx][f]) {
                        // End previous segment if active
                        if (active[satIdx][f]) {
                            for (int p = params.size() - 1; p >= 0; p--) {
                                AmbParam ap = params.get(p);
                                if (ap.sat == sat && ap.freq == f && ap.endEpoch < 0) {
                                    ap.endEpoch = ep - 1;
                                    break;
                                }
                            }
                            currentSegment[satIdx][f]++;
                        }

                        // Start new segment
                        AmbParam ap = new AmbParam(sat, f, currentSegment[satIdx][f], ep);
                        ap.blsIndex = 3 + params.size();
                        params.add(ap);
                        active[satIdx][f] = true;
                    }
                }
            }

            // End segments for satellites that disappeared
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
                    }
                }
            }
        }

        // Close any still-open segments
        for (AmbParam ap : params) {
            if (ap.endEpoch < 0) ap.endEpoch = epochs.size() - 1;
        }

        return params;
    }

    /**
     * Identify ref satellite indices (one per system/freq group) for zero-constraint.
     * Returns list of indices into ambParams that should be constrained to zero.
     */
    private static List<Integer> identifyRefSatIndices(List<AmbParam> ambParams,
                                                        ProcessingOptions opt) {
        int nf = FilterState.NF(opt);
        List<Integer> refs = new ArrayList<>();
        for (int m = 0; m < NSYS; m++) {
            for (int f = 0; f < nf; f++) {
                int bestIdx = -1;
                int bestSpan = 0;
                for (int j = 0; j < ambParams.size(); j++) {
                    AmbParam ap = ambParams.get(j);
                    if (ap.freq != f) continue;
                    int sys = SatelliteUtil.satsys(ap.sat)[0];
                    if (!Rtkpos.testSys(sys, m)) continue;
                    int span = ap.endEpoch - ap.startEpoch + 1;
                    if (span > bestSpan) { bestSpan = span; bestIdx = j; }
                }
                if (bestIdx >= 0) refs.add(bestIdx);
            }
        }
        return refs;
    }

    /**
     * Initialize a minimal RtkState for use with ddres.
     */
    private static RtkState initMinimalState(ProcessingOptions opt, double[] pos) {
        RtkState rtk = new RtkState();
        rtk.init(opt);
        System.arraycopy(pos, 0, rtk.x, 0, 3);
        System.arraycopy(opt.rb, 0, rtk.rb, 0, 3);
        // Initialize satellite system info (same as relpos)
        for (int i = 0; i < MAXSAT; i++) {
            rtk.ssat[i].sys = SatelliteUtil.satsys(i + 1)[0];
        }
        return rtk;
    }

    /**
     * Set ambiguity states in the RtkState for ddres to use.
     * Maps BLS ambiguity parameters to EKF IB() indices.
     */
    private static void setAmbiguityStates(RtkState rtk, List<AmbParam> ambParams,
                                            int epoch, double[] ambValues,
                                            boolean firstIter) {
        // Clear all ambiguity states
        int na = rtk.na;
        for (int i = na; i < rtk.nx; i++) {
            rtk.x[i] = 0.0;
        }

        // Set ambiguities for parameters active at this epoch
        for (AmbParam ap : ambParams) {
            if (epoch >= ap.startEpoch && epoch <= ap.endEpoch) {
                int ekfIdx = RtkState.IB(ap.sat, ap.freq, rtk.opt);
                if (ekfIdx < rtk.nx) {
                    if (ambValues != null) {
                        rtk.x[ekfIdx] = ambValues[ap.blsIndex - 3];
                    }
                    // Set non-zero to indicate active (ddres checks for x[IB]!=0)
                    if (rtk.x[ekfIdx] == 0.0 && !firstIter) {
                        rtk.x[ekfIdx] = 1e-10; // tiny non-zero to keep ddres happy
                    }
                }
            }
        }
    }

    /**
     * Remap H from EKF layout to BLS layout and accumulate into normal equations.
     * N += H_bls' * R^-1 * H_bls
     * b += H_bls' * R^-1 * v
     */
    private static void accumulateNormal(double[] N, double[] bvec,
                                          double[] Hekf, double[] v, double[] R,
                                          int nv, int nxEkf, int nxBls,
                                          List<AmbParam> ambParams, int epoch,
                                          ProcessingOptions opt) {
        // Build H_bls [nxBls x nv] from H_ekf [nxEkf x nv]
        // H_ekf is stored column-major: H_ekf[k + i*nxEkf] = dv[i]/dx[k]
        double[] Hbls = new double[nxBls * nv];

        for (int i = 0; i < nv; i++) {
            // Position: direct copy
            for (int k = 0; k < 3; k++) {
                Hbls[k + i * nxBls] = Hekf[k + i * nxEkf];
            }

            // Ambiguities: remap from EKF IB(sat,f) to BLS index
            for (AmbParam ap : ambParams) {
                if (epoch >= ap.startEpoch && epoch <= ap.endEpoch) {
                    int ekfIdx = RtkState.IB(ap.sat, ap.freq, opt);
                    if (ekfIdx < nxEkf) {
                        Hbls[ap.blsIndex + i * nxBls] = Hekf[ekfIdx + i * nxEkf];
                    }
                }
            }
        }

        // Hbls is stored as J^T (nxBls × nv, column-major) where J is the Jacobian
        // Hbls[k + i*nxBls] = J[i,k] = dv[i]/dx[k]
        // Normal equation: N += J^T * W * J, b += J^T * W * v
        //
        // For efficiency with block-diagonal R, use diagonal-only inverse
        // (DD covariance is block-diagonal but within small blocks).
        // Full inverse is expensive for large nv, so use explicit loops.

        // Full R^-1 (DD covariance is block-diagonal, NOT diagonal)
        double[] Rinv = new double[nv * nv];
        System.arraycopy(R, 0, Rinv, 0, nv * nv);
        if (MatrixUtil.matinv(Rinv, nv) != 0) {
            return; // skip this epoch if R is singular
        }

        // Compute W*J (nv × nxBls) and W*v (nv × 1) using full R^-1
        // J[i,p] = Hbls[p + i*nxBls]
        // (W*J)[i,p] = sum_k Rinv[i,k] * J[k,p]
        // N += J^T * W * J, b += J^T * W * v

        // Wv = Rinv * v
        double[] Wv = new double[nv];
        for (int i = 0; i < nv; i++) {
            for (int k = 0; k < nv; k++) {
                Wv[i] += Rinv[i + k * nv] * v[k];
            }
        }

        // b += J^T * Wv = sum_i J[i,p] * Wv[i]
        for (int i = 0; i < nv; i++) {
            for (int p = 0; p < nxBls; p++) {
                bvec[p] += Hbls[p + i * nxBls] * Wv[i];
            }
        }

        // WJ = Rinv * J (nv × nxBls)
        double[] WJ = new double[nv * nxBls];
        for (int i = 0; i < nv; i++) {
            for (int p = 0; p < nxBls; p++) {
                double sum = 0;
                for (int k = 0; k < nv; k++) {
                    sum += Rinv[i + k * nv] * Hbls[p + k * nxBls];
                }
                WJ[p + i * nxBls] = sum; // same storage as J
            }
        }

        // N += J^T * WJ
        for (int i = 0; i < nv; i++) {
            for (int p = 0; p < nxBls; p++) {
                double Jip = Hbls[p + i * nxBls];
                if (Jip == 0) continue;
                for (int q = p; q < nxBls; q++) {
                    double val = Jip * WJ[q + i * nxBls];
                    N[p + q * nxBls] += val;
                    if (p != q) N[q + p * nxBls] += val;
                }
            }
        }
    }

    /**
     * Initialize SD ambiguity estimates from zdres code-phase differences.
     * zdres removes geometry (range + tropo + sat clock), so the code-phase
     * difference directly gives the SD phase bias with minimal geometric bias.
     */
    static double[] initAmbFromZdres(List<EpochData> epochs,
                                              List<AmbParam> ambParams,
                                              RtkState rtk, Navigation nav,
                                              ProcessingOptions opt, double[] pos) {
        int nf = FilterState.NF(opt);
        double[] ambValues = new double[ambParams.size()];

        for (int j = 0; j < ambParams.size(); j++) {
            AmbParam ap = ambParams.get(j);
            int f = ap.freq;
            double sumBias = 0;
            int count = 0;

            int maxEp = Math.min(ap.endEpoch + 1, epochs.size());
            for (int ep = ap.startEpoch; ep < maxEp; ep++) {
                EpochData ed = epochs.get(ep);

                int satIdx = -1;
                for (int i = 0; i < ed.ns; i++) {
                    if (ed.sat[i] == ap.sat) { satIdx = i; break; }
                }
                if (satIdx < 0) continue;

                int n = ed.nu + ed.nr;

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

                int iu = ed.iu[satIdx], ir = ed.ir[satIdx] - ed.nu;
                if (iu >= ed.nu || ir < 0 || ir >= ed.nr) continue;

                // SD zdres: phase and code
                double yPhaseRov = yRov[f + iu * nf * 2];
                double yPhaseBase = yBase[f + ir * nf * 2];
                double yCodeRov = yRov[f + nf + iu * nf * 2];
                double yCodeBase = yBase[f + nf + ir * nf * 2];

                if (yPhaseRov == 0 || yPhaseBase == 0 || yCodeRov == 0 || yCodeBase == 0) continue;

                double sdPhase = yPhaseRov - yPhaseBase; // meters
                double sdCode = yCodeRov - yCodeBase;     // meters

                double freq = freqRov[f + iu * nf];
                if (freq <= 0) continue;

                // bias = (sd_phase - sd_code) * freq / CLIGHT (cycles)
                // sd_phase(m) = lambda * N_sd + noise → sd_phase(m) / lambda = N_sd + noise
                sumBias += (sdPhase - sdCode) * freq / CLIGHT;
                count++;

                // Only need a few epochs for averaging
                if (count >= 10) break;
            }

            if (count > 0) {
                ambValues[j] = sumBias / count;
            }
        }
        return ambValues;
    }

    /**
     * Initialize SD ambiguity estimates from code-phase differences.
     * Uses averaging over multiple epochs for noise reduction.
     * SD phase bias ≈ (SD_phase - SD_code) * freq / CLIGHT (in cycles).
     */
    private static double[] initAmbiguities(List<EpochData> epochs,
                                             List<AmbParam> ambParams,
                                             RtkState rtk, Navigation nav,
                                             ProcessingOptions opt, double[] pos) {
        double[] ambValues = new double[ambParams.size()];

        for (int j = 0; j < ambParams.size(); j++) {
            AmbParam ap = ambParams.get(j);
            int f = ap.freq;

            // Average code-phase bias over all epochs in this segment
            double sumBias = 0;
            int count = 0;
            int maxEp = Math.min(ap.endEpoch + 1, epochs.size());

            for (int ep = ap.startEpoch; ep < maxEp; ep++) {
                EpochData ed = epochs.get(ep);

                int satIdx = -1;
                for (int i = 0; i < ed.ns; i++) {
                    if (ed.sat[i] == ap.sat) { satIdx = i; break; }
                }
                if (satIdx < 0) continue;

                int iu = ed.iu[satIdx];
                int ir = ed.ir[satIdx];

                double L_rov = ed.obs[iu].L[f];
                double L_base = ed.obs[ir].L[f];
                double P_rov = ed.obs[iu].P[f];
                double P_base = ed.obs[ir].P[f];

                if (L_rov == 0.0 || L_base == 0.0 || P_rov == 0.0 || P_base == 0.0) continue;

                double freq = SignalUtil.sat2freq(ap.sat, ed.obs[iu].code[f], nav);
                if (freq == 0.0) continue;

                double sdPhase = L_rov - L_base; // cycles
                double sdCode = P_rov - P_base;  // meters

                sumBias += sdPhase - sdCode * freq / CLIGHT;
                count++;
            }

            if (count > 0) {
                ambValues[j] = sumBias / count;
            }
        }

        return ambValues;
    }

    /**
     * Count unique satellites in ambiguity parameters.
     */
    private static int countSatellites(List<AmbParam> ambParams) {
        boolean[] seen = new boolean[MAXSAT];
        for (AmbParam ap : ambParams) {
            if (ap.sat > 0 && ap.sat <= MAXSAT) seen[ap.sat - 1] = true;
        }
        int count = 0;
        for (boolean s : seen) if (s) count++;
        return count;
    }
}
