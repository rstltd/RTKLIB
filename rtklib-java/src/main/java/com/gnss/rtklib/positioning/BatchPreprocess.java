package com.gnss.rtklib.positioning;

import com.gnss.rtklib.core.*;
import com.gnss.rtklib.model.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static com.gnss.rtklib.core.Constants.*;

/**
 * BLS preprocessing: epoch matching, satellite position computation,
 * reference satellite selection, and DD ambiguity arc scanning.
 */
final class BatchPreprocess {

    private BatchPreprocess() {}

    /** DD ambiguity parameter descriptor for BLS state vector. */
    static class AmbParam {
        final int refSat;
        final int sat;
        final int freq;
        final int segment;
        int startEpoch;
        int endEpoch;

        AmbParam(int refSat, int sat, int freq, int segment, int startEpoch) {
            this.refSat = refSat;
            this.sat = sat;
            this.freq = freq;
            this.segment = segment;
            this.startEpoch = startEpoch;
            this.endEpoch = -1;
        }
    }

    /** Per-epoch preprocessed data. */
    static class EpochData {
        ObsData[] obs;
        int nu, nr;
        double[] rs, dts;
        double[] var;
        int[] svh;
        int[] sat, iu, ir;
        int ns;
    }

    // ---------------------------------------------------------------
    // sppPosition
    // ---------------------------------------------------------------

    static double[] sppPosition(List<List<ObsData>> roverEpochs,
                                 Navigation nav, ProcessingOptions opt) {
        // Try carrier-smoothed SPP first, fall back to plain SPP
        double[] csPos = carrierSmoothedPosition(roverEpochs, nav, opt);
        if (csPos != null) return csPos;

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
     * Carrier-smoothed SPP initial position using Hatch filter.
     * <p>
     * For each satellite/frequency, applies the Hatch filter:
     *   P_smooth(k) = (1/n)*P(k) + (1-1/n)*(P_smooth(k-1) + lambda*(L(k)-L(k-1)))
     * where n is the smoothing count (capped at MAX_SMOOTH).
     * <p>
     * Smooths across the first N epochs, then runs SPP on the smoothed pseudoranges
     * from the last smoothed epoch. Falls back to null if insufficient data.
     */
    private static final int MAX_SMOOTH = 100;  // cap smoothing window
    private static final int MIN_SMOOTH_EPOCHS = 10;  // need at least this many epochs

    private static double[] carrierSmoothedPosition(List<List<ObsData>> roverEpochs,
                                                     Navigation nav, ProcessingOptions opt) {
        // Per-satellite smoothing state: key = sat*16+freq
        Map<Integer, double[]> smoothState = new HashMap<>(); // [smoothedP, prevL, count]
        int nSmoothedEpochs = 0;
        List<ObsData> lastEpoch = null;

        // Process up to MAX_SMOOTH epochs for smoothing
        int maxEpochs = Math.min(roverEpochs.size(), MAX_SMOOTH);
        for (int ei = 0; ei < maxEpochs; ei++) {
            List<ObsData> epoch = roverEpochs.get(ei);
            if (epoch == null || epoch.isEmpty()) continue;

            for (ObsData obs : epoch) {
                if (obs.sat <= 0 || obs.sat > MAXSAT) continue;

                for (int f = 0; f < NFREQ + NEXOBS; f++) {
                    if (obs.P[f] == 0.0 || obs.L[f] == 0.0) continue;

                    double freq = Spp.sat2freq(obs.sat, obs.code[f], nav);
                    if (freq <= 0) continue;
                    double lambda = CLIGHT / freq;
                    double carrier_m = obs.L[f] * lambda; // carrier in meters

                    int key = obs.sat * 16 + f;
                    double[] state = smoothState.get(key);

                    if (state == null || (obs.LLI[f] & 1) != 0) {
                        // First observation or cycle slip: reset
                        smoothState.put(key, new double[]{obs.P[f], carrier_m, 1.0});
                    } else {
                        double prevSmoothed = state[0];
                        double prevCarrier = state[1];
                        double n = Math.min(state[2] + 1, MAX_SMOOTH);
                        double predicted = prevSmoothed + (carrier_m - prevCarrier);
                        double smoothed = obs.P[f] / n + predicted * (1.0 - 1.0 / n);
                        state[0] = smoothed;
                        state[1] = carrier_m;
                        state[2] = n;
                    }
                }
            }
            nSmoothedEpochs++;
            lastEpoch = epoch;
        }

        if (nSmoothedEpochs < MIN_SMOOTH_EPOCHS || lastEpoch == null) return null;

        // Create smoothed observation copies for the last epoch
        ObsData[] smoothedObs = new ObsData[lastEpoch.size()];
        for (int i = 0; i < lastEpoch.size(); i++) {
            ObsData orig = lastEpoch.get(i);
            ObsData copy = new ObsData();
            copy.time = orig.time;
            copy.sat = orig.sat;
            copy.rcv = orig.rcv;
            copy.freq = orig.freq;
            System.arraycopy(orig.LLI, 0, copy.LLI, 0, orig.LLI.length);
            System.arraycopy(orig.code, 0, copy.code, 0, orig.code.length);
            System.arraycopy(orig.L, 0, copy.L, 0, orig.L.length);
            System.arraycopy(orig.D, 0, copy.D, 0, orig.D.length);
            System.arraycopy(orig.SNR, 0, copy.SNR, 0, orig.SNR.length);
            System.arraycopy(orig.Lstd, 0, copy.Lstd, 0, orig.Lstd.length);
            System.arraycopy(orig.Pstd, 0, copy.Pstd, 0, orig.Pstd.length);
            // Replace pseudoranges with smoothed values
            System.arraycopy(orig.P, 0, copy.P, 0, orig.P.length);
            for (int f = 0; f < copy.P.length; f++) {
                int key = orig.sat * 16 + f;
                double[] state = smoothState.get(key);
                if (state != null && state[2] >= 3) { // at least 3 epochs smoothed
                    copy.P[f] = state[0];
                }
            }
            smoothedObs[i] = copy;
        }

        // Run SPP with smoothed pseudoranges
        Solution sol = new Solution();
        Spp.SatStatus[] ssat = new Spp.SatStatus[MAXSAT];
        for (int i = 0; i < MAXSAT; i++) ssat[i] = new Spp.SatStatus();
        StringBuilder msg = new StringBuilder();

        if (Spp.pntpos(smoothedObs, smoothedObs.length, nav, opt, sol, null, ssat, msg) != 0
            && sol.stat != SOLQ_NONE) {
            return new double[]{sol.rr[0], sol.rr[1], sol.rr[2]};
        }
        return null;
    }

    // ---------------------------------------------------------------
    // preprocessEpochs
    // ---------------------------------------------------------------

    static List<EpochData> preprocessEpochs(List<List<ObsData>> roverEpochs,
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

            double[] azelFull = new double[2 * n];
            Rtkpos.zdres(1, obsBase, ed.nr, rsBase, dtsBase, varBase, svhBase,
                          nav, opt.rb, opt, yBase, eBase, azelBase, freqBase);
            System.arraycopy(azelBase, 0, azelFull, ed.nu * 2, 2 * ed.nr);

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
    // chooseRefSats
    // ---------------------------------------------------------------

    static int[][] chooseRefSats(List<EpochData> epochs,
                                  ProcessingOptions opt, int nf) {
        int[] systems = {SYS_GPS, SYS_GLO, SYS_GAL, SYS_QZS, SYS_CMP, SYS_IRN, SYS_SBS};
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

        int[][] refSatMap = new int[systems.length][NFREQ];

        for (int si = 0; si < systems.length; si++) {
            int sys = systems[si];
            if ((sys & opt.navsys) == 0) continue;

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
                for (int f = 0; f < nf; f++) {
                    if (visCount[bestDualSat - 1][f] > 0) {
                        refSatMap[si][f] = bestDualSat;
                    } else {
                        int bestSat = 0, bestCount = 0;
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
                for (int f = 0; f < nf; f++) {
                    int bestSat = 0, bestCount = 0;
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

    static int sysIndex(int sys) {
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
    // scanDdAmbiguities
    // ---------------------------------------------------------------

    static List<AmbParam> scanDdAmbiguities(List<EpochData> epochs,
                                              ProcessingOptions opt, int nf,
                                              int[][] refSatMap) {
        List<AmbParam> params = new ArrayList<>();
        boolean[][] active = new boolean[MAXSAT][NFREQ];
        int[][] currentSegment = new int[MAXSAT][NFREQ];
        double[][] prevGf = new double[MAXSAT][NFREQ];
        boolean[][] hasGf = new boolean[MAXSAT][NFREQ];

        for (int ep = 0; ep < epochs.size(); ep++) {
            EpochData ed = epochs.get(ep);
            boolean[][] seenThisEpoch = new boolean[MAXSAT][NFREQ];

            // Phase 1: GF slip detection
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

            // Phase 2: Ref sat slip propagation
            int[] systems = {SYS_GPS, SYS_GLO, SYS_GAL, SYS_QZS, SYS_CMP, SYS_IRN, SYS_SBS};
            boolean[][] refSlipForce = new boolean[MAXSAT][NFREQ];
            for (int si = 0; si < systems.length; si++) {
                if ((systems[si] & opt.navsys) == 0) continue;
                for (int f = 0; f < nf; f++) {
                    int refSat = refSatMap[si][f];
                    if (refSat == 0) continue;
                    if (slipMap[refSat - 1][f]) {
                        for (int s = 0; s < MAXSAT; s++) {
                            if (active[s][f] && SatelliteUtil.satsys(s + 1)[0] == systems[si]) {
                                refSlipForce[s][f] = true;
                            }
                        }
                    }
                }
            }

            // Phase 3: Build/update segments
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

            // End segments for disappeared sats
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

    static int countSatellites(List<AmbParam> ambParams) {
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
