package com.gnss.rtklib.positioning;

import com.gnss.rtklib.PostProcessor;
import com.gnss.rtklib.core.GTime;
import com.gnss.rtklib.core.SatelliteUtil;
import com.gnss.rtklib.io.ConfigReader;
import com.gnss.rtklib.io.RinexReader;
import com.gnss.rtklib.io.SolutionWriter;
import com.gnss.rtklib.model.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIf;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static com.gnss.rtklib.core.Constants.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for SNR weighting in varerr() — validates the formula,
 * parameter sweep, and SNR=0 guard safety.
 * Uses combined-nophasereset mode (matching static.conf).
 */
class SnrWeightingTest {

    // Station B (L1+L5, has SNR data)
    private static final Path STATIC_DATA = findPath("test/data/static");
    private static final double DTTOL = 10.0;

    // Station A (RINEX 2.10, no SNR)
    private static final Path RINEX_DATA = findPath("test/data/rinex");

    private static Path findPath(String rel) {
        Path p = Path.of("").toAbsolutePath().resolve("../" + rel);
        if (Files.exists(p)) return p;
        return Path.of("").toAbsolutePath().resolve(rel);
    }

    static boolean stationBAvailable() {
        return Files.exists(STATIC_DATA.resolve("rover_L1L5.obs"))
            && Files.exists(STATIC_DATA.resolve("base_L1L5.obs"))
            && Files.exists(STATIC_DATA.resolve("rover_L1L5.nav"))
            && Files.exists(STATIC_DATA.resolve("static.conf"));
    }

    static boolean stationAAvailable() {
        return Files.exists(RINEX_DATA.resolve("07590920.05o"))
            && Files.exists(RINEX_DATA.resolve("30400920.05o"))
            && Files.exists(RINEX_DATA.resolve("07590920.05n"));
    }

    // ---------------------------------------------------------------
    // Test 1: varerr SNR term formula validation
    // ---------------------------------------------------------------

    @Test
    void testVarerrSnrTerm() {
        ProcessingOptions opt = new ProcessingOptions();
        opt.mode = PMODE_STATIC;
        opt.nf = 2;
        opt.navsys = SYS_GPS;
        opt.ionoopt = IONOOPT_BRDC;
        opt.eratio[0] = 300.0;
        opt.eratio[1] = 300.0;
        opt.err[1] = 0.003;
        opt.err[2] = 0.003;
        opt.err[5] = 52.0;  // snrMax
        opt.err[6] = 0.003; // enable SNR weighting

        double el = 45.0 * D2R;
        int sat = 1, sys = SYS_GPS;
        double bl = 0, dt = 0;
        int f = 0; // phase L1

        // Baseline: no SNR weighting
        opt.err[6] = 0.0;
        double varNoSnr = Rtkpos.varerr(sat, sys, el, 45.0, 45.0, bl, dt, f, opt);

        // Enable SNR weighting
        opt.err[6] = 0.003;

        // SNR=52 (ceiling) → 10^(0.1*max(52-52,0)) = 10^0 = 1 per side
        double varSnr52 = Rtkpos.varerr(sat, sys, el, 52.0, 52.0, bl, dt, f, opt);
        double snrExtra52 = varSnr52 - varNoSnr;
        assertEquals(0.000018, snrExtra52, 1e-10, "SNR=52 extra variance");

        // SNR=45 → 10^(0.1*7) = 10^0.7 ≈ 5.012 per side
        double varSnr45 = Rtkpos.varerr(sat, sys, el, 45.0, 45.0, bl, dt, f, opt);
        double snrExtra45 = varSnr45 - varNoSnr;
        double expected45 = 0.003 * 0.003 * 2 * Math.pow(10, 0.7);
        assertEquals(expected45, snrExtra45, 1e-10, "SNR=45 extra variance");

        // SNR=35 → 10^(0.1*17) = 10^1.7 ≈ 50.12 per side
        double varSnr35 = Rtkpos.varerr(sat, sys, el, 35.0, 35.0, bl, dt, f, opt);
        double snrExtra35 = varSnr35 - varNoSnr;
        double expected35 = 0.003 * 0.003 * 2 * Math.pow(10, 1.7);
        assertEquals(expected35, snrExtra35, 1e-10, "SNR=35 extra variance");

        // SNR=0 (missing) → guard skips, extra = 0
        double varSnr0 = Rtkpos.varerr(sat, sys, el, 0.0, 0.0, bl, dt, f, opt);
        assertEquals(varNoSnr, varSnr0, 1e-15, "SNR=0 should produce same variance as no SNR weighting");

        // Mixed: one side has SNR=0 → guard skips
        double varMixed = Rtkpos.varerr(sat, sys, el, 45.0, 0.0, bl, dt, f, opt);
        assertEquals(varNoSnr, varMixed, 1e-15, "One side SNR=0 should skip SNR weighting");

        // Print summary
        System.out.println("[SNR varerr] Formula validation:");
        System.out.printf("  No SNR weighting : var = %.10f%n", varNoSnr);
        System.out.printf("  SNR=52 (ceiling) : var = %.10f  (extra = %.10f)%n", varSnr52, snrExtra52);
        System.out.printf("  SNR=45           : var = %.10f  (extra = %.10f)%n", varSnr45, snrExtra45);
        System.out.printf("  SNR=35           : var = %.10f  (extra = %.10f)%n", varSnr35, snrExtra35);
        System.out.printf("  SNR=0  (missing) : var = %.10f  (extra = 0)%n", varSnr0);
    }

    // ---------------------------------------------------------------
    // Test 2: SNR weighting effect (Station B, combined-nophasereset)
    // ---------------------------------------------------------------

    @Test
    @EnabledIf("stationBAvailable")
    void testSnrWeightingEffect() throws Exception {
        RunResult baseline = runStationBCombined(0.0);
        RunResult snrEnabled = runStationBCombined(0.003);

        double[] statsBase = computeStdStats(baseline);
        double[] statsSnr = computeStdStats(snrEnabled);

        System.out.println("[SNR Effect] Station B comparison (combined-nophasereset):");
        System.out.println("  err[6] | fix  | float | total | fix%");
        System.out.printf("  0.000  | %4d | %5d | %5d | %4.1f%%%n",
            baseline.fixCount, baseline.floatCount, baseline.total,
            100.0 * baseline.fixCount / baseline.total);
        System.out.printf("  0.003  | %4d | %5d | %5d | %4.1f%%%n",
            snrEnabled.fixCount, snrEnabled.floatCount, snrEnabled.total,
            100.0 * snrEnabled.fixCount / snrEnabled.total);
        System.out.println();
        System.out.println("[SNR Effect] Position STD comparison (fix epochs only):");
        System.out.println("                 EKF-reported std (mm)     Coordinate scatter (mm)");
        System.out.println("  err[6] |  sdE     sdN     sdU    |  scatE   scatN   scatU");
        System.out.printf("  0.000  | %6.2f  %6.2f  %6.2f  | %6.2f  %6.2f  %6.2f%n",
            statsBase[0], statsBase[1], statsBase[2],
            statsBase[3], statsBase[4], statsBase[5]);
        System.out.printf("  0.003  | %6.2f  %6.2f  %6.2f  | %6.2f  %6.2f  %6.2f%n",
            statsSnr[0], statsSnr[1], statsSnr[2],
            statsSnr[3], statsSnr[4], statsSnr[5]);

        assertTrue(baseline.fixCount > 0, "Baseline should produce fix solutions");
        assertTrue(snrEnabled.fixCount > 0, "SNR-weighted should produce fix solutions");
        assertTrue(snrEnabled.fixCount >= baseline.fixCount / 2,
            "SNR weighting should not reduce fix count by more than half");
    }

    // ---------------------------------------------------------------
    // Test 3: Parameter sweep (combined-nophasereset)
    // ---------------------------------------------------------------

    @Test
    @EnabledIf("stationBAvailable")
    void testSnrWeightingParameterSweep() throws Exception {
        double[] errValues = {0.0, 0.001, 0.003, 0.005, 0.010};

        System.out.println("[SNR Sweep] Station B parameter sweep (combined-nophasereset):");
        System.out.println("  err[6] | fix  | float | total | fix%  |  sdE(mm)  sdN(mm)  sdU(mm) | scatE(mm) scatN(mm) scatU(mm)");

        RunResult firstResult = null;
        for (double err6 : errValues) {
            RunResult r = runStationBCombined(err6);
            if (firstResult == null) firstResult = r;
            double[] stats = computeStdStats(r);
            System.out.printf("  %.3f  | %4d | %5d | %5d | %4.1f%% | %8.2f %8.2f %8.2f | %9.2f %9.2f %9.2f%n",
                err6, r.fixCount, r.floatCount, r.total,
                100.0 * r.fixCount / r.total,
                stats[0], stats[1], stats[2],
                stats[3], stats[4], stats[5]);
        }

        assertTrue(firstResult.total > 10, "Should process more than 10 epochs");
    }

    // ---------------------------------------------------------------
    // Test 5: Output POS and STAT files (combined-nophasereset)
    // ---------------------------------------------------------------

    @Test
    @EnabledIf("stationBAvailable")
    void testOutputPosAndStatFiles() throws Exception {
        Path outDir = STATIC_DATA.resolve("snr_results");
        Files.createDirectories(outDir);

        String roverObs = STATIC_DATA.resolve("rover_L1L5.obs").toString();
        String baseObs = STATIC_DATA.resolve("base_L1L5.obs").toString();
        String navFile = STATIC_DATA.resolve("rover_L1L5.nav").toString();

        double[] errValues = {0.0, 0.003};
        for (double err6 : errValues) {
            String tag = String.format("err6_%.3f", err6);
            Path posFile = outDir.resolve(tag + ".pos");
            Path statFile = outDir.resolve(tag + ".pos.stat");

            // POS: use PostProcessor.processRtk (combined-nophasereset)
            ProcessingOptions popt = loadConf();
            popt.err[6] = err6;
            SolutionOptions sopt = new SolutionOptions();
            sopt.posf = SOLF_LLH;
            sopt.timef = 1;
            sopt.timeu = 3;
            sopt.outhead = 1;
            sopt.outopt = 1;
            PostProcessor.processRtk(roverObs, baseObs, navFile,
                posFile.toString(), popt, sopt);

            // STAT: forward + backward (matching C combined-nophasereset)
            try (PrintWriter statOut = new PrintWriter(Files.newBufferedWriter(statFile))) {
                writeStatBothPasses(err6, statOut);
            }

            System.out.printf("[SNR Output] Written: %s  (%d bytes)%n", posFile, Files.size(posFile));
            System.out.printf("[SNR Output] Written: %s  (%d bytes)%n", statFile, Files.size(statFile));
            assertTrue(Files.size(posFile) > 0, "POS file should not be empty");
            assertTrue(Files.size(statFile) > 0, "STAT file should not be empty");
        }
    }

    /**
     * Write STAT for both forward and backward passes (combined-nophasereset).
     * Backward pass continues from forward state without reset.
     */
    private void writeStatBothPasses(double err6, PrintWriter statOut) throws Exception {
        Navigation nav = new Navigation();
        RinexReader.readNav(STATIC_DATA.resolve("rover_L1L5.nav").toString(), nav);
        List<List<ObsData>> roverEpochs = RinexReader.readObs(
            STATIC_DATA.resolve("rover_L1L5.obs").toString(), nav, 1);
        List<List<ObsData>> baseEpochs = RinexReader.readObs(
            STATIC_DATA.resolve("base_L1L5.obs").toString(), nav, 2);

        ProcessingOptions opt = loadConf();
        opt.err[6] = err6;

        RtkState rtk = new RtkState();
        rtk.init(opt);
        int nf = FilterState.NF(opt);

        // Forward pass
        for (List<ObsData> roverEpoch : roverEpochs) {
            if (roverEpoch == null || roverEpoch.isEmpty()) continue;
            List<ObsData> baseEpoch = findMatchingBase(roverEpoch.get(0).time, baseEpochs);
            if (baseEpoch == null) continue;
            ObsData[] obs = mergeObs(roverEpoch, baseEpoch);
            Rtkpos.rtkpos(rtk, obs, obs.length, nav);
            if (rtk.sol.stat != SOLQ_NONE) writeStatEpoch(statOut, rtk, nf);
        }

        // Backward pass (no state reset = combined-nophasereset)
        List<List<ObsData>> bwdEpochs = new ArrayList<>(roverEpochs);
        Collections.reverse(bwdEpochs);
        for (List<ObsData> roverEpoch : bwdEpochs) {
            if (roverEpoch == null || roverEpoch.isEmpty()) continue;
            List<ObsData> baseEpoch = findMatchingBase(roverEpoch.get(0).time, baseEpochs);
            if (baseEpoch == null) continue;
            ObsData[] obs = mergeObs(roverEpoch, baseEpoch);
            Rtkpos.rtkpos(rtk, obs, obs.length, nav);
            if (rtk.sol.stat != SOLQ_NONE) writeStatEpoch(statOut, rtk, nf);
        }

        statOut.flush();
    }

    // ---------------------------------------------------------------
    // Test 4: No SNR data safe with guard (Station A)
    // ---------------------------------------------------------------

    @Test
    @EnabledIf("stationAAvailable")
    void testNoSnrDataSafeWithGuard() throws Exception {
        double[] basePosA = {-3978242.4348, 3382841.1715, 3649902.7667};

        Navigation nav = new Navigation();
        RinexReader.readNav(RINEX_DATA.resolve("07590920.05n").toString(), nav);
        List<List<ObsData>> roverEpochs = RinexReader.readObs(
            RINEX_DATA.resolve("07590920.05o").toString(), nav, 1);
        List<List<ObsData>> baseEpochs = RinexReader.readObs(
            RINEX_DATA.resolve("30400920.05o").toString(), nav, 2);

        int[] result0 = runStationA(roverEpochs, baseEpochs, nav, basePosA, 0.0);
        int[] result1 = runStationA(roverEpochs, baseEpochs, nav, basePosA, 0.003);

        System.out.println("[SNR Guard] Station A (no SNR data):");
        System.out.printf("  err[6]=0.000: fix=%d, float=%d, total=%d%n",
            result0[0], result0[1], result0[2]);
        System.out.printf("  err[6]=0.003: fix=%d, float=%d, total=%d%n",
            result1[0], result1[1], result1[2]);

        assertEquals(result0[0], result1[0],
            "Fix count should be identical when SNR=0 (guard active)");
        assertEquals(result0[1], result1[1],
            "Float count should be identical when SNR=0 (guard active)");
    }

    // ---------------------------------------------------------------
    // Combined-nophasereset runner
    // ---------------------------------------------------------------

    private static class RunResult {
        int fixCount, floatCount, total;
        List<Solution> solutions = new ArrayList<>();
    }

    /**
     * Run Station B with combined-nophasereset:
     * forward pass → backward pass (no state reset) → combres smoother.
     */
    private RunResult runStationBCombined(double err6) throws Exception {
        Navigation nav = new Navigation();
        RinexReader.readNav(STATIC_DATA.resolve("rover_L1L5.nav").toString(), nav);
        List<List<ObsData>> roverEpochs = RinexReader.readObs(
            STATIC_DATA.resolve("rover_L1L5.obs").toString(), nav, 1);
        List<List<ObsData>> baseEpochs = RinexReader.readObs(
            STATIC_DATA.resolve("base_L1L5.obs").toString(), nav, 2);

        ProcessingOptions opt = loadConf();
        opt.err[6] = err6;

        // Forward pass
        RtkState rtk = new RtkState();
        rtk.init(opt);
        List<Solution> solf = new ArrayList<>();
        processRtkPass(roverEpochs, baseEpochs, rtk, nav, solf);

        // Backward pass (no state reset = combined-nophasereset)
        List<List<ObsData>> bwdEpochs = new ArrayList<>(roverEpochs);
        Collections.reverse(bwdEpochs);
        List<Solution> solb = new ArrayList<>();
        processRtkPass(bwdEpochs, baseEpochs, rtk, nav, solb);

        // Combine via smoother
        SolutionOptions sopt = new SolutionOptions();
        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw);
        PostProcessor.combres(solf, solb, opt, sopt, pw);
        pw.flush();

        // Parse combined solutions from combres output
        RunResult result = new RunResult();
        // combres writes to pw via SolutionWriter — collect from solf/solb merge
        // Instead, re-collect combined solutions by running combres into a solution list
        result.solutions = collectCombined(solf, solb, opt);
        for (Solution s : result.solutions) {
            result.total++;
            if (s.stat == SOLQ_FIX) result.fixCount++;
            else if (s.stat == SOLQ_FLOAT) result.floatCount++;
        }

        return result;
    }

    /**
     * Replicate combres logic to collect combined Solution objects.
     */
    private List<Solution> collectCombined(List<Solution> solf, List<Solution> solb,
                                            ProcessingOptions popt) {
        List<Solution> combined = new ArrayList<>();
        int i = 0, j = solb.size() - 1;

        while (i < solf.size() && j >= 0) {
            Solution sf = solf.get(i);
            Solution sb = solb.get(j);
            double tt = sf.time.timediff(sb.time);

            Solution sols;
            if (tt < -0.005) {
                sols = sf.copy();
                i++;
            } else if (tt > 0.005) {
                sols = sb.copy();
                j--;
            } else {
                // Time match: pick better quality or smooth
                int[] PRI = {0, 7, 1, 6, 2, 5, 3, 4};
                if (PRI[sf.stat] < PRI[sb.stat]) {
                    sols = sf.copy();
                } else if (PRI[sf.stat] > PRI[sb.stat]) {
                    sols = sb.copy();
                } else {
                    // Same quality: apply smoother
                    sols = sf.copy();
                    double[] Qf = expandQr(sf.qr), Qb = expandQr(sb.qr);
                    double[] Qs = new double[9], xs = new double[3];
                    if (com.gnss.rtklib.core.MatrixUtil.smoother(
                            sf.rr, Qf, sb.rr, Qb, 3, xs, Qs) == 0) {
                        System.arraycopy(xs, 0, sols.rr, 0, 3);
                        sols.qr[0] = (float) Qs[0]; sols.qr[1] = (float) Qs[4];
                        sols.qr[2] = (float) Qs[8]; sols.qr[3] = (float) Qs[1];
                        sols.qr[4] = (float) Qs[5]; sols.qr[5] = (float) Qs[2];
                    }
                }
                i++; j--;
            }
            combined.add(sols);
        }
        // Remaining forward
        while (i < solf.size()) combined.add(solf.get(i++).copy());
        // Remaining backward
        while (j >= 0) combined.add(solb.get(j--).copy());

        return combined;
    }

    private static double[] expandQr(float[] qr) {
        double[] Q = new double[9];
        Q[0] = qr[0]; Q[4] = qr[1]; Q[8] = qr[2];
        Q[1] = Q[3] = qr[3]; Q[5] = Q[7] = qr[4]; Q[2] = Q[6] = qr[5];
        return Q;
    }

    private void processRtkPass(List<List<ObsData>> roverEpochs,
                                 List<List<ObsData>> baseEpochs,
                                 RtkState rtk, Navigation nav,
                                 List<Solution> solBuffer) {
        for (List<ObsData> roverEpoch : roverEpochs) {
            if (roverEpoch == null || roverEpoch.isEmpty()) continue;

            List<ObsData> baseEpoch = findMatchingBase(roverEpoch.get(0).time, baseEpochs);
            if (baseEpoch == null) continue;

            ObsData[] obs = mergeObs(roverEpoch, baseEpoch);
            int stat = Rtkpos.rtkpos(rtk, obs, obs.length, nav);
            if (stat != 0 && rtk.sol.stat != SOLQ_NONE) {
                solBuffer.add(rtk.sol.copy());
            }
        }
    }

    // ---------------------------------------------------------------
    // Station A runner (forward-only, for guard test)
    // ---------------------------------------------------------------

    private int[] runStationA(List<List<ObsData>> roverEpochs,
                               List<List<ObsData>> baseEpochs,
                               Navigation nav, double[] basePos,
                               double err6) {
        ProcessingOptions opt = createStationAOptions(basePos);
        opt.err[6] = err6;

        RtkState rtk = new RtkState();
        rtk.init(opt);

        int fixCount = 0, floatCount = 0, total = 0;
        int maxEpochs = Math.min(roverEpochs.size(), 120);

        for (int ep = 0; ep < maxEpochs; ep++) {
            List<ObsData> roverEpoch = roverEpochs.get(ep);
            if (roverEpoch == null || roverEpoch.isEmpty()) continue;

            List<ObsData> baseEpoch = findMatchingBase(roverEpoch.get(0).time, baseEpochs);
            if (baseEpoch == null) continue;

            ObsData[] obs = mergeObs(roverEpoch, baseEpoch);
            Rtkpos.rtkpos(rtk, obs, obs.length, nav);
            total++;

            if (rtk.sol.stat == SOLQ_FIX) fixCount++;
            else if (rtk.sol.stat == SOLQ_FLOAT) floatCount++;
        }

        return new int[]{fixCount, floatCount, total};
    }

    // ---------------------------------------------------------------
    // Options loading
    // ---------------------------------------------------------------

    /**
     * Load processing options from test/data/static/static.conf.
     * Uses combined-nophasereset as configured.
     */
    private ProcessingOptions loadConf() {
        try {
            ProcessingOptions opt = new ProcessingOptions();
            SolutionOptions sopt = new SolutionOptions();
            ConfigReader.load(STATIC_DATA.resolve("static.conf").toString(), opt, sopt);
            return opt;
        } catch (Exception e) {
            throw new RuntimeException("Failed to load static.conf", e);
        }
    }

    private ProcessingOptions createStationAOptions(double[] basePos) {
        ProcessingOptions opt = new ProcessingOptions();
        opt.mode = PMODE_STATIC;
        opt.nf = 2;
        opt.navsys = SYS_GPS;
        opt.elmin = 15.0 * D2R;
        opt.ionoopt = IONOOPT_BRDC;
        opt.tropopt = TROPOPT_SAAS;
        opt.modear = 3;
        opt.dynamics = 0;
        opt.rb[0] = basePos[0];
        opt.rb[1] = basePos[1];
        opt.rb[2] = basePos[2];
        opt.thresar[0] = 3.0;
        opt.eratio[0] = 300.0;
        opt.eratio[1] = 300.0;
        opt.err[1] = 0.003;
        opt.err[2] = 0.003;
        opt.minfix = 10;
        opt.niter = 1;
        return opt;
    }

    // ---------------------------------------------------------------
    // STAT output
    // ---------------------------------------------------------------

    private static void writeStatEpoch(PrintWriter out, RtkState rtk, int nf) {
        double[] wt = rtk.sol.time.time2gpst();
        int week = (int) wt[0];
        double tow = wt[1];

        double[] xa = new double[3];
        for (int i = 0; i < 3; i++) xa[i] = i < rtk.na ? rtk.xa[i] : 0.0;
        out.printf("$POS,%d,%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f%n",
            week, tow, rtk.sol.stat,
            rtk.x[0], rtk.x[1], rtk.x[2], xa[0], xa[1], xa[2]);

        out.printf("$CLK,%d,%.3f,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f%n",
            week, tow, rtk.sol.stat, 1,
            rtk.sol.dtr[0] * 1E9, rtk.sol.dtr[1] * 1E9,
            rtk.sol.dtr[2] * 1E9, rtk.sol.dtr[3] * 1E9,
            rtk.sol.dtr[4] * 1E9, rtk.sol.dtr[5] * 1E9);

        for (int i = 0; i < MAXSAT; i++) {
            RtkState.SatState ssat = rtk.ssat[i];
            if (ssat.vsat[0] == 0 && (nf < 2 || ssat.vsat[1] == 0)
                && (nf < 3 || ssat.vsat[2] == 0)) continue;

            String id = SatelliteUtil.satno2id(i + 1);
            for (int j = 0; j < nf; j++) {
                int k = RtkState.IB(i + 1, j, rtk.opt);
                double amb = k < rtk.nx ? rtk.x[k] : 0.0;
                double ambVar = k < rtk.nx ? rtk.P[k + k * rtk.nx] : 0.0;
                out.printf("$SAT,%d,%.3f,%s,%d,%.1f,%.1f,%.4f,%.4f,%d,%.0f,%d,%d,%d,%d,%d,%d,%.2f,%.6f,%.5f%n",
                    week, tow, id, j + 1,
                    ssat.azel[0] * R2D, ssat.azel[1] * R2D,
                    ssat.resp[j], ssat.resc[j],
                    ssat.vsat[j], ssat.snr_rover[j],
                    ssat.fix[j],
                    ssat.slip[j] & (LLI_SLIP | LLI_HALFC),
                    ssat.lock[j], ssat.outc[j],
                    ssat.slipc[j], ssat.rejc[j],
                    amb, ambVar, ssat.icbias[j]);
            }
        }
    }

    // ---------------------------------------------------------------
    // ENU std helpers
    // ---------------------------------------------------------------

    private static double[] ecef2enuStd(double[] rr, float[] qr) {
        double x = rr[0], y = rr[1], z = rr[2];
        double r = Math.sqrt(x * x + y * y + z * z);
        double lat = Math.asin(z / r);
        double lon = Math.atan2(y, x);

        double sinLat = Math.sin(lat), cosLat = Math.cos(lat);
        double sinLon = Math.sin(lon), cosLon = Math.cos(lon);

        double[][] E = {
            {-sinLon,             cosLon,              0},
            {-sinLat * cosLon,   -sinLat * sinLon,     cosLat},
            { cosLat * cosLon,    cosLat * sinLon,     sinLat}
        };

        double[][] Q = {
            {qr[0], qr[3], qr[5]},
            {qr[3], qr[1], qr[4]},
            {qr[5], qr[4], qr[2]}
        };

        double[][] EQ = new double[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                    EQ[i][j] += E[i][k] * Q[k][j];

        double[] enuVar = new double[3];
        for (int i = 0; i < 3; i++)
            for (int k = 0; k < 3; k++)
                enuVar[i] += EQ[i][k] * E[i][k];

        return new double[]{Math.sqrt(enuVar[0]), Math.sqrt(enuVar[1]), Math.sqrt(enuVar[2])};
    }

    private static double[] computeStdStats(RunResult r) {
        List<Solution> fixSols = new ArrayList<>();
        for (Solution s : r.solutions) {
            if (s.stat == SOLQ_FIX) fixSols.add(s);
        }
        if (fixSols.isEmpty()) return new double[6];

        double sumSdE = 0, sumSdN = 0, sumSdU = 0;
        double[] meanPos = new double[3];
        List<double[]> enuPositions = new ArrayList<>();

        for (Solution s : fixSols) {
            meanPos[0] += s.rr[0]; meanPos[1] += s.rr[1]; meanPos[2] += s.rr[2];
        }
        meanPos[0] /= fixSols.size(); meanPos[1] /= fixSols.size(); meanPos[2] /= fixSols.size();

        double rm = Math.sqrt(meanPos[0] * meanPos[0] + meanPos[1] * meanPos[1] + meanPos[2] * meanPos[2]);
        double lat = Math.asin(meanPos[2] / rm);
        double lon = Math.atan2(meanPos[1], meanPos[0]);
        double sinLat = Math.sin(lat), cosLat = Math.cos(lat);
        double sinLon = Math.sin(lon), cosLon = Math.cos(lon);
        double[][] E = {
            {-sinLon,            cosLon,             0},
            {-sinLat * cosLon,  -sinLat * sinLon,    cosLat},
            { cosLat * cosLon,   cosLat * sinLon,    sinLat}
        };

        for (Solution s : fixSols) {
            double[] enuStd = ecef2enuStd(s.rr, s.qr);
            sumSdE += enuStd[0]; sumSdN += enuStd[1]; sumSdU += enuStd[2];

            double dx = s.rr[0] - meanPos[0], dy = s.rr[1] - meanPos[1], dz = s.rr[2] - meanPos[2];
            double de = E[0][0] * dx + E[0][1] * dy + E[0][2] * dz;
            double dn = E[1][0] * dx + E[1][1] * dy + E[1][2] * dz;
            double du = E[2][0] * dx + E[2][1] * dy + E[2][2] * dz;
            enuPositions.add(new double[]{de, dn, du});
        }

        int n = fixSols.size();
        double ssE = 0, ssN = 0, ssU = 0;
        for (double[] enu : enuPositions) {
            ssE += enu[0] * enu[0]; ssN += enu[1] * enu[1]; ssU += enu[2] * enu[2];
        }

        return new double[]{
            sumSdE / n * 1000, sumSdN / n * 1000, sumSdU / n * 1000,
            Math.sqrt(ssE / n) * 1000, Math.sqrt(ssN / n) * 1000, Math.sqrt(ssU / n) * 1000
        };
    }

    // ---------------------------------------------------------------
    // Obs matching helpers
    // ---------------------------------------------------------------

    private List<ObsData> findMatchingBase(GTime roverTime, List<List<ObsData>> baseEpochs) {
        double bestDt = Double.MAX_VALUE;
        List<ObsData> best = null;
        for (List<ObsData> epoch : baseEpochs) {
            if (epoch == null || epoch.isEmpty()) continue;
            double dt = Math.abs(roverTime.timediff(epoch.get(0).time));
            if (dt < bestDt) {
                bestDt = dt;
                best = epoch;
            }
            if (epoch.get(0).time.timediff(roverTime) > 60) break;
        }
        return bestDt <= DTTOL ? best : null;
    }

    private ObsData[] mergeObs(List<ObsData> rover, List<ObsData> base) {
        List<ObsData> merged = new ArrayList<>(rover.size() + base.size());
        merged.addAll(rover);
        merged.addAll(base);
        return merged.toArray(new ObsData[0]);
    }
}
