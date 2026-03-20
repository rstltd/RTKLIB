package com.gnss.rtklib.positioning;

import com.gnss.rtklib.core.SatelliteUtil;
import com.gnss.rtklib.io.RinexReader;
import com.gnss.rtklib.model.*;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIf;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import static com.gnss.rtklib.core.Constants.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * BLS benchmark: sliding 10-minute windows across the full dataset.
 * Measures fix rate, 3D RMS, and elapsed time.
 */
class BlsBenchmarkTest {

    private static final Path STATIC_DATA = findStaticData();

    private static Path findStaticData() {
        Path p = Path.of("").toAbsolutePath().resolve("../test/data/static");
        if (Files.exists(p)) return p;
        p = Path.of("").toAbsolutePath().resolve("test/data/static");
        if (Files.exists(p)) return p;
        return p;
    }

    static boolean staticDataAvailable() {
        return Files.exists(STATIC_DATA.resolve("rover_L1L5.obs"))
            && Files.exists(STATIC_DATA.resolve("base_L1L5.obs"))
            && Files.exists(STATIC_DATA.resolve("rover_L1L5.nav"));
    }

    private static final double[] BASE_POS = {-3026183.3550, 4975933.2850, 2598179.5470};
    private static final double[] REF_FIX_POS = {-3026470.474, 4976167.052, 2597455.974};

    private static Navigation nav;
    private static List<List<ObsData>> roverEpochs;
    private static List<List<ObsData>> baseEpochs;
    private static int overlapStart;

    @BeforeAll
    static void loadData() throws Exception {
        if (!staticDataAvailable()) return;

        nav = new Navigation();
        RinexReader.readNav(STATIC_DATA.resolve("rover_L1L5.nav").toString(), nav);
        roverEpochs = RinexReader.readObs(STATIC_DATA.resolve("rover_L1L5.obs").toString(), nav, 1);
        baseEpochs = RinexReader.readObs(STATIC_DATA.resolve("base_L1L5.obs").toString(), nav, 2);

        // Find overlap start
        overlapStart = 0;
        for (int i = 0; i < roverEpochs.size(); i++) {
            List<ObsData> re = roverEpochs.get(i);
            if (re == null || re.isEmpty()) continue;
            double rt = re.get(0).time.time + re.get(0).time.sec;
            for (List<ObsData> be : baseEpochs) {
                if (be == null || be.isEmpty()) continue;
                double bt = be.get(0).time.time + be.get(0).time.sec;
                if (Math.abs(rt - bt) <= 1.0) { overlapStart = i; break; }
            }
            if (overlapStart > 0) break;
        }

        System.out.printf("Data loaded: rover=%d, base=%d, overlap starts at epoch %d%n",
                          roverEpochs.size(), baseEpochs.size(), overlapStart);
    }

    private static ProcessingOptions createOpt() {
        ProcessingOptions opt = new ProcessingOptions();
        opt.mode = PMODE_STATIC;
        opt.nf = 3;
        opt.navsys = SYS_GPS | SYS_GLO | SYS_GAL | SYS_QZS | SYS_CMP | SYS_SBS;
        opt.elmin = 15.0 * D2R;
        opt.ionoopt = IONOOPT_BRDC;
        opt.tropopt = TROPOPT_SAAS;
        opt.modear = 3;
        opt.dynamics = 0;
        opt.rb[0] = BASE_POS[0];
        opt.rb[1] = BASE_POS[1];
        opt.rb[2] = BASE_POS[2];
        opt.thresar[0] = 3.0;
        opt.eratio[0] = 150.0;
        opt.eratio[1] = 150.0;
        opt.err[1] = 0.003;
        opt.err[2] = 0.006;
        opt.niter = 1;
        opt.snrmask.ena[0] = 1;
        opt.snrmask.ena[1] = 1;
        for (int f = 0; f < opt.snrmask.mask.length; f++) {
            java.util.Arrays.fill(opt.snrmask.mask[f], 35.0);
        }
        return opt;
    }

    @Test
    @EnabledIf("staticDataAvailable")
    void benchmark10min() {
        int windowSize = 600; // 10 min @ 1Hz
        int step = 600;       // non-overlapping windows
        ProcessingOptions opt = createOpt();

        int maxEnd = roverEpochs.size(); // full dataset
        int nWindows = 0, nFix = 0, nFloat = 0, nFail = 0, nWrongFix = 0;
        double sumSqErr = 0;
        int nFixForRms = 0;
        long totalMs = 0;

        System.out.println("\n--- BLS 10min Sliding Windows ---");
        System.out.printf("%-6s %-5s %7s %7s %6s %6s %5s %6s%n",
                          "Window", "Stat", "Ratio", "3D(mm)", "nAmb", "nEp", "ms", "sigma0");

        for (int start = overlapStart; start + windowSize <= maxEnd; start += step) {
            List<List<ObsData>> rovSub = roverEpochs.subList(start, start + windowSize);

            long t0 = System.nanoTime();
            BatchSolver.BatchResult r = BatchSolver.solve(rovSub, baseEpochs, nav, opt);
            long elapsed = (System.nanoTime() - t0) / 1_000_000;
            totalMs += elapsed;

            double err3d = 999;
            if (r.stat == SOLQ_FIX || r.stat == SOLQ_FLOAT) {
                double dx = r.pos[0] - REF_FIX_POS[0];
                double dy = r.pos[1] - REF_FIX_POS[1];
                double dz = r.pos[2] - REF_FIX_POS[2];
                err3d = Math.sqrt(dx * dx + dy * dy + dz * dz);
            }

            String stat = r.stat == SOLQ_FIX ? "FIX" : r.stat == SOLQ_FLOAT ? "FLOAT" : "NONE";
            System.out.printf("%-6d %-5s %7.1f %7.1f %6d %6d %5d %6.2f%n",
                              nWindows, stat, r.ratio, err3d * 1000, r.nAmb, r.nEpochs, elapsed,
                              r.sigma0sq);

            if (r.stat == SOLQ_FIX) {
                nFix++;
                sumSqErr += err3d * err3d;
                nFixForRms++;
                if (err3d > 0.050) nWrongFix++;
            } else if (r.stat == SOLQ_FLOAT) {
                nFloat++;
            } else {
                nFail++;
            }
            nWindows++;
        }

        double fixRate = nWindows > 0 ? 100.0 * nFix / nWindows : 0;
        double rms3d = nFixForRms > 0 ? Math.sqrt(sumSqErr / nFixForRms) * 1000 : 0;

        System.out.println("\n========== BLS 10min Benchmark ==========");
        System.out.printf("Windows:    %d%n", nWindows);
        System.out.printf("Fix rate:   %.1f%% (%d/%d)%n", fixRate, nFix, nWindows);
        System.out.printf("Float:      %d, Fail: %d%n", nFloat, nFail);
        System.out.printf("Wrong fix:  %d (>50mm)%n", nWrongFix);
        System.out.printf("3D RMS fix: %.1f mm%n", rms3d);
        System.out.printf("Total time: %d ms (%.0f ms/window)%n",
                          totalMs, nWindows > 0 ? (double) totalMs / nWindows : 0);
        System.out.println("==========================================");

        assertTrue(nWindows > 0, "Should have at least one window");
    }

    @Test
    @EnabledIf("staticDataAvailable")
    void pfThresholdSweep() {
        double[] thresholds = {0.008, 0.010, 0.012, 0.015, 0.020};
        int windowSize = 600;
        int step = 600;
        ProcessingOptions opt = createOpt();
        int maxEnd = roverEpochs.size();

        System.out.printf("%n%-8s %6s %6s %8s %6s%n",
                          "pfThres", "Fix%", "nFix", "RMS(mm)", "WF>50");

        double savedThres = opt.thresar[7];
        try {
            for (double pf : thresholds) {
                opt.thresar[7] = pf;
                int nW = 0, nFix = 0, nWrongFix = 0;
                double sumSq = 0;
                int nFixRms = 0;

                for (int start = overlapStart; start + windowSize <= maxEnd; start += step) {
                    List<List<ObsData>> rovSub = roverEpochs.subList(start, start + windowSize);
                    BatchSolver.BatchResult r = BatchSolver.solve(rovSub, baseEpochs, nav, opt);
                    nW++;
                    if (r.stat == SOLQ_FIX) {
                        nFix++;
                        double dx = r.pos[0] - REF_FIX_POS[0];
                        double dy = r.pos[1] - REF_FIX_POS[1];
                        double dz = r.pos[2] - REF_FIX_POS[2];
                        double err = Math.sqrt(dx*dx + dy*dy + dz*dz);
                        sumSq += err * err;
                        nFixRms++;
                        if (err > 0.050) nWrongFix++;
                    }
                }

                double fixRate = 100.0 * nFix / nW;
                double rms = nFixRms > 0 ? Math.sqrt(sumSq / nFixRms) * 1000 : 0;
                System.out.printf("%.3f   %5.1f%% %5d  %7.1f  %5d%n",
                                  pf, fixRate, nFix, rms, nWrongFix);
            }
        } finally {
            opt.thresar[7] = savedThres;
        }
    }

    @Test
    @EnabledIf("staticDataAvailable")
    void diagnoseWindow() {
        int windowSize = 600;
        int windowIdx = 11; // change to diagnose different windows
        int start = overlapStart + windowIdx * windowSize;
        if (start + windowSize > roverEpochs.size()) return;

        List<List<ObsData>> rovSub = roverEpochs.subList(start, start + windowSize);
        ProcessingOptions opt = createOpt();

        // Run preprocessing manually to inspect internals
        int nf = FilterState.NF(opt);
        double[] pos = BatchPreprocess.sppPosition(rovSub, nav, opt);
        assertNotNull(pos, "SPP should succeed");

        List<BatchPreprocess.EpochData> epochs =
                BatchPreprocess.preprocessEpochs(rovSub, baseEpochs, nav, opt);
        System.out.printf("Epochs preprocessed: %d%n", epochs.size());

        int[][] refSatMap = BatchPreprocess.chooseRefSats(epochs, opt, nf);
        int[] systems = {SYS_GPS, SYS_GLO, SYS_GAL, SYS_QZS, SYS_CMP, SYS_IRN, SYS_SBS};
        String[] sysNames = {"GPS", "GLO", "GAL", "QZS", "CMP", "IRN", "SBS"};
        System.out.println("\nRef sats:");
        for (int si = 0; si < systems.length; si++) {
            for (int f = 0; f < nf; f++) {
                if (refSatMap[si][f] != 0) {
                    System.out.printf("  %s freq=%d: sat %d%n", sysNames[si], f, refSatMap[si][f]);
                }
            }
        }

        List<BatchPreprocess.AmbParam> ambParams =
                BatchPreprocess.scanDdAmbiguities(epochs, opt, nf, refSatMap);

        double epochInterval = 1.0;
        if (epochs.size() >= 2) {
            double t0 = epochs.get(0).obs[0].time.time + epochs.get(0).obs[0].time.sec;
            double t1 = epochs.get(1).obs[0].time.time + epochs.get(1).obs[0].time.sec;
            epochInterval = Math.max(1.0, t1 - t0);
        }
        int MIN_SEG_LEN = Math.max(4, (int)(30.0 / epochInterval));
        ambParams.removeIf(ap -> (ap.endEpoch - ap.startEpoch + 1) < MIN_SEG_LEN);

        System.out.printf("\nAmb params: %d (after short segment removal)%n", ambParams.size());
        for (int j = 0; j < ambParams.size(); j++) {
            BatchPreprocess.AmbParam ap = ambParams.get(j);
            int sys = SatelliteUtil.satsys(ap.sat)[0];
            String sn = sys == SYS_GPS ? "G" : sys == SYS_GLO ? "R" :
                         sys == SYS_GAL ? "E" : sys == SYS_QZS ? "J" :
                         sys == SYS_CMP ? "C" : sys == SYS_SBS ? "S" : "?";
            System.out.printf("  [%2d] ref=%3d sat=%s%d freq=%d ep=%d-%d (%d epochs)%n",
                    j, ap.refSat, sn, ap.sat, ap.freq,
                    ap.startEpoch, ap.endEpoch, ap.endEpoch - ap.startEpoch + 1);
        }

        // Run full solve and get result
        BatchSolver.BatchResult r = BatchSolver.solve(rovSub, baseEpochs, nav, opt);
        double dx = r.pos[0] - REF_FIX_POS[0];
        double dy = r.pos[1] - REF_FIX_POS[1];
        double dz = r.pos[2] - REF_FIX_POS[2];
        double err3d = Math.sqrt(dx * dx + dy * dy + dz * dz);

        System.out.printf("\nResult: stat=%d ratio=%.1f 3D=%.1fmm nAmb=%d%n",
                r.stat, r.ratio, err3d * 1000, r.nAmb);

        // Show float ambiguity fractional parts if available
        if (r.ambValues != null && r.ambParams != null) {
            System.out.println("\nFloat ambiguity fractional parts:");
            for (int j = 0; j < r.ambParams.size(); j++) {
                BatchPreprocess.AmbParam ap = r.ambParams.get(j);
                int sys = SatelliteUtil.satsys(ap.sat)[0];
                if (sys == SYS_GLO) continue;
                double frac = r.ambValues[j] - Math.round(r.ambValues[j]);
                System.out.printf("  [%2d] ref=%3d sat=%3d f=%d: %.4f (frac=%.3f)%n",
                        j, ap.refSat, ap.sat, ap.freq, r.ambValues[j], frac);
            }
        }
    }
}
