package com.gnss.rtklib.positioning;

import com.gnss.rtklib.PostProcessor;
import com.gnss.rtklib.core.Coord;
import com.gnss.rtklib.core.GTime;
import com.gnss.rtklib.io.ConfigReader;
import com.gnss.rtklib.model.ProcessingOptions;
import com.gnss.rtklib.model.SolutionOptions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIf;

import java.io.BufferedReader;
import java.io.FileReader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import static com.gnss.rtklib.core.Constants.*;

/**
 * RTK performance benchmark: 3h@1Hz scenario.
 * Baseline (pre-optimization): 38,120 ms / 57 GC / 710 ms GC time.
 */
class RtkBenchmarkTest {

    private static final Path STATIC_DATA = findStaticData();

    private static Path findStaticData() {
        Path p = Path.of("").toAbsolutePath().resolve("../test/data/static");
        if (Files.exists(p)) return p;
        return Path.of("").toAbsolutePath().resolve("test/data/static");
    }

    static boolean dataAvailable() {
        return Files.exists(STATIC_DATA.resolve("rover_L1L5.obs"))
            && Files.exists(STATIC_DATA.resolve("base_L1L5.obs"))
            && Files.exists(STATIC_DATA.resolve("rover_L1L5.nav"))
            && Files.exists(STATIC_DATA.resolve("static.conf"))
            && Files.exists(STATIC_DATA.resolve("rover_L1L5.pos"));
    }

    @Test
    @EnabledIf("dataAvailable")
    void benchmark3h() throws Exception {
        List<double[]> cSols = loadPos(STATIC_DATA.resolve("rover_L1L5.pos"));
        double[] ref = computeRefFromFix(cSols);

        int maxEpochs = 10800;

        // Warm up JVM
        runScenario("Warmup (600 ep)", 600, ref, p -> {});

        // A/B comparison: collect stats for table
        System.out.printf("%n%n============================================================%n");
        System.out.printf("  A/B Comparison: 3h@1Hz (%d epochs)%n", maxEpochs);
        System.out.printf("============================================================%n");

        double[] sBase = runScenario("A) Baseline (elevation model, no AR validation)",
                maxEpochs, ref, p -> {});

        double[] sPos1 = runScenario("B) posthres=0.5m",
                maxEpochs, ref, p -> { p.arposthres = 0.5; });

        double[] sPos2 = runScenario("C) posthres=0.1m",
                maxEpochs, ref, p -> { p.arposthres = 0.1; });

        double[] sChisq = runScenario("D) chisq=1e-4",
                maxEpochs, ref, p -> { p.archisqthres = 1e-4; });

        double[] sBoth = runScenario("E) pos=0.5m + chisq=1e-4",
                maxEpochs, ref, p -> { p.arposthres = 0.5; p.archisqthres = 1e-4; });

        double[] sSNR = runScenario("F) SNR model only",
                maxEpochs, ref, p -> { p.snrmodel = 1; });

        // Summary table
        double[][] all = {sBase, sPos1, sPos2, sChisq, sBoth, sSNR};
        String[] names = {"A:Base", "B:.5m", "C:.1m", "D:chi", "E:B+D", "F:SNR"};
        System.out.printf("%n%-30s", "Metric");
        for (String n : names) System.out.printf(" %8s", n);
        System.out.println();
        System.out.printf("%-30s", "──────────────────────────────");
        for (int i = 0; i < names.length; i++) System.out.printf(" %8s", "────────");
        System.out.println();
        printRow("Fix rate (%)", all, 2, 100, "%.1f");
        printRow("3D RMS 2nd (mm)", all, 1, 1000, "%.2f");
        printRow("H RMS 2nd (mm)", all, 5, 1000, "%.2f");
        printRow("V RMS 2nd (mm)", all, 6, 1000, "%.2f");
        System.out.printf("============================================================%n%n");
    }

    private static void printRow(String label, double[][] stats, int idx, double scale, String fmt) {
        System.out.printf("%-30s", label);
        for (double[] s : stats) System.out.printf(" %8s", String.format(fmt, s[idx] * scale));
        System.out.println();
    }

    @FunctionalInterface
    interface OptionsTweak {
        void apply(ProcessingOptions p);
    }

    private double[] runScenario(String label, int maxEpochs, double[] ref,
                                 OptionsTweak tweak) throws Exception {
        ProcessingOptions popt = new ProcessingOptions();
        SolutionOptions sopt = new SolutionOptions();
        ConfigReader.load(STATIC_DATA.resolve("static.conf").toString(), popt, sopt);
        popt.soltype = SOLTYPE_FORWARD;
        sopt.posf = 1;
        sopt.outhead = 1;
        tweak.apply(popt);

        Path outFile = Files.createTempFile("rtk_bench_", ".pos");

        System.gc();
        Thread.sleep(100);

        long gcCountBefore = getGcCount();
        long gcTimeBefore = getGcTime();
        long t0 = System.currentTimeMillis();

        int ret = PostProcessor.processRtk(
                STATIC_DATA.resolve("rover_L1L5.obs").toString(),
                STATIC_DATA.resolve("base_L1L5.obs").toString(),
                STATIC_DATA.resolve("rover_L1L5.nav").toString(),
                outFile.toString(), popt, sopt, maxEpochs);

        long elapsed = System.currentTimeMillis() - t0;
        long gcCount = getGcCount() - gcCountBefore;
        long gcTime = getGcTime() - gcTimeBefore;

        List<double[]> sols = loadPos(outFile);
        double[] stats = computeStats(sols, ref);
        Files.deleteIfExists(outFile);

        System.out.printf("%n--- %s ---%n", label);
        System.out.printf("  Elapsed:      %d ms (%.1f ms/epoch)%n", elapsed,
                sols.size() > 0 ? (double) elapsed / sols.size() : 0);
        System.out.printf("  Fix rate:     %.1f%% (%d/%d)%n", stats[2] * 100, (int) stats[3], sols.size());
        System.out.printf("  3D RMS all:   %.4f m  |  2nd half: %.4f m%n", stats[0], stats[1]);
        System.out.printf("  H RMS 2nd:    %.4f m  |  V RMS 2nd: %.4f m%n", stats[5], stats[6]);
        System.out.printf("  GC count:     %d  |  GC time: %d ms%n", gcCount, gcTime);

        return stats;
    }

    private long getGcCount() {
        long count = 0;
        for (java.lang.management.GarbageCollectorMXBean gc :
                java.lang.management.ManagementFactory.getGarbageCollectorMXBeans()) {
            if (gc.getCollectionCount() >= 0) count += gc.getCollectionCount();
        }
        return count;
    }

    private long getGcTime() {
        long time = 0;
        for (java.lang.management.GarbageCollectorMXBean gc :
                java.lang.management.ManagementFactory.getGarbageCollectorMXBeans()) {
            if (gc.getCollectionTime() >= 0) time += gc.getCollectionTime();
        }
        return time;
    }

    private double[] computeRefFromFix(List<double[]> sols) {
        double sx = 0, sy = 0, sz = 0;
        int cnt = 0;
        for (double[] s : sols) {
            if ((int) s[4] == SOLQ_FIX) {
                sx += s[1]; sy += s[2]; sz += s[3];
                cnt++;
            }
        }
        if (cnt == 0) return new double[]{0, 0, 0};
        return new double[]{sx / cnt, sy / cnt, sz / cnt};
    }

    private double[] computeStats(List<double[]> sols, double[] ref) {
        double sumSq = 0, sumSq2 = 0;
        double sumH2 = 0, sumV2 = 0;
        int n = sols.size();
        int half = n / 2;
        int fixCount = 0, floatCount = 0;
        double[] refPos = Coord.ecef2pos(ref);
        for (int i = 0; i < n; i++) {
            double[] s = sols.get(i);
            double[] dr = {s[1] - ref[0], s[2] - ref[1], s[3] - ref[2]};
            double d3d = dr[0] * dr[0] + dr[1] * dr[1] + dr[2] * dr[2];
            sumSq += d3d;
            if (i >= half) {
                sumSq2 += d3d;
                double[] enu = Coord.ecef2enu(refPos, dr);
                sumH2 += enu[0] * enu[0] + enu[1] * enu[1];
                sumV2 += enu[2] * enu[2];
            }
            if ((int) s[4] == SOLQ_FIX) fixCount++;
            if ((int) s[4] == SOLQ_FLOAT) floatCount++;
        }
        double rmsAll = n > 0 ? Math.sqrt(sumSq / n) : 0;
        int n2 = n - half;
        double rms2 = n2 > 0 ? Math.sqrt(sumSq2 / n2) : 0;
        double hRms2 = n2 > 0 ? Math.sqrt(sumH2 / n2) : 0;
        double vRms2 = n2 > 0 ? Math.sqrt(sumV2 / n2) : 0;
        double fixRate = n > 0 ? (double) fixCount / n : 0;
        return new double[]{rmsAll, rms2, fixRate, fixCount, floatCount, hRms2, vRms2};
    }

    private List<double[]> loadPos(Path posFile) throws Exception {
        List<double[]> sols = new ArrayList<>();
        try (BufferedReader br = new BufferedReader(new FileReader(posFile.toFile()))) {
            String line;
            while ((line = br.readLine()) != null) {
                if (line.startsWith("%") || line.trim().isEmpty()) continue;
                String[] parts = line.trim().split("\\s+");
                if (parts.length < 7) continue;
                try {
                    String[] dp = parts[0].split("/");
                    String[] tp = parts[1].split(":");
                    double x = Double.parseDouble(parts[2]);
                    double y = Double.parseDouble(parts[3]);
                    double z = Double.parseDouble(parts[4]);
                    int Q = Integer.parseInt(parts[5]);
                    int year = Integer.parseInt(dp[0]);
                    int month = Integer.parseInt(dp[1]);
                    int day = Integer.parseInt(dp[2]);
                    int hour = Integer.parseInt(tp[0]);
                    int min = Integer.parseInt(tp[1]);
                    double sec = Double.parseDouble(tp[2]);
                    GTime gt = GTime.epoch2time(new double[]{year, month, day, hour, min, sec});
                    sols.add(new double[]{gt.time + gt.sec, x, y, z, Q});
                } catch (Exception e) { /* skip */ }
            }
        }
        return sols;
    }
}
