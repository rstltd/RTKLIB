package com.gnss.rtklib.positioning;

import com.gnss.rtklib.PostProcessor;
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

        // Warm up JVM
        runScenario("Warmup (600 ep)", 600, ref);

        // Primary benchmark: 3h@1Hz = 10800 epochs
        runScenario("3h@1Hz (10800 ep)", 10800, ref);

        // Baseline reference (pre-optimization, 2026-03-19):
        //   Elapsed: 38,120 ms | GC count: 57 | GC time: 710 ms
        //   Fix rate: 92.4% | 3D RMS 2nd half: 0.0048 m
    }

    private void runScenario(String label, int maxEpochs, double[] ref) throws Exception {
        ProcessingOptions popt = new ProcessingOptions();
        SolutionOptions sopt = new SolutionOptions();
        ConfigReader.load(STATIC_DATA.resolve("static.conf").toString(), popt, sopt);
        popt.soltype = SOLTYPE_FORWARD;
        sopt.posf = 1;
        sopt.outhead = 1;

        Path outFile = Files.createTempFile("rtk_bench_", ".pos");

        System.gc();
        Thread.sleep(100);

        Runtime rt = Runtime.getRuntime();
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
        System.out.printf("  GC count:     %d  |  GC time: %d ms%n", gcCount, gcTime);
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
        int n = sols.size();
        int half = n / 2;
        int fixCount = 0, floatCount = 0;
        for (int i = 0; i < n; i++) {
            double[] s = sols.get(i);
            double dx = s[1] - ref[0], dy = s[2] - ref[1], dz = s[3] - ref[2];
            double d3d = dx * dx + dy * dy + dz * dz;
            sumSq += d3d;
            if (i >= half) sumSq2 += d3d;
            if ((int) s[4] == SOLQ_FIX) fixCount++;
            if ((int) s[4] == SOLQ_FLOAT) floatCount++;
        }
        double rmsAll = n > 0 ? Math.sqrt(sumSq / n) : 0;
        double rms2 = (n - half) > 0 ? Math.sqrt(sumSq2 / (n - half)) : 0;
        double fixRate = n > 0 ? (double) fixCount / n : 0;
        return new double[]{rmsAll, rms2, fixRate, fixCount, floatCount};
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
