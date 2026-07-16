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
import static org.junit.jupiter.api.Assertions.*;

/**
 * RTK static integration test: run forward RTK on static baseline data
 * and compare against C RTKLIB reference solution.
 */
class RtkStaticTest {

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
    void testRtkStaticForward() throws Exception {
        ProcessingOptions popt = new ProcessingOptions();
        SolutionOptions sopt = new SolutionOptions();
        ConfigReader.load(STATIC_DATA.resolve("static.conf").toString(), popt, sopt);

        // Force forward-only to avoid combined double-pass
        popt.soltype = SOLTYPE_FORWARD;
        sopt.posf = 1; // XYZ ECEF
        sopt.outhead = 1;

        Path outFile = Files.createTempFile("rtk_static_", ".pos");

        long t0 = System.currentTimeMillis();
        int ret = PostProcessor.processRtk(
                STATIC_DATA.resolve("rover_L1L5.obs").toString(),
                STATIC_DATA.resolve("base_L1L5.obs").toString(),
                STATIC_DATA.resolve("rover_L1L5.nav").toString(),
                outFile.toString(), popt, sopt, 600);
        long elapsed = System.currentTimeMillis() - t0;

        assertEquals(0, ret, "processRtk should succeed");

        // Load Java output
        List<double[]> javaSols = loadPos(outFile);

        // Load C reference (first 600 epochs)
        List<double[]> cSols = loadPos(STATIC_DATA.resolve("rover_L1L5.pos"));
        if (cSols.size() > 600) cSols = cSols.subList(0, 600);

        // Reference position from C converged solution (median of Q=1 epochs)
        double[] ref = computeRefFromFix(cSols);

        // Java stats
        double[] javaStats = computeStats(javaSols, ref);
        // C stats (for comparison)
        double[] cStats = computeStats(cSols, ref);

        System.out.printf("%n========== RTK Static Forward (600 epochs) ==========%n");
        System.out.printf("%-25s %12s %12s%n", "", "Java", "C ref");
        System.out.printf("%-25s %12d %12d%n", "Total epochs", javaSols.size(), cSols.size());
        System.out.printf("%-25s %11.1f%% %11.1f%%%n", "Fix rate (Q=1)",
                javaStats[2] * 100, cStats[2] * 100);
        System.out.printf("%-25s %12d %12d%n", "Q=1 (FIX) epochs",
                (int) javaStats[3], (int) cStats[3]);
        System.out.printf("%-25s %12.4f %12.4f%n", "3D RMS all (m)",
                javaStats[0], cStats[0]);
        System.out.printf("%-25s %12.4f %12.4f%n", "3D RMS 2nd half (m)",
                javaStats[1], cStats[1]);
        System.out.printf("%-25s %12.4f %12.4f%n", "H RMS 2nd half (m)",
                javaStats[5], cStats[5]);
        System.out.printf("%-25s %12.4f %12.4f%n", "V RMS 2nd half (m)",
                javaStats[6], cStats[6]);
        System.out.printf("%-25s %12d ms%n", "Elapsed time", elapsed);
        System.out.printf("=====================================================%n");

        Files.deleteIfExists(outFile);

        // Assertions: initial thresholds (relax for first run)
        assertTrue(javaSols.size() > 100, "Should produce >100 solutions");
        assertTrue(javaStats[2] > 0.50, "Fix rate should be > 50%");
        assertTrue(javaStats[1] < 0.05, "3D RMS (2nd half) should be < 50mm");
    }

    /**
     * Compute reference position as mean of Q=1 (FIX) epochs.
     */
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

    /**
     * Compute statistics: [rmsAll, rms2ndHalf, fixRate, fixCount, floatCount, hRms2, vRms2]
     */
    private double[] computeStats(List<double[]> sols, double[] ref) {
        double sumSq = 0, sumSq2 = 0;
        double sumH2 = 0, sumV2 = 0;
        int n = sols.size();
        int half = n / 2;
        int fixCount = 0;
        int floatCount = 0;
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
                    int ns = Integer.parseInt(parts[6]);

                    int year = Integer.parseInt(dp[0]);
                    int month = Integer.parseInt(dp[1]);
                    int day = Integer.parseInt(dp[2]);
                    int hour = Integer.parseInt(tp[0]);
                    int min = Integer.parseInt(tp[1]);
                    double sec = Double.parseDouble(tp[2]);

                    GTime gt = GTime.epoch2time(new double[]{year, month, day, hour, min, sec});
                    sols.add(new double[]{gt.time + gt.sec, x, y, z, Q, ns});
                } catch (Exception e) { /* skip */ }
            }
        }
        return sols;
    }
}
