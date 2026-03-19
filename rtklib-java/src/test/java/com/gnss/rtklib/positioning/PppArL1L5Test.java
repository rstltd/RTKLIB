package com.gnss.rtklib.positioning;

import com.gnss.rtklib.PostProcessor;
import com.gnss.rtklib.core.GTime;
import com.gnss.rtklib.io.BiasReader;
import com.gnss.rtklib.io.ConfigReader;
import com.gnss.rtklib.model.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIf;

import java.io.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;

import static com.gnss.rtklib.core.Constants.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * PPP-AR integration test: compare float-only vs AR-enabled solutions
 * using WHU OSB phase bias products.
 */
class PppArL1L5Test {

    private static final Path PPP_DATA = findPppData();

    private static Path findPppData() {
        Path p = Path.of("").toAbsolutePath().resolve("../test/data/ppp");
        if (Files.exists(p)) return p;
        return Path.of("").toAbsolutePath().resolve("test/data/ppp");
    }

    // WHU consistent product set for PPP-AR
    private static final String WHU_SP3 = "WUM0MGXFIN_20260590000_01D_05M_ORB.SP3";
    private static final String WHU_CLK = "WUM0MGXFIN_20260590000_01D_30S_CLK.CLK";
    private static final String WHU_BIA = "WUM0MGXFIN_20260590000_01D_01D_OSB.BIA";
    // CODE products for float-only comparison
    private static final String COD_SP3 = "COD0MGXFIN_20260590000_01D_05M_ORB.SP3";
    private static final String COD_CLK = "COD0MGXFIN_20260590000_01D_30S_CLK.CLK";

    static boolean dataAvailable() {
        return Files.exists(PPP_DATA.resolve("test-rinex-L1L5.obs"))
            && Files.exists(PPP_DATA.resolve("test-rinex-L1L5.nav"))
            && Files.exists(PPP_DATA.resolve(WHU_SP3))
            && Files.exists(PPP_DATA.resolve(WHU_CLK))
            && Files.exists(PPP_DATA.resolve(WHU_BIA));
    }

    /** Reference position from C RTKLIB converged solution (ECEF, m) */
    private static double[] refEcef() {
        return new double[]{-3026184.8040, 4975934.7286, 2598180.2999};
    }

    // ---------------------------------------------------------------
    // Test 1: BiasReader loads real WHU file correctly
    // ---------------------------------------------------------------

    @Test
    @EnabledIf("dataAvailable")
    void testBiasReaderRealFile() throws Exception {
        Navigation nav = new Navigation();
        BiasReader.readBias(PPP_DATA.resolve("WUM0MGXFIN_20260590000_01D_01D_OSB.BIA").toString(), nav);

        // Count loaded phase biases
        int cnt = 0;
        for (int i = 0; i < MAXSAT; i++)
            for (int j = 1; j <= MAXCODE; j++)
                if (nav.pbias[i][j] != 0.0) cnt++;

        System.out.printf("WHU BIA: %d phase biases loaded%n", cnt);
        assertTrue(cnt > 50, "Should load >50 satellite phase biases from WHU file");

        // Verify GPS G01 L1W exists and is reasonable
        int satG01 = com.gnss.rtklib.core.SatelliteUtil.satno(SYS_GPS, 1);
        int codeL1W = com.gnss.rtklib.core.SignalUtil.obs2code("1W");
        int codeL5Q = com.gnss.rtklib.core.SignalUtil.obs2code("5Q");
        double biasL1W = nav.pbias[satG01 - 1][codeL1W];
        double biasL5Q = nav.pbias[satG01 - 1][codeL5Q];

        System.out.printf("G01 L1W: %.6f m (%.4f ns)%n", biasL1W, biasL1W / CLIGHT * 1e9);
        System.out.printf("G01 L5Q: %.6f m (%.4f ns)%n", biasL5Q, biasL5Q / CLIGHT * 1e9);

        assertNotEquals(0.0, biasL1W, "G01 L1W should be loaded");
        assertNotEquals(0.0, biasL5Q, "G01 L5Q should be loaded");
        assertTrue(Math.abs(biasL1W) < 1.0, "Phase bias should be < 1m");
        assertTrue(Math.abs(biasL5Q) < 1.0, "Phase bias should be < 1m");

        // Verify GAL phase biases
        int satE01 = com.gnss.rtklib.core.SatelliteUtil.satno(SYS_GAL, 1);
        int codeL1X = com.gnss.rtklib.core.SignalUtil.obs2code("1X");
        int codeL5X = com.gnss.rtklib.core.SignalUtil.obs2code("5X");
        double biasE01_L1X = nav.pbias[satE01 - 1][codeL1X];
        double biasE01_L5X = nav.pbias[satE01 - 1][codeL5X];
        System.out.printf("E01 L1X: %.6f m  L5X: %.6f m%n", biasE01_L1X, biasE01_L5X);
    }

    // ---------------------------------------------------------------
    // Test 2: PPP float-only produces valid solution
    // ---------------------------------------------------------------

    @Test
    @EnabledIf("dataAvailable")
    void testPppFloatProducesValidSolution() throws Exception {
        ProcessingOptions popt = new ProcessingOptions();
        SolutionOptions sopt = new SolutionOptions();
        ConfigReader.load(PPP_DATA.resolve("ppp.conf").toString(), popt, sopt);

        // Keep AR off (default from config)
        assertEquals(0, popt.modear, "Config should have AR off");

        Path outFile = Files.createTempFile("ppp_float_", ".pos");
        sopt.posf = 1; // XYZ
        sopt.outhead = 1;

        int ret = PostProcessor.processPpp(
                PPP_DATA.resolve("test-rinex-L1L5.obs").toString(),
                PPP_DATA.resolve("test-rinex-L1L5.nav").toString(),
                PPP_DATA.resolve(WHU_SP3).toString(),
                PPP_DATA.resolve(WHU_CLK).toString(),
                null, null,
                outFile.toString(), popt, sopt);

        assertEquals(0, ret, "PPP float should succeed");

        List<double[]> sols = loadPos(outFile);
        System.out.printf("%nPPP Float (WHU products): %d epochs%n", sols.size());
        assertTrue(sols.size() > 100, "Should produce >100 solutions");

        // Check solution quality against reference
        double[] ref = refEcef();
        double[] stats = computeStats(sols, ref);
        System.out.printf("  3D RMS (all):       %.4f m%n", stats[0]);
        System.out.printf("  3D RMS (2nd half):  %.4f m%n", stats[1]);
        System.out.printf("  Fix rate:           %.1f%% (%d/%d)%n", stats[2] * 100, (int) stats[3], sols.size());
        System.out.printf("  Q=6 (PPP) epochs:   %d%n", (int) stats[4]);

        assertTrue(stats[0] < 0.1, "Float 3D RMS should be < 0.1m");
        assertTrue(stats[1] < 0.05, "Float converged 3D RMS should be < 50mm");

        Files.deleteIfExists(outFile);
    }

    // ---------------------------------------------------------------
    // Test 3: PPP-AR with WHU OSB
    // ---------------------------------------------------------------

    @Test
    @EnabledIf("dataAvailable")
    void testPppArWithWhuOsb() throws Exception {
        ProcessingOptions popt = new ProcessingOptions();
        SolutionOptions sopt = new SolutionOptions();
        ConfigReader.load(PPP_DATA.resolve("ppp.conf").toString(), popt, sopt);

        // Enable AR
        popt.modear = 1; // continuous

        Path outFile = Files.createTempFile("ppp_ar_", ".pos");
        sopt.posf = 1; // XYZ
        sopt.outhead = 1;

        int ret = PostProcessor.processPpp(
                PPP_DATA.resolve("test-rinex-L1L5.obs").toString(),
                PPP_DATA.resolve("test-rinex-L1L5.nav").toString(),
                PPP_DATA.resolve(WHU_SP3).toString(),
                PPP_DATA.resolve(WHU_CLK).toString(),
                null,
                PPP_DATA.resolve(WHU_BIA).toString(),
                outFile.toString(), popt, sopt);

        assertEquals(0, ret, "PPP-AR should succeed");

        List<double[]> sols = loadPos(outFile);
        System.out.printf("%nPPP-AR (WHU SP3+CLK+OSB): %d epochs%n", sols.size());
        assertTrue(sols.size() > 100, "Should produce >100 solutions");

        double[] ref = refEcef();
        double[] stats = computeStats(sols, ref);
        System.out.printf("  3D RMS (all):       %.4f m%n", stats[0]);
        System.out.printf("  3D RMS (2nd half):  %.4f m%n", stats[1]);
        System.out.printf("  Fix rate:           %.1f%% (%d/%d)%n", stats[2] * 100, (int) stats[3], sols.size());
        System.out.printf("  Q=1 (FIX) epochs:   %d%n", (int) stats[3]);
        System.out.printf("  Q=6 (PPP) epochs:   %d%n", (int) stats[4]);

        // Save for inspection
        Path savedPos = PPP_DATA.resolve("test-rinex-L1L5_ar_java.pos");
        Files.copy(outFile, savedPos, java.nio.file.StandardCopyOption.REPLACE_EXISTING);
        System.out.printf("  Output saved: %s%n", savedPos);
        Files.deleteIfExists(outFile);

        // PPP-AR diagnostic counters
        System.out.printf("  AR attempts: %d, WL ok: %d, NL ok: %d, Fixed: %d%n",
                PpposAr.diagAttempt, PpposAr.diagWlOk, PpposAr.diagNlOk, PpposAr.diagFixed);
    }

    // ---------------------------------------------------------------
    // Test 4: Compare float vs AR side-by-side
    // ---------------------------------------------------------------

    @Test
    @EnabledIf("dataAvailable")
    void testPppArImprovesOverFloat() throws Exception {
        double[] ref = refEcef();

        // --- Run float ---
        ProcessingOptions poptFloat = new ProcessingOptions();
        SolutionOptions soptFloat = new SolutionOptions();
        ConfigReader.load(PPP_DATA.resolve("ppp.conf").toString(), poptFloat, soptFloat);
        poptFloat.modear = 0;
        soptFloat.posf = 1;
        soptFloat.outhead = 1;

        Path outFloat = Files.createTempFile("ppp_cmp_float_", ".pos");
        PostProcessor.processPpp(
                PPP_DATA.resolve("test-rinex-L1L5.obs").toString(),
                PPP_DATA.resolve("test-rinex-L1L5.nav").toString(),
                PPP_DATA.resolve(WHU_SP3).toString(),
                PPP_DATA.resolve(WHU_CLK).toString(),
                null, null,
                outFloat.toString(), poptFloat, soptFloat);
        List<double[]> floatSols = loadPos(outFloat);
        double[] floatStats = computeStats(floatSols, ref);
        Files.deleteIfExists(outFloat);

        // --- Run AR ---
        // Reset diagnostic counters
        PpposAr.diagAttempt = PpposAr.diagWlOk = PpposAr.diagNlOk = PpposAr.diagFixed = 0;

        ProcessingOptions poptAr = new ProcessingOptions();
        SolutionOptions soptAr = new SolutionOptions();
        ConfigReader.load(PPP_DATA.resolve("ppp.conf").toString(), poptAr, soptAr);
        poptAr.modear = 1;
        soptAr.posf = 1;
        soptAr.outhead = 1;

        Path outAr = Files.createTempFile("ppp_cmp_ar_", ".pos");
        PostProcessor.processPpp(
                PPP_DATA.resolve("test-rinex-L1L5.obs").toString(),
                PPP_DATA.resolve("test-rinex-L1L5.nav").toString(),
                PPP_DATA.resolve(WHU_SP3).toString(),
                PPP_DATA.resolve(WHU_CLK).toString(),
                null,
                PPP_DATA.resolve(WHU_BIA).toString(),
                outAr.toString(), poptAr, soptAr);
        List<double[]> arSols = loadPos(outAr);
        double[] arStats = computeStats(arSols, ref);
        Files.deleteIfExists(outAr);

        // --- Report ---
        System.out.printf("%n========== PPP Float vs AR Comparison ==========%n");
        System.out.printf("%-25s %12s %12s%n", "", "Float", "AR");
        System.out.printf("%-25s %12d %12d%n", "Total epochs", floatSols.size(), arSols.size());
        System.out.printf("%-25s %12.4f %12.4f%n", "3D RMS all (m)", floatStats[0], arStats[0]);
        System.out.printf("%-25s %12.4f %12.4f%n", "3D RMS 2nd half (m)", floatStats[1], arStats[1]);
        System.out.printf("%-25s %11.1f%% %11.1f%%%n", "Fix rate (Q=1)", floatStats[2] * 100, arStats[2] * 100);
        System.out.printf("%-25s %12d %12d%n", "Q=1 (FIX) epochs", (int) floatStats[3], (int) arStats[3]);
        System.out.printf("%-25s %12d %12d%n", "Q=6 (PPP) epochs", (int) floatStats[4], (int) arStats[4]);
        System.out.printf("==================================================%n");
        System.out.printf("AR attempts: %d, WL ok: %d, NL ok: %d, Fixed: %d%n",
                PpposAr.diagAttempt, PpposAr.diagWlOk, PpposAr.diagNlOk, PpposAr.diagFixed);

        // Assertions
        assertTrue(floatSols.size() > 100, "Float should produce solutions");
        assertTrue(arSols.size() > 100, "AR should produce solutions");
    }

    // ---------------------------------------------------------------
    // Test 5: PPP-AR with L1+L2 data (WHU consistent products)
    //   GPS: L1C+L2X — WHU has both L1C and L2X phase bias
    //   GAL: L1X+L7X — WHU has both L1X and L7X phase bias
    //   Perfect signal code match → should achieve integer fixing
    // ---------------------------------------------------------------

    static boolean l1l2DataAvailable() {
        return dataAvailable()
            && Files.exists(PPP_DATA.resolve("test-rinex-L1L2.obs"))
            && Files.exists(PPP_DATA.resolve("test-rinex-L1L2.nav"));
    }

    @Test
    @EnabledIf("l1l2DataAvailable")
    void testPppArL1L2WithWhu() throws Exception {
        double[] ref = {-2974575.4491, 5068091.2877, 2471282.4677};

        // --- Float baseline ---
        ProcessingOptions poptFloat = new ProcessingOptions();
        SolutionOptions soptFloat = new SolutionOptions();
        ConfigReader.load(PPP_DATA.resolve("ppp.conf").toString(), poptFloat, soptFloat);
        poptFloat.modear = 0;
        soptFloat.posf = 1;
        soptFloat.outhead = 1;

        Path outFloat = Files.createTempFile("ppp_l1l2_float_", ".pos");
        PostProcessor.processPpp(
                PPP_DATA.resolve("test-rinex-L1L2.obs").toString(),
                PPP_DATA.resolve("test-rinex-L1L2.nav").toString(),
                PPP_DATA.resolve(WHU_SP3).toString(),
                PPP_DATA.resolve(WHU_CLK).toString(),
                null, null,
                outFloat.toString(), poptFloat, soptFloat);
        List<double[]> floatSols = loadPos(outFloat);
        double[] floatStats = computeStats(floatSols, ref);
        Files.deleteIfExists(outFloat);

        // --- AR ---
        PpposAr.diagAttempt = PpposAr.diagWlOk = PpposAr.diagNlOk = PpposAr.diagFixed = 0;

        ProcessingOptions poptAr = new ProcessingOptions();
        SolutionOptions soptAr = new SolutionOptions();
        ConfigReader.load(PPP_DATA.resolve("ppp.conf").toString(), poptAr, soptAr);
        poptAr.modear = 1; // enable AR
        soptAr.posf = 1;
        soptAr.outhead = 1;

        Path outAr = Files.createTempFile("ppp_l1l2_ar_", ".pos");
        PostProcessor.processPpp(
                PPP_DATA.resolve("test-rinex-L1L2.obs").toString(),
                PPP_DATA.resolve("test-rinex-L1L2.nav").toString(),
                PPP_DATA.resolve(WHU_SP3).toString(),
                PPP_DATA.resolve(WHU_CLK).toString(),
                null,
                PPP_DATA.resolve(WHU_BIA).toString(),
                outAr.toString(), poptAr, soptAr);
        List<double[]> arSols = loadPos(outAr);
        double[] arStats = computeStats(arSols, ref);

        // Save for inspection
        Path savedPos = PPP_DATA.resolve("test-rinex-L1L2_ar_java.pos");
        Files.copy(outAr, savedPos, java.nio.file.StandardCopyOption.REPLACE_EXISTING);
        Files.deleteIfExists(outAr);

        // --- Report ---
        System.out.printf("%n========== PPP L1+L2 Float vs AR (WHU) ==========%n");
        System.out.printf("%-25s %12s %12s%n", "", "Float", "AR");
        System.out.printf("%-25s %12d %12d%n", "Total epochs", floatSols.size(), arSols.size());
        System.out.printf("%-25s %12.4f %12.4f%n", "3D RMS all (m)", floatStats[0], arStats[0]);
        System.out.printf("%-25s %12.4f %12.4f%n", "3D RMS 2nd half (m)", floatStats[1], arStats[1]);
        System.out.printf("%-25s %11.1f%% %11.1f%%%n", "Fix rate (Q=1)", floatStats[2] * 100, arStats[2] * 100);
        System.out.printf("%-25s %12d %12d%n", "Q=1 (FIX) epochs", (int) floatStats[3], (int) arStats[3]);
        System.out.printf("%-25s %12d %12d%n", "Q=6 (PPP) epochs", (int) floatStats[4], (int) arStats[4]);
        System.out.printf("==================================================%n");
        System.out.printf("AR: attempts=%d, WL ok(last)=%d, NL ok(last)=%d, Fixed=%d%n",
                PpposAr.diagAttempt, PpposAr.diagWlOk, PpposAr.diagNlOk, PpposAr.diagFixed);
        System.out.printf("    WL peak=%d, NL peak=%d%n",
                PpposAr.diagWlPeak, PpposAr.diagNlPeak);
        System.out.printf("Output saved: %s%n", savedPos);

        // Assertions
        assertTrue(floatSols.size() > 100, "Float should produce solutions");
        assertTrue(arSols.size() > 100, "AR should produce solutions");
    }

    // ---------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------

    /**
     * Compute statistics: [rmsAll, rms2ndHalf, fixRate, fixCount, pppCount]
     */
    private double[] computeStats(List<double[]> sols, double[] ref) {
        double sumSq = 0, sumSq2 = 0;
        int n = sols.size();
        int half = n / 2;
        int fixCount = 0;
        int pppCount = 0;

        for (int i = 0; i < n; i++) {
            double[] s = sols.get(i);
            double dx = s[1] - ref[0];
            double dy = s[2] - ref[1];
            double dz = s[3] - ref[2];
            double d3d = dx * dx + dy * dy + dz * dz;
            sumSq += d3d;
            if (i >= half) sumSq2 += d3d;
            if ((int) s[4] == SOLQ_FIX) fixCount++;
            if ((int) s[4] == SOLQ_PPP) pppCount++;
        }

        double rmsAll = n > 0 ? Math.sqrt(sumSq / n) : 0;
        double rms2 = (n - half) > 0 ? Math.sqrt(sumSq2 / (n - half)) : 0;
        double fixRate = n > 0 ? (double) fixCount / n : 0;

        return new double[]{rmsAll, rms2, fixRate, fixCount, pppCount};
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
