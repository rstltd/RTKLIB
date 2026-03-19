package com.gnss.rtklib.positioning;

import com.gnss.rtklib.PostProcessor;
import com.gnss.rtklib.core.GTime;
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
 * Compare Java PPP L1+L5 solution against C RTKLIB reference.
 * Uses identical config (ppp.conf) for fair comparison.
 */
class PppL1L5CompareTest {

    private static final Path PPP_DATA = findPppData();

    private static Path findPppData() {
        Path p = Path.of("").toAbsolutePath().resolve("../test/data/ppp");
        if (Files.exists(p)) return p;
        return Path.of("").toAbsolutePath().resolve("test/data/ppp");
    }

    static boolean dataAvailable() {
        return Files.exists(PPP_DATA.resolve("test-rinex-L1L5.obs"))
            && Files.exists(PPP_DATA.resolve("test-rinex-L1L5.nav"))
            && Files.exists(PPP_DATA.resolve("COD0MGXFIN_20260590000_01D_05M_ORB.SP3"))
            && Files.exists(PPP_DATA.resolve("COD0MGXFIN_20260590000_01D_30S_CLK.CLK"))
            && Files.exists(PPP_DATA.resolve("test-rinex-L1L5.pos"));
    }

    @Test
    @EnabledIf("dataAvailable")
    void comparePppL1L5() throws Exception {
        // Use PostProcessor.processPpp with identical config
        ProcessingOptions popt = new ProcessingOptions();
        SolutionOptions sopt = new SolutionOptions();
        ConfigReader.load(PPP_DATA.resolve("ppp.conf").toString(), popt, sopt);

        System.out.printf("Config: mode=%d nf=%d soltype=%d ionoopt=%d tropopt=%d navsys=%d dynamics=%d%n",
                popt.mode, popt.nf, popt.soltype, popt.ionoopt, popt.tropopt, popt.navsys, popt.dynamics);

        // Output to temp file
        Path outFile = Files.createTempFile("ppp_l1l5_java_", ".pos");
        sopt.posf = 1;     // XYZ format (matches reference)
        sopt.outhead = 1;  // output header (required by rtkplot)

        int ret = PostProcessor.processPpp(
                PPP_DATA.resolve("test-rinex-L1L5.obs").toString(),
                PPP_DATA.resolve("test-rinex-L1L5.nav").toString(),
                PPP_DATA.resolve("COD0MGXFIN_20260590000_01D_05M_ORB.SP3").toString(),
                PPP_DATA.resolve("COD0MGXFIN_20260590000_01D_30S_CLK.CLK").toString(),
                null, // no ANTEX
                outFile.toString(),
                popt, sopt);

        assertEquals(0, ret, "PostProcessor.processPpp should succeed");

        // Load Java result
        List<double[]> javaSols = loadPos(outFile);
        System.out.printf("Java solutions: %d epochs%n", javaSols.size());

        // Load C reference
        List<double[]> cSols = loadPos(PPP_DATA.resolve("test-rinex-L1L5.pos"));
        System.out.printf("C reference:    %d epochs%n", cSols.size());

        assertTrue(javaSols.size() > 0, "Should produce PPP solutions");

        // Compare: match by time, compute statistics
        int matched = 0;
        double sumSq = 0, maxDiff = 0;
        double sumDx = 0, sumDy = 0, sumDz = 0;
        // Statistics for last 50% (converged) epochs
        int matchedConv = 0;
        double sumSqConv = 0;
        int cIdx = 0;
        int halfPoint = javaSols.size() / 2;
        int jIdx = 0;

        for (double[] js : javaSols) {
            while (cIdx < cSols.size() - 1 && cSols.get(cIdx)[0] < js[0] - 0.5) cIdx++;
            if (cIdx >= cSols.size()) break;
            if (Math.abs(cSols.get(cIdx)[0] - js[0]) > 1.0) { jIdx++; continue; }

            double[] cs = cSols.get(cIdx);
            double dx = js[1] - cs[1];
            double dy = js[2] - cs[2];
            double dz = js[3] - cs[3];
            double d3d = Math.sqrt(dx*dx + dy*dy + dz*dz);

            sumDx += dx; sumDy += dy; sumDz += dz;
            sumSq += d3d * d3d;
            if (d3d > maxDiff) maxDiff = d3d;
            matched++;

            if (jIdx >= halfPoint) {
                sumSqConv += d3d * d3d;
                matchedConv++;
            }
            jIdx++;
        }

        if (matched > 0) {
            double rms = Math.sqrt(sumSq / matched);
            double rmsConv = matchedConv > 0 ? Math.sqrt(sumSqConv / matchedConv) : 0;
            double meanDx = sumDx / matched;
            double meanDy = sumDy / matched;
            double meanDz = sumDz / matched;
            double meanBias = Math.sqrt(meanDx*meanDx + meanDy*meanDy + meanDz*meanDz);

            System.out.printf("%nJava vs C RTKLIB — PPP L1+L5 (same config):%n");
            System.out.printf("  Matched:           %d / %d epochs%n", matched, javaSols.size());
            System.out.printf("  3D RMS diff (all): %.4f m%n", rms);
            System.out.printf("  3D RMS diff (2nd half): %.4f m%n", rmsConv);
            System.out.printf("  3D max diff:       %.4f m%n", maxDiff);
            System.out.printf("  Mean bias:         %.4f m (dx=%.4f dy=%.4f dz=%.4f)%n",
                              meanBias, meanDx, meanDy, meanDz);

            if (javaSols.size() >= 2) {
                double[] first = javaSols.get(0), last = javaSols.get(javaSols.size()-1);
                double[] cfirst = cSols.get(0), clast = cSols.get(cSols.size()-1);
                System.out.printf("  First Java: %.4f %.4f %.4f Q=%d ns=%d%n",
                        first[1], first[2], first[3], (int)first[4], (int)first[5]);
                System.out.printf("  First C:    %.4f %.4f %.4f Q=%d ns=%d%n",
                        cfirst[1], cfirst[2], cfirst[3], (int)cfirst[4], (int)cfirst[5]);
                System.out.printf("  Last  Java: %.4f %.4f %.4f Q=%d ns=%d%n",
                        last[1], last[2], last[3], (int)last[4], (int)last[5]);
                System.out.printf("  Last  C:    %.4f %.4f %.4f Q=%d ns=%d%n",
                        clast[1], clast[2], clast[3], (int)clast[4], (int)clast[5]);
            }
        }

        // Copy to project dir for inspection
        Path savedPos = PPP_DATA.resolve("test-rinex-L1L5_java.pos");
        Files.copy(outFile, savedPos, java.nio.file.StandardCopyOption.REPLACE_EXISTING);
        System.out.printf("  Output saved: %s%n", savedPos);
        Files.deleteIfExists(outFile);

        assertTrue(matched > 0, "Should have matched epochs");
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
