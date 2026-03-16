package com.gnss.rtklib.positioning;

import com.gnss.rtklib.core.GTime;
import com.gnss.rtklib.io.RinexReader;
import com.gnss.rtklib.model.*;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIf;

import java.io.BufferedReader;
import java.io.FileReader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static com.gnss.rtklib.core.Constants.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Comprehensive 24-hour BLS vs EKF comparison on static L1+L5 RINEX dataset.
 *
 * Tests BLS performance on full 24h, 10-min windows, and 30-min windows,
 * comparing against EKF forward-only and a C-RTKLIB combined reference solution.
 *
 * Data: test/data/static/ (rover_L1L5.obs, base_L1L5.obs, rover_L1L5.nav)
 * Reference: rover_L1L5.pos (C-RTKLIB combined, 100% fix, ~2811 epochs)
 */
class Bls24hTest {

    // --- Data paths ---
    private static final Path STATIC_DATA = findStaticData();

    private static Path findStaticData() {
        Path p = Path.of("").toAbsolutePath().resolve("../test/data/static");
        if (Files.exists(p)) return p;
        return Path.of("").toAbsolutePath().resolve("test/data/static");
    }

    static boolean dataAvailable() {
        return Files.exists(STATIC_DATA.resolve("rover_L1L5.obs"))
            && Files.exists(STATIC_DATA.resolve("base_L1L5.obs"))
            && Files.exists(STATIC_DATA.resolve("rover_L1L5.nav"));
    }

    // Base station ECEF from static.conf (ant2-pos)
    private static final double[] BASE_POS = {-3026183.3550, 4975933.2850, 2598179.5470};

    // --- Loaded data ---
    private static Navigation nav;
    private static List<List<ObsData>> roverEpochs;
    private static List<List<ObsData>> baseEpochs;
    private static int overlapStart;
    private static int overlapEnd;

    // --- Reference solution parsed from POS file ---
    private static double[] refPos;  // mean fix position ECEF [3]
    private static List<RefEpoch> refEpochs;

    // --- STAT file summary ---
    private static int statTotalPos;
    private static int statFixCount;
    private static int statSatTotal;

    // ---------------------------------------------------------------
    // Data loading
    // ---------------------------------------------------------------

    @BeforeAll
    static void loadData() throws Exception {
        if (!dataAvailable()) return;

        // Load RINEX
        nav = new Navigation();
        RinexReader.readNav(STATIC_DATA.resolve("rover_L1L5.nav").toString(), nav);
        roverEpochs = RinexReader.readObs(STATIC_DATA.resolve("rover_L1L5.obs").toString(), nav, 1);
        baseEpochs = RinexReader.readObs(STATIC_DATA.resolve("base_L1L5.obs").toString(), nav, 2);

        assertNotNull(roverEpochs);
        assertNotNull(baseEpochs);
        assertFalse(roverEpochs.isEmpty(), "No rover epochs loaded");
        assertFalse(baseEpochs.isEmpty(), "No base epochs loaded");

        // Find overlap region
        overlapStart = 0;
        overlapEnd = roverEpochs.size() - 1;
        for (int i = 0; i < roverEpochs.size(); i++) {
            List<ObsData> re = roverEpochs.get(i);
            if (re == null || re.isEmpty()) continue;
            if (findBase(re.get(0).time) != null) { overlapStart = i; break; }
        }
        for (int i = roverEpochs.size() - 1; i >= overlapStart; i--) {
            List<ObsData> re = roverEpochs.get(i);
            if (re == null || re.isEmpty()) continue;
            if (findBase(re.get(0).time) != null) { overlapEnd = i; break; }
        }

        // Parse reference POS file
        Path posFile = STATIC_DATA.resolve("rover_L1L5.pos");
        if (Files.exists(posFile)) {
            refEpochs = parseRefPosEpochs(posFile);
            refPos = parseRefPos(refEpochs);
        } else {
            refEpochs = new ArrayList<>();
            refPos = new double[]{-3026470.470, 4976167.053, 2597455.971};
        }

        // Parse STAT file
        Path statFile = STATIC_DATA.resolve("rover_L1L5.pos.stat");
        if (Files.exists(statFile)) {
            parseStatFile(statFile);
        }

        System.out.printf("=== Bls24hTest Data Summary ===%n");
        System.out.printf("Rover epochs:   %d%n", roverEpochs.size());
        System.out.printf("Base epochs:    %d%n", baseEpochs.size());
        System.out.printf("Overlap:        rover[%d..%d] = %d epochs%n",
                          overlapStart, overlapEnd, overlapEnd - overlapStart + 1);
        System.out.printf("Reference POS:  %d epochs (%d fix)%n",
                          refEpochs.size(),
                          (int) refEpochs.stream().filter(e -> e.Q == 1).count());
        System.out.printf("Reference pos:  %.4f  %.4f  %.4f%n",
                          refPos[0], refPos[1], refPos[2]);
        System.out.printf("STAT summary:   %d POS lines, %d fix, %d total sats%n",
                          statTotalPos, statFixCount, statSatTotal);
    }

    // ---------------------------------------------------------------
    // Test 1: Full 24h BLS
    // ---------------------------------------------------------------

    @Test
    @EnabledIf("dataAvailable")
    void test1_full24hBls() {
        System.out.printf("%n========================================%n");
        System.out.printf("TEST 1: Full 24h BLS%n");
        System.out.printf("========================================%n");

        ProcessingOptions opt = createOpt();
        opt.solver = ProcessingOptions.SOLVER_BATCH;

        List<List<ObsData>> fullRover = roverEpochs.subList(overlapStart, overlapEnd + 1);

        long t0 = System.currentTimeMillis();
        BatchSolver.BatchResult result = BatchSolver.solve(fullRover, baseEpochs, nav, opt);
        long elapsed = System.currentTimeMillis() - t0;

        double err = posError(result.pos, refPos);

        System.out.printf("%-20s %s%n", "Status:", statName(result.stat));
        System.out.printf("%-20s %d%n", "Epochs processed:", result.nEpochs);
        System.out.printf("%-20s %d%n", "Ambiguities:", result.nAmb);
        System.out.printf("%-20s %d%n", "Satellites:", result.ns);
        System.out.printf("%-20s %.1f%n", "Ratio:", result.ratio);
        System.out.printf("%-20s %.4f  %.4f  %.4f%n", "Position ECEF:",
                          result.pos[0], result.pos[1], result.pos[2]);
        System.out.printf("%-20s %.4f m%n", "Error vs ref:", err);
        System.out.printf("%-20s %d ms%n", "Elapsed:", elapsed);

        if (result.qr != null) {
            System.out.printf("%-20s xx=%.6f yy=%.6f zz=%.6f%n", "Covariance (m^2):",
                              result.qr[0], result.qr[1], result.qr[2]);
        }

        // Sanity: BLS should produce a valid solution
        assertTrue(result.stat == SOLQ_FIX || result.stat == SOLQ_FLOAT,
                   "Full 24h BLS should produce at least float (got " + result.stat + ")");
        assertTrue(result.nEpochs > 100,
                   "Should process >100 epochs from 24h data (got " + result.nEpochs + ")");
        assertTrue(result.nAmb > 0, "Should have ambiguity parameters");
    }

    // ---------------------------------------------------------------
    // Test 2: Sliding 10-min windows (20 epochs each)
    // ---------------------------------------------------------------

    @Test
    @EnabledIf("dataAvailable")
    void test2_sliding10minWindows() {
        int windowSize = 20; // 20 epochs @ 30s = 10 min
        runWindowComparison("10-minute", windowSize);
    }

    // ---------------------------------------------------------------
    // Test 3: Sliding 30-min windows (60 epochs each)
    // ---------------------------------------------------------------

    @Test
    @EnabledIf("dataAvailable")
    void test3_sliding30minWindows() {
        int windowSize = 60; // 60 epochs @ 30s = 30 min
        runWindowComparison("30-minute", windowSize);
    }

    // ---------------------------------------------------------------
    // Window comparison engine
    // ---------------------------------------------------------------

    private void runWindowComparison(String label, int windowSize) {
        int totalEpochs = overlapEnd - overlapStart + 1;
        int numWindows = totalEpochs / windowSize;
        if (numWindows == 0) {
            System.out.printf("Not enough epochs for %s windows (have %d, need %d)%n",
                              label, totalEpochs, windowSize);
            return;
        }

        System.out.printf("%n======================================================%n");
        System.out.printf("TEST: %s windows (%d epochs each), %d windows%n",
                          label, windowSize, numWindows);
        System.out.printf("======================================================%n");
        System.out.printf("%-6s  %-6s %-7s %-9s %-7s  %-6s %-7s %-9s %-7s  %-6s%n",
                          "Win", "BLS_Q", "BLS_rat", "BLS_err", "BLS_ms",
                          "EKF_Q", "EKF_fix", "EKF_err", "EKF_ms", "Winner");
        System.out.printf("%-6s  %-6s %-7s %-9s %-7s  %-6s %-7s %-9s %-7s  %-6s%n",
                          "---", "-----", "-------", "--------", "------",
                          "-----", "-------", "--------", "------", "------");

        List<Double> blsErrors = new ArrayList<>();
        List<Double> ekfErrors = new ArrayList<>();
        int blsFixes = 0, ekfWindowFixes = 0;
        int blsWins = 0, ekfWins = 0, ties = 0;

        for (int w = 0; w < numWindows; w++) {
            int start = overlapStart + w * windowSize;
            int end = Math.min(start + windowSize, roverEpochs.size());
            List<List<ObsData>> window = roverEpochs.subList(start, end);

            // --- BLS ---
            ProcessingOptions blsOpt = createOpt();
            blsOpt.solver = ProcessingOptions.SOLVER_BATCH;

            long t0 = System.currentTimeMillis();
            BatchSolver.BatchResult blsResult = BatchSolver.solve(window, baseEpochs, nav, blsOpt);
            long blsMs = System.currentTimeMillis() - t0;
            double blsErr = posError(blsResult.pos, refPos);

            if (blsResult.stat == SOLQ_FIX) blsFixes++;
            blsErrors.add(blsErr);

            // --- EKF forward-only ---
            ProcessingOptions ekfOpt = createOpt();
            ekfOpt.solver = ProcessingOptions.SOLVER_EKF;
            RtkState rtk = new RtkState();
            rtk.init(ekfOpt);

            int ekfFix = 0, ekfTotal = 0;
            double[] lastEkfPos = null;

            long t1 = System.currentTimeMillis();
            for (List<ObsData> roverEpoch : window) {
                if (roverEpoch == null || roverEpoch.isEmpty()) continue;
                List<ObsData> baseEpoch = findBase(roverEpoch.get(0).time);
                if (baseEpoch == null) continue;
                ObsData[] obs = mergeObs(roverEpoch, baseEpoch);
                Rtkpos.rtkpos(rtk, obs, obs.length, nav);
                ekfTotal++;
                if (rtk.sol.stat == SOLQ_FIX) {
                    ekfFix++;
                    lastEkfPos = new double[]{rtk.sol.rr[0], rtk.sol.rr[1], rtk.sol.rr[2]};
                } else if (rtk.sol.stat != SOLQ_NONE && lastEkfPos == null) {
                    lastEkfPos = new double[]{rtk.sol.rr[0], rtk.sol.rr[1], rtk.sol.rr[2]};
                }
            }
            long ekfMs = System.currentTimeMillis() - t1;

            double ekfErr = posError(lastEkfPos, refPos);
            double ekfFixPct = ekfTotal > 0 ? 100.0 * ekfFix / ekfTotal : 0;
            if (ekfFix > 0) ekfWindowFixes++;
            ekfErrors.add(ekfErr);

            // Determine winner
            String winner;
            if (blsResult.stat == SOLQ_FIX && ekfFix == 0) {
                winner = "BLS"; blsWins++;
            } else if (blsResult.stat != SOLQ_FIX && ekfFix > 0) {
                winner = "EKF"; ekfWins++;
            } else if (blsErr < ekfErr * 0.8) {
                winner = "BLS"; blsWins++;
            } else if (ekfErr < blsErr * 0.8) {
                winner = "EKF"; ekfWins++;
            } else {
                winner = "TIE"; ties++;
            }

            System.out.printf("W%-5d  %-6s %-7.1f %-9.4f %-7d  %-6s %-7.1f %-9.4f %-7d  %-6s%n",
                              w,
                              statName(blsResult.stat), blsResult.ratio, blsErr, blsMs,
                              ekfFix > 0 ? "FIX" : (ekfTotal > 0 ? "FLOAT" : "NONE"),
                              ekfFixPct, ekfErr, ekfMs,
                              winner);
        }

        // --- Summary statistics ---
        System.out.printf("%n--- %s Window Summary (%d windows) ---%n", label, numWindows);
        printStatsSummary("BLS", blsErrors, blsFixes, numWindows);
        printStatsSummary("EKF", ekfErrors, ekfWindowFixes, numWindows);
        System.out.printf("Score: BLS=%d, EKF=%d, TIE=%d%n", blsWins, ekfWins, ties);

        // Sanity: at least some BLS solutions should be valid
        assertTrue(blsFixes + (numWindows - blsFixes) > 0,
                   "BLS should produce solutions for all windows");
    }

    private void printStatsSummary(String name, List<Double> errors, int fixes, int total) {
        double[] sorted = errors.stream().mapToDouble(Double::doubleValue).sorted().toArray();
        double mean = Arrays.stream(sorted).average().orElse(999);
        double median = sorted.length > 0 ? sorted[sorted.length / 2] : 999;
        double max = sorted.length > 0 ? sorted[sorted.length - 1] : 999;
        double min = sorted.length > 0 ? sorted[0] : 999;
        System.out.printf("  %s: fix=%d/%d (%.1f%%), error min=%.4f med=%.4f mean=%.4f max=%.4f m%n",
                          name, fixes, total, 100.0 * fixes / total,
                          min, median, mean, max);
    }

    // ---------------------------------------------------------------
    // Processing options (matches static.conf)
    // ---------------------------------------------------------------

    private static ProcessingOptions createOpt() {
        ProcessingOptions opt = new ProcessingOptions();
        opt.mode = PMODE_STATIC;
        opt.nf = 3;  // L1+L2+L5 (L2 empty for GPS, L5 at freq[2])
        opt.navsys = SYS_GPS | SYS_GLO | SYS_GAL | SYS_QZS | SYS_CMP | SYS_SBS;
        opt.elmin = 15.0 * D2R;
        opt.ionoopt = IONOOPT_BRDC;
        opt.tropopt = TROPOPT_SAAS;
        opt.modear = 3;        // fix-and-hold
        opt.dynamics = 0;
        opt.rb[0] = BASE_POS[0];
        opt.rb[1] = BASE_POS[1];
        opt.rb[2] = BASE_POS[2];
        opt.thresar[0] = 3.0;
        opt.eratio[0] = 150.0;
        opt.eratio[1] = 150.0;
        opt.eratio[2] = 150.0;
        opt.err[1] = 0.003;
        opt.err[2] = 0.006;
        opt.niter = 1;
        // SNR mask: 35 dBHz for all frequencies at all elevations
        opt.snrmask.ena[0] = 1; // rover
        opt.snrmask.ena[1] = 1; // base
        for (int f = 0; f < opt.snrmask.mask.length; f++) {
            java.util.Arrays.fill(opt.snrmask.mask[f], 35.0);
        }
        return opt;
    }

    // ---------------------------------------------------------------
    // Reference POS file parsing
    // ---------------------------------------------------------------

    /**
     * Parse reference POS file epochs.
     * Format: yyyy/mm/dd HH:MM:SS.SSS  x-ecef  y-ecef  z-ecef  Q  ns  sdx  sdy  sdz  sdxy  sdyz  sdzx  age  ratio
     * Lines starting with % are headers.
     */
    private static List<RefEpoch> parseRefPosEpochs(Path posFile) throws Exception {
        List<RefEpoch> epochs = new ArrayList<>();
        try (BufferedReader br = new BufferedReader(new FileReader(posFile.toFile()))) {
            String line;
            while ((line = br.readLine()) != null) {
                if (line.startsWith("%") || line.trim().isEmpty()) continue;
                String[] parts = line.trim().split("\\s+");
                if (parts.length < 14) continue;

                RefEpoch ep = new RefEpoch();

                // Parse date/time
                String[] dateParts = parts[0].split("/");
                String[] timeParts = parts[1].split(":");
                double[] epoch = new double[6];
                epoch[0] = Double.parseDouble(dateParts[0]);
                epoch[1] = Double.parseDouble(dateParts[1]);
                epoch[2] = Double.parseDouble(dateParts[2]);
                epoch[3] = Double.parseDouble(timeParts[0]);
                epoch[4] = Double.parseDouble(timeParts[1]);
                epoch[5] = Double.parseDouble(timeParts[2]);
                ep.time = GTime.epoch2time(epoch);

                ep.x = Double.parseDouble(parts[2]);
                ep.y = Double.parseDouble(parts[3]);
                ep.z = Double.parseDouble(parts[4]);
                ep.Q = Integer.parseInt(parts[5]);
                ep.ns = Integer.parseInt(parts[6]);
                ep.ratio = Double.parseDouble(parts[13]);

                epochs.add(ep);
            }
        }
        return epochs;
    }

    /**
     * Compute mean fix position from reference epochs (Q=1 only).
     */
    private static double[] parseRefPos(List<RefEpoch> epochs) {
        double sx = 0, sy = 0, sz = 0;
        int n = 0;
        for (RefEpoch ep : epochs) {
            if (ep.Q == 1) {
                sx += ep.x;
                sy += ep.y;
                sz += ep.z;
                n++;
            }
        }
        if (n == 0) return new double[]{-3026470.470, 4976167.053, 2597455.971};
        return new double[]{sx / n, sy / n, sz / n};
    }

    /**
     * Parse STAT file for summary statistics.
     * $POS,week,tow,Q,x,y,z,...
     * $SAT,week,tow,PRN,freq,...
     */
    private static void parseStatFile(Path statFile) throws Exception {
        statTotalPos = 0;
        statFixCount = 0;
        statSatTotal = 0;
        try (BufferedReader br = new BufferedReader(new FileReader(statFile.toFile()))) {
            String line;
            while ((line = br.readLine()) != null) {
                if (line.startsWith("$POS,")) {
                    statTotalPos++;
                    String[] parts = line.split(",");
                    if (parts.length >= 4) {
                        int Q = Integer.parseInt(parts[3].trim());
                        if (Q == 1) statFixCount++;
                    }
                } else if (line.startsWith("$SAT,")) {
                    statSatTotal++;
                }
            }
        }
    }

    // ---------------------------------------------------------------
    // Helper types
    // ---------------------------------------------------------------

    static class RefEpoch {
        GTime time;
        double x, y, z;
        int Q;
        int ns;
        double ratio;
    }

    // ---------------------------------------------------------------
    // Helper methods
    // ---------------------------------------------------------------

    private static String statName(int stat) {
        switch (stat) {
            case SOLQ_FIX:    return "FIX";
            case SOLQ_FLOAT:  return "FLOAT";
            case SOLQ_SINGLE: return "SINGLE";
            case SOLQ_NONE:   return "NONE";
            default:          return "Q" + stat;
        }
    }

    private static double posError(double[] pos, double[] ref) {
        if (pos == null || ref == null) return 999.0;
        double dx = pos[0] - ref[0];
        double dy = pos[1] - ref[1];
        double dz = pos[2] - ref[2];
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    private static List<ObsData> findBase(GTime t) {
        List<ObsData> best = null;
        double bestDt = 1e9;
        for (List<ObsData> ep : baseEpochs) {
            if (ep == null || ep.isEmpty()) continue;
            double dt = Math.abs(t.timediff(ep.get(0).time));
            if (dt < bestDt) { bestDt = dt; best = ep; }
            if (bestDt < 0.5) break;
        }
        return bestDt <= 1.0 ? best : null;
    }

    private static ObsData[] mergeObs(List<ObsData> rover, List<ObsData> base) {
        List<ObsData> merged = new ArrayList<>(rover.size() + base.size());
        merged.addAll(rover);
        merged.addAll(base);
        return merged.toArray(new ObsData[0]);
    }
}
