package com.gnss.rtklib.positioning;

import com.gnss.rtklib.PostProcessor;
import com.gnss.rtklib.core.Constants;
import com.gnss.rtklib.core.Coord;
import com.gnss.rtklib.core.GTime;
import com.gnss.rtklib.io.RinexReader;
import com.gnss.rtklib.io.SolutionWriter;
import com.gnss.rtklib.model.*;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIf;

import java.io.BufferedReader;
import java.io.FileReader;
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
 * End-to-end integration test for RTK static baseline positioning.
 * Uses test data 0759 (rover) and 3040 (base) from RTKLIB test suite.
 */
class RtkEndToEndTest {

    private static final Path TEST_DATA = findTestData();

    private static Path findTestData() {
        Path p = Path.of("").toAbsolutePath().resolve("../test/data/rinex");
        if (Files.exists(p)) return p;
        p = Path.of("").toAbsolutePath().resolve("test/data/rinex");
        if (Files.exists(p)) return p;
        return p;
    }

    static boolean testDataAvailable() {
        return Files.exists(TEST_DATA.resolve("07590920.05o"))
            && Files.exists(TEST_DATA.resolve("30400920.05o"))
            && Files.exists(TEST_DATA.resolve("07590920.05n"));
    }

    static boolean refDataAvailable() {
        return testDataAvailable()
            && Files.exists(TEST_DATA.resolve("ref_rtk_07590920.pos"));
    }

    // Base station position ECEF from RINEX header (3040)
    private static final double[] BASE_POS = {-3978242.4348, 3382841.1715, 3649902.7667};

    private static Navigation nav;
    private static List<List<ObsData>> roverEpochs;
    private static List<List<ObsData>> baseEpochs;

    @BeforeAll
    static void loadData() throws Exception {
        String roverPath = TEST_DATA.resolve("07590920.05o").toString();
        String basePath  = TEST_DATA.resolve("30400920.05o").toString();
        String navPath   = TEST_DATA.resolve("07590920.05n").toString();

        nav = new Navigation();
        RinexReader.readNav(navPath, nav);
        roverEpochs = RinexReader.readObs(roverPath, nav, 1);
        baseEpochs = RinexReader.readObs(basePath, nav, 2);

        assertNotNull(roverEpochs);
        assertNotNull(baseEpochs);
        assertFalse(roverEpochs.isEmpty(), "No rover epochs loaded");
        assertFalse(baseEpochs.isEmpty(), "No base epochs loaded");
    }

    @Test
    @EnabledIf("testDataAvailable")
    void testRtkInitialization() {
        ProcessingOptions opt = createRtkOptions();
        RtkState rtk = new RtkState();
        rtk.init(opt);

        assertTrue(rtk.nx > 0, "nx should be positive");
        assertTrue(rtk.na > 0, "na should be positive");
        assertEquals(3, RtkState.NP(opt), "NP should be 3 for static mode");
        assertTrue(rtk.nx > rtk.na, "nx > na (phase biases included)");
    }

    @Test
    @EnabledIf("testDataAvailable")
    void testRtkSingleEpoch() {
        ProcessingOptions opt = createRtkOptions();
        RtkState rtk = new RtkState();
        rtk.init(opt);

        // Get first rover and base epochs
        List<ObsData> roverEpoch = roverEpochs.get(0);
        List<ObsData> baseEpoch = findMatchingBase(roverEpoch.get(0).time);
        assertNotNull(baseEpoch, "No matching base epoch found");

        // Merge
        ObsData[] obs = mergeObs(roverEpoch, baseEpoch);
        int stat = Rtkpos.rtkpos(rtk, obs, obs.length, nav);

        assertEquals(1, stat, "rtkpos should return success");
        // First epoch is typically single/float
        assertTrue(rtk.sol.stat >= SOLQ_FLOAT || rtk.sol.stat == SOLQ_SINGLE ||
                   rtk.sol.stat == SOLQ_DGPS,
                   "Solution status should be valid: " + rtk.sol.stat);
    }

    @Test
    @EnabledIf("testDataAvailable")
    void testRtkMultiEpochConvergence() {
        ProcessingOptions opt = createRtkOptions();
        RtkState rtk = new RtkState();
        rtk.init(opt);

        int fixCount = 0;
        int floatCount = 0;
        int totalProcessed = 0;
        int maxEpochs = Math.min(roverEpochs.size(), 120); // first ~60 min at 30s

        for (int ep = 0; ep < maxEpochs; ep++) {
            List<ObsData> roverEpoch = roverEpochs.get(ep);
            if (roverEpoch == null || roverEpoch.isEmpty()) continue;

            List<ObsData> baseEpoch = findMatchingBase(roverEpoch.get(0).time);
            if (baseEpoch == null) continue;

            ObsData[] obs = mergeObs(roverEpoch, baseEpoch);
            Rtkpos.rtkpos(rtk, obs, obs.length, nav);
            totalProcessed++;

            if (rtk.sol.stat == SOLQ_FIX) fixCount++;
            else if (rtk.sol.stat == SOLQ_FLOAT) floatCount++;
        }

        assertTrue(totalProcessed > 10, "Should process at least 10 epochs");
        System.out.printf("RTK test: %d epochs processed, %d fix, %d float%n",
                          totalProcessed, fixCount, floatCount);

        // For static baseline, we expect some float solutions at minimum
        assertTrue(floatCount + fixCount > 0,
                   "Should have at least some float or fix solutions");
    }

    @Test
    @EnabledIf("testDataAvailable")
    void testRtkPositionAccuracy() {
        ProcessingOptions opt = createRtkOptions();
        RtkState rtk = new RtkState();
        rtk.init(opt);

        int maxEpochs = Math.min(roverEpochs.size(), 120);
        double[] lastFixPos = null;
        int fixCount = 0;

        for (int ep = 0; ep < maxEpochs; ep++) {
            List<ObsData> roverEpoch = roverEpochs.get(ep);
            if (roverEpoch == null || roverEpoch.isEmpty()) continue;

            List<ObsData> baseEpoch = findMatchingBase(roverEpoch.get(0).time);
            if (baseEpoch == null) continue;

            ObsData[] obs = mergeObs(roverEpoch, baseEpoch);
            Rtkpos.rtkpos(rtk, obs, obs.length, nav);

            if (rtk.sol.stat == SOLQ_FIX) {
                lastFixPos = new double[]{rtk.sol.rr[0], rtk.sol.rr[1], rtk.sol.rr[2]};
                fixCount++;
            }
        }

        if (fixCount > 0 && lastFixPos != null) {
            // Verify the fixed solution is reasonable (within 100m of approx position)
            // Rover approx pos from RINEX: -3976219.5082, 3382372.5671, 3652512.9849
            double dx = lastFixPos[0] - (-3976219.5082);
            double dy = lastFixPos[1] - 3382372.5671;
            double dz = lastFixPos[2] - 3652512.9849;
            double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
            System.out.printf("RTK fix position offset from approx: %.3f m%n", dist);
            assertTrue(dist < 100.0, "Fixed solution should be within 100m of approx position");
        }

        // Also verify float solution is reasonable
        if (rtk.sol.stat >= SOLQ_FLOAT) {
            double dx = rtk.sol.rr[0] - (-3976219.5082);
            double dy = rtk.sol.rr[1] - 3382372.5671;
            double dz = rtk.sol.rr[2] - 3652512.9849;
            double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
            System.out.printf("RTK solution offset from approx: %.3f m (Q=%d)%n",
                              dist, rtk.sol.stat);
            assertTrue(dist < 100.0, "Solution should be within 100m of approx position");
        }
    }

    @Test
    @EnabledIf("testDataAvailable")
    void testBaselineLength() {
        ProcessingOptions opt = createRtkOptions();
        RtkState rtk = new RtkState();
        rtk.init(opt);

        // Process enough epochs to get a solution
        int maxEpochs = Math.min(roverEpochs.size(), 60);
        for (int ep = 0; ep < maxEpochs; ep++) {
            List<ObsData> roverEpoch = roverEpochs.get(ep);
            if (roverEpoch == null || roverEpoch.isEmpty()) continue;

            List<ObsData> baseEpoch = findMatchingBase(roverEpoch.get(0).time);
            if (baseEpoch == null) continue;

            ObsData[] obs = mergeObs(roverEpoch, baseEpoch);
            Rtkpos.rtkpos(rtk, obs, obs.length, nav);
        }

        if (rtk.sol.stat != SOLQ_NONE) {
            // Compute baseline length
            double dx = rtk.sol.rr[0] - BASE_POS[0];
            double dy = rtk.sol.rr[1] - BASE_POS[1];
            double dz = rtk.sol.rr[2] - BASE_POS[2];
            double bl = Math.sqrt(dx * dx + dy * dy + dz * dz);
            System.out.printf("Baseline length: %.3f m%n", bl);

            // Baseline between 0759 and 3040 should be ~2-3 km
            assertTrue(bl > 100.0 && bl < 10000.0,
                       "Baseline should be reasonable (got " + bl + " m)");
        }
    }

    @Test
    @EnabledIf("refDataAvailable")
    void testRtkVsCReference() throws Exception {
        // Parse C reference solution (ECEF format)
        Path refFile = TEST_DATA.resolve("ref_rtk_07590920.pos");
        List<RtkRefSolution> refs = parseRefRtkPos(refFile);
        assertFalse(refs.isEmpty(), "Reference file should not be empty");

        // Run Java RTK
        ProcessingOptions opt = createRtkOptions();
        RtkState rtk = new RtkState();
        rtk.init(opt);

        List<RtkRefSolution> javaResults = new ArrayList<>();
        int maxEpochs = Math.min(roverEpochs.size(), 120);

        for (int ep = 0; ep < maxEpochs; ep++) {
            List<ObsData> roverEpoch = roverEpochs.get(ep);
            if (roverEpoch == null || roverEpoch.isEmpty()) continue;

            List<ObsData> baseEpoch = findMatchingBase(roverEpoch.get(0).time);
            if (baseEpoch == null) continue;

            ObsData[] obs = mergeObs(roverEpoch, baseEpoch);
            Rtkpos.rtkpos(rtk, obs, obs.length, nav);

            if (rtk.sol.stat != SOLQ_NONE) {
                RtkRefSolution r = new RtkRefSolution();
                double[] wt = rtk.sol.time.time2gpst();
                r.week = (int) wt[0];
                r.tow = wt[1];
                r.x = rtk.sol.rr[0];
                r.y = rtk.sol.rr[1];
                r.z = rtk.sol.rr[2];
                r.Q = rtk.sol.stat;
                r.ns = rtk.sol.ns;
                javaResults.add(r);
            }
        }

        System.out.printf("[RTK] C ref epochs: %d, Java epochs: %d%n",
                          refs.size(), javaResults.size());

        // Match and compare
        int matched = 0;
        int fixMatched = 0;
        double maxDx = 0, maxDy = 0, maxDz = 0;
        double maxFixDx = 0, maxFixDy = 0, maxFixDz = 0;
        int qMismatch = 0;

        for (RtkRefSolution ref : refs) {
            RtkRefSolution match = null;
            for (RtkRefSolution r : javaResults) {
                if (r.week == ref.week && Math.abs(r.tow - ref.tow) < 0.01) {
                    match = r;
                    break;
                }
            }
            if (match == null) continue;
            matched++;

            double dx = Math.abs(match.x - ref.x);
            double dy = Math.abs(match.y - ref.y);
            double dz = Math.abs(match.z - ref.z);
            maxDx = Math.max(maxDx, dx);
            maxDy = Math.max(maxDy, dy);
            maxDz = Math.max(maxDz, dz);

            if (match.Q != ref.Q) qMismatch++;

            // For fix-fix comparison, track separately
            if (ref.Q == SOLQ_FIX && match.Q == SOLQ_FIX) {
                fixMatched++;
                maxFixDx = Math.max(maxFixDx, dx);
                maxFixDy = Math.max(maxFixDy, dy);
                maxFixDz = Math.max(maxFixDz, dz);
            }

            // Log epochs with large differences
            double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
            if (dist > 0.01) {
                System.out.printf("  tow=%.1f: dx=%.4f dy=%.4f dz=%.4f (%.4fm) Q=%d/%d%n",
                    ref.tow, match.x - ref.x, match.y - ref.y, match.z - ref.z,
                    dist, match.Q, ref.Q);
            }
        }

        System.out.printf("[RTK] Matched: %d/%d, Fix-Fix: %d, Q mismatch: %d%n",
                          matched, refs.size(), fixMatched, qMismatch);
        System.out.printf("[RTK] Max diff (all): dX=%.4fm dY=%.4fm dZ=%.4fm%n",
                          maxDx, maxDy, maxDz);
        System.out.printf("[RTK] Max diff (fix): dX=%.4fm dY=%.4fm dZ=%.4fm%n",
                          maxFixDx, maxFixDy, maxFixDz);

        assertTrue(matched > 10, "Should match at least 10 epochs");

        // Primary assertion: fixed solutions should match < 1mm in ECEF
        if (fixMatched > 0) {
            double maxFixDist = Math.sqrt(maxFixDx * maxFixDx + maxFixDy * maxFixDy + maxFixDz * maxFixDz);
            System.out.printf("[RTK] Max 3D fix diff: %.4f mm%n", maxFixDist * 1000);
            assertTrue(maxFixDist < 0.001,
                String.format("Fixed solution 3D diff should be < 1mm, got %.4f mm", maxFixDist * 1000));
        }
    }

    // ---------------------------------------------------------------
    // Combined solution tests
    // ---------------------------------------------------------------

    @Test
    @EnabledIf("testDataAvailable")
    void testRtkCombined() {
        ProcessingOptions opt = createRtkOptions();
        opt.soltype = SOLTYPE_COMBINED;

        // Forward pass
        RtkState rtk = new RtkState();
        rtk.init(opt);
        List<Solution> solf = new ArrayList<>();
        processRtkPass(roverEpochs, rtk, opt, solf);
        assertFalse(solf.isEmpty(), "Forward pass should produce solutions");

        // Reset state for combined mode
        rtk = new RtkState();
        rtk.init(opt);

        // Backward pass
        List<List<ObsData>> bwdEpochs = new ArrayList<>(roverEpochs);
        Collections.reverse(bwdEpochs);
        List<Solution> solb = new ArrayList<>();
        processRtkPass(bwdEpochs, rtk, opt, solb);
        assertFalse(solb.isEmpty(), "Backward pass should produce solutions");

        // Combine
        SolutionOptions sopt = new SolutionOptions();
        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw);
        PostProcessor.combres(solf, solb, opt, sopt, pw);
        pw.flush();

        String output = sw.toString();
        String[] lines = output.split("\n");
        int combCount = 0;
        for (String line : lines) {
            if (!line.trim().isEmpty() && !line.startsWith("%")) combCount++;
        }

        System.out.printf("RTK combined: fwd=%d, bwd=%d, combined=%d lines%n",
            solf.size(), solb.size(), combCount);
        assertTrue(combCount > 0, "Combined should produce output");
        assertTrue(combCount >= Math.max(solf.size(), solb.size()),
            String.format("Combined (%d) should have >= max(fwd=%d, bwd=%d)",
                combCount, solf.size(), solb.size()));
    }

    /** Helper: run an RTK pass, collecting solutions. */
    private void processRtkPass(List<List<ObsData>> epochs, RtkState rtk,
                                 ProcessingOptions opt, List<Solution> solBuffer) {
        for (List<ObsData> roverEpoch : epochs) {
            if (roverEpoch == null || roverEpoch.isEmpty()) continue;
            List<ObsData> baseEpoch = findMatchingBase(roverEpoch.get(0).time);
            if (baseEpoch == null) continue;

            ObsData[] obs = mergeObs(roverEpoch, baseEpoch);
            int stat = Rtkpos.rtkpos(rtk, obs, obs.length, nav);
            if (stat != 0 && rtk.sol.stat != SOLQ_NONE) {
                solBuffer.add(rtk.sol.copy());
            }
        }
    }

    // ---------------------------------------------------------------
    // Reference solution parsing
    // ---------------------------------------------------------------

    /**
     * Parse C reference RTK .pos file (ECEF format from -e -t options).
     * Format: yyyy/mm/dd hh:mm:ss.ssss  x-ecef  y-ecef  z-ecef  Q  ns  ...
     */
    private List<RtkRefSolution> parseRefRtkPos(Path file) throws Exception {
        List<RtkRefSolution> refs = new ArrayList<>();
        try (BufferedReader br = new BufferedReader(new FileReader(file.toFile()))) {
            String line;
            while ((line = br.readLine()) != null) {
                if (line.startsWith("%") || line.trim().isEmpty()) continue;
                String[] parts = line.trim().split("\\s+");
                if (parts.length < 7) continue;
                // parts[0]=date parts[1]=time parts[2]=x parts[3]=y parts[4]=z parts[5]=Q parts[6]=ns
                RtkRefSolution r = new RtkRefSolution();
                // Parse date/time to GPS week/tow
                String[] dateParts = parts[0].split("/");
                String[] timeParts = parts[1].split(":");
                double[] ep = new double[6];
                ep[0] = Double.parseDouble(dateParts[0]);
                ep[1] = Double.parseDouble(dateParts[1]);
                ep[2] = Double.parseDouble(dateParts[2]);
                ep[3] = Double.parseDouble(timeParts[0]);
                ep[4] = Double.parseDouble(timeParts[1]);
                ep[5] = Double.parseDouble(timeParts[2]);
                GTime t = GTime.epoch2time(ep);
                double[] wt = t.time2gpst();
                r.week = (int) wt[0];
                r.tow = wt[1];
                r.x = Double.parseDouble(parts[2]);
                r.y = Double.parseDouble(parts[3]);
                r.z = Double.parseDouble(parts[4]);
                r.Q = Integer.parseInt(parts[5]);
                r.ns = Integer.parseInt(parts[6]);
                refs.add(r);
            }
        }
        return refs;
    }

    static class RtkRefSolution {
        int week, Q, ns;
        double tow, x, y, z;
    }

    // ---------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------

    private static ProcessingOptions createRtkOptions() {
        ProcessingOptions opt = new ProcessingOptions();
        opt.mode = PMODE_STATIC;
        opt.nf = 2;
        opt.navsys = SYS_GPS;
        opt.elmin = 15.0 * D2R;
        opt.ionoopt = IONOOPT_BRDC;
        opt.tropopt = TROPOPT_SAAS;
        opt.modear = 3; // fix-and-hold
        opt.dynamics = 0;
        opt.rb[0] = BASE_POS[0];
        opt.rb[1] = BASE_POS[1];
        opt.rb[2] = BASE_POS[2];
        opt.thresar[0] = 3.0;
        opt.eratio[0] = 300.0;
        opt.eratio[1] = 300.0;
        opt.err[1] = 0.003;
        opt.err[2] = 0.003;
        opt.minfix = 10;
        opt.niter = 1;
        return opt;
    }

    private List<ObsData> findMatchingBase(GTime roverTime) {
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
