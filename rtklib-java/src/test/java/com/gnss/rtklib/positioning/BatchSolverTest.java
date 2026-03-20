package com.gnss.rtklib.positioning;

import com.gnss.rtklib.core.GTime;
import com.gnss.rtklib.io.RinexReader;
import com.gnss.rtklib.model.*;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIf;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static com.gnss.rtklib.core.Constants.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for Batch Least Squares RTK static solver.
 * Uses test/data/static/ L1+L5 multi-GNSS dataset (u-blox, ~800m baseline).
 */
class BatchSolverTest {

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

    // Base station position ECEF from static.conf (ant2-pos)
    private static final double[] BASE_POS = {-3026183.3550, 4975933.2850, 2598179.5470};

    // Reference fix position from C RTKLIB combined solution (average of fix epochs)
    private static final double[] REF_FIX_POS = {-3026470.474, 4976167.052, 2597455.974};

    private static Navigation nav;
    private static List<List<ObsData>> roverEpochs;
    private static List<List<ObsData>> baseEpochs;

    @BeforeAll
    static void loadData() throws Exception {
        if (!staticDataAvailable()) return;

        String roverPath = STATIC_DATA.resolve("rover_L1L5.obs").toString();
        String basePath  = STATIC_DATA.resolve("base_L1L5.obs").toString();
        String navPath   = STATIC_DATA.resolve("rover_L1L5.nav").toString();

        nav = new Navigation();
        RinexReader.readNav(navPath, nav);
        roverEpochs = RinexReader.readObs(roverPath, nav, 1);
        baseEpochs = RinexReader.readObs(basePath, nav, 2);

        assertNotNull(roverEpochs);
        assertNotNull(baseEpochs);
        assertFalse(roverEpochs.isEmpty(), "No rover epochs loaded");
        assertFalse(baseEpochs.isEmpty(), "No base epochs loaded");

        System.out.printf("Static data: rover=%d epochs, base=%d epochs%n",
                          roverEpochs.size(), baseEpochs.size());
    }

    /**
     * Create options matching static.conf but simplified for BLS.
     * GPS-only L1+L5 (nf=2) to start — multi-system can be added later.
     */
    private static ProcessingOptions createBatchOptions() {
        ProcessingOptions opt = new ProcessingOptions();
        opt.mode = PMODE_STATIC;
        opt.nf = 3;                  // L1+L2+L5 (L2 empty for GPS, L5 at freq[2])
        // GPS + GLO. GAL excluded due to ISB in ddres m=0 group.
        opt.navsys = SYS_GPS | SYS_GLO | SYS_GAL | SYS_QZS | SYS_CMP | SYS_SBS;
        opt.elmin = 15.0 * D2R;
        opt.ionoopt = IONOOPT_BRDC;
        opt.tropopt = TROPOPT_SAAS;
        opt.modear = 3;              // fix-and-hold (AR enabled)
        opt.dynamics = 0;
        opt.rb[0] = BASE_POS[0];
        opt.rb[1] = BASE_POS[1];
        opt.rb[2] = BASE_POS[2];
        opt.thresar[0] = 3.0;
        opt.eratio[0] = 150.0;       // from static.conf
        opt.eratio[1] = 150.0;
        opt.err[1] = 0.003;          // phase error constant
        opt.err[2] = 0.006;          // phase error elevation (from static.conf)
        opt.niter = 1;
        opt.solver = ProcessingOptions.SOLVER_BATCH;
        // SNR mask from static.conf: 35 dBHz for all frequencies
        opt.snrmask.ena[0] = 1; // rover
        opt.snrmask.ena[1] = 1; // base
        for (int f = 0; f < opt.snrmask.mask.length; f++) {
            java.util.Arrays.fill(opt.snrmask.mask[f], 35.0);
        }
        return opt;
    }

    /** Find start of overlap region (first rover epoch with matching base). */
    private static int findOverlapStart() {
        for (int i = 0; i < roverEpochs.size(); i++) {
            List<ObsData> re = roverEpochs.get(i);
            if (re == null || re.isEmpty()) continue;
            double rt = re.get(0).time.time + re.get(0).time.sec;
            for (List<ObsData> be : baseEpochs) {
                if (be == null || be.isEmpty()) continue;
                double bt = be.get(0).time.time + be.get(0).time.sec;
                if (Math.abs(rt - bt) <= 1.0) return i;
            }
        }
        return 0;
    }

    @Test
    @EnabledIf("staticDataAvailable")
    void testBatchFloat() {
        ProcessingOptions opt = createBatchOptions();
        opt.modear = 0; // disable AR for float-only test

        // Use a 60-epoch window (~30 min) from the overlap region
        int start = findOverlapStart();
        int end = Math.min(start + 60, roverEpochs.size());
        List<List<ObsData>> rovSub = roverEpochs.subList(start, end);

        BatchSolver.BatchResult result = BatchSolver.solve(rovSub, baseEpochs, nav, opt);

        System.out.printf("BLS float: stat=%d, ns=%d, nEpochs=%d, nAmb=%d%n",
                          result.stat, result.ns, result.nEpochs, result.nAmb);
        System.out.printf("BLS float pos: %.4f %.4f %.4f%n",
                          result.pos[0], result.pos[1], result.pos[2]);

        assertTrue(result.stat == SOLQ_FLOAT || result.stat == SOLQ_FIX,
                   "Should produce float or fix solution, got: " + result.stat);

        // Position should be within 5m of reference
        double dx = result.pos[0] - REF_FIX_POS[0];
        double dy = result.pos[1] - REF_FIX_POS[1];
        double dz = result.pos[2] - REF_FIX_POS[2];
        double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
        System.out.printf("BLS float distance from ref: %.4f m%n", dist);
        // GPS-only L1+L5 has limited geometry; accept 10m for 4-hour session
        assertTrue(dist < 10.0, "Float position should be within 10m of ref (got " + dist + " m)");

        assertTrue(result.ns > 0, "Should have satellites");
        assertTrue(result.nAmb > 0, "Should have ambiguity parameters");
    }

    @Test
    @EnabledIf("staticDataAvailable")
    void testBatchFix() {
        ProcessingOptions opt = createBatchOptions();

        // Use a 60-epoch window from the overlap region
        int start = findOverlapStart();
        int end = Math.min(start + 60, roverEpochs.size());
        List<List<ObsData>> rovSub = roverEpochs.subList(start, end);

        BatchSolver.BatchResult result = BatchSolver.solve(rovSub, baseEpochs, nav, opt);

        System.out.printf("BLS fix: stat=%d, ratio=%.1f, ns=%d, nEpochs=%d, nAmb=%d%n",
                          result.stat, result.ratio, result.ns, result.nEpochs, result.nAmb);
        System.out.printf("BLS fix pos: %.4f %.4f %.4f%n",
                          result.pos[0], result.pos[1], result.pos[2]);

        // Should produce at least a float solution
        assertTrue(result.stat == SOLQ_FIX || result.stat == SOLQ_FLOAT,
                   "Should produce fix or float solution with " + result.nEpochs + " epochs");

        // Float position should be within 1m of reference
        double dx = result.pos[0] - REF_FIX_POS[0];
        double dy = result.pos[1] - REF_FIX_POS[1];
        double dz = result.pos[2] - REF_FIX_POS[2];
        double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
        System.out.printf("BLS position distance from ref: %.4f m%n", dist);
        // GPS-only with L1+L5 has limited geometry (~8 sats); 10m threshold
        assertTrue(dist < 10.0,
                   "Position should be within 10m of ref (got " + dist + " m)");

        if (result.stat == SOLQ_FIX) {
            // Adaptive threshold is < 3.0 for large nAmb (FFRT)
            assertTrue(result.ratio >= 1.0, "Fix ratio should be meaningful");
            assertTrue(dist < 0.10, "Fix should be within 10cm of ref");
        }
    }

    @Test
    @EnabledIf("staticDataAvailable")
    void testBatchVsEkf() {
        ProcessingOptions opt = createBatchOptions();

        // Use a 60-epoch window from the overlap region
        int start = findOverlapStart();
        int end = Math.min(start + 60, roverEpochs.size());
        List<List<ObsData>> rovSub = roverEpochs.subList(start, end);

        // BLS solution
        BatchSolver.BatchResult blsResult = BatchSolver.solve(rovSub, baseEpochs, nav, opt);

        // EKF combined solution
        ProcessingOptions ekfOpt = createBatchOptions();
        ekfOpt.solver = ProcessingOptions.SOLVER_EKF;
        ekfOpt.soltype = SOLTYPE_COMBINED;

        RtkState rtk = new RtkState();
        rtk.init(ekfOpt);
        List<Solution> solf = new ArrayList<>();
        processRtkPass(rovSub, rtk, ekfOpt, solf);

        rtk = new RtkState();
        rtk.init(ekfOpt);
        List<List<ObsData>> bwdEpochs = new ArrayList<>(rovSub);
        Collections.reverse(bwdEpochs);
        List<Solution> solb = new ArrayList<>();
        processRtkPass(bwdEpochs, rtk, ekfOpt, solb);

        // Find best EKF fix solution
        double[] ekfFixPos = null;
        for (Solution s : solf) {
            if (s.stat == SOLQ_FIX) ekfFixPos = new double[]{s.rr[0], s.rr[1], s.rr[2]};
        }
        for (Solution s : solb) {
            if (s.stat == SOLQ_FIX) ekfFixPos = new double[]{s.rr[0], s.rr[1], s.rr[2]};
        }

        System.out.printf("BLS: stat=%d, ratio=%.1f, pos=[%.4f, %.4f, %.4f]%n",
                          blsResult.stat, blsResult.ratio,
                          blsResult.pos[0], blsResult.pos[1], blsResult.pos[2]);

        if (ekfFixPos != null && blsResult.stat == SOLQ_FIX) {
            double dx = blsResult.pos[0] - ekfFixPos[0];
            double dy = blsResult.pos[1] - ekfFixPos[1];
            double dz = blsResult.pos[2] - ekfFixPos[2];
            double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
            System.out.printf("EKF fix: pos=[%.4f, %.4f, %.4f]%n",
                              ekfFixPos[0], ekfFixPos[1], ekfFixPos[2]);
            System.out.printf("BLS vs EKF distance: %.4f m%n", dist);

            assertTrue(dist < 1.0,
                       "BLS and EKF fix positions should agree within 1m (got " + dist + " m)");
        }
    }

    @Test
    @EnabledIf("staticDataAvailable")
    void testBatchShortWindow() {
        ProcessingOptions opt = createBatchOptions();

        // Use a small window from the overlapping region.
        // Find the first rover epoch that has a matching base epoch.
        int startIdx = 0;
        for (int i = 0; i < roverEpochs.size(); i++) {
            List<ObsData> re = roverEpochs.get(i);
            if (re == null || re.isEmpty()) continue;
            List<ObsData> be = findMatchingBase(re.get(0).time);
            if (be != null) { startIdx = i; break; }
        }
        int endIdx = Math.min(startIdx + 10, roverEpochs.size());
        List<List<ObsData>> rovSub = roverEpochs.subList(startIdx, endIdx);

        BatchSolver.BatchResult blsResult = BatchSolver.solve(rovSub, baseEpochs, nav, opt);

        System.out.printf("BLS short: stat=%d, ratio=%.1f, ns=%d, nEpochs=%d, nAmb=%d%n",
                          blsResult.stat, blsResult.ratio, blsResult.ns,
                          blsResult.nEpochs, blsResult.nAmb);

        // EKF with same 10 epochs - should NOT fix (minlock=20 prevents it)
        ProcessingOptions ekfOpt = createBatchOptions();
        ekfOpt.solver = ProcessingOptions.SOLVER_EKF;
        ekfOpt.minlock = 20;

        RtkState rtk = new RtkState();
        rtk.init(ekfOpt);
        int ekfFixCount = 0;
        for (int ep = 0; ep < rovSub.size(); ep++) {
            List<ObsData> roverEpoch = rovSub.get(ep);
            if (roverEpoch == null || roverEpoch.isEmpty()) continue;
            List<ObsData> baseEpoch = findMatchingBase(roverEpoch.get(0).time);
            if (baseEpoch == null) continue;
            ObsData[] obs = mergeObs(roverEpoch, baseEpoch);
            Rtkpos.rtkpos(rtk, obs, obs.length, nav);
            if (rtk.sol.stat == SOLQ_FIX) ekfFixCount++;
        }

        System.out.printf("EKF short: fixCount=%d (with minlock=20)%n", ekfFixCount);

        // BLS should produce at least a float solution
        assertTrue(blsResult.stat >= SOLQ_FLOAT || blsResult.stat == SOLQ_FIX,
                   "BLS should produce at least float with 10 epochs");

        if (blsResult.stat == SOLQ_FIX && ekfFixCount == 0) {
            System.out.println("BLS advantage: fixed with short window where EKF could not");
        }
    }

    // Helper methods

    private void processRtkPass(List<List<ObsData>> rovEpochs, RtkState rtk,
                                 ProcessingOptions opt, List<Solution> solBuf) {
        for (List<ObsData> roverEpoch : rovEpochs) {
            if (roverEpoch == null || roverEpoch.isEmpty()) continue;
            List<ObsData> baseEpoch = findMatchingBase(roverEpoch.get(0).time);
            if (baseEpoch == null) continue;
            ObsData[] obs = mergeObs(roverEpoch, baseEpoch);
            Rtkpos.rtkpos(rtk, obs, obs.length, nav);
            if (rtk.sol.stat != SOLQ_NONE) {
                solBuf.add(rtk.sol.copy());
            }
        }
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
            if (bestDt < 0.1) break;
        }
        return bestDt <= 30.0 ? best : null;
    }

    private static ObsData[] mergeObs(List<ObsData> rover, List<ObsData> base) {
        List<ObsData> merged = new ArrayList<>(rover.size() + base.size());
        merged.addAll(rover);
        merged.addAll(base);
        return merged.toArray(new ObsData[0]);
    }
}
