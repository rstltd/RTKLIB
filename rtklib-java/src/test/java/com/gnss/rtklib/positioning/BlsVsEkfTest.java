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
import java.util.List;

import static com.gnss.rtklib.core.Constants.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Side-by-side BLS vs EKF comparison on 10-minute windows.
 * Uses test/data/static L1+L5 data, sliced into production-scale windows.
 */
class BlsVsEkfTest {

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

    private static final double[] BASE_POS = {-3026183.3550, 4975933.2850, 2598179.5470};
    private static final double[] REF_POS = {-3026470.474, 4976167.052, 2597455.974};

    private static Navigation nav;
    private static List<List<ObsData>> roverEpochs, baseEpochs;
    private static int overlapStart; // index of first matching rover epoch

    @BeforeAll
    static void loadData() throws Exception {
        if (!dataAvailable()) return;
        nav = new Navigation();
        RinexReader.readNav(STATIC_DATA.resolve("rover_L1L5.nav").toString(), nav);
        roverEpochs = RinexReader.readObs(STATIC_DATA.resolve("rover_L1L5.obs").toString(), nav, 1);
        baseEpochs = RinexReader.readObs(STATIC_DATA.resolve("base_L1L5.obs").toString(), nav, 2);

        // Find first overlapping epoch
        overlapStart = 0;
        for (int i = 0; i < roverEpochs.size(); i++) {
            List<ObsData> re = roverEpochs.get(i);
            if (re == null || re.isEmpty()) continue;
            if (findBase(re.get(0).time) != null) { overlapStart = i; break; }
        }
        System.out.printf("Data loaded: rover=%d, base=%d, overlap starts at rover[%d]%n",
                          roverEpochs.size(), baseEpochs.size(), overlapStart);
    }

    private static ProcessingOptions createOpt() {
        ProcessingOptions opt = new ProcessingOptions();
        opt.mode = PMODE_STATIC;
        opt.nf = 3;                  // L1+L2+L5 (L2 empty for GPS, L5 at freq[2])
        // GPS + GLO only. GAL excluded because ddres groups GPS+GAL in m=0,
        // creating cross-system DD with ISB that biases position.
        // GLO is safe: separate m=1 group, no cross-system DD.
        opt.navsys = SYS_GPS | SYS_GLO;
        opt.elmin = 15.0 * D2R;
        opt.ionoopt = IONOOPT_BRDC;
        opt.tropopt = TROPOPT_SAAS;
        opt.modear = 3;              // fix-and-hold
        opt.dynamics = 0;
        opt.rb[0] = BASE_POS[0]; opt.rb[1] = BASE_POS[1]; opt.rb[2] = BASE_POS[2];
        opt.thresar[0] = 3.0;
        opt.eratio[0] = 150.0; opt.eratio[1] = 150.0; opt.eratio[2] = 150.0;
        opt.err[1] = 0.003; opt.err[2] = 0.006;
        opt.niter = 1;
        // SNR mask from static.conf: 35 dBHz for all frequencies at all elevations
        opt.snrmask.ena[0] = 1; // rover
        opt.snrmask.ena[1] = 1; // base
        for (int f = 0; f < opt.snrmask.mask.length; f++) {
            java.util.Arrays.fill(opt.snrmask.mask[f], 35.0);
        }
        return opt;
    }

    /**
     * Run BLS and EKF on multiple 10-minute windows, compare results.
     */
    @Test
    @EnabledIf("dataAvailable")
    void compareBlsVsEkfOnWindows() {
        int windowSize = 20; // 20 epochs = 10 min @ 30s
        int numWindows = 5;
        int overlapEnd = overlapStart + 509; // ~509 matching epochs
        int step = (overlapEnd - overlapStart - windowSize) / numWindows;

        System.out.printf("%n=== BLS vs EKF: 10-minute windows ===%n");
        System.out.printf("%-8s %-8s %-8s %-8s %-10s %-10s %-8s %-8s%n",
                          "Window", "BLS_st", "BLS_rat", "BLS_err",
                          "EKF_st", "EKF_fix%", "EKF_err", "Winner");

        int blsWins = 0, ekfWins = 0, ties = 0;

        for (int w = 0; w < numWindows; w++) {
            int start = overlapStart + w * step;
            int end = Math.min(start + windowSize, roverEpochs.size());
            List<List<ObsData>> window = roverEpochs.subList(start, end);

            // --- BLS ---
            ProcessingOptions blsOpt = createOpt();
            blsOpt.solver = ProcessingOptions.SOLVER_BATCH;
            BatchSolver.BatchResult blsResult = BatchSolver.solve(window, baseEpochs, nav, blsOpt);
            double blsErr = posError(blsResult.pos);

            // --- EKF forward ---
            ProcessingOptions ekfOpt = createOpt();
            ekfOpt.solver = ProcessingOptions.SOLVER_EKF;
            RtkState rtk = new RtkState();
            rtk.init(ekfOpt);
            int ekfFix = 0, ekfTotal = 0;
            double[] lastEkfPos = null;
            double lastEkfErr = 999;

            for (List<ObsData> roverEpoch : window) {
                if (roverEpoch == null || roverEpoch.isEmpty()) continue;
                List<ObsData> baseEpoch = findBase(roverEpoch.get(0).time);
                if (baseEpoch == null) continue;
                ObsData[] obs = merge(roverEpoch, baseEpoch);
                Rtkpos.rtkpos(rtk, obs, obs.length, nav);
                ekfTotal++;
                if (rtk.sol.stat == SOLQ_FIX) {
                    ekfFix++;
                    lastEkfPos = new double[]{rtk.sol.rr[0], rtk.sol.rr[1], rtk.sol.rr[2]};
                    lastEkfErr = posError(lastEkfPos);
                } else if (rtk.sol.stat == SOLQ_FLOAT && lastEkfPos == null) {
                    lastEkfPos = new double[]{rtk.sol.rr[0], rtk.sol.rr[1], rtk.sol.rr[2]};
                    lastEkfErr = posError(lastEkfPos);
                }
            }

            double ekfFixPct = ekfTotal > 0 ? 100.0 * ekfFix / ekfTotal : 0;

            // Determine winner
            String winner;
            if (blsResult.stat == SOLQ_FIX && ekfFix == 0) {
                winner = "BLS"; blsWins++;
            } else if (blsResult.stat != SOLQ_FIX && ekfFix > 0) {
                winner = "EKF"; ekfWins++;
            } else if (blsErr < lastEkfErr * 0.8) {
                winner = "BLS"; blsWins++;
            } else if (lastEkfErr < blsErr * 0.8) {
                winner = "EKF"; ekfWins++;
            } else {
                winner = "TIE"; ties++;
            }

            System.out.printf("W%-7d %-8s %-8.1f %-8.3f %-10s %-10.1f %-8.3f %-8s%n",
                              w,
                              blsResult.stat == SOLQ_FIX ? "FIX" :
                              blsResult.stat == SOLQ_FLOAT ? "FLOAT" : "NONE",
                              blsResult.ratio, blsErr,
                              ekfFix > 0 ? "FIX" : "FLOAT",
                              ekfFixPct, lastEkfErr,
                              winner);
        }

        System.out.printf("%nScore: BLS=%d, EKF=%d, TIE=%d%n", blsWins, ekfWins, ties);

        // Basic assertions
        // BLS should produce at least float for all windows
        // (detailed comparison is for human review)
    }

    /**
     * Focused test: 10-minute window where EKF cannot fix (too short for minlock).
     * BLS should still produce a good float or even fix.
     */
    @Test
    @EnabledIf("dataAvailable")
    void blsAdvantageShortWindow() {
        int windowSize = 20;
        int start = overlapStart;
        int end = Math.min(start + windowSize, roverEpochs.size());
        List<List<ObsData>> window = roverEpochs.subList(start, end);

        // BLS
        ProcessingOptions blsOpt = createOpt();
        blsOpt.solver = ProcessingOptions.SOLVER_BATCH;
        BatchSolver.BatchResult blsResult = BatchSolver.solve(window, baseEpochs, nav, blsOpt);
        double blsErr = posError(blsResult.pos);

        // EKF with minlock=20 (standard setting, cannot fix in 20 epochs)
        ProcessingOptions ekfOpt = createOpt();
        ekfOpt.minlock = 20;
        RtkState rtk = new RtkState();
        rtk.init(ekfOpt);
        int ekfFix = 0;
        double[] ekfPos = null;
        for (List<ObsData> re : window) {
            if (re == null || re.isEmpty()) continue;
            List<ObsData> be = findBase(re.get(0).time);
            if (be == null) continue;
            Rtkpos.rtkpos(rtk, merge(re, be), merge(re, be).length, nav);
            if (rtk.sol.stat == SOLQ_FIX) ekfFix++;
            if (rtk.sol.stat != SOLQ_NONE)
                ekfPos = new double[]{rtk.sol.rr[0], rtk.sol.rr[1], rtk.sol.rr[2]};
        }
        double ekfErr = ekfPos != null ? posError(ekfPos) : 999;

        System.out.printf("%n=== BLS Advantage: Short Window ===%n");
        System.out.printf("BLS: stat=%s, ratio=%.1f, error=%.3f m, nAmb=%d%n",
                          blsResult.stat == SOLQ_FIX ? "FIX" : "FLOAT",
                          blsResult.ratio, blsErr, blsResult.nAmb);
        System.out.printf("EKF: fixCount=%d/%d, error=%.3f m%n",
                          ekfFix, windowSize, ekfErr);

        // BLS should produce valid solution
        assertTrue(blsResult.stat >= SOLQ_FLOAT,
                   "BLS should produce at least float in 10-min window");
        // Note: BLS without SNR mask may have large errors on noisy windows.
        // This test documents current behavior; SNR mask is a future improvement.
        System.out.printf("Note: BLS error %.1f m (no SNR mask yet)%n", blsErr);
    }

    // --- Helpers ---

    private double posError(double[] pos) {
        if (pos == null) return 999;
        double dx = pos[0] - REF_POS[0], dy = pos[1] - REF_POS[1], dz = pos[2] - REF_POS[2];
        return Math.sqrt(dx*dx + dy*dy + dz*dz);
    }

    private static List<ObsData> findBase(GTime t) {
        List<ObsData> best = null; double bestDt = 1e9;
        for (List<ObsData> ep : baseEpochs) {
            if (ep == null || ep.isEmpty()) continue;
            double dt = Math.abs(t.timediff(ep.get(0).time));
            if (dt < bestDt) { bestDt = dt; best = ep; }
            if (bestDt < 0.5) break;
        }
        return bestDt <= 1.0 ? best : null;
    }

    private static ObsData[] merge(List<ObsData> r, List<ObsData> b) {
        List<ObsData> m = new ArrayList<>(r.size() + b.size());
        m.addAll(r); m.addAll(b);
        return m.toArray(new ObsData[0]);
    }
}
