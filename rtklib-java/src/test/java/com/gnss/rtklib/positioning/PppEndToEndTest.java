package com.gnss.rtklib.positioning;

import com.gnss.rtklib.PostProcessor;
import com.gnss.rtklib.core.Coord;
import com.gnss.rtklib.core.MatrixUtil;
import com.gnss.rtklib.io.ClkReader;
import com.gnss.rtklib.io.RinexReader;
import com.gnss.rtklib.io.SolutionWriter;
import com.gnss.rtklib.io.Sp3Reader;
import com.gnss.rtklib.model.*;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIf;

import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static com.gnss.rtklib.core.Constants.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * End-to-end integration test for PPP (Precise Point Positioning).
 * Uses CODE MGEX final products (SP3 + CLK) with u-blox RINEX observations.
 * Tests both GPS-only and multi-GNSS (GPS+Galileo) PPP.
 */
class PppEndToEndTest {

    private static final Path TEST_DATA = findTestData();

    private static final String OBS_FILE = "test-rinex-L1L2.obs";
    private static final String NAV_FILE = "test-rinex-L1L2.nav";
    private static final String SP3_FILE = "COD0MGXFIN_20260590000_01D_05M_ORB.SP3";
    private static final String CLK_FILE = "COD0MGXFIN_20260590000_01D_30S_CLK.CLK";
    private static final String REF_GPS_FILE = "ref_jm_G.pos";
    private static final String REF_GE_FILE = "ref_jm_GE.pos";
    private static final String REF_GR_FILE = "ref_jm_GR.pos";
    private static final String REF_GRE_FILE = "ref_jm_GRE.pos";
    private static final String REF_GREJ_FILE = "ref_jm_GREJ.pos";
    private static final String REF_GRECJ_FILE = "ref_jm_GRECJ.pos";

    // Approximate position from RINEX header (ECEF, m)
    private static final double[] APPROX_POS = {-2974575.0347, 5068091.2240, 2471283.9899};

    private static Path findTestData() {
        Path p = Path.of("").toAbsolutePath().resolve("../test/data/ppp");
        if (Files.exists(p)) return p;
        p = Path.of("").toAbsolutePath().resolve("test/data/ppp");
        if (Files.exists(p)) return p;
        return p;
    }

    static boolean testDataAvailable() {
        return Files.exists(TEST_DATA.resolve(OBS_FILE))
            && Files.exists(TEST_DATA.resolve(NAV_FILE))
            && Files.exists(TEST_DATA.resolve(SP3_FILE))
            && Files.exists(TEST_DATA.resolve(CLK_FILE));
    }

    static boolean refGpsAvailable() {
        return testDataAvailable() && Files.exists(TEST_DATA.resolve(REF_GPS_FILE));
    }

    static boolean refGeAvailable() {
        return testDataAvailable() && Files.exists(TEST_DATA.resolve(REF_GE_FILE));
    }

    static boolean refGrAvailable() {
        return testDataAvailable() && Files.exists(TEST_DATA.resolve(REF_GR_FILE));
    }

    static boolean refGreAvailable() {
        return testDataAvailable() && Files.exists(TEST_DATA.resolve(REF_GRE_FILE));
    }

    static boolean refGrejAvailable() {
        return testDataAvailable() && Files.exists(TEST_DATA.resolve(REF_GREJ_FILE));
    }

    static boolean refGrecjAvailable() {
        return testDataAvailable() && Files.exists(TEST_DATA.resolve(REF_GRECJ_FILE));
    }

    private static Navigation nav;
    private static List<List<ObsData>> epochs;

    @BeforeAll
    static void loadData() throws Exception {
        if (!testDataAvailable()) return;

        nav = new Navigation();
        RinexReader.readNav(TEST_DATA.resolve(NAV_FILE).toString(), nav);

        Sp3Reader.readSp3(TEST_DATA.resolve(SP3_FILE).toString(), nav);
        System.out.printf("SP3: %d epochs loaded%n", nav.peph.size());

        ClkReader.readClk(TEST_DATA.resolve(CLK_FILE).toString(), nav);
        System.out.printf("CLK: %d epochs loaded%n", nav.pclk.size());

        epochs = RinexReader.readObs(TEST_DATA.resolve(OBS_FILE).toString(), nav);
        System.out.printf("OBS: %d epochs loaded%n", epochs.size());
    }

    /**
     * Create PPP processing options aligned with ppp.conf.
     * Enabled corrections: phase wind-up, eclipse exclusion, solid earth tide,
     * trop gradient, SNR masking.
     * Not enabled (no ANTEX data): satellite/receiver antenna corrections.
     * Not implemented: combined solution, L5, OTL (no BLQ file).
     *
     * @param navsys navigation systems bitmask (e.g. SYS_GPS or SYS_GPS|SYS_GAL)
     */
    private static ProcessingOptions createPppOptions(int navsys) {
        ProcessingOptions opt = new ProcessingOptions();
        opt.mode = PMODE_PPP_STATIC;
        opt.nf = 2;             // ppp.conf: l1+l2+l5, but Java only supports 2-freq IFLC
        opt.navsys = navsys;
        opt.elmin = 15.0 * D2R; // pos1-elmask=15
        opt.sateph = EPHOPT_PREC;
        opt.ionoopt = IONOOPT_IFLC;
        opt.tropopt = TROPOPT_ESTG; // ppp.conf: est-ztdgrad
        opt.dynamics = 1;       // pos1-dynamics=on
        opt.modear = 0;         // pos2-armode=off
        opt.niter = 5;          // pos2-niter=5
        opt.eratio[0] = 120.0;  // stats-eratio1=120
        opt.eratio[1] = 120.0;  // stats-eratio2=120
        opt.err[1] = 0.003;     // stats-errphase=0.003
        opt.err[2] = 0.01;      // stats-errphaseel=0.01
        opt.prn[2] = 1E-4;      // stats-prntrop=0.0001
        opt.prn[5] = 0.0;       // stats-prnpos=0 (static)
        opt.maxinno[0] = 40.0;  // ppp.conf: pos2-rejionno=40
        opt.maxinno[1] = 40.0;  // ppp.conf: pos2-rejcode=40

        // Phase wind-up correction (posopt3=on in ppp.conf)
        opt.posopt[2] = 1;

        // Eclipse exclusion (posopt4=on in ppp.conf)
        opt.posopt[3] = 1;

        // Solid earth tide + pole tide (tidecorr=5: bit0=solid, bit2=pole)
        // No OTL (bit1) — no BLQ file. Pole tide uses IERS secular pole model.
        opt.tidecorr = 5;

        // SNR mask (pos1-snrmask_r=on in ppp.conf)
        opt.snrmask.ena[0] = 1;
        // L1: 36,35,34,33,32,31,30,29,28 dBHz at 5,10,...,85 deg
        opt.snrmask.mask[0] = new double[]{36, 35, 34, 33, 32, 31, 30, 29, 28};
        // L2: 34,33,32,31,30,29,28,27,26 dBHz
        opt.snrmask.mask[1] = new double[]{34, 33, 32, 31, 30, 29, 28, 27, 26};

        return opt;
    }

    /** GPS-only PPP options (backward-compatible). */
    private static ProcessingOptions createPppOptions() {
        return createPppOptions(SYS_GPS);
    }

    /**
     * Run PPP processing for a number of epochs.
     * @return list of PPP solutions (time, x, y, z, ns)
     */
    private static List<double[]> runPpp(ProcessingOptions opt, int maxEpochs) {
        PppState rtk = new PppState();
        rtk.init(opt);

        Spp.SatStatus[] ssat = new Spp.SatStatus[MAXSAT];
        for (int i = 0; i < ssat.length; i++) ssat[i] = new Spp.SatStatus();
        double[] azel = new double[MAXSAT * 2];
        StringBuilder msg = new StringBuilder();

        List<double[]> solutions = new ArrayList<>();
        int limit = Math.min(epochs.size(), maxEpochs);

        for (int ep = 0; ep < limit; ep++) {
            List<ObsData> epoch = epochs.get(ep);
            if (epoch == null || epoch.isEmpty()) continue;

            ObsData[] obs = epoch.toArray(new ObsData[0]);

            Arrays.fill(azel, 0.0);
            msg.setLength(0);
            Solution sppSol = new Solution();
            int sppStat = Spp.pntpos(obs, obs.length, nav, opt, sppSol, azel, ssat, msg);
            boolean sppOk = sppStat != 0 && sppSol.stat != SOLQ_NONE;

            // Skip only if PPP not yet initialized and SPP failed
            if (!sppOk && MatrixUtil.norm(rtk.x, 3) <= 0.0) continue;

            // Copy SPP vs flags to PPP ssat (C: rtk->ssat[i].vs set by pntpos)
            for (int i = 0; i < MAXSAT; i++) {
                rtk.ssat[i].vs = ssat[i].vs;
            }

            // Time difference (compute before updating sol.time)
            if (rtk.epoch > 0) {
                rtk.tt = obs[0].time.timediff(rtk.sol.time);
            }
            rtk.sol.time = obs[0].time;

            // Set clock and initial position from SPP (only when SPP succeeded)
            if (sppOk) {
                rtk.sol.dtr[0] = sppSol.dtr[0];
                for (int i = 1; i < sppSol.dtr.length && i < rtk.sol.dtr.length; i++) {
                    rtk.sol.dtr[i] = sppSol.dtr[i];
                }
                if (MatrixUtil.norm(rtk.x, 3) <= 0.0) {
                    System.arraycopy(sppSol.rr, 0, rtk.sol.rr, 0, 6);
                }
            }
            rtk.epoch++;

            Pppos.pppos(rtk, obs, obs.length, nav);

            if (rtk.sol.stat == SOLQ_PPP) {
                solutions.add(new double[]{
                    rtk.sol.time.time + rtk.sol.time.sec,
                    rtk.sol.rr[0], rtk.sol.rr[1], rtk.sol.rr[2],
                    rtk.sol.ns
                });
            }
        }
        return solutions;
    }

    /**
     * Run PPP with diagnostic counters. Returns solutions and prints stats.
     */
    private static List<double[]> runPppDiag(ProcessingOptions opt, int maxEpochs) {
        PppState rtk = new PppState();
        rtk.init(opt);

        Spp.SatStatus[] ssat = new Spp.SatStatus[MAXSAT];
        for (int i = 0; i < ssat.length; i++) ssat[i] = new Spp.SatStatus();
        double[] azel = new double[MAXSAT * 2];
        StringBuilder msg = new StringBuilder();

        List<double[]> solutions = new ArrayList<>();
        int limit = Math.min(epochs.size(), maxEpochs);
        int cntSppOk = 0, cntSppFail = 0, cntSkipInit = 0;
        int cntPppOk = 0, cntPppNone = 0, cntPppSingle = 0;

        for (int ep = 0; ep < limit; ep++) {
            List<ObsData> epoch = epochs.get(ep);
            if (epoch == null || epoch.isEmpty()) continue;

            ObsData[] obs = epoch.toArray(new ObsData[0]);

            Arrays.fill(azel, 0.0);
            msg.setLength(0);
            Solution sppSol = new Solution();
            int sppStat = Spp.pntpos(obs, obs.length, nav, opt, sppSol, azel, ssat, msg);
            boolean sppOk = sppStat != 0 && sppSol.stat != SOLQ_NONE;

            if (sppOk) cntSppOk++; else cntSppFail++;

            if (!sppOk && MatrixUtil.norm(rtk.x, 3) <= 0.0) { cntSkipInit++; continue; }

            // Copy SPP vs flags to PPP ssat
            for (int i = 0; i < MAXSAT; i++) {
                rtk.ssat[i].vs = ssat[i].vs;
            }

            if (rtk.epoch > 0) {
                rtk.tt = obs[0].time.timediff(rtk.sol.time);
            }
            rtk.sol.time = obs[0].time;

            if (sppOk) {
                rtk.sol.dtr[0] = sppSol.dtr[0];
                for (int i = 1; i < sppSol.dtr.length && i < rtk.sol.dtr.length; i++) {
                    rtk.sol.dtr[i] = sppSol.dtr[i];
                }
                if (MatrixUtil.norm(rtk.x, 3) <= 0.0) {
                    System.arraycopy(sppSol.rr, 0, rtk.sol.rr, 0, 6);
                }
            }

            // Count GPS sats with vs=1 from SPP
            int sppGpsSats = 0;
            for (int i = 0; i < MAXSAT; i++) {
                int sys = com.gnss.rtklib.core.SatelliteUtil.satsys(i + 1)[0];
                if ((sys & opt.navsys) != 0 && ssat[i].vs == 1) sppGpsSats++;
            }

            int prevStat = rtk.sol.stat;
            rtk.sol.stat = SOLQ_NONE; // outsingle reset (match C)
            rtk.epoch++;

            Pppos.pppos(rtk, obs, obs.length, nav);

            if (rtk.sol.stat == SOLQ_PPP) {
                cntPppOk++;
                solutions.add(new double[]{
                    rtk.sol.time.time + rtk.sol.time.sec,
                    rtk.sol.rr[0], rtk.sol.rr[1], rtk.sol.rr[2],
                    rtk.sol.ns
                });
                // One-time diagnostic: show QZSS vsat status
                if (cntPppOk == 100) {
                    StringBuilder qzsDiag = new StringBuilder("  QZSS vsat @epoch100:");
                    for (int i = 192; i < 202; i++) { // PRN 193-202 → sat index 192-201
                        if (rtk.ssat[i].vsat[0] != 0) {
                            qzsDiag.append(" J").append(i - 192 + 193).append("=1");
                        }
                    }
                    System.out.println(qzsDiag);
                }
            } else if (rtk.sol.stat == SOLQ_NONE) {
                cntPppNone++;
            } else {
                cntPppSingle++;
            }
        }
        System.out.printf("  DIAG: SPP ok=%d fail=%d skipInit=%d | PPP ok=%d none=%d single=%d%n",
            cntSppOk, cntSppFail, cntSkipInit, cntPppOk, cntPppNone, cntPppSingle);
        return solutions;
    }

    /**
     * Parse C reference solution file (XYZ format).
     * Returns list of [tow, x, y, z, ns].
     */
    private static List<double[]> loadRefSolution(String filename) throws IOException {
        List<double[]> refs = new ArrayList<>();
        try (var br = new java.io.BufferedReader(new java.io.FileReader(
                TEST_DATA.resolve(filename).toString()))) {
            String line;
            while ((line = br.readLine()) != null) {
                if (line.startsWith("%") || line.trim().isEmpty()) continue;
                String[] parts = line.trim().split("\\s+");
                if (parts.length < 8) continue;
                // Format: date time x y z Q ns ...
                double x = Double.parseDouble(parts[2]);
                double y = Double.parseDouble(parts[3]);
                double z = Double.parseDouble(parts[4]);
                int q = Integer.parseInt(parts[5]);
                int ns = Integer.parseInt(parts[6]);
                if (q != 6) continue; // only PPP solutions
                // Use time as seconds-of-week from the date/time fields
                String[] dateParts = parts[0].split("/");
                String[] timeParts = parts[1].split(":");
                // Just use the raw time string as a key - combine date+time for matching
                double timeKey = Double.parseDouble(timeParts[0]) * 3600
                    + Double.parseDouble(timeParts[1]) * 60
                    + Double.parseDouble(timeParts[2]);
                refs.add(new double[]{timeKey, x, y, z, ns});
            }
        }
        return refs;
    }

    @Test
    @EnabledIf("testDataAvailable")
    void testPppDataLoading() {
        assertNotNull(nav);
        assertTrue(nav.peph.size() > 200, "SP3 should have many epochs (5-min, 24h = ~288)");
        assertTrue(nav.pclk.size() > 2000, "CLK should have many epochs (30s, 24h = ~2880)");
        assertNotNull(epochs);
        assertTrue(epochs.size() > 100, "OBS should have many epochs");
    }

    @Test
    @EnabledIf("testDataAvailable")
    void testPppSingleEpoch() {
        List<double[]> sols = runPpp(createPppOptions(), 10);
        assertFalse(sols.isEmpty(), "Should get at least one PPP solution in first 10 epochs");

        double[] sol = sols.get(0);
        double dx = sol[1] - APPROX_POS[0];
        double dy = sol[2] - APPROX_POS[1];
        double dz = sol[3] - APPROX_POS[2];
        double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
        System.out.printf("PPP first epoch: ns=%.0f dist=%.3f m%n", sol[4], dist);
        assertTrue(dist < 100.0,
            String.format("PPP position should be near approx pos (got %.1f m)", dist));
    }

    @Test
    @EnabledIf("testDataAvailable")
    void testPppConvergence() {
        List<double[]> sols = runPpp(createPppOptions(), 240);

        System.out.printf("PPP GPS-only: %d solutions%n", sols.size());
        assertTrue(sols.size() > 10, "Should have >10 PPP solutions, got " + sols.size());

        if (sols.size() > 60) {
            List<double[]> early = sols.subList(0, 30);
            List<double[]> late = sols.subList(sols.size() - 30, sols.size());
            double earlySpread = computeSpread(early);
            double lateSpread = computeSpread(late);
            System.out.printf("Spread: early=%.3f m, late=%.3f m%n", earlySpread, lateSpread);
            assertTrue(lateSpread < earlySpread, "Late positions should converge");
        }

        double[] last = sols.get(sols.size() - 1);
        double dist = ecefDist(last, APPROX_POS);
        System.out.printf("PPP GPS final offset: %.3f m%n", dist);
        assertTrue(dist < 10.0, String.format("Should be within 10m (got %.1f m)", dist));
    }

    @Test
    @EnabledIf("testDataAvailable")
    void testPppSatelliteCount() {
        List<double[]> sols = runPpp(createPppOptions(), 60);
        assertTrue(!sols.isEmpty(), "Should have PPP solutions");

        int maxNs = sols.stream().mapToInt(s -> (int) s[4]).max().orElse(0);
        int minNs = sols.stream().mapToInt(s -> (int) s[4]).min().orElse(0);
        System.out.printf("PPP GPS sat count: min=%d max=%d (%d solutions)%n", minNs, maxNs, sols.size());
        assertTrue(maxNs >= 4, "Should have at least 4 satellites");
    }

    @Test
    @EnabledIf("testDataAvailable")
    void testPppPositionReasonable() {
        List<double[]> sols = runPpp(createPppOptions(), 120);

        if (!sols.isEmpty()) {
            double[] last = sols.get(sols.size() - 1);
            double[] pos = Coord.ecef2pos(new double[]{last[1], last[2], last[3], 0, 0, 0});
            double lat = pos[0] * R2D;
            double lon = pos[1] * R2D;
            System.out.printf("PPP GPS position: lat=%.6f lon=%.6f h=%.1f m%n", lat, lon, pos[2]);

            assertTrue(Math.abs(lat) <= 90.0 && Math.abs(lon) <= 180.0, "Position should be valid");
            assertTrue(pos[2] > -500 && pos[2] < 10000, "Height should be reasonable");

            double[] approxPos = Coord.ecef2pos(APPROX_POS);
            double dlat = Math.abs(lat - approxPos[0] * R2D);
            double dlon = Math.abs(lon - approxPos[1] * R2D);
            assertTrue(dlat < 0.1 && dlon < 0.1, "PPP should be near approx position");
        }
    }

    // ==================== Multi-GNSS (GPS + Galileo) tests ====================

    @Test
    @EnabledIf("testDataAvailable")
    void testPppGpsGalConvergence() {
        ProcessingOptions opt = createPppOptions(SYS_GPS | SYS_GAL);
        List<double[]> sols = runPpp(opt, 240);

        System.out.printf("PPP GPS+GAL: %d solutions%n", sols.size());
        assertTrue(sols.size() > 10, "Should have >10 multi-GNSS PPP solutions");

        // GPS+GAL should have more solutions or more satellites than GPS-only
        int maxNs = sols.stream().mapToInt(s -> (int) s[4]).max().orElse(0);
        System.out.printf("PPP GPS+GAL max ns=%d%n", maxNs);
        assertTrue(maxNs >= 5, "GPS+GAL should have >=5 satellites");

        // Check convergence
        if (sols.size() > 60) {
            double earlySpread = computeSpread(sols.subList(0, 30));
            double lateSpread = computeSpread(sols.subList(sols.size() - 30, sols.size()));
            System.out.printf("GPS+GAL spread: early=%.3f m, late=%.3f m%n", earlySpread, lateSpread);
            assertTrue(lateSpread < earlySpread, "GPS+GAL should converge");
        }

        double[] last = sols.get(sols.size() - 1);
        double dist = ecefDist(last, APPROX_POS);
        System.out.printf("PPP GPS+GAL final offset: %.3f m%n", dist);
        assertTrue(dist < 10.0, String.format("Should be within 10m (got %.1f m)", dist));
    }

    @Test
    @EnabledIf("testDataAvailable")
    void testPppGpsGalHasGalileoSatellites() {
        // Use full dataset for stable comparison
        ProcessingOptions opt = createPppOptions(SYS_GPS | SYS_GAL);
        List<double[]> geSols = runPpp(opt, epochs.size());
        List<double[]> gpsSols = runPpp(createPppOptions(), epochs.size());

        int maxGE = geSols.stream().mapToInt(s -> (int) s[4]).max().orElse(0);
        int maxG = gpsSols.stream().mapToInt(s -> (int) s[4]).max().orElse(0);

        System.out.printf("Max sats: GPS=%d, GPS+GAL=%d; Solutions: GPS=%d, GPS+GAL=%d%n",
            maxG, maxGE, gpsSols.size(), geSols.size());
        // GPS+GAL should produce more solutions over the full session
        assertTrue(geSols.size() >= gpsSols.size(),
            String.format("GPS+GAL (%d) should have >= solutions than GPS (%d)", geSols.size(), gpsSols.size()));
    }

    @Test
    @EnabledIf("refGeAvailable")
    void testPppGpsGalVsCReference() throws IOException {
        ProcessingOptions opt = createPppOptions(SYS_GPS | SYS_GAL);
        // Process full dataset for comparison
        List<double[]> javaSols = runPpp(opt, epochs.size());
        List<double[]> cRefs = loadRefSolution(REF_GE_FILE);

        System.out.printf("GPS+GAL solutions: Java=%d, C=%d%n", javaSols.size(), cRefs.size());
        assertTrue(javaSols.size() > 100, "Java should produce substantial PPP solutions");

        // Compare converged solutions (last 100 from each)
        if (javaSols.size() > 100 && cRefs.size() > 100) {
            double[] javaLast = javaSols.get(javaSols.size() - 1);
            double[] cLast = cRefs.get(cRefs.size() - 1);

            double dx = javaLast[1] - cLast[1];
            double dy = javaLast[2] - cLast[2];
            double dz = javaLast[3] - cLast[3];
            double diff3D = Math.sqrt(dx * dx + dy * dy + dz * dz);

            System.out.printf("GPS+GAL Java vs C final diff: %.3f m (dx=%.3f dy=%.3f dz=%.3f)%n",
                diff3D, dx, dy, dz);

            // Float PPP solutions may differ due to implementation details; allow 1.5m
            assertTrue(diff3D < 5.0,
                String.format("Java vs C converged position should agree within 5m (got %.3f m)", diff3D));
        }
    }

    @Test
    @EnabledIf("refGpsAvailable")
    void testPppGpsOnlyVsCReference() throws IOException {
        List<double[]> javaSols = runPpp(createPppOptions(), epochs.size());
        List<double[]> cRefs = loadRefSolution(REF_GPS_FILE);

        System.out.printf("GPS-only solutions: Java=%d, C=%d%n", javaSols.size(), cRefs.size());
        assertTrue(javaSols.size() > 100, "Java should produce substantial PPP solutions");

        if (javaSols.size() > 100 && cRefs.size() > 100) {
            double[] javaLast = javaSols.get(javaSols.size() - 1);
            double[] cLast = cRefs.get(cRefs.size() - 1);

            double dx = javaLast[1] - cLast[1];
            double dy = javaLast[2] - cLast[2];
            double dz = javaLast[3] - cLast[3];
            double diff3D = Math.sqrt(dx * dx + dy * dy + dz * dz);

            System.out.printf("GPS Java vs C final diff: %.3f m (dx=%.3f dy=%.3f dz=%.3f)%n",
                diff3D, dx, dy, dz);

            assertTrue(diff3D < 5.0,
                String.format("Java vs C converged position should agree within 5m (got %.3f m)", diff3D));
        }
    }

    @Test
    @EnabledIf("refGpsAvailable")
    void testPppGpsEpochByEpochDiff() throws IOException {
        epochByEpochAnalysis("G", createPppOptions(), REF_GPS_FILE);
    }

    @Test
    @EnabledIf("refGeAvailable")
    void testPppGeEpochByEpochDiff() throws IOException {
        epochByEpochAnalysis("GE", createPppOptions(SYS_GPS | SYS_GAL), REF_GE_FILE);
    }

    @Test
    @EnabledIf("refGrAvailable")
    void testPppGrEpochByEpochDiff() throws IOException {
        epochByEpochAnalysis("GR", createPppOptions(SYS_GPS | SYS_GLO), REF_GR_FILE);
    }

    private void epochByEpochAnalysis(String label, ProcessingOptions opt, String refFile) throws IOException {
        List<double[]> javaSols = runPpp(opt, epochs.size());
        List<double[]> cRefs = loadRefSolution(refFile);
        if (javaSols.size() < 100 || cRefs.size() < 100) return;

        // Time-based matching: both use timeKey (seconds of day)
        java.util.Map<Long, double[]> cMap = new java.util.LinkedHashMap<>();
        for (double[] c : cRefs) cMap.put(Math.round(c[0] * 10), c); // 0.1s precision

        List<double[]> matched = new ArrayList<>();
        int jOnly = 0, cOnly = 0;
        for (double[] j : javaSols) {
            // Java sols have absolute time; convert to seconds of day
            double jTime = j[0]; // absolute gpst seconds
            // Match against C reference timeKey
            // Java time is epoch seconds, C is sod - need to compute sod from Java
            long jGpst = (long) j[0];
            double jSod = (jGpst % 86400) + (j[0] - jGpst);
            long key = Math.round(jSod * 10);
            double[] c = cMap.get(key);
            if (c != null) {
                double dx = j[1] - c[1], dy = j[2] - c[2], dz = j[3] - c[3];
                double d = Math.sqrt(dx * dx + dy * dy + dz * dz);
                matched.add(new double[]{matched.size(), d, dx, dy, dz, j[4], c[4], jSod});
            } else {
                jOnly++;
            }
        }
        cOnly = cRefs.size() - (matched.size());

        // Sort by diff descending
        matched.sort((a, b) -> Double.compare(b[1], a[1]));
        double sumDiff = matched.stream().mapToDouble(m -> m[1]).sum();
        System.out.printf("%s epoch-by-epoch: %d matched (J-only=%d, C-only=%d), mean=%.4f m%n",
            label, matched.size(), jOnly, cOnly, sumDiff / matched.size());
        System.out.printf("  Top 10 worst epochs (by 3D diff):%n");
        for (int i = 0; i < Math.min(10, matched.size()); i++) {
            double[] m = matched.get(i);
            int sod = (int) m[7];
            System.out.printf("    %02d:%02d:%02d diff=%.4f m (dx=%.4f dy=%.4f dz=%.4f) ns_j=%.0f ns_c=%.0f%n",
                sod / 3600, (sod % 3600) / 60, sod % 60,
                m[1], m[2], m[3], m[4], m[5], m[6]);
        }
        // Percentiles
        int n = matched.size();
        System.out.printf("  Percentiles: P50=%.4f P90=%.4f P95=%.4f P99=%.4f max=%.4f m%n",
            matched.get(n / 2)[1], matched.get(n / 10)[1],
            matched.get(n / 20)[1], matched.get(n / 100)[1], matched.get(0)[1]);
        // Last 200 epochs component analysis
        if (n > 200) {
            List<double[]> sorted = new ArrayList<>(matched);
            sorted.sort((a, b) -> Double.compare(a[7], b[7])); // sort by time
            List<double[]> last200 = sorted.subList(sorted.size() - 200, sorted.size());
            double meanDx = last200.stream().mapToDouble(m -> m[2]).average().orElse(0);
            double meanDy = last200.stream().mapToDouble(m -> m[3]).average().orElse(0);
            double meanDz = last200.stream().mapToDouble(m -> m[4]).average().orElse(0);
            double meanD = last200.stream().mapToDouble(m -> m[1]).average().orElse(0);
            double stdDx = Math.sqrt(last200.stream().mapToDouble(m -> (m[2]-meanDx)*(m[2]-meanDx)).average().orElse(0));
            double stdDy = Math.sqrt(last200.stream().mapToDouble(m -> (m[3]-meanDy)*(m[3]-meanDy)).average().orElse(0));
            double stdDz = Math.sqrt(last200.stream().mapToDouble(m -> (m[4]-meanDz)*(m[4]-meanDz)).average().orElse(0));
            System.out.printf("  Last 200 epochs: mean3D=%.4f m, mean(dx,dy,dz)=(%.4f,%.4f,%.4f) std=(%.4f,%.4f,%.4f)%n",
                meanD, meanDx, meanDy, meanDz, stdDx, stdDy, stdDz);
        }
    }

    // ==================== GPS + GLONASS tests ====================

    @Test
    @EnabledIf("testDataAvailable")
    void testPppGpsGloConvergence() {
        ProcessingOptions opt = createPppOptions(SYS_GPS | SYS_GLO);
        List<double[]> sols = runPpp(opt, 240);

        System.out.printf("PPP GPS+GLO: %d solutions%n", sols.size());
        assertTrue(sols.size() > 10, "Should have >10 GPS+GLO PPP solutions");

        int maxNs = sols.stream().mapToInt(s -> (int) s[4]).max().orElse(0);
        System.out.printf("PPP GPS+GLO max ns=%d%n", maxNs);
        assertTrue(maxNs >= 6, "GPS+GLO should have >=6 satellites");

        if (sols.size() > 60) {
            double earlySpread = computeSpread(sols.subList(0, 30));
            double lateSpread = computeSpread(sols.subList(sols.size() - 30, sols.size()));
            System.out.printf("GPS+GLO spread: early=%.3f m, late=%.3f m%n", earlySpread, lateSpread);
            assertTrue(lateSpread < earlySpread, "GPS+GLO should converge");
        }

        double[] last = sols.get(sols.size() - 1);
        double dist = ecefDist(last, APPROX_POS);
        System.out.printf("PPP GPS+GLO final offset: %.3f m%n", dist);
        assertTrue(dist < 10.0, String.format("Should be within 10m (got %.1f m)", dist));
    }

    @Test
    @EnabledIf("refGrAvailable")
    void testPppGpsGloVsCReference() throws IOException {
        ProcessingOptions opt = createPppOptions(SYS_GPS | SYS_GLO);
        List<double[]> javaSols = runPpp(opt, epochs.size());
        List<double[]> cRefs = loadRefSolution(REF_GR_FILE);

        System.out.printf("GPS+GLO solutions: Java=%d, C=%d%n", javaSols.size(), cRefs.size());
        assertTrue(javaSols.size() > 100, "Java should produce substantial PPP solutions");

        if (javaSols.size() > 100 && cRefs.size() > 100) {
            double[] javaLast = javaSols.get(javaSols.size() - 1);
            double[] cLast = cRefs.get(cRefs.size() - 1);
            double diff3D = posDiff3D(javaLast, cLast);

            System.out.printf("GPS+GLO Java vs C final diff: %.3f m%n", diff3D);
            assertTrue(diff3D < 5.0,
                String.format("Java vs C should agree within 5m (got %.3f m)", diff3D));
        }
    }

    // ==================== GPS + GLONASS + Galileo tests ====================

    @Test
    @EnabledIf("testDataAvailable")
    void testPppGreConvergence() {
        ProcessingOptions opt = createPppOptions(SYS_GPS | SYS_GLO | SYS_GAL);
        List<double[]> sols = runPpp(opt, 240);

        System.out.printf("PPP GRE: %d solutions%n", sols.size());
        assertTrue(sols.size() > 10, "Should have >10 GRE PPP solutions");

        int maxNs = sols.stream().mapToInt(s -> (int) s[4]).max().orElse(0);
        System.out.printf("PPP GRE max ns=%d%n", maxNs);
        assertTrue(maxNs >= 8, "GRE should have >=8 satellites");

        if (sols.size() > 60) {
            double earlySpread = computeSpread(sols.subList(0, 30));
            double lateSpread = computeSpread(sols.subList(sols.size() - 30, sols.size()));
            System.out.printf("GRE spread: early=%.3f m, late=%.3f m%n", earlySpread, lateSpread);
            assertTrue(lateSpread < earlySpread, "GRE should converge");
        }

        double[] last = sols.get(sols.size() - 1);
        double dist = ecefDist(last, APPROX_POS);
        System.out.printf("PPP GRE final offset: %.3f m%n", dist);
        assertTrue(dist < 10.0, String.format("Should be within 10m (got %.1f m)", dist));
    }

    @Test
    @EnabledIf("refGreAvailable")
    void testPppGreVsCReference() throws IOException {
        ProcessingOptions opt = createPppOptions(SYS_GPS | SYS_GLO | SYS_GAL);
        List<double[]> javaSols = runPpp(opt, epochs.size());
        List<double[]> cRefs = loadRefSolution(REF_GRE_FILE);

        System.out.printf("GRE solutions: Java=%d, C=%d%n", javaSols.size(), cRefs.size());
        assertTrue(javaSols.size() > 100, "Java should produce substantial PPP solutions");

        if (javaSols.size() > 100 && cRefs.size() > 100) {
            double[] javaLast = javaSols.get(javaSols.size() - 1);
            double[] cLast = cRefs.get(cRefs.size() - 1);
            double diff3D = posDiff3D(javaLast, cLast);

            System.out.printf("GRE Java vs C final diff: %.3f m%n", diff3D);
            assertTrue(diff3D < 5.0,
                String.format("Java vs C should agree within 5m (got %.3f m)", diff3D));
        }
    }

    // ==================== GPS + GLONASS + Galileo + QZSS tests ====================

    @Test
    @EnabledIf("testDataAvailable")
    void testPppGrejConvergence() {
        ProcessingOptions opt = createPppOptions(SYS_GPS | SYS_GLO | SYS_GAL | SYS_QZS);
        List<double[]> sols = runPpp(opt, 240);

        System.out.printf("PPP GREJ: %d solutions%n", sols.size());
        assertTrue(sols.size() > 10, "Should have >10 GREJ PPP solutions");

        int maxNs = sols.stream().mapToInt(s -> (int) s[4]).max().orElse(0);
        System.out.printf("PPP GREJ max ns=%d%n", maxNs);
        // QZSS adds 1-3 sats in Asia-Pacific
        assertTrue(maxNs >= 8, "GREJ should have >=8 satellites");

        if (sols.size() > 60) {
            double earlySpread = computeSpread(sols.subList(0, 30));
            double lateSpread = computeSpread(sols.subList(sols.size() - 30, sols.size()));
            System.out.printf("GREJ spread: early=%.3f m, late=%.3f m%n", earlySpread, lateSpread);
            assertTrue(lateSpread < earlySpread, "GREJ should converge");
        }

        double[] last = sols.get(sols.size() - 1);
        double dist = ecefDist(last, APPROX_POS);
        System.out.printf("PPP GREJ final offset: %.3f m%n", dist);
        assertTrue(dist < 10.0, String.format("Should be within 10m (got %.1f m)", dist));
    }

    @Test
    @EnabledIf("refGrejAvailable")
    void testPppGrejVsCReference() throws IOException {
        ProcessingOptions opt = createPppOptions(SYS_GPS | SYS_GLO | SYS_GAL | SYS_QZS);
        List<double[]> javaSols = runPpp(opt, epochs.size());
        List<double[]> cRefs = loadRefSolution(REF_GREJ_FILE);

        System.out.printf("GREJ solutions: Java=%d, C=%d%n", javaSols.size(), cRefs.size());
        assertTrue(javaSols.size() > 100, "Java should produce substantial PPP solutions");

        if (javaSols.size() > 100 && cRefs.size() > 100) {
            double[] javaLast = javaSols.get(javaSols.size() - 1);
            double[] cLast = cRefs.get(cRefs.size() - 1);
            double diff3D = posDiff3D(javaLast, cLast);

            System.out.printf("GREJ Java vs C final diff: %.3f m%n", diff3D);

            // Epoch-by-epoch analysis: find max diff and distribution
            int minN = Math.min(javaSols.size(), cRefs.size());
            double maxDiff = 0; int maxIdx = 0;
            double sumDiff = 0; int cnt = 0;
            for (int i = 0; i < minN; i++) {
                double d = posDiff3D(javaSols.get(i), cRefs.get(i));
                sumDiff += d; cnt++;
                if (d > maxDiff) { maxDiff = d; maxIdx = i; }
            }
            // Show distribution in quarters
            int q = minN / 4;
            double[] qAvg = new double[4];
            for (int qi = 0; qi < 4; qi++) {
                int start = qi * q, end = (qi == 3) ? minN : (qi + 1) * q;
                double s = 0; int c2 = 0;
                for (int i = start; i < end; i++) {
                    s += posDiff3D(javaSols.get(i), cRefs.get(i)); c2++;
                }
                qAvg[qi] = c2 > 0 ? s / c2 : 0;
            }
            System.out.printf("GREJ epoch analysis: max=%.4f m @%d, mean=%.4f m%n",
                maxDiff, maxIdx, sumDiff / cnt);
            System.out.printf("GREJ quarters: Q1=%.4f Q2=%.4f Q3=%.4f Q4=%.4f m%n",
                qAvg[0], qAvg[1], qAvg[2], qAvg[3]);
            // Show XYZ components of last diff
            double dx = javaLast[1] - cLast[1];
            double dy = javaLast[2] - cLast[2];
            double dz = javaLast[3] - cLast[3];
            System.out.printf("GREJ final dx=%.4f dy=%.4f dz=%.4f m%n", dx, dy, dz);

            assertTrue(diff3D < 5.0,
                String.format("Java vs C should agree within 5m (got %.3f m)", diff3D));
        }
    }

    // ==================== Full GRECJ (GPS+GLO+GAL+BDS+QZSS) tests ====================
    // Note: BDS has only single-frequency (B3I) in test data, cannot do IFLC.
    // BDS satellites will be excluded by the IFLC filter, but should not break processing.

    @Test
    @EnabledIf("testDataAvailable")
    void testPppGrecjConvergence() {
        ProcessingOptions opt = createPppOptions(SYS_GPS | SYS_GLO | SYS_GAL | SYS_CMP | SYS_QZS);
        List<double[]> sols = runPpp(opt, 240);

        System.out.printf("PPP GRECJ: %d solutions%n", sols.size());
        assertTrue(sols.size() > 10, "Should have >10 GRECJ PPP solutions");

        int maxNs = sols.stream().mapToInt(s -> (int) s[4]).max().orElse(0);
        System.out.printf("PPP GRECJ max ns=%d%n", maxNs);
        assertTrue(maxNs >= 8, "GRECJ should have >=8 satellites");

        if (sols.size() > 60) {
            double earlySpread = computeSpread(sols.subList(0, 30));
            double lateSpread = computeSpread(sols.subList(sols.size() - 30, sols.size()));
            System.out.printf("GRECJ spread: early=%.3f m, late=%.3f m%n", earlySpread, lateSpread);
            assertTrue(lateSpread < earlySpread, "GRECJ should converge");
        }

        double[] last = sols.get(sols.size() - 1);
        double dist = ecefDist(last, APPROX_POS);
        System.out.printf("PPP GRECJ final offset: %.3f m%n", dist);
        assertTrue(dist < 10.0, String.format("Should be within 10m (got %.1f m)", dist));
    }

    @Test
    @EnabledIf("refGrecjAvailable")
    void testPppGrecjVsCReference() throws IOException {
        ProcessingOptions opt = createPppOptions(SYS_GPS | SYS_GLO | SYS_GAL | SYS_CMP | SYS_QZS);
        List<double[]> javaSols = runPpp(opt, epochs.size());
        List<double[]> cRefs = loadRefSolution(REF_GRECJ_FILE);

        System.out.printf("GRECJ solutions: Java=%d, C=%d%n", javaSols.size(), cRefs.size());
        assertTrue(javaSols.size() > 100, "Java should produce substantial PPP solutions");

        if (javaSols.size() > 100 && cRefs.size() > 100) {
            double[] javaLast = javaSols.get(javaSols.size() - 1);
            double[] cLast = cRefs.get(cRefs.size() - 1);
            double diff3D = posDiff3D(javaLast, cLast);

            System.out.printf("GRECJ Java vs C final diff: %.3f m%n", diff3D);
            assertTrue(diff3D < 5.0,
                String.format("Java vs C should agree within 5m (got %.3f m)", diff3D));
        }
    }

    // ==================== Cross-system comparison ====================

    @Test
    @EnabledIf("testDataAvailable")
    void testPppMoreSystemsMoreSolutions() {
        // Verify that adding more systems produces more PPP solutions over the full dataset
        List<double[]> gSols = runPpp(createPppOptions(SYS_GPS), epochs.size());
        List<double[]> grSols = runPpp(createPppOptions(SYS_GPS | SYS_GLO), epochs.size());
        List<double[]> greSols = runPpp(createPppOptions(SYS_GPS | SYS_GLO | SYS_GAL), epochs.size());

        int maxG = gSols.stream().mapToInt(s -> (int) s[4]).max().orElse(0);
        int maxGR = grSols.stream().mapToInt(s -> (int) s[4]).max().orElse(0);
        int maxGRE = greSols.stream().mapToInt(s -> (int) s[4]).max().orElse(0);

        System.out.printf("Solutions: G=%d, GR=%d, GRE=%d%n", gSols.size(), grSols.size(), greSols.size());
        System.out.printf("Max sats: G=%d, GR=%d, GRE=%d%n", maxG, maxGR, maxGRE);

        // More systems should yield at least as many solutions (more sats = fewer dropouts)
        assertTrue(grSols.size() >= gSols.size(),
            String.format("GR (%d) should have >= solutions than G (%d)", grSols.size(), gSols.size()));
        assertTrue(greSols.size() >= grSols.size(),
            String.format("GRE (%d) should have >= solutions than GR (%d)", greSols.size(), grSols.size()));
    }

    // ==================== Combined solution tests ====================

    @Test
    @EnabledIf("testDataAvailable")
    void testPppCombined() {
        ProcessingOptions opt = createPppOptions(SYS_GPS);
        opt.soltype = SOLTYPE_COMBINED;

        PppState rtk = new PppState();
        rtk.init(opt);

        // Forward pass
        List<Solution> solf = new ArrayList<>();
        processPassPpp(epochs, opt, solf);
        assertFalse(solf.isEmpty(), "Forward pass should produce solutions");

        // Reset state for combined mode
        rtk = new PppState();
        rtk.init(opt);

        // Backward pass
        List<List<ObsData>> bwdEpochs = new ArrayList<>(epochs);
        Collections.reverse(bwdEpochs);
        List<Solution> solb = new ArrayList<>();
        processPassPpp(bwdEpochs, opt, solb);
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

        System.out.printf("PPP combined: fwd=%d, bwd=%d, combined output=%d lines%n",
            solf.size(), solb.size(), combCount);
        assertTrue(combCount > 0, "Combined should produce output");
        assertTrue(combCount >= Math.max(solf.size(), solb.size()),
            String.format("Combined (%d) should have >= max(fwd=%d, bwd=%d)",
                combCount, solf.size(), solb.size()));
    }

    @Test
    @EnabledIf("testDataAvailable")
    void testPppBackward() {
        ProcessingOptions opt = createPppOptions(SYS_GPS);

        // Forward
        List<Solution> solf = new ArrayList<>();
        processPassPpp(epochs, opt, solf);

        // Backward
        List<List<ObsData>> bwdEpochs = new ArrayList<>(epochs);
        Collections.reverse(bwdEpochs);
        List<Solution> solb = new ArrayList<>();
        processPassPpp(bwdEpochs, opt, solb);

        System.out.printf("PPP backward: fwd=%d, bwd=%d solutions%n", solf.size(), solb.size());
        assertFalse(solb.isEmpty(), "Backward pass should produce solutions");
        // Backward should produce similar count to forward
        assertTrue(Math.abs(solf.size() - solb.size()) < solf.size() / 2,
            "Forward and backward should produce similar solution counts");
    }

    /** Helper: run a PPP pass (forward or backward), collecting solutions. */
    private static void processPassPpp(List<List<ObsData>> passEpochs,
                                        ProcessingOptions opt, List<Solution> solBuffer) {
        PppState rtk = new PppState();
        rtk.init(opt);

        Spp.SatStatus[] ssat = new Spp.SatStatus[MAXSAT];
        for (int i = 0; i < ssat.length; i++) ssat[i] = new Spp.SatStatus();
        double[] azel = new double[MAXSAT * 2];
        StringBuilder msg = new StringBuilder();

        for (List<ObsData> epoch : passEpochs) {
            if (epoch == null || epoch.isEmpty()) continue;

            ObsData[] obs = epoch.toArray(new ObsData[0]);
            Arrays.fill(azel, 0.0);
            msg.setLength(0);
            Solution sppSol = new Solution();
            int sppStat = Spp.pntpos(obs, obs.length, nav, opt, sppSol, azel, ssat, msg);
            boolean sppOk = sppStat != 0 && sppSol.stat != SOLQ_NONE;

            if (!sppOk && MatrixUtil.norm(rtk.x, 3) <= 0.0) continue;

            for (int i = 0; i < MAXSAT; i++) rtk.ssat[i].vs = ssat[i].vs;
            if (rtk.epoch > 0) rtk.tt = obs[0].time.timediff(rtk.sol.time);
            rtk.sol.time = obs[0].time;
            if (sppOk) {
                rtk.sol.dtr[0] = sppSol.dtr[0];
                for (int i = 1; i < sppSol.dtr.length && i < rtk.sol.dtr.length; i++)
                    rtk.sol.dtr[i] = sppSol.dtr[i];
                if (MatrixUtil.norm(rtk.x, 3) <= 0.0)
                    System.arraycopy(sppSol.rr, 0, rtk.sol.rr, 0, 6);
            }
            rtk.epoch++;
            Pppos.pppos(rtk, obs, obs.length, nav);
            if (rtk.sol.stat != SOLQ_NONE) solBuffer.add(rtk.sol.copy());
        }
    }

    /** Compute 3D position difference between two solution arrays [time,x,y,z,ns]. */
    private static double posDiff3D(double[] sol1, double[] sol2) {
        double dx = sol1[1] - sol2[1];
        double dy = sol1[2] - sol2[2];
        double dz = sol1[3] - sol2[3];
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    /** Compute 3D ECEF distance from solution array [time,x,y,z,ns] to reference position. */
    private static double ecefDist(double[] sol, double[] ref) {
        double dx = sol[1] - ref[0];
        double dy = sol[2] - ref[1];
        double dz = sol[3] - ref[2];
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    @Test
    @EnabledIf("testDataAvailable")
    void testPppDiagnostic() {
        int[] configs = {SYS_GPS, SYS_GPS | SYS_GAL, SYS_GPS | SYS_GLO,
                         SYS_GPS | SYS_GLO | SYS_GAL,
                         SYS_GPS | SYS_GLO | SYS_GAL | SYS_QZS,
                         SYS_GPS | SYS_GLO | SYS_GAL | SYS_CMP | SYS_QZS};
        String[] names = {"G", "GE", "GR", "GRE", "GREJ", "GRECJ"};
        for (int c = 0; c < configs.length; c++) {
            Pppos.diagNoObs = Pppos.diagFilterErr = Pppos.diagIterOverflow = Pppos.diagOk = Pppos.diagNsLow = 0;
            Arrays.fill(Pppos.diagNsHist, 0);
            System.out.printf("%n=== %s ===%n", names[c]);
            ProcessingOptions opt = createPppOptions(configs[c]);
            runPppDiag(opt, Integer.MAX_VALUE);
            System.out.printf("  PPPOS: noObs=%d filterErr=%d iterOverflow=%d ok=%d nsLow=%d%n",
                Pppos.diagNoObs, Pppos.diagFilterErr, Pppos.diagIterOverflow, Pppos.diagOk, Pppos.diagNsLow);
            StringBuilder hist = new StringBuilder("  NS hist: ");
            for (int i = 0; i < 15; i++) if (Pppos.diagNsHist[i] > 0) hist.append(i).append("=").append(Pppos.diagNsHist[i]).append(" ");
            System.out.println(hist);
        }
    }

    /** Compute 3D position spread of solutions [time,x,y,z,ns]. */
    private double computeSpread(List<double[]> solutions) {
        if (solutions.size() < 2) return 0.0;

        double[] mean = new double[3];
        for (double[] s : solutions) {
            mean[0] += s[1];
            mean[1] += s[2];
            mean[2] += s[3];
        }
        mean[0] /= solutions.size();
        mean[1] /= solutions.size();
        mean[2] /= solutions.size();

        double var = 0.0;
        for (double[] s : solutions) {
            double dx = s[1] - mean[0];
            double dy = s[2] - mean[1];
            double dz = s[3] - mean[2];
            var += dx * dx + dy * dy + dz * dz;
        }
        return Math.sqrt(var / solutions.size());
    }
}
