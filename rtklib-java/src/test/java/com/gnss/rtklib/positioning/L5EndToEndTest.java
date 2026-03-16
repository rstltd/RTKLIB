package com.gnss.rtklib.positioning;

import com.gnss.rtklib.PostProcessor;
import com.gnss.rtklib.core.GTime;
import com.gnss.rtklib.io.RinexReader;
import com.gnss.rtklib.io.ClkReader;
import com.gnss.rtklib.io.Sp3Reader;
import com.gnss.rtklib.model.*;
import com.gnss.rtklib.core.SatelliteUtil;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIf;

import java.io.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static com.gnss.rtklib.core.Constants.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * End-to-end tests for L1+L5 dual-frequency data processing.
 * Tests both RTK static and PPP modes with L5 observations.
 */
class L5EndToEndTest {

    // ---------------------------------------------------------------
    // Data paths
    // ---------------------------------------------------------------

    private static final Path STATIC_DATA = findPath("test/data/static");
    private static final Path PPP_DATA = findPath("test/data/ppp");

    private static Path findPath(String rel) {
        Path p = Path.of("").toAbsolutePath().resolve("../" + rel);
        if (Files.exists(p)) return p;
        p = Path.of("").toAbsolutePath().resolve(rel);
        return p;
    }

    // ---------------------------------------------------------------
    // Availability checks
    // ---------------------------------------------------------------

    static boolean rtkL5DataAvailable() {
        return Files.exists(STATIC_DATA.resolve("rover_L1L5.obs"))
            && Files.exists(STATIC_DATA.resolve("base_L1L5.obs"))
            && Files.exists(STATIC_DATA.resolve("rover_L1L5.nav"));
    }

    static boolean rtkL5RefAvailable() {
        return rtkL5DataAvailable()
            && Files.exists(STATIC_DATA.resolve("ref_L1L5_fwd.pos"));
    }

    static boolean pppL5DataAvailable() {
        return Files.exists(PPP_DATA.resolve("test-rinex-L1L5.obs"))
            && Files.exists(PPP_DATA.resolve("test-rinex-L1L5.nav"))
            && Files.exists(PPP_DATA.resolve("COD0MGXFIN_20260590000_01D_05M_ORB.SP3"))
            && Files.exists(PPP_DATA.resolve("COD0MGXFIN_20260590000_01D_30S_CLK.CLK"));
    }

    static boolean pppL5RefAvailable() {
        return pppL5DataAvailable()
            && Files.exists(PPP_DATA.resolve("ref_L1L5.pos"));
    }

    // ---------------------------------------------------------------
    // RTK Static L1+L5
    // ---------------------------------------------------------------

    // Base station ECEF from static.conf
    private static final double[] BASE_POS_L5 = {-3026183.3550, 4975933.2850, 2598179.5470};
    private static final double DTTOL = 10.0; // time match tolerance (s)

    @Test
    @EnabledIf("rtkL5DataAvailable")
    void testRtkL5Convergence() throws Exception {
        Navigation nav = new Navigation();
        RinexReader.readNav(STATIC_DATA.resolve("rover_L1L5.nav").toString(), nav);
        List<List<ObsData>> roverEpochs = RinexReader.readObs(
            STATIC_DATA.resolve("rover_L1L5.obs").toString(), nav, 1);
        List<List<ObsData>> baseEpochs = RinexReader.readObs(
            STATIC_DATA.resolve("base_L1L5.obs").toString(), nav, 2);

        assertFalse(roverEpochs.isEmpty());
        assertFalse(baseEpochs.isEmpty());

        ProcessingOptions opt = createRtkL5Options();
        RtkState rtk = new RtkState();
        rtk.init(opt);

        int fixCount = 0, floatCount = 0, totalProcessed = 0;
        int maxEpochs = Math.min(roverEpochs.size(), 600);

        for (int ep = 0; ep < maxEpochs; ep++) {
            List<ObsData> roverEpoch = roverEpochs.get(ep);
            if (roverEpoch == null || roverEpoch.isEmpty()) continue;

            List<ObsData> baseEpoch = findMatchingBase(roverEpoch.get(0).time, baseEpochs);
            if (baseEpoch == null) continue;

            ObsData[] obs = mergeObs(roverEpoch, baseEpoch);
            Rtkpos.rtkpos(rtk, obs, obs.length, nav);
            totalProcessed++;

            if (rtk.sol.stat == SOLQ_FIX) fixCount++;
            else if (rtk.sol.stat == SOLQ_FLOAT) floatCount++;
        }

        System.out.printf("[RTK L5] %d epochs: %d fix, %d float%n",
            totalProcessed, fixCount, floatCount);
        assertTrue(totalProcessed > 10);
        assertTrue(fixCount > 0, "L1+L5 RTK should produce fix solutions");
    }

    @Test
    @EnabledIf("rtkL5RefAvailable")
    void testRtkL5VsCReference() throws Exception {
        Navigation nav = new Navigation();
        RinexReader.readNav(STATIC_DATA.resolve("rover_L1L5.nav").toString(), nav);
        List<List<ObsData>> roverEpochs = RinexReader.readObs(
            STATIC_DATA.resolve("rover_L1L5.obs").toString(), nav, 1);
        List<List<ObsData>> baseEpochs = RinexReader.readObs(
            STATIC_DATA.resolve("base_L1L5.obs").toString(), nav, 2);

        // Parse C forward-only reference
        List<RefSolution> refs = parseRefPos(STATIC_DATA.resolve("ref_L1L5_fwd.pos"));
        assertFalse(refs.isEmpty());

        // Run Java RTK forward-only
        ProcessingOptions opt = createRtkL5Options();
        RtkState rtk = new RtkState();
        rtk.init(opt);

        List<RefSolution> javaResults = new ArrayList<>();
        int maxEpochs = roverEpochs.size();

        for (int ep = 0; ep < maxEpochs; ep++) {
            List<ObsData> roverEpoch = roverEpochs.get(ep);
            if (roverEpoch == null || roverEpoch.isEmpty()) continue;

            List<ObsData> baseEpoch = findMatchingBase(roverEpoch.get(0).time, baseEpochs);
            if (baseEpoch == null) continue;

            ObsData[] obs = mergeObs(roverEpoch, baseEpoch);
            Rtkpos.rtkpos(rtk, obs, obs.length, nav);

            if (rtk.sol.stat != SOLQ_NONE) {
                RefSolution r = new RefSolution();
                double[] wt = rtk.sol.time.time2gpst();
                r.week = (int) wt[0];
                r.tow = wt[1];
                r.x = rtk.sol.rr[0];
                r.y = rtk.sol.rr[1];
                r.z = rtk.sol.rr[2];
                r.Q = rtk.sol.stat;
                r.ns = rtk.sol.ns;
                javaResults.add(r);

                // Dump satellite state for target epoch
                if (Math.abs(r.tow - 303090.0) < 0.5) {
                    System.out.printf("[JAVA SAT] epoch TOW=%.0f Q=%d ns=%d%n", r.tow, r.Q, r.ns);
                    for (int s = 0; s < MAXSAT; s++) {
                        RtkState.SatState ss = rtk.ssat[s];
                        if (ss.vsat[0] == 1 || ss.vsat[2] == 1) {
                            String id = SatelliteUtil.satno2id(s + 1);
                            System.out.printf("[JAVA SAT] %-4s el=%5.1f vsat=[%d,%d,%d] fix=[%d,%d,%d] lock=[%d,%d,%d] slip=[%d,%d,%d] resc=[%8.4f,%8.4f,%8.4f] resp=[%8.4f,%8.4f,%8.4f]%n",
                                id, ss.azel[1] * 180 / Math.PI,
                                ss.vsat[0], ss.vsat[1], ss.vsat[2],
                                ss.fix[0], ss.fix[1], ss.fix[2],
                                ss.lock[0], ss.lock[1], ss.lock[2],
                                ss.slip[0], ss.slip[1], ss.slip[2],
                                ss.resc[0], ss.resc[1], ss.resc[2],
                                ss.resp[0], ss.resp[1], ss.resp[2]);
                        }
                    }
                }
            }
        }

        System.out.printf("[RTK L5] C ref: %d epochs, Java fwd: %d epochs%n",
            refs.size(), javaResults.size());

        // Match and compare
        int matched = 0, fixMatched = 0, qMismatch = 0;
        int javaFixCFloat = 0, javaFloatCFix = 0; // Q mismatch breakdown
        double maxDist = 0, maxFixDist = 0;
        double sumDist = 0;

        // Track top 5 worst fix-fix epochs
        int TOP_N = 5;
        double[] worstDists = new double[TOP_N];
        RefSolution[] worstJava = new RefSolution[TOP_N];
        RefSolution[] worstCRef = new RefSolution[TOP_N];

        for (RefSolution ref : refs) {
            RefSolution match = findMatch(ref, javaResults);
            if (match == null) continue;
            matched++;

            double dx = match.x - ref.x;
            double dy = match.y - ref.y;
            double dz = match.z - ref.z;
            double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
            sumDist += dist;
            maxDist = Math.max(maxDist, dist);

            if (match.Q != ref.Q) {
                qMismatch++;
                if (match.Q == SOLQ_FIX && ref.Q != SOLQ_FIX) javaFixCFloat++;
                if (match.Q != SOLQ_FIX && ref.Q == SOLQ_FIX) javaFloatCFix++;
            }
            if (ref.Q == SOLQ_FIX && match.Q == SOLQ_FIX) {
                fixMatched++;
                maxFixDist = Math.max(maxFixDist, dist);

                // Insert into top-N worst if qualifies
                int insertIdx = -1;
                for (int i = TOP_N - 1; i >= 0; i--) {
                    if (dist > worstDists[i]) insertIdx = i;
                    else break;
                }
                if (insertIdx >= 0) {
                    // Shift down
                    for (int i = TOP_N - 1; i > insertIdx; i--) {
                        worstDists[i] = worstDists[i - 1];
                        worstJava[i] = worstJava[i - 1];
                        worstCRef[i] = worstCRef[i - 1];
                    }
                    worstDists[insertIdx] = dist;
                    worstJava[insertIdx] = match;
                    worstCRef[insertIdx] = ref;
                }
            }
        }

        System.out.printf("[RTK L5] Matched: %d/%d, Fix-Fix: %d, Q mismatch: %d%n",
            matched, refs.size(), fixMatched, qMismatch);
        System.out.printf("[RTK L5] Q mismatch breakdown: Java=fix/C=float: %d, Java=float/C=fix: %d%n",
            javaFixCFloat, javaFloatCFix);
        if (matched > 0) {
            System.out.printf("[RTK L5] Mean 3D diff: %.4f mm, Max: %.4f mm%n",
                sumDist / matched * 1000, maxDist * 1000);
        }
        if (fixMatched > 0) {
            System.out.printf("[RTK L5] Max fix 3D diff: %.4f mm%n", maxFixDist * 1000);
            System.out.println("[RTK L5] Top 5 worst fix-fix epochs:");
            System.out.println("[RTK L5]   Rank | Week   TOW        | 3D diff (mm) | Java XYZ                              | C ref XYZ                              | ns(J/C)");
            for (int i = 0; i < TOP_N; i++) {
                if (worstJava[i] == null) break;
                RefSolution j = worstJava[i], c = worstCRef[i];
                System.out.printf("[RTK L5]   #%d   | %4d %10.1f | %12.4f | %14.4f %14.4f %14.4f | %14.4f %14.4f %14.4f | %d/%d%n",
                    i + 1, j.week, j.tow, worstDists[i] * 1000,
                    j.x, j.y, j.z, c.x, c.y, c.z, j.ns, c.ns);
            }
        }

        // Statistics excluding first 50 fix epochs (convergence period)
        double[] fixDists = new double[fixMatched];
        int fi = 0;
        for (RefSolution ref : refs) {
            RefSolution match = findMatch(ref, javaResults);
            if (match == null) continue;
            if (ref.Q == SOLQ_FIX && match.Q == SOLQ_FIX) {
                double dx2 = match.x - ref.x;
                double dy2 = match.y - ref.y;
                double dz2 = match.z - ref.z;
                fixDists[fi++] = Math.sqrt(dx2 * dx2 + dy2 * dy2 + dz2 * dz2);
            }
        }
        if (fi > 50) {
            double maxLate = 0, sumLate = 0;
            for (int i = 50; i < fi; i++) {
                maxLate = Math.max(maxLate, fixDists[i]);
                sumLate += fixDists[i];
            }
            System.out.printf("[RTK L5] Post-convergence (epoch 50+): mean=%.4f mm, max=%.4f mm%n",
                sumLate / (fi - 50) * 1000, maxLate * 1000);
        }

        assertTrue(matched > 50, "Should match significant epochs");
        // RTK fix solutions: tolerance 30mm (multi-constellation + convergence differences)
        if (fixMatched > 0) {
            assertTrue(maxFixDist < 0.030,
                String.format("Fix 3D diff should be < 30mm, got %.4f mm", maxFixDist * 1000));
        }
    }

    // ---------------------------------------------------------------
    // PPP L1+L5
    // ---------------------------------------------------------------

    @Test
    @EnabledIf("pppL5DataAvailable")
    void testPppL5Convergence() throws Exception {
        Navigation nav = new Navigation();
        RinexReader.readNav(PPP_DATA.resolve("test-rinex-L1L5.nav").toString(), nav);
        List<List<ObsData>> epochs = RinexReader.readObs(
            PPP_DATA.resolve("test-rinex-L1L5.obs").toString(), nav, 1);

        Sp3Reader.readSp3(PPP_DATA.resolve("COD0MGXFIN_20260590000_01D_05M_ORB.SP3").toString(), nav);
        ClkReader.readClk(PPP_DATA.resolve("COD0MGXFIN_20260590000_01D_30S_CLK.CLK").toString(), nav);

        assertFalse(epochs.isEmpty());

        ProcessingOptions opt = createPppL5Options();
        PppState rtk = new PppState();
        rtk.init(opt);

        int pppCount = 0, totalProcessed = 0;
        int maxEpochs = Math.min(epochs.size(), 600);
        GTime prevTime = null;

        for (int ep = 0; ep < maxEpochs; ep++) {
            List<ObsData> epoch = epochs.get(ep);
            if (epoch == null || epoch.isEmpty()) continue;

            // SPP first
            ObsData[] obsArr = epoch.toArray(new ObsData[0]);
            Solution sol = new Solution();
            double[] azel = new double[2 * obsArr.length];
            Spp.SatStatus[] ssat = new Spp.SatStatus[MAXSAT];
            for (int j = 0; j < MAXSAT; j++) ssat[j] = new Spp.SatStatus();
            int stat = Spp.pntpos(obsArr, obsArr.length, nav, opt, sol, azel, ssat, new StringBuilder());
            if (stat == 0 || sol.stat == SOLQ_NONE) continue;

            rtk.sol = sol;
            for (int j = 0; j < MAXSAT; j++) rtk.ssat[j].vs = ssat[j].vs;
            if (prevTime != null) {
                rtk.tt = sol.time.timediff(prevTime);
            }
            prevTime = sol.time;

            Pppos.pppos(rtk, obsArr, obsArr.length, nav);
            totalProcessed++;

            if (rtk.sol.stat == SOLQ_PPP) pppCount++;
        }

        System.out.printf("[PPP L5] %d epochs: %d PPP solutions%n",
            totalProcessed, pppCount);
        System.out.printf("[PPP L5] Diag: noObs=%d filterErr=%d iterOverflow=%d ok=%d nsLow=%d%n",
            Pppos.diagNoObs, Pppos.diagFilterErr, Pppos.diagIterOverflow,
            Pppos.diagOk, Pppos.diagNsLow);
        assertTrue(totalProcessed > 10);
        assertTrue(pppCount > 0, "L1+L5 PPP should produce solutions");
    }

    @Test
    @EnabledIf("pppL5RefAvailable")
    void testPppL5VsCReference() throws Exception {
        Navigation nav = new Navigation();
        RinexReader.readNav(PPP_DATA.resolve("test-rinex-L1L5.nav").toString(), nav);
        List<List<ObsData>> epochs = RinexReader.readObs(
            PPP_DATA.resolve("test-rinex-L1L5.obs").toString(), nav, 1);

        Sp3Reader.readSp3(PPP_DATA.resolve("COD0MGXFIN_20260590000_01D_05M_ORB.SP3").toString(), nav);
        ClkReader.readClk(PPP_DATA.resolve("COD0MGXFIN_20260590000_01D_30S_CLK.CLK").toString(), nav);

        // Parse C forward-only reference
        Path refFile = PPP_DATA.resolve("ref_L1L5_fwd.pos");
        if (!Files.exists(refFile)) {
            // Fall back to combined reference if forward-only not available
            refFile = PPP_DATA.resolve("ref_L1L5.pos");
        }
        List<RefSolution> refs = parseRefPos(refFile);
        assertFalse(refs.isEmpty());

        // Run Java PPP forward-only
        ProcessingOptions opt = createPppL5Options();
        PppState rtk = new PppState();
        rtk.init(opt);

        List<RefSolution> javaResults = new ArrayList<>();
        GTime prevTime = null;

        for (List<ObsData> epoch : epochs) {
            if (epoch == null || epoch.isEmpty()) continue;

            ObsData[] obsArr = epoch.toArray(new ObsData[0]);
            Solution sol = new Solution();
            double[] azel = new double[2 * obsArr.length];
            Spp.SatStatus[] ssat = new Spp.SatStatus[MAXSAT];
            for (int j = 0; j < MAXSAT; j++) ssat[j] = new Spp.SatStatus();
            int stat = Spp.pntpos(obsArr, obsArr.length, nav, opt, sol, azel, ssat, new StringBuilder());
            if (stat == 0 || sol.stat == SOLQ_NONE) continue;

            rtk.sol = sol;
            for (int j = 0; j < MAXSAT; j++) rtk.ssat[j].vs = ssat[j].vs;
            if (prevTime != null) {
                rtk.tt = sol.time.timediff(prevTime);
            }
            prevTime = sol.time;

            Pppos.pppos(rtk, obsArr, obsArr.length, nav);

            if (rtk.sol.stat != SOLQ_NONE) {
                RefSolution r = new RefSolution();
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

        System.out.printf("[PPP L5] C ref: %d epochs, Java fwd: %d epochs%n",
            refs.size(), javaResults.size());

        // Match and compare
        int matched = 0, qMismatch = 0;
        double maxDist = 0, sumDist = 0;

        for (RefSolution ref : refs) {
            RefSolution match = findMatch(ref, javaResults);
            if (match == null) continue;
            matched++;

            double dx = match.x - ref.x;
            double dy = match.y - ref.y;
            double dz = match.z - ref.z;
            double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
            sumDist += dist;
            maxDist = Math.max(maxDist, dist);

            if (match.Q != ref.Q) qMismatch++;
        }

        System.out.printf("[PPP L5] Matched: %d/%d, Q mismatch: %d%n",
            matched, refs.size(), qMismatch);
        if (matched > 0) {
            System.out.printf("[PPP L5] Mean 3D diff: %.4f m, Max: %.4f m%n",
                sumDist / matched, maxDist);
        }

        assertTrue(matched > 50, "Should match significant epochs");
        // PPP differences expected due to config differences (SNR mask, tide corr, etc.)
        assertTrue(maxDist < 10.0,
            String.format("PPP 3D diff should be < 10m, got %.4f m", maxDist));
    }

    // ---------------------------------------------------------------
    // PPP pass helper
    // ---------------------------------------------------------------

    private void processPppPass(List<List<ObsData>> epochs, PppState rtk,
                                 Navigation nav, ProcessingOptions opt,
                                 List<Solution> solBuffer) {
        GTime prevTime = null;
        for (List<ObsData> epoch : epochs) {
            if (epoch == null || epoch.isEmpty()) continue;

            ObsData[] obsArr = epoch.toArray(new ObsData[0]);
            Solution sol = new Solution();
            double[] azel = new double[2 * obsArr.length];
            Spp.SatStatus[] ssat = new Spp.SatStatus[MAXSAT];
            for (int j = 0; j < MAXSAT; j++) ssat[j] = new Spp.SatStatus();
            int stat = Spp.pntpos(obsArr, obsArr.length, nav, opt, sol, azel, ssat, new StringBuilder());
            if (stat == 0 || sol.stat == SOLQ_NONE) continue;

            rtk.sol = sol;
            for (int j = 0; j < MAXSAT; j++) rtk.ssat[j].vs = ssat[j].vs;
            if (prevTime != null) {
                rtk.tt = sol.time.timediff(prevTime);
            }
            prevTime = sol.time;

            Pppos.pppos(rtk, obsArr, obsArr.length, nav);

            if (rtk.sol.stat != SOLQ_NONE) {
                solBuffer.add(rtk.sol.copy());
            }
        }
    }

    // ---------------------------------------------------------------
    // RTK pass helper
    // ---------------------------------------------------------------

    private void processRtkPass(List<List<ObsData>> roverEpochs,
                                 List<List<ObsData>> baseEpochs,
                                 RtkState rtk, Navigation nav,
                                 List<Solution> solBuffer) {
        for (List<ObsData> roverEpoch : roverEpochs) {
            if (roverEpoch == null || roverEpoch.isEmpty()) continue;

            List<ObsData> baseEpoch = findMatchingBase(roverEpoch.get(0).time, baseEpochs);
            if (baseEpoch == null) continue;

            ObsData[] obs = mergeObs(roverEpoch, baseEpoch);
            int stat = Rtkpos.rtkpos(rtk, obs, obs.length, nav);
            if (stat != 0 && rtk.sol.stat != SOLQ_NONE) {
                solBuffer.add(rtk.sol.copy());
            }
        }
    }

    // ---------------------------------------------------------------
    // Options
    // ---------------------------------------------------------------

    private ProcessingOptions createRtkL5Options() {
        ProcessingOptions opt = new ProcessingOptions();
        opt.mode = PMODE_STATIC;
        opt.nf = 3;  // L1+L2+L5
        opt.navsys = SYS_GPS | SYS_GLO | SYS_GAL | SYS_QZS | SYS_CMP | SYS_IRN; // 127
        opt.elmin = 15.0 * D2R;
        opt.ionoopt = IONOOPT_BRDC;
        opt.tropopt = TROPOPT_SAAS;
        opt.modear = 3; // fix-and-hold
        opt.dynamics = 0;
        opt.rb[0] = BASE_POS_L5[0];
        opt.rb[1] = BASE_POS_L5[1];
        opt.rb[2] = BASE_POS_L5[2];
        opt.thresar[0] = 3.0;
        opt.eratio[0] = 150.0;
        opt.eratio[1] = 150.0;
        opt.eratio[2] = 150.0;
        opt.err[1] = 0.003;
        opt.err[2] = 0.006;
        opt.std[0] = 30.0;
        opt.std[1] = 0.03;
        opt.std[2] = 0.3;
        opt.prn[0] = 0.0001;
        opt.prn[1] = 0.0015;
        opt.prn[2] = 0.00002;
        opt.minfix = 20;
        opt.minlock = 20;
        opt.niter = 1;
        opt.maxout = 5;
        opt.elmaskar = 15.0 * D2R;
        opt.elmaskhold = 0.0;
        // SNR mask
        opt.snrmask.ena[0] = 1;
        opt.snrmask.ena[1] = 1;
        for (int i = 0; i < 9; i++) {
            opt.snrmask.mask[0][i] = 35.0;
            opt.snrmask.mask[1][i] = 35.0;
            opt.snrmask.mask[2][i] = 35.0;
        }
        return opt;
    }

    private ProcessingOptions createPppL5Options() {
        ProcessingOptions opt = new ProcessingOptions();
        opt.mode = PMODE_PPP_STATIC;
        opt.nf = 3;  // L1+L2+L5
        opt.navsys = SYS_GPS | SYS_GLO | SYS_GAL | SYS_QZS | SYS_CMP; // 61
        opt.elmin = 15.0 * D2R;
        opt.ionoopt = IONOOPT_IFLC;
        opt.tropopt = TROPOPT_ESTG;
        opt.sateph = EPHOPT_PREC;
        opt.dynamics = 1;
        opt.posopt[0] = 0; // no sat antenna (no ANTEX)
        opt.posopt[1] = 0; // no rcv antenna
        opt.posopt[2] = 1; // phase wind-up
        opt.posopt[3] = 1; // eclipse
        opt.posopt[4] = 1; // raim
        opt.posopt[5] = 1; // day boundary
        opt.tidecorr = 5;  // solid + pole
        opt.eratio[0] = 150.0;
        opt.eratio[1] = 150.0;
        opt.eratio[2] = 150.0;
        opt.err[1] = 0.003;
        opt.err[2] = 0.003;
        opt.prn[0] = 0.0001;
        opt.prn[2] = 0.00002;
        opt.prn[5] = 0.0;
        // SNR mask disabled for L1+L5 PPP (SNR[1]=0 for absent L2 would reject sats)
        opt.snrmask.ena[0] = 0;
        opt.niter = 5;         // match C ppp_java_L5.conf
        opt.maxinno[0] = 30.0; // phase
        opt.maxinno[1] = 30.0; // code
        return opt;
    }

    // ---------------------------------------------------------------
    // Parsing and matching helpers
    // ---------------------------------------------------------------

    private List<RefSolution> parseRefPos(Path file) throws Exception {
        return parseRefPos(new BufferedReader(new FileReader(file.toFile())));
    }

    private List<RefSolution> parseRefPos(String text) {
        try {
            return parseRefPos(new BufferedReader(new StringReader(text)));
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    private List<RefSolution> parseRefPos(BufferedReader br) throws Exception {
        List<RefSolution> refs = new ArrayList<>();
        String line;
        while ((line = br.readLine()) != null) {
            if (line.startsWith("%") || line.trim().isEmpty()) continue;
            String[] parts = line.trim().split("\\s+");
            if (parts.length < 7) continue;
            RefSolution r = new RefSolution();
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
        return refs;
    }

    private RefSolution findMatch(RefSolution ref, List<RefSolution> list) {
        for (RefSolution r : list) {
            if (r.week == ref.week && Math.abs(r.tow - ref.tow) < 0.1) return r;
        }
        return null;
    }

    private List<ObsData> findMatchingBase(GTime roverTime, List<List<ObsData>> baseEpochs) {
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

    static class RefSolution {
        int week, Q, ns;
        double tow, x, y, z;
    }
}
