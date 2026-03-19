package com.gnss.rtklib;

import com.gnss.rtklib.core.Constants;
import com.gnss.rtklib.core.SatelliteUtil;
import com.gnss.rtklib.io.RinexReader;
import com.gnss.rtklib.model.*;
import com.gnss.rtklib.positioning.Spp;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIf;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

class DiagTest {

    private static final Path TEST_DATA = findTestData();

    private static Path findTestData() {
        Path p = Path.of("").toAbsolutePath().resolve("../test/data/rinex");
        if (Files.exists(p)) return p;
        return Path.of("").toAbsolutePath().resolve("test/data/rinex");
    }

    static boolean testDataAvailable() {
        return Files.exists(TEST_DATA.resolve("30400920.05o"))
            && Files.exists(TEST_DATA.resolve("30400920.05n"));
    }

    @Test
    @EnabledIf("testDataAvailable")
    void diagRinexRead() throws Exception {
        String navFile = TEST_DATA.resolve("30400920.05n").toString();
        String obsFile = TEST_DATA.resolve("30400920.05o").toString();

        // Read nav
        Navigation nav = new Navigation();
        RinexReader.readNav(navFile, nav);
        System.out.printf("Nav: %d GPS eph, %d GLO eph%n", nav.eph.size(), nav.geph.size());
        System.out.printf("Ion GPS: [%.4e, %.4e, %.4e, %.4e, %.4e, %.4e, %.4e, %.4e]%n",
            nav.ionGps[0], nav.ionGps[1], nav.ionGps[2], nav.ionGps[3],
            nav.ionGps[4], nav.ionGps[5], nav.ionGps[6], nav.ionGps[7]);

        if (!nav.eph.isEmpty()) {
            Ephemeris e = nav.eph.get(0);
            System.out.printf("First eph: sat=%d(%s) iode=%d A=%.3f e=%.6f toe=%s%n",
                e.sat, SatelliteUtil.satno2id(e.sat), e.iode, e.A, e.e,
                e.toe != null ? e.toe.format(0) : "null");
        }

        // Read obs
        List<List<ObsData>> epochs = RinexReader.readObs(obsFile, nav);
        System.out.printf("Obs: %d epochs%n", epochs.size());

        if (!epochs.isEmpty()) {
            List<ObsData> first = epochs.get(0);
            System.out.printf("First epoch: %d observations%n", first.size());
            for (ObsData obs : first) {
                System.out.printf("  sat=%d(%s) P=[%.3f, %.3f, %.3f] code=[%d,%d,%d] time=%s%n",
                    obs.sat, SatelliteUtil.satno2id(obs.sat),
                    obs.P[0], obs.P[1], obs.P[2],
                    obs.code[0], obs.code[1], obs.code[2],
                    obs.time != null ? obs.time.format(3) : "null");
            }
        }

        // Try SPP on first epoch
        if (!epochs.isEmpty()) {
            List<ObsData> epoch = epochs.get(0);
            ObsData[] obs = epoch.toArray(new ObsData[0]);
            int n = obs.length;

            ProcessingOptions popt = new ProcessingOptions();
            popt.mode = Constants.PMODE_SINGLE;
            popt.ionoopt = Constants.IONOOPT_BRDC;
            popt.tropopt = Constants.TROPOPT_SAAS;
            popt.navsys = Constants.SYS_GPS;

            Solution sol = new Solution();
            double[] azel = new double[Constants.MAXSAT * 2];
            Spp.SatStatus[] ssat = new Spp.SatStatus[Constants.MAXSAT];
            for (int i = 0; i < ssat.length; i++) ssat[i] = new Spp.SatStatus();
            StringBuilder msg = new StringBuilder();

            System.out.println("\n--- Running SPP on first epoch ---");
            int stat = Spp.pntpos(obs, n, nav, popt, sol, azel, ssat, msg);
            System.out.printf("pntpos returned: %d, sol.stat=%d, msg=%s%n", stat, sol.stat, msg);
            if (stat != 0) {
                System.out.printf("Position: [%.3f, %.3f, %.3f]%n", sol.rr[0], sol.rr[1], sol.rr[2]);
                double[] pos = com.gnss.rtklib.core.Coord.ecef2pos(
                    new double[]{sol.rr[0], sol.rr[1], sol.rr[2]});
                System.out.printf("LLH: lat=%.9f lon=%.9f h=%.4f%n",
                    pos[0] * Constants.R2D, pos[1] * Constants.R2D, pos[2]);
            }
        }
    }
}
