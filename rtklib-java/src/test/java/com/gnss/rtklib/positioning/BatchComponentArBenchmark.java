package com.gnss.rtklib.positioning;

import com.gnss.rtklib.io.RinexReader;
import com.gnss.rtklib.model.*;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import static com.gnss.rtklib.core.Constants.*;

/**
 * Benchmark: Connected Component AR with Block-diagonal Naa inverse.
 * Tests 10min, 30min, 60min, and 24h windows at 30s.
 */
public class BatchComponentArBenchmark {

    private static final double[] BASE_POS = {-3026183.3550, 4975933.2850, 2598179.5470};
    private static final double[] REF_FIX_POS = {-3026470.474, 4976167.052, 2597455.974};

    public static void main(String[] args) throws Exception {
        Path staticData = findStaticData();
        if (staticData == null) {
            System.out.println("Static data not found!");
            return;
        }

        String roverPath = staticData.resolve("rover_L1L5.obs").toString();
        String basePath  = staticData.resolve("base_L1L5.obs").toString();
        String navPath   = staticData.resolve("rover_L1L5.nav").toString();

        Navigation nav = new Navigation();
        RinexReader.readNav(navPath, nav);
        List<List<ObsData>> roverEpochs = RinexReader.readObs(roverPath, nav, 1);
        List<List<ObsData>> baseEpochs = RinexReader.readObs(basePath, nav, 2);

        System.out.printf("Loaded: rover=%d epochs, base=%d epochs%n",
                          roverEpochs.size(), baseEpochs.size());

        // Find overlap start
        int start = findOverlapStart(roverEpochs, baseEpochs);
        System.out.printf("Overlap starts at epoch %d%n", start);

        // Decimate to 30s for all tests
        int decimStep = 30; // take every 30th epoch (1s -> 30s)
        List<List<ObsData>> rovDec = new java.util.ArrayList<>();
        for (int i = start; i < roverEpochs.size(); i += decimStep) {
            rovDec.add(roverEpochs.get(i));
        }
        System.out.printf("Decimated to %d epochs (30s)%n", rovDec.size());

        // Test windows in 30s-decimated epochs:
        // 10min=20ep, 30min=60ep, 60min=120ep, 24h=all
        int[][] windows = {
            {20, 10},     // 10 min @ 30s
            {60, 30},     // 30 min @ 30s
            {120, 60},    // 60 min @ 30s
            {rovDec.size(), -1}  // all available (24h)
        };
        String[] labels = {"10min", "30min", "60min", "24h"};
        start = 0; // already adjusted

        System.out.println();
        System.out.printf("%-8s %-6s %8s %10s %6s %12s%n",
                "Window", "Stat", "Ratio", "3D_err(mm)", "nAmb", "Time(ms)");
        System.out.println("----------------------------------------------------------");

        for (int w = 0; w < windows.length; w++) {
            int nEp = windows[w][0];
            int end = Math.min(start + nEp, rovDec.size());
            List<List<ObsData>> rovSub = rovDec.subList(start, end);

            ProcessingOptions opt = createOptions();

            long t0 = System.currentTimeMillis();
            BatchSolver.BatchResult result = BatchSolver.solve(rovSub, baseEpochs, nav, opt);
            long dt = System.currentTimeMillis() - t0;

            String statStr;
            switch (result.stat) {
                case SOLQ_FIX: statStr = "FIX"; break;
                case SOLQ_FLOAT: statStr = "FLOAT"; break;
                default: statStr = "NONE"; break;
            }

            double dx = result.pos[0] - REF_FIX_POS[0];
            double dy = result.pos[1] - REF_FIX_POS[1];
            double dz = result.pos[2] - REF_FIX_POS[2];
            double err3d = Math.sqrt(dx * dx + dy * dy + dz * dz) * 1000; // mm

            System.out.printf("%-8s %-6s %8.1f %10.1f %6d %12d%n",
                    labels[w], statStr, result.ratio, err3d, result.nAmb, dt);
        }
    }

    private static ProcessingOptions createOptions() {
        ProcessingOptions opt = new ProcessingOptions();
        opt.mode = PMODE_STATIC;
        opt.nf = 3;
        opt.navsys = SYS_GPS | SYS_GLO | SYS_GAL | SYS_QZS | SYS_CMP | SYS_SBS;
        opt.elmin = 15.0 * D2R;
        opt.ionoopt = IONOOPT_BRDC;
        opt.tropopt = TROPOPT_SAAS;
        opt.modear = 3;
        opt.dynamics = 0;
        opt.rb[0] = BASE_POS[0];
        opt.rb[1] = BASE_POS[1];
        opt.rb[2] = BASE_POS[2];
        opt.thresar[0] = 3.0;
        opt.eratio[0] = 150.0;
        opt.eratio[1] = 150.0;
        opt.err[1] = 0.003;
        opt.err[2] = 0.006;
        opt.niter = 1;
        opt.solver = ProcessingOptions.SOLVER_BATCH;
        opt.snrmask.ena[0] = 1;
        opt.snrmask.ena[1] = 1;
        for (int f = 0; f < opt.snrmask.mask.length; f++) {
            java.util.Arrays.fill(opt.snrmask.mask[f], 35.0);
        }
        return opt;
    }

    private static int findOverlapStart(List<List<ObsData>> rover, List<List<ObsData>> base) {
        for (int i = 0; i < rover.size(); i++) {
            List<ObsData> re = rover.get(i);
            if (re == null || re.isEmpty()) continue;
            double rt = re.get(0).time.time + re.get(0).time.sec;
            for (List<ObsData> be : base) {
                if (be == null || be.isEmpty()) continue;
                double bt = be.get(0).time.time + be.get(0).time.sec;
                if (Math.abs(rt - bt) <= 1.0) return i;
            }
        }
        return 0;
    }

    private static Path findStaticData() {
        Path p = Path.of("").toAbsolutePath().resolve("../test/data/static");
        if (Files.exists(p.resolve("rover_L1L5.obs"))) return p;
        p = Path.of("").toAbsolutePath().resolve("test/data/static");
        if (Files.exists(p.resolve("rover_L1L5.obs"))) return p;
        return null;
    }
}
