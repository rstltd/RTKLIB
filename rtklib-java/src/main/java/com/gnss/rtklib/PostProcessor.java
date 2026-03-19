package com.gnss.rtklib;

import com.gnss.rtklib.core.GTime;
import com.gnss.rtklib.core.MatrixUtil;
import com.gnss.rtklib.io.AntexReader;
import com.gnss.rtklib.io.ClkReader;
import com.gnss.rtklib.io.ConfigReader;
import com.gnss.rtklib.io.RinexReader;
import com.gnss.rtklib.io.SolutionWriter;
import com.gnss.rtklib.io.Sp3Reader;
import com.gnss.rtklib.model.FileOptions;
import com.gnss.rtklib.model.ObsData;
import com.gnss.rtklib.model.Navigation;
import com.gnss.rtklib.model.PppState;
import com.gnss.rtklib.model.ProcessingOptions;
import com.gnss.rtklib.model.RtkState;
import com.gnss.rtklib.model.Solution;
import com.gnss.rtklib.model.SolutionOptions;
import com.gnss.rtklib.positioning.Pppos;
import com.gnss.rtklib.positioning.Rtkpos;
import com.gnss.rtklib.positioning.Spp;

import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static com.gnss.rtklib.core.Constants.*;

/**
 * Main post-processing class and CLI entry point.
 * <p>
 * Reads RINEX observation and navigation files, computes positions using
 * single-point positioning (SPP), and writes results in RTKLIB .pos format.
 * <p>
 * Ported from RTKLIB postpos.c / rnx2rtkp.c.
 */
public class PostProcessor {

    private PostProcessor() {}

    // Priority table for solution status: lower = better
    // Index: SOLQ_NONE=0, FIX=1, FLOAT=2, SBAS=3, DGPS=4, SINGLE=5, PPP=6, DR=7
    private static final int[] PRI = {7, 1, 2, 3, 4, 5, 1, 6};

    /**
     * Process RINEX observation and navigation files.
     *
     * @param obsFile path to RINEX observation file
     * @param navFile path to RINEX navigation file
     * @param outFile path to output .pos file (null for stdout)
     * @param popt    processing options
     * @param sopt    solution output options
     * @return 0 on success, -1 on error
     */
    public static int process(String obsFile, String navFile, String outFile,
                              ProcessingOptions popt, SolutionOptions sopt) {
        // Read navigation data
        Navigation nav = new Navigation();

        try {
            RinexReader.readNav(navFile, nav);
        } catch (Exception e) {
            System.err.printf("error: failed to read nav file: %s (%s)%n",
                              navFile, e.getMessage());
            return -1;
        }

        // Read observation data (epochs)
        List<List<ObsData>> epochs;
        try {
            epochs = RinexReader.readObs(obsFile, nav);
        } catch (Exception e) {
            System.err.printf("error: failed to read obs file: %s (%s)%n",
                              obsFile, e.getMessage());
            return -1;
        }

        if (epochs == null || epochs.isEmpty()) {
            System.err.println("error: no observation data");
            return -1;
        }

        // Open output
        PrintWriter out;
        try {
            if (outFile != null && !outFile.isEmpty()) {
                out = new PrintWriter(new FileOutputStream(outFile), true);
            } else {
                out = new PrintWriter(System.out, true);
            }
        } catch (Exception e) {
            System.err.printf("error: failed to open output file: %s (%s)%n",
                              outFile, e.getMessage());
            return -1;
        }

        // Write header
        if (sopt.outhead != 0) {
            SolutionWriter.writeHeader(out, popt, sopt);
        }

        // Process each epoch
        int totalEpochs = epochs.size();
        int solvedEpochs = 0;
        int failedEpochs = 0;

        double[] azel = new double[MAXSAT * 2];
        Spp.SatStatus[] ssat = new Spp.SatStatus[MAXSAT];
        for (int i = 0; i < ssat.length; i++) ssat[i] = new Spp.SatStatus();
        StringBuilder msg = new StringBuilder();

        for (List<ObsData> epoch : epochs) {
            if (epoch == null || epoch.isEmpty()) {
                failedEpochs++;
                continue;
            }

            ObsData[] obs = epoch.toArray(new ObsData[0]);
            int n = obs.length;
            Solution sol = new Solution();

            // Reset work arrays
            java.util.Arrays.fill(azel, 0.0);
            msg.setLength(0);

            // Single point positioning
            int stat = Spp.pntpos(obs, n, nav, popt, sol, azel, ssat, msg);

            if (stat != 0 && sol.stat != SOLQ_NONE) {
                SolutionWriter.writeSolution(out, sol, popt.rb, sopt);
                solvedEpochs++;
            } else {
                failedEpochs++;
            }
        }

        // Close output file (but not stdout)
        if (outFile != null && !outFile.isEmpty()) {
            out.close();
        } else {
            out.flush();
        }

        // Print summary
        System.err.printf("total epochs: %d, solved: %d, failed: %d%n",
                          totalEpochs, solvedEpochs, failedEpochs);

        return 0;
    }

    // ---------------------------------------------------------------
    // Epoch processor interface for unified pipeline
    // ---------------------------------------------------------------

    /**
     * Functional interface for processing a sequence of observation epochs.
     * Implementations handle mode-specific logic (base matching for RTK,
     * SPP pre-processing for PPP) while the pipeline handles
     * forward/backward/combined orchestration.
     */
    @FunctionalInterface
    interface EpochProcessor {
        /**
         * Process epochs and output/buffer solutions.
         *
         * @param epochs    observation epochs to process
         * @param out       writer for direct output (may be null)
         * @param solBuffer buffer for storing solutions (may be null)
         * @return {solvedEpochs, failedEpochs}
         */
        int[] processEpochs(List<List<ObsData>> epochs, PrintWriter out, List<Solution> solBuffer);
    }

    /**
     * Unified forward/backward/combined processing pipeline.
     * Handles solution direction and combining logic shared by RTK and PPP.
     *
     * @param epochs     observation epochs (forward order)
     * @param popt       processing options (soltype determines pipeline mode)
     * @param sopt       solution output options
     * @param out        output writer
     * @param processor  mode-specific epoch processor
     * @param resetState called before backward pass in combined mode to reset filter state
     */
    private static void runPipeline(List<List<ObsData>> epochs,
                                     ProcessingOptions popt, SolutionOptions sopt,
                                     PrintWriter out,
                                     EpochProcessor processor,
                                     Runnable resetState) {
        int soltype = popt.soltype;

        if (soltype == SOLTYPE_FORWARD || soltype == SOLTYPE_BACKWARD) {
            // Single-pass: forward or backward
            List<List<ObsData>> passEpochs = epochs;
            if (soltype == SOLTYPE_BACKWARD) {
                passEpochs = new ArrayList<>(epochs);
                Collections.reverse(passEpochs);
            }
            int[] counts = processor.processEpochs(passEpochs, out, null);
            System.err.printf("total epochs: %d, solved: %d, failed: %d%n",
                              epochs.size(), counts[0], counts[1]);
        } else {
            // Combined: forward + backward + smoother
            List<Solution> solf = new ArrayList<>();
            processor.processEpochs(epochs, null, solf);

            // Reset state for backward pass (combined mode)
            if (soltype == SOLTYPE_COMBINED) {
                resetState.run();
            }

            // Backward pass
            List<List<ObsData>> bwdEpochs = new ArrayList<>(epochs);
            Collections.reverse(bwdEpochs);
            List<Solution> solb = new ArrayList<>();
            processor.processEpochs(bwdEpochs, null, solb);

            // Combine
            combres(solf, solb, popt, sopt, out);
            System.err.printf("combined: fwd=%d, bwd=%d solutions%n", solf.size(), solb.size());
        }
    }

    /**
     * Process RTK (relative positioning) using rover and base RINEX files.
     *
     * @param roverObsFile path to rover RINEX observation file
     * @param baseObsFile  path to base station RINEX observation file
     * @param navFile      path to RINEX navigation file
     * @param outFile      path to output .pos file (null for stdout)
     * @param popt         processing options
     * @param sopt         solution output options
     * @return 0 on success, -1 on error
     */
    public static int processRtk(String roverObsFile, String baseObsFile,
                                   String navFile, String outFile,
                                   ProcessingOptions popt, SolutionOptions sopt) {
        // Read navigation data
        Navigation nav = new Navigation();
        try {
            RinexReader.readNav(navFile, nav);
        } catch (Exception e) {
            System.err.printf("error: failed to read nav file: %s (%s)%n",
                              navFile, e.getMessage());
            return -1;
        }

        // Read rover observations (rcv=1)
        List<List<ObsData>> roverEpochs;
        try {
            roverEpochs = RinexReader.readObs(roverObsFile, nav, 1);
        } catch (Exception e) {
            System.err.printf("error: failed to read rover obs: %s (%s)%n",
                              roverObsFile, e.getMessage());
            return -1;
        }

        // Read base observations (rcv=2)
        List<List<ObsData>> baseEpochs;
        try {
            baseEpochs = RinexReader.readObs(baseObsFile, nav, 2);
        } catch (Exception e) {
            System.err.printf("error: failed to read base obs: %s (%s)%n",
                              baseObsFile, e.getMessage());
            return -1;
        }

        if (roverEpochs == null || roverEpochs.isEmpty()) {
            System.err.println("error: no rover observation data");
            return -1;
        }
        if (baseEpochs == null || baseEpochs.isEmpty()) {
            System.err.println("error: no base observation data");
            return -1;
        }

        // Open output
        PrintWriter out;
        try {
            if (outFile != null && !outFile.isEmpty()) {
                out = new PrintWriter(new FileOutputStream(outFile), true);
            } else {
                out = new PrintWriter(System.out, true);
            }
        } catch (Exception e) {
            System.err.printf("error: failed to open output file: %s (%s)%n",
                              outFile, e.getMessage());
            return -1;
        }

        // Write header
        if (sopt.outhead != 0) {
            SolutionWriter.writeHeader(out, popt, sopt);
        }



        // Create RTK state and epoch processor
        RtkState[] rtkHolder = { new RtkState() };
        rtkHolder[0].init(popt);

        EpochProcessor rtkProcessor = (epochs, writer, solBuf) ->
            processEpochsRtk(epochs, baseEpochs, rtkHolder[0], nav, popt, sopt, writer, solBuf);

        Runnable resetRtk = () -> {
            rtkHolder[0] = new RtkState();
            rtkHolder[0].init(popt);
        };

        runPipeline(roverEpochs, popt, sopt, out, rtkProcessor, resetRtk);

        if (outFile != null && !outFile.isEmpty()) {
            out.close();
        } else {
            out.flush();
        }

        return 0;
    }

    /**
     * Process RTK epochs. Outputs to writer or buffers into solBuffer.
     *
     * @return {solvedEpochs, failedEpochs}
     */
    private static int[] processEpochsRtk(List<List<ObsData>> roverEpochs,
                                            List<List<ObsData>> baseEpochs,
                                            RtkState rtk, Navigation nav,
                                            ProcessingOptions popt, SolutionOptions sopt,
                                            PrintWriter out, List<Solution> solBuffer) {
        int solvedEpochs = 0, failedEpochs = 0;
        int baseIdx = 0;

        for (List<ObsData> roverEpoch : roverEpochs) {
            if (roverEpoch == null || roverEpoch.isEmpty()) {
                failedEpochs++;
                continue;
            }

            // Find matching base epoch (within maxtdiff)
            double roverTime = roverEpoch.get(0).time.time + roverEpoch.get(0).time.sec;
            List<ObsData> bestBase = null;
            double bestDt = Double.MAX_VALUE;

            for (int j = Math.max(0, baseIdx - 1); j < baseEpochs.size(); j++) {
                List<ObsData> be = baseEpochs.get(j);
                if (be == null || be.isEmpty()) continue;
                double baseTime = be.get(0).time.time + be.get(0).time.sec;
                double dt = Math.abs(roverTime - baseTime);
                if (dt < bestDt) {
                    bestDt = dt;
                    bestBase = be;
                    baseIdx = j;
                }
                if (baseTime > roverTime + popt.maxtdiff) break;
            }

            if (bestBase == null || bestDt > popt.maxtdiff) {
                failedEpochs++;
                continue;
            }

            // Merge rover and base observations
            List<ObsData> merged = new ArrayList<>(roverEpoch.size() + bestBase.size());
            merged.addAll(roverEpoch);
            merged.addAll(bestBase);
            ObsData[] obs = merged.toArray(new ObsData[0]);

            // RTK processing
            int stat = Rtkpos.rtkpos(rtk, obs, obs.length, nav);

            if (stat != 0 && rtk.sol.stat != SOLQ_NONE) {
                if (solBuffer != null) {
                    solBuffer.add(rtk.sol.copy());
                }
                if (out != null) {
                    SolutionWriter.writeSolution(out, rtk.sol, popt.rb, sopt);
                }
                solvedEpochs++;
            } else {
                failedEpochs++;
            }
        }
        return new int[]{solvedEpochs, failedEpochs};
    }

    /**
     * Process PPP (Precise Point Positioning) using precise ephemeris/clock files.
     *
     * @param obsFile  path to RINEX observation file
     * @param navFile  path to RINEX navigation file (for SPP initial position)
     * @param sp3File  path to SP3 precise ephemeris file
     * @param clkFile  path to RINEX CLK file (may be null to use SP3 clocks)
     * @param atxFile  path to ANTEX antenna file (may be null)
     * @param outFile  path to output .pos file
     * @param popt     processing options
     * @param sopt     solution output options
     * @return 0 on success, -1 on error
     */
    public static int processPpp(String obsFile, String navFile,
                                   String sp3File, String clkFile,
                                   String atxFile, String outFile,
                                   ProcessingOptions popt,
                                   SolutionOptions sopt) {
        Navigation nav = new Navigation();
        try {
            RinexReader.readNav(navFile, nav);
        } catch (Exception e) {
            System.err.printf("error: failed to read nav file: %s (%s)%n",
                              navFile, e.getMessage());
            return -1;
        }

        // Read SP3
        try {
            Sp3Reader.readSp3(sp3File, nav);
        } catch (Exception e) {
            System.err.printf("error: failed to read sp3 file: %s (%s)%n",
                              sp3File, e.getMessage());
            return -1;
        }
        System.err.printf("sp3: %d epochs loaded%n", nav.peph.size());

        // Read CLK (optional)
        if (clkFile != null && !clkFile.isEmpty()) {
            try {
                ClkReader.readClk(clkFile, nav);
            } catch (Exception e) {
                System.err.printf("error: failed to read clk file: %s (%s)%n",
                                  clkFile, e.getMessage());
                return -1;
            }
            System.err.printf("clk: %d epochs loaded%n", nav.pclk.size());
        }

        // Read ANTEX (optional)
        if (atxFile != null && !atxFile.isEmpty()) {
            try {
                nav.pcvs = AntexReader.readAntex(atxFile);
            } catch (Exception e) {
                System.err.printf("warning: failed to read antex file: %s (%s)%n",
                                  atxFile, e.getMessage());
            }
            System.err.printf("antex: %d antenna models loaded%n", nav.pcvs.size());
        }

        // Read observations
        List<List<ObsData>> epochs;
        try {
            epochs = RinexReader.readObs(obsFile, nav);
        } catch (Exception e) {
            System.err.printf("error: failed to read obs file: %s (%s)%n",
                              obsFile, e.getMessage());
            return -1;
        }
        if (epochs == null || epochs.isEmpty()) {
            System.err.println("error: no observation data");
            return -1;
        }

        // Open output
        PrintWriter out;
        try {
            if (outFile != null && !outFile.isEmpty()) {
                out = new PrintWriter(new FileOutputStream(outFile), true);
            } else {
                out = new PrintWriter(System.out, true);
            }
        } catch (Exception e) {
            System.err.printf("error: failed to open output file: %s (%s)%n",
                              outFile, e.getMessage());
            return -1;
        }

        if (sopt.outhead != 0) {
            SolutionWriter.writeHeader(out, popt, sopt);
        }

        // Initialize PPP options
        popt.sateph = EPHOPT_PREC;
        popt.ionoopt = IONOOPT_IFLC;
        if (popt.tropopt < TROPOPT_EST) popt.tropopt = TROPOPT_EST;

        // Create PPP state and epoch processor
        PppState[] rtkHolder = { new PppState() };
        rtkHolder[0].init(popt);

        EpochProcessor pppProcessor = (epochList, writer, solBuf) ->
            processEpochsPpp(epochList, rtkHolder[0], nav, popt, sopt, writer, solBuf);

        Runnable resetPpp = () -> {
            rtkHolder[0] = new PppState();
            rtkHolder[0].init(popt);
        };

        runPipeline(epochs, popt, sopt, out, pppProcessor, resetPpp);

        if (outFile != null && !outFile.isEmpty()) out.close();
        else out.flush();

        return 0;
    }

    /**
     * Process PPP epochs. Outputs to writer or buffers into solBuffer.
     *
     * @return {solvedEpochs, failedEpochs}
     */
    private static int[] processEpochsPpp(List<List<ObsData>> epochs,
                                           PppState rtk, Navigation nav,
                                           ProcessingOptions popt, SolutionOptions sopt,
                                           PrintWriter out, List<Solution> solBuffer) {
        int solvedEpochs = 0, failedEpochs = 0;
        double[] azel = new double[MAXSAT * 2];
        Spp.SatStatus[] ssat = new Spp.SatStatus[MAXSAT];
        for (int i = 0; i < ssat.length; i++) ssat[i] = new Spp.SatStatus();
        StringBuilder msg = new StringBuilder();

        for (List<ObsData> epoch : epochs) {
            if (epoch == null || epoch.isEmpty()) { failedEpochs++; continue; }

            ObsData[] obs = epoch.toArray(new ObsData[0]);
            int n = obs.length;

            // SPP for initial position and clock
            java.util.Arrays.fill(azel, 0.0);
            msg.setLength(0);
            Solution sppSol = new Solution();
            int sppStat = Spp.pntpos(obs, n, nav, popt, sppSol, azel, ssat, msg);

            boolean sppOk = sppStat != 0 && sppSol.stat != SOLQ_NONE;

            // Skip only if PPP not yet initialized and SPP failed
            if (!sppOk && MatrixUtil.norm(rtk.x, 3) <= 0.0) {
                failedEpochs++;
                continue;
            }

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

            // PPP
            Pppos.pppos(rtk, obs, n, nav);

            if (rtk.sol.stat != SOLQ_NONE) {
                if (solBuffer != null) {
                    solBuffer.add(rtk.sol.copy());
                }
                if (out != null) {
                    SolutionWriter.writeSolution(out, rtk.sol, popt.rb, sopt);
                }
                solvedEpochs++;
            } else {
                failedEpochs++;
            }
        }
        return new int[]{solvedEpochs, failedEpochs};
    }

    // ---------------------------------------------------------------
    // Combined solution: smoother fusion
    // ---------------------------------------------------------------

    /**
     * Validate combined forward/backward solutions (4-sigma test).
     * Only applies to kinematic FIX solutions.
     * Ported from C postpos.c:valcomb().
     *
     * @return true if valid, false if should degrade to FLOAT
     */
    public static boolean valcomb(Solution sf, Solution sb, ProcessingOptions popt) {
        if (popt.mode != PMODE_KINEMA && popt.mode != PMODE_MOVEB) return true;
        if (sf.stat != SOLQ_FIX) return true;

        for (int k = 0; k < 3; k++) {
            double dr = sf.rr[k] - sb.rr[k];
            double var = (double) sf.qr[k] + (double) sb.qr[k];
            if (dr * dr > 16.0 * var) return false;
        }
        return true;
    }

    /**
     * Combine forward/backward solutions using fixed-interval smoother.
     * Ported from C postpos.c:combres().
     */
    public static void combres(List<Solution> solf, List<Solution> solb,
                         ProcessingOptions popt, SolutionOptions sopt,
                         PrintWriter out) {
        boolean solstatic = sopt.solstatic != 0 &&
            (popt.mode == PMODE_STATIC || popt.mode == PMODE_STATIC_START ||
             popt.mode == PMODE_PPP_STATIC);

        GTime bestTime = new GTime(0, 0.0);
        Solution bestSol = null;
        double[] bestRb = new double[3];

        int i = 0, j = solb.size() - 1;

        while (i < solf.size() && j >= 0) {
            Solution sf = solf.get(i);
            Solution sb = solb.get(j);
            double tt = sf.time.timediff(sb.time);

            Solution sols;
            if (tt < -DTTOL) {
                // Forward solution is earlier
                sols = sf.copy();
                i++;
            } else if (tt > DTTOL) {
                // Backward solution is earlier
                sols = sb.copy();
                j--;
            } else {
                // Time match: compare quality or smooth
                if (PRI[sf.stat] < PRI[sb.stat]) {
                    sols = sf.copy();
                } else if (PRI[sf.stat] > PRI[sb.stat]) {
                    sols = sb.copy();
                } else {
                    // Same quality: apply smoother
                    sols = sf.copy();
                    sols.time = sols.time.timeadd(-tt / 2.0);

                    // Degrade fix to float if validation failed
                    if ((popt.mode == PMODE_KINEMA || popt.mode == PMODE_MOVEB)
                            && sols.stat == SOLQ_FIX) {
                        if (!valcomb(sf, sb, popt)) sols.stat = SOLQ_FLOAT;
                    }

                    // Expand qr[0..5] to 3x3 symmetric covariance matrices
                    double[] Qf = new double[9], Qb = new double[9];
                    double[] Qs = new double[9], xs = new double[3];
                    Qf[0] = sf.qr[0]; Qf[4] = sf.qr[1]; Qf[8] = sf.qr[2];
                    Qf[1] = Qf[3] = sf.qr[3]; Qf[5] = Qf[7] = sf.qr[4]; Qf[2] = Qf[6] = sf.qr[5];
                    Qb[0] = sb.qr[0]; Qb[4] = sb.qr[1]; Qb[8] = sb.qr[2];
                    Qb[1] = Qb[3] = sb.qr[3]; Qb[5] = Qb[7] = sb.qr[4]; Qb[2] = Qb[6] = sb.qr[5];

                    if (MatrixUtil.smoother(sf.rr, Qf, sb.rr, Qb, 3, xs, Qs) != 0) {
                        i++; j--;
                        continue;
                    }
                    System.arraycopy(xs, 0, sols.rr, 0, 3);
                    sols.qr[0] = (float) Qs[0];
                    sols.qr[1] = (float) Qs[4];
                    sols.qr[2] = (float) Qs[8];
                    sols.qr[3] = (float) Qs[1];
                    sols.qr[4] = (float) Qs[5];
                    sols.qr[5] = (float) Qs[2];
                }
                i++; j--;
            }

            if (!solstatic) {
                SolutionWriter.writeSolution(out, sols, popt.rb, sopt);
            } else if (bestTime.time == 0 || PRI[sols.stat] <= PRI[bestSol.stat]) {
                bestSol = sols;
                if (bestTime.time == 0 || sols.time.timediff(bestTime) < 0.0) {
                    bestTime = sols.time;
                }
            }
        }

        // Output remaining forward solutions
        while (i < solf.size()) {
            Solution sols = solf.get(i++);
            if (!solstatic) {
                SolutionWriter.writeSolution(out, sols, popt.rb, sopt);
            } else if (bestTime.time == 0 || PRI[sols.stat] <= PRI[bestSol.stat]) {
                bestSol = sols;
                if (bestTime.time == 0 || sols.time.timediff(bestTime) < 0.0) bestTime = sols.time;
            }
        }

        // Output remaining backward solutions
        while (j >= 0) {
            Solution sols = solb.get(j--);
            if (!solstatic) {
                SolutionWriter.writeSolution(out, sols, popt.rb, sopt);
            } else if (bestTime.time == 0 || PRI[sols.stat] <= PRI[bestSol.stat]) {
                bestSol = sols;
                if (bestTime.time == 0 || sols.time.timediff(bestTime) < 0.0) bestTime = sols.time;
            }
        }

        // Static mode: output single best solution
        if (solstatic && bestSol != null && bestTime.time != 0) {
            bestSol.time = bestTime;
            SolutionWriter.writeSolution(out, bestSol, popt.rb, sopt);
        }
    }

    /**
     * CLI entry point for post-processing RINEX files.
     * <p>
     * Usage: java -jar rtklib.jar [options] obsfile navfile [baseobsfile]
     * <pre>
     * Options:
     *   -p mode    positioning mode (0:single, 3:static RTK, default 0)
     *   -sol type  solution type (0:fwd, 1:bwd, 2:combined, 3:combined-noreset)
     *   -o file    output file (default: stdout or obsfile.pos)
     *   -f nfreq   number of frequencies (1-3, default 2)
     *   -sys sys   navigation systems (G:GPS,R:GLO,E:GAL,C:BDS,J:QZS,I:IRN, default GRE)
     *   -el deg    elevation mask (degrees, default 15)
     *   -ion opt   ionosphere option (0:off,1:brdc,3:iflc, default 1)
     *   -trop opt  troposphere option (0:off,1:saas, default 1)
     *   -t level   trace level (0-4, default 0)
     *   -rb x,y,z  base station position ECEF (m)
     * </pre>
     * Example: java -jar rtklib.jar -p 3 -o output.pos rover.obs nav.nav base.obs
     */
    public static void main(String[] args) {
        ProcessingOptions popt = new ProcessingOptions();
        SolutionOptions sopt = new SolutionOptions();
        FileOptions fopt = new FileOptions();
        String outFile = null;
        String obsFile = null;
        String navFile = null;
        String baseObsFile = null;
        String sp3File = null;
        String clkFile = null;
        String atxFile = null;



        // Pass 1: find -k and load config file first
        for (int i = 0; i < args.length - 1; i++) {
            if (args[i].equals("-k")) {
                try {
                    fopt = ConfigReader.load(args[i + 1], popt, sopt);
                } catch (Exception e) {
                    System.err.printf("error: failed to load config: %s (%s)%n",
                                      args[i + 1], e.getMessage());
                    System.exit(1);
                }
                break;
            }
        }

        // Pass 2: CLI arguments override config
        int i = 0;
        while (i < args.length) {
            String arg = args[i];

            if (arg.equals("-k") && i + 1 < args.length) {
                i++; // skip -k and its argument (already processed)
            } else if (arg.equals("-p") && i + 1 < args.length) {
                popt.mode = Integer.parseInt(args[++i]);
            } else if (arg.equals("-sol") && i + 1 < args.length) {
                popt.soltype = Integer.parseInt(args[++i]);
            } else if (arg.equals("-o") && i + 1 < args.length) {
                outFile = args[++i];
            } else if (arg.equals("-f") && i + 1 < args.length) {
                popt.nf = Integer.parseInt(args[++i]);
                if (popt.nf < 1) popt.nf = 1;
                if (popt.nf > 3) popt.nf = 3;
            } else if (arg.equals("-sys") && i + 1 < args.length) {
                popt.navsys = parseNavSys(args[++i]);
            } else if (arg.equals("-el") && i + 1 < args.length) {
                popt.elmin = Double.parseDouble(args[++i]) * D2R;
            } else if (arg.equals("-ion") && i + 1 < args.length) {
                popt.ionoopt = Integer.parseInt(args[++i]);
            } else if (arg.equals("-trop") && i + 1 < args.length) {
                popt.tropopt = Integer.parseInt(args[++i]);
            } else if (arg.equals("-t") && i + 1 < args.length) {
                sopt.trace = Integer.parseInt(args[++i]);
            } else if (arg.equals("-sp3") && i + 1 < args.length) {
                sp3File = args[++i];
            } else if (arg.equals("-clk") && i + 1 < args.length) {
                clkFile = args[++i];
            } else if (arg.equals("-atx") && i + 1 < args.length) {
                atxFile = args[++i];
            } else if (arg.equals("-rb") && i + 1 < args.length) {
                String[] xyz = args[++i].split(",");
                if (xyz.length >= 3) {
                    popt.rb[0] = Double.parseDouble(xyz[0]);
                    popt.rb[1] = Double.parseDouble(xyz[1]);
                    popt.rb[2] = Double.parseDouble(xyz[2]);
                }
            } else if (!arg.startsWith("-")) {
                // Positional arguments: obsfile navfile [baseobsfile]
                if (obsFile == null) {
                    obsFile = arg;
                } else if (navFile == null) {
                    navFile = arg;
                } else if (baseObsFile == null) {
                    baseObsFile = arg;
                }
            } else {
                System.err.printf("unknown option: %s%n", arg);
                printUsage();
                System.exit(1);
            }
            i++;
        }

        // Use file options if CLI didn't specify
        if (sp3File == null && !fopt.satantp.isEmpty()) {
            // satantp in fopt is for antenna file, not sp3 — handled separately
        }
        if (atxFile == null && !fopt.satantp.isEmpty()) {
            atxFile = fopt.satantp;
        }

        // Validate required arguments
        if (obsFile == null || navFile == null) {
            printUsage();
            System.exit(1);
        }

        // Default output file: obsfile with .pos extension
        if (outFile == null) {
            int dot = obsFile.lastIndexOf('.');
            outFile = (dot > 0 ? obsFile.substring(0, dot) : obsFile) + ".pos";
        }

        sopt.prog = "rtklib-java";

        int ret;
        if (popt.mode >= PMODE_PPP_KINEMA && popt.mode <= PMODE_PPP_FIXED && sp3File != null) {
            ret = processPpp(obsFile, navFile, sp3File, clkFile, atxFile, outFile, popt, sopt);
        } else if (popt.mode >= PMODE_KINEMA && popt.mode <= PMODE_STATIC_START && baseObsFile != null) {
            ret = processRtk(obsFile, baseObsFile, navFile, outFile, popt, sopt);
        } else {
            ret = process(obsFile, navFile, outFile, popt, sopt);
        }
        System.exit(ret);
    }

    /**
     * Parse navigation system string (e.g., "GRE" -> SYS_GPS|SYS_GLO|SYS_GAL).
     */
    private static int parseNavSys(String sysStr) {
        int navsys = SYS_NONE;
        for (char c : sysStr.toUpperCase().toCharArray()) {
            switch (c) {
                case 'G': navsys |= SYS_GPS; break;
                case 'R': navsys |= SYS_GLO; break;
                case 'E': navsys |= SYS_GAL; break;
                case 'C': navsys |= SYS_CMP; break;
                case 'J': navsys |= SYS_QZS; break;
                case 'I': navsys |= SYS_IRN; break;
                default:
                    System.err.printf("warning: unknown system '%c' ignored%n", c);
                    break;
            }
        }
        return navsys;
    }

    private static void printUsage() {
        System.err.println("Usage: java -jar rtklib.jar [options] obsfile navfile [baseobsfile]");
        System.err.println("Options:");
        System.err.println("  -k config  load config file (.conf)");
        System.err.println("  -p mode    positioning mode (0:single, default 0)");
        System.err.println("  -sol type  solution type (0:fwd, 1:bwd, 2:combined, 3:combined-noreset)");
        System.err.println("  -o file    output file (default: obsfile.pos)");
        System.err.println("  -f nfreq   number of frequencies (1-3, default 2)");
        System.err.println("  -sys sys   navigation systems (G:GPS,R:GLO,E:GAL,C:BDS,J:QZS,I:IRN)");
        System.err.println("  -el deg    elevation mask (degrees, default 15)");
        System.err.println("  -ion opt   ionosphere option (0:off,1:brdc,3:iflc, default 1)");
        System.err.println("  -trop opt  troposphere option (0:off,1:saas, default 1)");
        System.err.println("  -t level   trace level (0-4, default 0)");
        System.err.println("  -sp3 file  SP3 precise ephemeris file");
        System.err.println("  -clk file  RINEX clock file");
        System.err.println("  -atx file  ANTEX antenna file");
        System.err.println("  -rb x,y,z  base station position ECEF (m)");

        System.err.println();
        System.err.println("Example: java -jar rtklib.jar -k config.conf -o output.pos obs.24o nav.24n");
    }
}
