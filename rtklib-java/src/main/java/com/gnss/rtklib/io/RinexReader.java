/*------------------------------------------------------------------------------
 * RinexReader.java : RINEX file reader ported from rinex.c
 *
 *          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
 *          Java port Copyright (C) 2026
 *
 * Licensed under BSD 2-clause license.
 *
 * Supports:
 *   - RINEX 2.x and 3.x observation files (.obs, .o, .*o)
 *   - RINEX 2.x and 3.x navigation files (.nav, .n, .*n, .p, .g, .l, etc.)
 *
 * reference:
 *     [1] RINEX Version 2.11, December 10, 2007
 *     [2] RINEX Version 3.00, November 28, 2007
 *     [3] IS-GPS-200D, 7 March, 2006
 *     [4] RINEX Version 3.04, November 23, 2018
 *-----------------------------------------------------------------------------*/
package com.gnss.rtklib.io;

import com.gnss.rtklib.core.Constants;
import com.gnss.rtklib.core.GTime;
import com.gnss.rtklib.core.SatelliteUtil;
import com.gnss.rtklib.model.Ephemeris;
import com.gnss.rtklib.model.GloEphemeris;
import com.gnss.rtklib.model.Navigation;
import com.gnss.rtklib.model.ObsData;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static com.gnss.rtklib.core.Constants.*;

/**
 * RINEX file reader supporting RINEX 2.x and 3.x observation and navigation files.
 * Ported from RTKLIB rinex.c.
 */
public class RinexReader {

    // Time system identifiers
    private static final int TSYS_GPS = 0;
    private static final int TSYS_UTC = 1;
    private static final int TSYS_GLO = 2;
    private static final int TSYS_GAL = 3;
    private static final int TSYS_QZS = 4;
    private static final int TSYS_CMP = 5;
    private static final int TSYS_IRN = 6;

    private static final int MAXOBSTYPE = 64;
    private static final int MAXOBS = 96;
    private static final int NOBS = NFREQ + NEXOBS;

    // Number of RINEX satellite systems
    private static final int RNX_NUMSYS = 7;
    private static final int RNX_SYS_GPS = 0;
    private static final int RNX_SYS_GLO = 1;
    private static final int RNX_SYS_GAL = 2;
    private static final int RNX_SYS_QZS = 3;
    private static final int RNX_SYS_SBS = 4;
    private static final int RNX_SYS_CMP = 5;
    private static final int RNX_SYS_IRN = 6;

    private static final String SYSCODES = "GREJSCI";

    /** URA values (ref [3] 20.3.3.3.1.1) */
    private static final double[] URA_EPH = {
        2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0, 96.0, 192.0,
        384.0, 768.0, 1536.0, 3072.0, 6144.0, 0.0
    };

    /** URA nominal values */
    private static final double[] URA_NOMINAL = {
        2.0, 2.8, 4.0, 5.7, 8.0, 11.3, 16.0, 32.0, 64.0, 128.0,
        256.0, 512.0, 1024.0, 2048.0, 4096.0, 8192.0
    };

    /** Observation code strings (index = CODE_xxx) */
    private static final String[] OBSCODES = {
        "",  "1C","1P","1W","1Y", "1M","1N","1S","1L","1E", //  0- 9
        "1A","1B","1X","1Z","2C", "2D","2S","2L","2X","2P", // 10-19
        "2W","2Y","2M","2N","5I", "5Q","5X","7I","7Q","7X", // 20-29
        "6A","6B","6C","6X","6Z", "6S","6L","8I","8Q","8X", // 30-39
        "2I","2Q","6I","6Q","3I", "3Q","3X","1I","1Q","5A", // 40-49
        "5B","5C","9A","9B","9C", "9X","1D","5D","5P","5Z", // 50-59
        "6E","7D","7P","7Z","8D", "8P","4A","4B","4X","6D", // 60-69
        "6P"
    };

    /** Code priority for each freq-index [sys][freq] */
    private static final String[][] CODEPRIS = {
        // L1/E1/B1   L2/E5b/B2b  L5/E5a/B2a  E6/LEX/B3  E5(a+b)
        {"CPYWMNSLX","CPYWMNDLSX","IQX"       ,""         ,""         ,""}, // GPS
        {"CPABX"    ,"CPABX"     ,"IQX"        ,""         ,""         ,""}, // GLO
        {"CABXZ"    ,"XIQ"       ,"XIQ"        ,"ABCXZ"    ,"IQX"      ,""}, // GAL
        {"CLSXZBE"  ,"LSX"       ,"IQXDPZ"     ,"LSXEZ"    ,""         ,""}, // QZS
        {"C"        ,"IQX"       ,""           ,""         ,""         ,""}, // SBS
        {"IQX"      ,"IQXDPZ"    ,"DPX"        ,"IQXDPZA"  ,"DPXSLZAN" ,"DPX"}, // BDS
        {"ABCX"     ,"ABCX"      ,"DPX"        ,""         ,""         ,""}  // IRN
    };

    // -----------------------------------------------------------------------
    // Signal index (per system)
    // -----------------------------------------------------------------------
    private static class SigInd {
        int n;                      // number of signal types
        int[] idx = new int[MAXOBSTYPE];   // freq-index
        int[] pos = new int[MAXOBSTYPE];   // position in obs data (-1: no)
        int[] pri = new int[MAXOBSTYPE];   // priority (15-0)
        int[] type = new int[MAXOBSTYPE];  // 0:C, 1:L, 2:D, 3:S
        int[] code = new int[MAXOBSTYPE];  // obs-code (CODE_xxx)
        double[] shift = new double[MAXOBSTYPE]; // phase shift (cycle)
    }

    // -----------------------------------------------------------------------
    // Header state passed between header and body parsing
    // -----------------------------------------------------------------------
    private static class RinexHeader {
        double ver = 2.10;
        char type = ' ';
        int sys = SYS_GPS;
        int tsys = TSYS_GPS;
        String[][] tobs = new String[RNX_NUMSYS][MAXOBSTYPE]; // 3-char obs codes

        RinexHeader() {
            for (int i = 0; i < RNX_NUMSYS; i++) {
                for (int j = 0; j < MAXOBSTYPE; j++) {
                    tobs[i][j] = "";
                }
            }
        }
    }

    // =======================================================================
    // Public API
    // =======================================================================

    /**
     * Read observation file and return observations grouped by epoch.
     *
     * @param filename path to RINEX observation file
     * @param nav      navigation data (for storing ionosphere/UTC params from header); may be null
     * @return list of epoch groups, where each group is a list of ObsData for that epoch
     * @throws IOException on file I/O error
     */
    public static List<List<ObsData>> readObs(String filename, Navigation nav) throws IOException {
        return readObs(filename, nav, 1);
    }

    /**
     * Read observation file with specified receiver number.
     *
     * @param filename path to RINEX observation file
     * @param nav      navigation data (for GLONASS FCN, etc.)
     * @param rcv      receiver number (1=rover, 2=base)
     * @return list of observation epochs
     * @throws IOException on file I/O error
     */
    public static List<List<ObsData>> readObs(String filename, Navigation nav, int rcv) throws IOException {
        if (nav == null) nav = new Navigation();
        List<List<ObsData>> obs = new ArrayList<>();
        read(filename, rcv, obs, nav);
        return obs;
    }

    /**
     * Read navigation file.
     *
     * @param filename path to RINEX navigation file
     * @param nav      navigation data to populate; if null, a new one is created
     * @return the populated Navigation
     * @throws IOException on file I/O error
     */
    public static Navigation readNav(String filename, Navigation nav) throws IOException {
        if (nav == null) nav = new Navigation();
        read(filename, 0, null, nav);
        return nav;
    }

    /**
     * Auto-detect file type and read observation or navigation data.
     *
     * @param filename path to RINEX file
     * @param rcv      receiver number (1-based, used for obs files)
     * @param obs      observation epochs (output, may be null if nav-only)
     * @param nav      navigation data (output)
     * @throws IOException on file I/O error
     */
    public static void read(String filename, int rcv, List<List<ObsData>> obs,
                            Navigation nav) throws IOException {
        try (BufferedReader br = new BufferedReader(new FileReader(filename))) {
            RinexHeader hdr = new RinexHeader();

            if (!readHeader(br, hdr, nav)) {
                return;
            }

            switch (hdr.type) {
                case 'O':
                    if (obs != null) {
                        readObsBody(br, hdr, rcv, obs, nav);
                    }
                    break;
                case 'N': readNavBody(br, hdr, hdr.sys, nav); break;
                case 'G': readNavBody(br, hdr, SYS_GLO, nav); break;
                case 'H': readNavBody(br, hdr, SYS_SBS, nav); break;
                case 'J': readNavBody(br, hdr, SYS_QZS, nav); break;
                case 'L': readNavBody(br, hdr, SYS_GAL, nav); break;
                default:
                    break;
            }
        }
    }

    // =======================================================================
    // Header parsing
    // =======================================================================

    private static boolean readHeader(BufferedReader br, RinexHeader hdr,
                                      Navigation nav) throws IOException {
        String line;
        int lineCount = 0;

        while ((line = br.readLine()) != null) {
            if (line.length() <= 60) {
                continue;
            }
            String label = line.substring(60).trim();

            if (label.contains("RINEX VERSION / TYPE")) {
                hdr.ver = parseDouble(line.substring(0, 9));
                hdr.type = line.length() > 20 ? line.charAt(20) : ' ';

                char sysChar = line.length() > 40 ? line.charAt(40) : ' ';
                switch (sysChar) {
                    case ' ': case 'G': hdr.sys = SYS_GPS; hdr.tsys = TSYS_GPS; break;
                    case 'R': hdr.sys = SYS_GLO; hdr.tsys = TSYS_UTC; break;
                    case 'E': hdr.sys = SYS_GAL; hdr.tsys = TSYS_GAL; break;
                    case 'S': hdr.sys = SYS_SBS; hdr.tsys = TSYS_GPS; break;
                    case 'J': hdr.sys = SYS_QZS; hdr.tsys = TSYS_QZS; break;
                    case 'C': hdr.sys = SYS_CMP; hdr.tsys = TSYS_CMP; break;
                    case 'I': hdr.sys = SYS_IRN; hdr.tsys = TSYS_IRN; break;
                    case 'M': hdr.sys = SYS_NONE; hdr.tsys = TSYS_GPS; break;
                    default: break;
                }
                continue;
            }
            if (label.contains("PGM / RUN BY / DATE")) continue;
            if (label.contains("COMMENT")) continue;

            switch (hdr.type) {
                case 'O': decodeObsHeader(br, line, hdr, nav); break;
                case 'N': case 'J': case 'L': decodeNavHeader(line, nav); break;
                case 'G': break; // GLONASS nav header - minimal
                case 'H': break; // GEO nav header - minimal
            }

            if (label.contains("END OF HEADER")) return true;

            if (++lineCount >= 1024 && hdr.type == ' ') break;
        }
        return false;
    }

    // -----------------------------------------------------------------------
    // Observation header
    // -----------------------------------------------------------------------

    private static void decodeObsHeader(BufferedReader br, String line,
                                        RinexHeader hdr, Navigation nav) throws IOException {
        String label = line.substring(60).trim();

        if (label.contains("SYS / # / OBS TYPES")) {
            // RINEX 3 observation types
            int sysIdx = SYSCODES.indexOf(line.charAt(0));
            if (sysIdx < 0) return;

            int n = (int) str2num(line, 3, 3);
            int nt = 0;
            int k = 7;
            for (int j = 0; j < n; j++, k += 4) {
                if (k > 58) {
                    line = br.readLine();
                    if (line == null) break;
                    k = 7;
                }
                if (nt < MAXOBSTYPE - 1) {
                    hdr.tobs[sysIdx][nt++] = safeSubstring(line, k, k + 3);
                }
            }
            // Mark end
            if (nt < MAXOBSTYPE) hdr.tobs[sysIdx][nt] = "";

            // BDS B1 code change for ver 3.02
            if (sysIdx == RNX_SYS_CMP && Math.abs(hdr.ver - 3.02) < 1e-3) {
                for (int j = 0; j < nt; j++) {
                    String t = hdr.tobs[sysIdx][j];
                    if (t.length() >= 2 && t.charAt(1) == '1') {
                        hdr.tobs[sysIdx][j] = t.charAt(0) + "2" + (t.length() > 2 ? t.substring(2) : "");
                    }
                }
            }
        } else if (label.contains("# / TYPES OF OBSERV")) {
            // RINEX 2 observation types
            int n = (int) str2num(line, 0, 6);
            int nt = 0;
            int k = 10;
            for (int i = 0; i < n; i++, k += 6) {
                if (k > 58) {
                    line = br.readLine();
                    if (line == null) break;
                    k = 10;
                }
                if (nt >= MAXOBSTYPE - 1) continue;
                String str2 = safeSubstring(line, k, k + 2).trim();
                if (str2.length() < 2) { nt++; continue; }

                hdr.tobs[RNX_SYS_GPS][nt] = convcode2(hdr.ver, SYS_GPS, str2);
                hdr.tobs[RNX_SYS_GLO][nt] = convcode2(hdr.ver, SYS_GLO, str2);
                hdr.tobs[RNX_SYS_GAL][nt] = convcode2(hdr.ver, SYS_GAL, str2);
                hdr.tobs[RNX_SYS_QZS][nt] = convcode2(hdr.ver, SYS_QZS, str2);
                hdr.tobs[RNX_SYS_SBS][nt] = convcode2(hdr.ver, SYS_SBS, str2);
                hdr.tobs[RNX_SYS_CMP][nt] = convcode2(hdr.ver, SYS_CMP, str2);
                nt++;
            }
            if (nt < MAXOBSTYPE) {
                for (int s = 0; s < RNX_NUMSYS; s++) {
                    hdr.tobs[s][nt] = "";
                }
            }
        } else if (label.contains("TIME OF FIRST OBS")) {
            String tsysStr = safeSubstring(line, 48, 51).trim();
            switch (tsysStr) {
                case "GPS": hdr.tsys = TSYS_GPS; break;
                case "GLO": hdr.tsys = TSYS_UTC; break;
                case "GAL": hdr.tsys = TSYS_GAL; break;
                case "QZS": hdr.tsys = TSYS_QZS; break;
                case "BDT": hdr.tsys = TSYS_CMP; break;
                case "IRN": hdr.tsys = TSYS_IRN; break;
            }
        } else if (label.contains("GLONASS SLOT / FRQ #")) {
            if (nav != null) {
                for (int i = 0; i < 8; i++) {
                    int col = 4 + i * 7;
                    if (col >= line.length()) break;
                    if (line.charAt(col) != 'R') continue;
                    int prn = (int) str2num(line, 5 + i * 7, 2);
                    int fcn = (int) str2num(line, 8 + i * 7, 2);
                    if (prn < 1 || prn > MAXPRNGLO || fcn < -7 || fcn > 6) continue;
                    nav.gloFcn[prn - 1] = fcn + 8;
                }
            }
        } else if (label.contains("ION ALPHA")) {
            // RINEX 2 GPS ionosphere alpha
            if (nav != null) {
                for (int i = 0, j = 2; i < 4; i++, j += 12) {
                    nav.ionGps[i] = str2num(line, j, 12);
                }
            }
        } else if (label.contains("ION BETA")) {
            // RINEX 2 GPS ionosphere beta
            if (nav != null) {
                for (int i = 0, j = 2; i < 4; i++, j += 12) {
                    nav.ionGps[i + 4] = str2num(line, j, 12);
                }
            }
        } else if (label.contains("DELTA-UTC: A0,A1,T,W")) {
            // RINEX 2 UTC parameters
            if (nav != null) {
                int j = 3;
                for (int i = 0; i < 2; i++, j += 19) {
                    nav.utcGps[i] = str2num(line, j, 19);
                }
                for (int i = 2; i < 4; i++, j += 9) {
                    nav.utcGps[i] = str2num(line, j, 9);
                }
            }
        } else if (label.contains("IONOSPHERIC CORR")) {
            decodeIonosphericCorr(line, nav);
        } else if (label.contains("TIME SYSTEM CORR")) {
            decodeTimeSystemCorr(line, nav);
        } else if (label.contains("LEAP SECONDS")) {
            if (nav != null) {
                nav.utcGps[4] = str2num(line, 0, 6);
                nav.utcGps[7] = str2num(line, 6, 6);
                nav.utcGps[5] = str2num(line, 12, 6);
                nav.utcGps[6] = str2num(line, 18, 6);
            }
        }
        // Other optional headers are silently ignored
    }

    // -----------------------------------------------------------------------
    // Navigation header
    // -----------------------------------------------------------------------

    private static void decodeNavHeader(String line, Navigation nav) {
        if (line.length() <= 60) return;
        String label = line.substring(60).trim();

        if (label.contains("ION ALPHA")) {
            if (nav != null) {
                for (int i = 0, j = 2; i < 4; i++, j += 12) {
                    nav.ionGps[i] = str2num(line, j, 12);
                }
            }
        } else if (label.contains("ION BETA")) {
            if (nav != null) {
                for (int i = 0, j = 2; i < 4; i++, j += 12) {
                    nav.ionGps[i + 4] = str2num(line, j, 12);
                }
            }
        } else if (label.contains("DELTA-UTC: A0,A1,T,W")) {
            if (nav != null) {
                int j = 3;
                for (int i = 0; i < 2; i++, j += 19) {
                    nav.utcGps[i] = str2num(line, j, 19);
                }
                for (int i = 2; i < 4; i++, j += 9) {
                    nav.utcGps[i] = str2num(line, j, 9);
                }
            }
        } else if (label.contains("IONOSPHERIC CORR")) {
            decodeIonosphericCorr(line, nav);
        } else if (label.contains("TIME SYSTEM CORR")) {
            decodeTimeSystemCorr(line, nav);
        } else if (label.contains("LEAP SECONDS")) {
            if (nav != null) {
                nav.utcGps[4] = str2num(line, 0, 6);
                nav.utcGps[7] = str2num(line, 6, 6);
                nav.utcGps[5] = str2num(line, 12, 6);
                nav.utcGps[6] = str2num(line, 18, 6);
            }
        }
    }

    private static void decodeIonosphericCorr(String line, Navigation nav) {
        if (nav == null) return;
        String id = safeSubstring(line, 0, 4).trim();
        switch (id) {
            case "GPSA":
                for (int i = 0, j = 5; i < 4; i++, j += 12) nav.ionGps[i] = str2num(line, j, 12);
                break;
            case "GPSB":
                for (int i = 0, j = 5; i < 4; i++, j += 12) nav.ionGps[i + 4] = str2num(line, j, 12);
                break;
            case "GAL":
                for (int i = 0, j = 5; i < 4; i++, j += 12) nav.ionGal[i] = str2num(line, j, 12);
                break;
            case "QZSA":
                for (int i = 0, j = 5; i < 4; i++, j += 12) nav.ionQzs[i] = str2num(line, j, 12);
                break;
            case "QZSB":
                for (int i = 0, j = 5; i < 4; i++, j += 12) nav.ionQzs[i + 4] = str2num(line, j, 12);
                break;
            case "BDSA":
                for (int i = 0, j = 5; i < 4; i++, j += 12) nav.ionCmp[i] = str2num(line, j, 12);
                break;
            case "BDSB":
                for (int i = 0, j = 5; i < 4; i++, j += 12) nav.ionCmp[i + 4] = str2num(line, j, 12);
                break;
            case "IRNA":
                for (int i = 0, j = 5; i < 4; i++, j += 12) nav.ionIrn[i] = str2num(line, j, 12);
                break;
            case "IRNB":
                for (int i = 0, j = 5; i < 4; i++, j += 12) nav.ionIrn[i + 4] = str2num(line, j, 12);
                break;
        }
    }

    private static void decodeTimeSystemCorr(String line, Navigation nav) {
        if (nav == null) return;
        String id = safeSubstring(line, 0, 4).trim();
        switch (id) {
            case "GPUT":
                nav.utcGps[0] = str2num(line, 5, 17);
                nav.utcGps[1] = str2num(line, 22, 16);
                nav.utcGps[2] = str2num(line, 38, 7);
                nav.utcGps[3] = str2num(line, 45, 5);
                break;
            case "GLUT":
                nav.utcGlo[0] = -str2num(line, 5, 17); // tau_C
                break;
            case "GLGP":
                nav.utcGlo[1] = str2num(line, 5, 17); // tau_GPS
                break;
            case "GAUT":
                nav.utcGal[0] = str2num(line, 5, 17);
                nav.utcGal[1] = str2num(line, 22, 16);
                nav.utcGal[2] = str2num(line, 38, 7);
                nav.utcGal[3] = str2num(line, 45, 5);
                break;
            case "QZUT":
                nav.utcQzs[0] = str2num(line, 5, 17);
                nav.utcQzs[1] = str2num(line, 22, 16);
                nav.utcQzs[2] = str2num(line, 38, 7);
                nav.utcQzs[3] = str2num(line, 45, 5);
                break;
            case "BDUT":
                nav.utcCmp[0] = str2num(line, 5, 17);
                nav.utcCmp[1] = str2num(line, 22, 16);
                nav.utcCmp[2] = str2num(line, 38, 7);
                nav.utcCmp[3] = str2num(line, 45, 5);
                break;
            case "IRUT":
                nav.utcIrn[0] = str2num(line, 5, 17);
                nav.utcIrn[1] = str2num(line, 22, 16);
                nav.utcIrn[2] = str2num(line, 38, 7);
                nav.utcIrn[3] = str2num(line, 45, 5);
                nav.utcIrn[8] = 0.0; // A2
                break;
        }
    }

    // =======================================================================
    // Observation body parsing
    // =======================================================================

    private static void readObsBody(BufferedReader br, RinexHeader hdr,
                                    int rcv, List<List<ObsData>> obs,
                                    Navigation nav) throws IOException {
        SigInd[] index = new SigInd[RNX_NUMSYS];
        int[] sysList = {SYS_GPS, SYS_GLO, SYS_GAL, SYS_QZS, SYS_SBS, SYS_CMP, SYS_IRN};
        for (int s = 0; s < RNX_NUMSYS; s++) {
            index[s] = setIndex(hdr.ver, sysList[s], hdr.tobs[s]);
        }

        String line;
        while ((line = br.readLine()) != null) {
            // Decode epoch line
            EpochResult epoch = decodeObsEpoch(br, line, hdr.ver);
            if (epoch == null || epoch.nsat <= 0) continue;
            if (epoch.flag >= 3 && epoch.flag <= 5) {
                // Skip special records (new site, header info, external event)
                // For flag 3/4, consume nsat lines
                if (epoch.flag == 3 || epoch.flag == 4) {
                    for (int i = 0; i < epoch.nsat; i++) {
                        br.readLine();
                    }
                }
                continue;
            }

            List<ObsData> epochData = new ArrayList<>();

            for (int i = 0; i < epoch.nsat; i++) {
                if (hdr.ver <= 2.99) {
                    // RINEX 2: satellite IDs already parsed from epoch line
                    // Read data lines (5 obs per line, 16-char fields)
                    ObsData data = decodeObsData2(br, hdr.ver, epoch.sats[i], index);
                    if (data != null) {
                        data.time = convertTimeSystem(epoch.time, hdr.tsys);
                        data.rcv = rcv;
                        epochData.add(data);
                    }
                } else {
                    // RINEX 3: read one line per satellite
                    line = br.readLine();
                    if (line == null) break;
                    ObsData data = decodeObsData3(line, hdr.ver, index);
                    if (data != null) {
                        data.time = convertTimeSystem(epoch.time, hdr.tsys);
                        data.rcv = rcv;
                        epochData.add(data);
                    }
                }
            }
            if (!epochData.isEmpty()) {
                obs.add(epochData);
            }
        }
    }

    private static class EpochResult {
        GTime time;
        int flag;
        int nsat;
        int[] sats; // satellite numbers (RINEX 2 only)
    }

    private static EpochResult decodeObsEpoch(BufferedReader br, String line,
                                              double ver) throws IOException {
        EpochResult result = new EpochResult();

        if (ver <= 2.99) {
            // RINEX 2 epoch line
            result.flag = (int) str2num(line, 28, 1);
            int n = (int) str2num(line, 29, 3);
            if (n <= 0) return null;
            result.nsat = n;

            if (result.flag >= 3 && result.flag <= 5) {
                if (result.flag == 5) {
                    result.time = str2time(line, 0, 26);
                }
                return result;
            }

            result.time = str2time(line, 0, 26);
            if (result.time == null) return null;

            result.sats = new int[n];
            int j = 32;
            for (int i = 0; i < n; i++, j += 3) {
                if (j >= 68) {
                    line = br.readLine();
                    if (line == null) break;
                    j = 32;
                }
                if (i < MAXOBS) {
                    String satid = safeSubstring(line, j, j + 3).trim();
                    result.sats[i] = SatelliteUtil.satid2no(satid);
                }
            }
        } else {
            // RINEX 3 epoch line: "> yyyy mm dd hh mm ss.sssssss  flag  numsat"
            result.flag = (int) str2num(line, 31, 1);
            int n = (int) str2num(line, 32, 3);
            if (n <= 0) return null;
            result.nsat = n;

            if (result.flag >= 3 && result.flag <= 5) {
                if (result.flag == 5) {
                    result.time = str2time(line, 1, 28);
                }
                return result;
            }

            if (line.charAt(0) != '>') return null;
            result.time = str2time(line, 1, 28);
            if (result.time == null) return null;
        }
        return result;
    }

    /**
     * Decode RINEX 2 observation data for one satellite.
     * Data lines contain 5 observations per line, each 16 chars.
     */
    private static ObsData decodeObsData2(BufferedReader br, double ver,
                                          int sat, SigInd[] index) throws IOException {
        /* Determine the number of observation types from the header (GPS index)
         * to know how many data lines to consume per satellite. */
        SigInd ind = null;
        boolean skip = false;

        if (sat <= 0) {
            skip = true;
        } else {
            int[] sysPrn = SatelliteUtil.satsys(sat);
            int sys = sysPrn[0];
            if (sys == SYS_NONE) {
                skip = true;
            } else {
                ind = getIndexForSys(sys, index);
                if (ind == null || ind.n == 0) {
                    skip = true;
                }
            }
        }

        /* Determine number of obs types to read (for line consumption).
         * Use GPS index as fallback since all systems share the same obs types
         * in RINEX 2. */
        int nobs;
        if (ind != null) {
            nobs = ind.n;
        } else {
            /* Use GPS signal index to determine number of lines to skip */
            SigInd gpsInd = getIndexForSys(SYS_GPS, index);
            nobs = (gpsInd != null) ? gpsInd.n : 0;
        }

        // Read observation values from data lines
        double[] val = new double[MAXOBSTYPE];
        int[] lli = new int[MAXOBSTYPE];
        double[] std = new double[MAXOBSTYPE];

        String line = null;
        int j = 0;
        int nlines = (nobs + 4) / 5; /* 5 obs per line (16 chars each, max 80 chars) */
        int lineRead = 0;
        for (int i = 0; i < nobs; i++, j += 16) {
            if (j >= 80 || i == 0) {
                if (i == 0 || j >= 80) {
                    line = br.readLine();
                    lineRead++;
                    if (line == null) break;
                    j = 0;
                }
            }
            if (!skip) {
                val[i] = str2num(line, j, 14) + ind.shift[i];
                lli[i] = ((int) str2num(line, j + 14, 1)) & 3;
                std[i] = str2num(line, j + 15, 1);
            }
        }
        /* Consume any remaining lines if fewer were read than expected */
        while (lineRead < nlines) {
            br.readLine();
            lineRead++;
        }

        if (skip) return null;
        return buildObsData(ver, sat, ind, val, lli, std);
    }

    /**
     * Decode RINEX 3 observation data for one satellite from a single line.
     */
    private static ObsData decodeObsData3(String line, double ver,
                                          SigInd[] index) {
        if (line.length() < 3) return null;
        String satid = line.substring(0, 3).trim();
        int sat = SatelliteUtil.satid2no(satid);
        if (sat <= 0) return null;

        int[] sysPrn = SatelliteUtil.satsys(sat);
        int sys = sysPrn[0];
        if (sys == SYS_NONE) return null;

        SigInd ind = getIndexForSys(sys, index);
        if (ind == null || ind.n == 0) return null;

        double[] val = new double[MAXOBSTYPE];
        int[] lli = new int[MAXOBSTYPE];
        double[] std = new double[MAXOBSTYPE];

        for (int i = 0, j = 3; i < ind.n; i++, j += 16) {
            val[i] = str2num(line, j, 14) + ind.shift[i];
            lli[i] = ((int) str2num(line, j + 14, 1)) & 3;
            std[i] = str2num(line, j + 15, 1);
        }

        return buildObsData(ver, sat, ind, val, lli, std);
    }

    /**
     * Build ObsData from parsed values, applying position assignment logic.
     */
    private static ObsData buildObsData(double ver, int sat, SigInd ind,
                                        double[] val, int[] lli, double[] std) {
        ObsData obs = new ObsData();
        obs.sat = sat;

        // Determine position mapping
        int[] p = new int[MAXOBSTYPE];
        for (int i = 0; i < ind.n; i++) {
            p[i] = (ver <= 2.11) ? ind.idx[i] : ind.pos[i];
        }

        // For RINEX 2.11 and earlier, handle duplicate codes (C1/P1, C2/P2)
        if (ver <= 2.11) {
            resolveDuplicateCodes(ind, val, p, 0); // L1 codes
            resolveDuplicateCodes(ind, val, p, 1); // L2 codes
            resolveDuplicateCodes(ind, val, p, 2); // L3 codes
        }

        // Assign observation data
        for (int i = 0; i < ind.n; i++) {
            if (p[i] < 0 || p[i] >= NOBS) continue;
            if (val[i] == 0.0 && lli[i] == 0) continue;

            switch (ind.type[i]) {
                case 0: // Pseudorange
                    obs.P[p[i]] = val[i];
                    obs.code[p[i]] = ind.code[i];
                    obs.Pstd[p[i]] = std[i] > 0 ? 0.01 * Math.pow(2, std[i] + 5) : 0;
                    break;
                case 1: // Carrier-phase
                    obs.L[p[i]] = val[i];
                    obs.LLI[p[i]] = lli[i];
                    obs.Lstd[p[i]] = std[i] > 0 ? std[i] * 0.004 : 0;
                    break;
                case 2: // Doppler
                    obs.D[p[i]] = val[i];
                    break;
                case 3: // Signal strength
                    obs.SNR[p[i]] = val[i];
                    break;
            }
        }
        return obs;
    }

    /**
     * Resolve duplicate codes for a given frequency index in RINEX 2.11 and earlier.
     * When multiple pseudorange codes exist for the same frequency (e.g., C1 and P1),
     * keep the higher-priority one in the primary slot and move the other to NEXOBS.
     */
    private static void resolveDuplicateCodes(SigInd ind, double[] val, int[] p, int freqIdx) {
        List<Integer> candidates = new ArrayList<>();
        for (int i = 0; i < ind.n; i++) {
            if (ind.type[i] == 0 && p[i] == freqIdx) {
                candidates.add(i);
            }
        }
        if (candidates.size() >= 2) {
            int k0 = candidates.get(0);
            int k1 = candidates.get(1);
            if (val[k0] == 0.0 && val[k1] == 0.0) {
                p[k0] = -1;
                p[k1] = -1;
            } else if (val[k0] != 0.0 && val[k1] == 0.0) {
                p[k0] = freqIdx;
                p[k1] = -1;
            } else if (val[k0] == 0.0 && val[k1] != 0.0) {
                p[k0] = -1;
                p[k1] = freqIdx;
            } else if (ind.pri[k1] > ind.pri[k0]) {
                p[k1] = freqIdx;
                p[k0] = (freqIdx < NEXOBS) ? NFREQ + freqIdx : -1;
            } else {
                p[k0] = freqIdx;
                p[k1] = (freqIdx < NEXOBS) ? NFREQ + freqIdx : -1;
            }
        }
    }

    // =======================================================================
    // Navigation body parsing
    // =======================================================================

    private static void readNavBody(BufferedReader br, RinexHeader hdr,
                                    int sys, Navigation nav) throws IOException {
        String line;
        while ((line = br.readLine()) != null) {
            // Skip blank lines
            if (line.trim().isEmpty()) continue;

            // Decode satellite ID and epoch
            int sat;
            int sp; // field start position after satellite ID
            int navSys = sys;

            if (hdr.ver >= 3.0 || sys == SYS_GAL || sys == SYS_QZS) {
                // RINEX 3 or RINEX 2 GAL/QZS extension
                String satid = safeSubstring(line, 0, 3).trim();
                sat = SatelliteUtil.satid2no(satid);
                sp = 4;
                if (hdr.ver >= 3.0 && sat > 0) {
                    int[] sysPrn = SatelliteUtil.satsys(sat);
                    navSys = sysPrn[0];
                    if (navSys == SYS_NONE) {
                        navSys = (satid.charAt(0) == 'S') ? SYS_SBS :
                                 (satid.charAt(0) == 'R') ? SYS_GLO : SYS_GPS;
                    }
                }
            } else {
                // RINEX 2
                int prn = (int) str2num(line, 0, 2);
                sp = 3;
                if (sys == SYS_SBS) {
                    sat = SatelliteUtil.satno(SYS_SBS, prn + 100);
                } else if (sys == SYS_GLO) {
                    sat = SatelliteUtil.satno(SYS_GLO, prn);
                } else if (prn >= 93 && prn <= 97) {
                    sat = SatelliteUtil.satno(SYS_QZS, prn + 100);
                } else {
                    sat = SatelliteUtil.satno(SYS_GPS, prn);
                }
            }

            if (sat <= 0) {
                // Skip this record (consume remaining lines)
                skipNavRecord(br, navSys);
                continue;
            }

            // Decode Toc
            GTime toc = str2time(line, sp, 19);
            if (toc == null) {
                skipNavRecord(br, navSys);
                continue;
            }

            // Read data fields
            double[] data = new double[64];
            int idx = 0;

            // Line 0: 3 data fields after epoch
            String p = safeSubstring(line, sp + 19, line.length());
            for (int j = 0; j < 3; j++) {
                data[idx++] = str2num(p, j * 19, 19);
            }

            // Determine number of remaining lines and total data count
            int nlines;
            int totalData;
            if (navSys == SYS_GLO) {
                nlines = 3; // 3 more lines
                totalData = 15;
            } else if (navSys == SYS_SBS) {
                nlines = 3;
                totalData = 15;
            } else {
                nlines = 7; // 7 more lines for GPS/GAL/QZS/BDS/IRN
                totalData = 31;
            }

            // Read remaining data lines (4 values per line)
            for (int ln = 0; ln < nlines; ln++) {
                line = br.readLine();
                if (line == null) break;
                for (int j = 0; j < 4 && idx < totalData; j++) {
                    data[idx++] = str2num(line, sp + j * 19, 19);
                }
            }

            // Decode ephemeris based on system
            if (navSys == SYS_GLO) {
                GloEphemeris geph = decodeGloEph(hdr.ver, sat, toc, data);
                if (geph != null) {
                    // Store FCN in nav
                    int[] sp2 = SatelliteUtil.satsys(sat);
                    int prn = sp2[1];
                    if (prn >= 1 && prn <= nav.gloFcn.length) {
                        nav.gloFcn[prn - 1] = geph.frq + 8;
                    }
                    nav.geph.add(geph);
                }
            } else if (navSys == SYS_SBS) {
                // SBAS not supported in current Java model - skip
            } else {
                // GPS/GAL/QZS/BDS/IRN
                Ephemeris eph = decodeEph(hdr.ver, sat, toc, data);
                if (eph != null) {
                    nav.eph.add(eph);
                }
            }
        }
    }

    /**
     * Skip remaining lines in a navigation record.
     */
    private static void skipNavRecord(BufferedReader br, int sys) throws IOException {
        int nlines = (sys == SYS_GLO || sys == SYS_SBS) ? 3 : 7;
        for (int i = 0; i < nlines; i++) {
            br.readLine();
        }
    }

    // -----------------------------------------------------------------------
    // Ephemeris decoding
    // -----------------------------------------------------------------------

    /**
     * Decode GPS/GAL/QZS/BDS/IRN ephemeris from data array.
     * Data array indices match C RTKLIB decode_eph():
     * [0-2]: f0,f1,f2  [3]:iode  [4]:crs  [5]:deln  [6]:M0
     * [7]:cuc  [8]:e  [9]:cus  [10]:sqrtA
     * [11]:toes  [12]:cic  [13]:OMG0  [14]:cis
     * [15]:i0  [16]:crc  [17]:omg  [18]:OMGd
     * [19]:idot  [20]:codes  [21]:week  [22]:flag
     * [23]:sva  [24]:svh  [25]:tgd0  [26]:tgd1/iodc
     * [27]:ttr  [28]:fit/iodc
     */
    private static Ephemeris decodeEph(double ver, int sat, GTime toc, double[] data) {
        int[] sysPrn = SatelliteUtil.satsys(sat);
        int sys = sysPrn[0];

        if ((sys & (SYS_GPS | SYS_GAL | SYS_QZS | SYS_CMP | SYS_IRN)) == 0) {
            return null;
        }

        Ephemeris eph = new Ephemeris();
        eph.sat = sat;
        eph.toc = toc;

        eph.f0 = data[0];
        eph.f1 = data[1];
        eph.f2 = data[2];

        eph.A = data[10] * data[10]; // sqrtA^2
        eph.e = data[8];
        eph.i0 = data[15];
        eph.OMG0 = data[13];
        eph.omg = data[17];
        eph.M0 = data[6];
        eph.deln = data[5];
        eph.OMGd = data[18];
        eph.idot = data[19];
        eph.crc = data[16];
        eph.crs = data[4];
        eph.cuc = data[7];
        eph.cus = data[9];
        eph.cic = data[12];
        eph.cis = data[14];

        if (sys == SYS_GPS || sys == SYS_QZS) {
            eph.iode = (int) data[3];
            eph.iodc = (int) data[26];
            eph.toes = data[11];
            eph.week = (int) data[21];
            eph.toe = adjweek(GTime.gpst2time(eph.week, data[11]), toc);
            eph.ttr = adjweek(GTime.gpst2time(eph.week, data[27]), toc);
            eph.code = (int) data[20];
            eph.svh = (int) data[24];
            eph.sva = uraindex(data[23]);
            eph.flag = (int) data[22];
            eph.tgd[0] = data[25];
            if (sys == SYS_GPS) {
                eph.fit = data[28];
            } else {
                eph.fit = data[28] == 0.0 ? 2 : 4;
            }
        } else if (sys == SYS_GAL) {
            eph.iode = (int) data[3];
            eph.toes = data[11];
            eph.week = (int) data[21];
            eph.toe = adjweek(GTime.gpst2time(eph.week, data[11]), toc);
            eph.ttr = adjweek(GTime.gpst2time(eph.week, data[27]), toc);
            eph.code = (int) data[20];
            eph.svh = (int) data[24];
            eph.sva = sisaIndex(data[23]);
            eph.tgd[0] = data[25]; // BGD E5a/E1
            eph.tgd[1] = data[26]; // BGD E5b/E1
        } else if (sys == SYS_CMP) {
            eph.toc = toc.bdt2gpst(); // BDT -> GPST
            eph.iode = (int) data[3];
            eph.iodc = (int) data[28];
            eph.toes = data[11];
            eph.week = (int) data[21];
            eph.toe = GTime.bdt2time(eph.week, data[11]).bdt2gpst();
            eph.ttr = GTime.bdt2time(eph.week, data[27]).bdt2gpst();
            eph.toe = adjweek(eph.toe, toc);
            eph.ttr = adjweek(eph.ttr, toc);
            eph.svh = (int) data[24];
            eph.sva = uraindex(data[23]);
            eph.tgd[0] = data[25]; // TGD1 B1/B3
            eph.tgd[1] = data[26]; // TGD2 B2/B3
        } else if (sys == SYS_IRN) {
            eph.iode = (int) data[3];
            eph.toes = data[11];
            eph.week = (int) data[21];
            eph.toe = adjweek(GTime.gpst2time(eph.week, data[11]), toc);
            eph.ttr = adjweek(GTime.gpst2time(eph.week, data[27]), toc);
            eph.svh = (int) data[24];
            eph.sva = uraindex(data[23]);
            eph.tgd[0] = data[25];
        }
        return eph;
    }

    /**
     * Decode GLONASS ephemeris from data array.
     * Data indices:
     * [0]:-TauN  [1]:GammaN  [2]:message frame time
     * [3]:X(km)  [4]:Xdot(km/s)  [5]:Xdotdot(km/s^2)  [6]:health
     * [7]:Y(km)  [8]:Ydot(km/s)  [9]:Ydotdot(km/s^2)  [10]:freq number
     * [11]:Z(km)  [12]:Zdot(km/s)  [13]:Zdotdot(km/s^2)  [14]:age
     */
    private static GloEphemeris decodeGloEph(double ver, int sat, GTime toc,
                                             double[] data) {
        int[] sysPrn = SatelliteUtil.satsys(sat);
        if (sysPrn[0] != SYS_GLO) return null;

        GloEphemeris geph = new GloEphemeris();
        geph.sat = sat;

        // Toc rounded to 15 min in UTC
        double[] wt = toc.time2gpst();
        int week = (int) wt[0];
        double tow = wt[1];
        toc = GTime.gpst2time(week, Math.floor((tow + 450.0) / 900.0) * 900);
        int dow = (int) Math.floor(tow / 86400.0);

        // Time of frame in UTC
        double tod = (ver <= 2.99) ? data[2] : (data[2] % 86400.0);
        GTime tof = GTime.gpst2time(week, tod + dow * 86400.0);
        tof = adjday(tof, toc);

        geph.toe = toc.utc2gpst();  // Toc -> GPST
        geph.tof = tof.utc2gpst();  // Tof -> GPST

        // IODE = Tb (7bit)
        geph.iode = (int) (((tow + 10800.0) % 86400.0) / 900.0 + 0.5);

        geph.taun = -data[0]; // -TauN
        geph.gamn = data[1];  // GammaN

        geph.pos[0] = data[3] * 1E3;
        geph.pos[1] = data[7] * 1E3;
        geph.pos[2] = data[11] * 1E3;
        geph.vel[0] = data[4] * 1E3;
        geph.vel[1] = data[8] * 1E3;
        geph.vel[2] = data[12] * 1E3;
        geph.acc[0] = data[5] * 1E3;
        geph.acc[1] = data[9] * 1E3;
        geph.acc[2] = data[13] * 1E3;

        geph.svh = (int) data[6];
        geph.frq = (int) data[10];
        geph.age = (int) data[14];

        // Handle receivers that output >128 for negative freq numbers
        if (geph.frq > 128) geph.frq -= 256;

        return geph;
    }

    // =======================================================================
    // Signal index setup
    // =======================================================================

    /**
     * Set up signal index for a satellite system, matching C set_index().
     */
    private static SigInd setIndex(double ver, int sys, String[] tobs) {
        SigInd ind = new SigInd();
        int n = 0;

        for (int i = 0; i < tobs.length && tobs[i] != null && !tobs[i].isEmpty(); i++) {
            String t = tobs[i];
            if (t.length() < 2) continue;

            ind.code[n] = obs2code(t.substring(1)); // 2-char code after type char
            ind.type[n] = "CLDS".indexOf(t.charAt(0));
            if (ind.type[n] < 0) ind.type[n] = 0;
            ind.idx[n] = code2idx(sys, ind.code[n]);
            ind.pri[n] = getcodepri(sys, ind.code[n]);
            ind.pos[n] = -1;
            n++;
        }

        // Assign position for highest priority code per frequency
        for (int i = 0; i < NFREQ; i++) {
            int best = -1;
            for (int j = 0; j < n; j++) {
                if (ind.idx[j] == i && ind.pri[j] > 0 &&
                    (best < 0 || ind.pri[j] > ind.pri[best])) {
                    best = j;
                }
            }
            if (best < 0) continue;

            for (int j = 0; j < n; j++) {
                if (ind.code[j] == ind.code[best]) {
                    ind.pos[j] = i;
                }
            }
        }

        // Assign extended observation slots
        for (int i = 0; i < NEXOBS; i++) {
            int j;
            for (j = 0; j < n; j++) {
                if (ind.code[j] != 0 && ind.pri[j] > 0 && ind.pos[j] < 0) break;
            }
            if (j >= n) break;

            for (int k = 0; k < n; k++) {
                if (ind.code[k] == ind.code[j]) {
                    ind.pos[k] = NFREQ + i;
                }
            }
        }

        ind.n = n;
        return ind;
    }

    private static SigInd getIndexForSys(int sys, SigInd[] index) {
        switch (sys) {
            case SYS_GPS: return index[RNX_SYS_GPS];
            case SYS_GLO: return index[RNX_SYS_GLO];
            case SYS_GAL: return index[RNX_SYS_GAL];
            case SYS_QZS: return index[RNX_SYS_QZS];
            case SYS_SBS: return index[RNX_SYS_SBS];
            case SYS_CMP: return index[RNX_SYS_CMP];
            case SYS_IRN: return index[RNX_SYS_IRN];
            default:      return index[RNX_SYS_GPS];
        }
    }

    // =======================================================================
    // RINEX 2 obs code conversion
    // =======================================================================

    /**
     * Convert RINEX 2 observation code (2-char) to RINEX 3 code (3-char).
     * Matches C convcode().
     */
    private static String convcode2(double ver, int sys, String str) {
        if (str.equals("P1")) {
            if (sys == SYS_GPS) return "C1W";
            if (sys == SYS_GLO) return "C1P";
        } else if (str.equals("P2")) {
            if (sys == SYS_GPS) return "C2W";
            if (sys == SYS_GLO) return "C2P";
        } else if (str.equals("C1")) {
            if (ver >= 2.12) return "   "; // reject C1 for 2.12
            if (sys == SYS_GPS) return "C1C";
            if (sys == SYS_GLO) return "C1C";
            if (sys == SYS_GAL) return "C1X";
            if (sys == SYS_QZS) return "C1C";
            if (sys == SYS_SBS) return "C1C";
        } else if (str.equals("C2")) {
            if (sys == SYS_GPS) {
                return (ver >= 2.12) ? "C2W" : "C2X";
            }
            if (sys == SYS_GLO) return "C2C";
            if (sys == SYS_QZS) return "C2X";
            if (sys == SYS_CMP) return "C2X";
        } else if (str.equals("C5")) {
            if (sys == SYS_GPS) return "C5X";
            if (sys == SYS_GAL) return "C5X";
            if (sys == SYS_QZS) return "C5X";
            if (sys == SYS_SBS) return "C5X";
        }

        if (str.length() < 2) return "   ";
        char typeChar = str.charAt(0);
        char freqChar = str.charAt(1);

        // ver >= 2.12 special single-letter frequency codes
        if (ver >= 2.12) {
            if (freqChar == 'A') {
                if (sys == SYS_GPS) return typeChar + "1C";
                if (sys == SYS_GLO) return typeChar + "1C";
                if (sys == SYS_QZS) return typeChar + "1C";
                if (sys == SYS_SBS) return typeChar + "1C";
            } else if (freqChar == 'B') {
                if (sys == SYS_GPS) return typeChar + "1X";
                if (sys == SYS_QZS) return typeChar + "1X";
            } else if (freqChar == 'C') {
                if (sys == SYS_GPS) return typeChar + "2X";
                if (sys == SYS_QZS) return typeChar + "2X";
            } else if (freqChar == 'D') {
                if (sys == SYS_GLO) return typeChar + "2C";
            } else if (freqChar == '1') {
                if (sys == SYS_GPS) return typeChar + "1W";
                if (sys == SYS_GLO) return typeChar + "1P";
                if (sys == SYS_GAL) return typeChar + "1X";
                if (sys == SYS_CMP) return typeChar + "2X";
            }
        }

        // General frequency number mapping
        if (freqChar == '1') {
            if (ver < 2.12) {
                if (sys == SYS_GPS) return typeChar + "1C";
                if (sys == SYS_GLO) return typeChar + "1C";
                if (sys == SYS_GAL) return typeChar + "1X";
                if (sys == SYS_QZS) return typeChar + "1C";
                if (sys == SYS_SBS) return typeChar + "1C";
            }
        } else if (freqChar == '2') {
            if (sys == SYS_GPS) return typeChar + "2W";
            if (sys == SYS_GLO) return typeChar + "2P";
            if (sys == SYS_QZS) return typeChar + "2X";
            if (sys == SYS_CMP) return typeChar + "2X";
        } else if (freqChar == '5') {
            if (sys == SYS_GPS) return typeChar + "5X";
            if (sys == SYS_GAL) return typeChar + "5X";
            if (sys == SYS_QZS) return typeChar + "5X";
            if (sys == SYS_SBS) return typeChar + "5X";
        } else if (freqChar == '6') {
            if (sys == SYS_GAL) return typeChar + "6X";
            if (sys == SYS_QZS) return typeChar + "6X";
            if (sys == SYS_CMP) return typeChar + "6X";
        } else if (freqChar == '7') {
            if (sys == SYS_GAL) return typeChar + "7X";
            if (sys == SYS_CMP) return typeChar + "7X";
        } else if (freqChar == '8') {
            if (sys == SYS_GAL) return typeChar + "8X";
        }

        return "   ";
    }

    // =======================================================================
    // Obs code / frequency index utilities
    // =======================================================================

    /**
     * Convert 2-char observation code string to code number.
     * Matches C obs2code().
     */
    static int obs2code(String obs) {
        if (obs == null || obs.length() < 2) return CODE_NONE;
        for (int i = 1; i < OBSCODES.length; i++) {
            if (OBSCODES[i].equals(obs)) return i;
        }
        return CODE_NONE;
    }

    /**
     * Convert code number to 2-char observation code string.
     */
    static String code2obs(int code) {
        if (code <= CODE_NONE || code >= OBSCODES.length) return "";
        return OBSCODES[code];
    }

    /**
     * Get frequency index for a given system and obs code.
     * Matches C code2idx().
     */
    static int code2idx(int sys, int code) {
        String obs = code2obs(code);
        if (obs.isEmpty()) return -1;
        char freq = obs.charAt(0);

        switch (sys) {
            case SYS_GPS:
                switch (freq) {
                    case '1': return 0; // L1
                    case '2': return 1; // L2
                    case '5': return 2; // L5
                }
                break;
            case SYS_GLO:
                switch (freq) {
                    case '1': case '4': return 0; // G1, G1a
                    case '2': case '6': return 1; // G2, G2a
                    case '3': return 2; // G3
                }
                break;
            case SYS_GAL:
                switch (freq) {
                    case '1': return 0; // E1
                    case '7': return 1; // E5b
                    case '5': return 2; // E5a
                    case '6': return 3; // E6
                    case '8': return 4; // E5ab
                }
                break;
            case SYS_QZS:
                switch (freq) {
                    case '1': return 0; // L1
                    case '2': return 1; // L2
                    case '5': return 2; // L5
                    case '6': return 3; // L6
                }
                break;
            case SYS_SBS:
                switch (freq) {
                    case '1': return 0; // L1
                    case '5': return 1; // L5
                }
                break;
            case SYS_CMP:
                switch (freq) {
                    case '2': return 0; // B1I
                    case '7': return 1; // B2, B2b
                    case '5': return 2; // B2a
                    case '6': return 3; // B3
                    case '1': return 4; // B1C, B1A
                    case '8': return 5; // B2ab
                }
                break;
            case SYS_IRN:
                switch (freq) {
                    case '5': return 0; // L5
                    case '9': return 1; // S
                    case '1': return 2; // L1
                }
                break;
        }
        return -1;
    }

    /**
     * Get code priority for a system and obs code.
     * Matches C getcodepri() without options.
     */
    static int getcodepri(int sys, int code) {
        int sysIdx;
        switch (sys) {
            case SYS_GPS: sysIdx = 0; break;
            case SYS_GLO: sysIdx = 1; break;
            case SYS_GAL: sysIdx = 2; break;
            case SYS_QZS: sysIdx = 3; break;
            case SYS_SBS: sysIdx = 4; break;
            case SYS_CMP: sysIdx = 5; break;
            case SYS_IRN: sysIdx = 6; break;
            default: return 0;
        }
        int freqIdx = code2idx(sys, code);
        if (freqIdx < 0 || freqIdx >= CODEPRIS[sysIdx].length) return 0;

        String obs = code2obs(code);
        if (obs.isEmpty() || obs.length() < 2) return 0;

        String pri = CODEPRIS[sysIdx][freqIdx];
        int p = pri.indexOf(obs.charAt(1));
        return p >= 0 ? 14 - p : 0;
    }

    // =======================================================================
    // Time utilities
    // =======================================================================

    /**
     * Adjust time for week handover.
     */
    private static GTime adjweek(GTime t, GTime t0) {
        double tt = t.diff(t0);
        if (tt < -302400.0) return t.add(604800.0);
        if (tt > 302400.0) return t.add(-604800.0);
        return t;
    }

    /**
     * Adjust time for day handover.
     */
    private static GTime adjday(GTime t, GTime t0) {
        double tt = t.diff(t0);
        if (tt < -43200.0) return t.add(86400.0);
        if (tt > 43200.0) return t.add(-86400.0);
        return t;
    }

    /**
     * Convert time based on time system.
     */
    private static GTime convertTimeSystem(GTime time, int tsys) {
        if (tsys == TSYS_UTC) {
            return time.utc2gpst();
        }
        return time;
    }

    /**
     * Parse time from RINEX string.
     * Handles both RINEX 2 format (" yy mm dd hh mm  ss.sssssss") and
     * RINEX 3 format ("yyyy mm dd hh mm ss.sssssss").
     *
     * @param line source string
     * @param pos  start position
     * @param len  field length (26 for v2, 28 for v3, or 19 for nav Toc)
     * @return GTime or null on parse error
     */
    private static GTime str2time(String line, int pos, int len) {
        String s = safeSubstring(line, pos, pos + len).trim();
        if (s.isEmpty()) return null;

        try {
            String[] parts = s.split("\\s+");
            if (parts.length < 6) return null;

            double year = Double.parseDouble(parts[0]);
            double month = Double.parseDouble(parts[1]);
            double day = Double.parseDouble(parts[2]);
            double hour = Double.parseDouble(parts[3]);
            double min = Double.parseDouble(parts[4]);
            double sec = Double.parseDouble(parts[5]);

            // 2-digit year handling
            if (year < 80) year += 2000;
            else if (year < 100) year += 1900;

            return GTime.epoch2time(new double[]{year, month, day, hour, min, sec});
        } catch (NumberFormatException e) {
            return null;
        }
    }

    // =======================================================================
    // Numeric utilities
    // =======================================================================

    /**
     * Extract a numeric value from a substring, handling Fortran 'D'/'d' exponent.
     * Matches C str2num().
     */
    private static double str2num(String line, int pos, int len) {
        String s = safeSubstring(line, pos, pos + len).trim();
        if (s.isEmpty()) return 0.0;
        // Replace Fortran-style exponent
        s = s.replace('D', 'E').replace('d', 'e');
        try {
            return Double.parseDouble(s);
        } catch (NumberFormatException e) {
            return 0.0;
        }
    }

    /**
     * Parse a double value from a string.
     */
    private static double parseDouble(String s) {
        if (s == null) return 0.0;
        s = s.trim().replace('D', 'E').replace('d', 'e');
        if (s.isEmpty()) return 0.0;
        try {
            return Double.parseDouble(s);
        } catch (NumberFormatException e) {
            return 0.0;
        }
    }

    /**
     * Safe substring that handles out-of-bounds gracefully.
     */
    private static String safeSubstring(String s, int begin, int end) {
        if (s == null) return "";
        if (begin < 0) begin = 0;
        if (begin >= s.length()) return "";
        if (end > s.length()) end = s.length();
        if (end <= begin) return "";
        return s.substring(begin, end);
    }

    /**
     * URA value (m) to URA index.
     */
    private static int uraindex(double value) {
        for (int i = 0; i < 15; i++) {
            if (URA_EPH[i] >= value) return i;
        }
        return 15;
    }

    /**
     * Galileo SISA value (m) to SISA index.
     */
    private static int sisaIndex(double value) {
        if (value < 0.0 || value > 6.0) return 255;
        if (value <= 0.49) return (int) Math.round(value / 0.01);
        if (value <= 0.98) return (int) Math.round((value - 0.5) / 0.02) + 50;
        if (value <= 1.96) return (int) Math.round((value - 1.0) / 0.04) + 75;
        return (int) Math.round((value - 2.0) / 0.16) + 100;
    }
}
