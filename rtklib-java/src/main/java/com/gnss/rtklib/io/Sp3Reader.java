package com.gnss.rtklib.io;

import com.gnss.rtklib.core.Constants;
import com.gnss.rtklib.core.GTime;
import com.gnss.rtklib.core.SatelliteUtil;
import com.gnss.rtklib.model.Navigation;
import com.gnss.rtklib.model.PrecEph;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static com.gnss.rtklib.core.Constants.*;

/**
 * SP3 precise ephemeris file reader.
 * Ported from RTKLIB preceph.c readsp3h()+readsp3b().
 */
public final class Sp3Reader {

    private Sp3Reader() {}

    /**
     * Read SP3 file and add precise ephemerides to navigation data.
     *
     * @param file SP3 file path
     * @param nav  navigation data (peph list will be appended)
     * @throws IOException on read error
     */
    public static void readSp3(String file, Navigation nav) throws IOException {
        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            // Parse header
            String[] tsys = {""};
            double[] bfact = new double[2];
            int[] sats = new int[MAXSAT];
            char[] type = {'P'};
            String[] firstEpochLine = {null};
            int ns = readSp3Header(br, sats, bfact, tsys, type, firstEpochLine);

            // Parse body
            readSp3Body(br, type[0], ns, bfact, tsys[0], 0, nav, firstEpochLine[0]);
        }

        // Sort by time
        nav.peph.sort((a, b) -> {
            double dt = a.time.timediff(b.time);
            if (dt < -1E-9) return -1;
            if (dt > 1E-9) return 1;
            return Integer.compare(a.index, b.index);
        });

        // Combine same-epoch entries
        combPeph(nav);
    }

    /**
     * Read SP3 header. Returns ns (number of satellites).
     * firstEpochLine[0] is set to the first epoch line ('*' line) if found during header parsing.
     */
    private static int readSp3Header(BufferedReader br, int[] sats, double[] bfact,
                                      String[] tsys, char[] type,
                                      String[] firstEpochLine) throws IOException {
        int k = 0, ns = 0;
        boolean tsysRead = false;
        boolean bfactRead = false;
        String line;

        while ((line = br.readLine()) != null) {
            if (line.length() < 2) continue;

            char c0 = line.charAt(0);
            char c1 = line.charAt(1);

            if (c0 == '#' && (c1 == 'c' || c1 == 'd')) {
                type[0] = line.charAt(2);
            } else if (c0 == '+' && c1 == ' ') {
                // Satellite list line
                if (ns == 0 && line.length() >= 6) {
                    ns = parseInt(line, 3, 3);
                }
                for (int j = 0; j < 17 && k < ns; j++) {
                    int sys = code2sys(safeChar(line, 9 + 3 * j));
                    if (sys == 0) { k++; continue; } // padding '0' or ' '
                    int prn = parseInt(line, 10 + 3 * j, 2);
                    if (k < MAXSAT) {
                        int sat = SatelliteUtil.satno(sys, prn);
                        if (sat > 0) sats[k] = sat;
                    }
                    k++;
                }
            } else if (c0 == '+' && c1 == '+') {
                // Accuracy lines — skip
            } else if (c0 == '%' && c1 == 'c') {
                if (!tsysRead && line.length() >= 12) {
                    tsys[0] = line.substring(9, 12).trim();
                    tsysRead = true;
                }
            } else if (c0 == '%' && c1 == 'f') {
                if (!bfactRead) {
                    bfact[0] = parseDouble(line, 3, 10);
                    bfact[1] = parseDouble(line, 14, 12);
                    bfactRead = true;
                }
            } else if (c0 == '%' && c1 == 'i') {
                // Comment lines — skip
            } else if (c0 == '/' && c1 == '*') {
                // Comment lines — skip
            } else if (c0 == '*') {
                // First epoch line — header is done, pass it back
                firstEpochLine[0] = line;
                break;
            }
        }
        return ns;
    }

    private static void readSp3Body(BufferedReader br, char type, int ns,
                                     double[] bfact, String tsys, int index,
                                     Navigation nav, String firstEpochLine) throws IOException {
        int n = ns * (type == 'P' ? 1 : 2);
        String line = firstEpochLine;
        boolean isUtc = "UTC".equals(tsys);
        boolean firstLine = (line != null && line.length() > 0 && line.charAt(0) == '*');

        if (!firstLine) line = null;

        while (true) {
            if (!firstLine) {
                line = br.readLine();
                if (line == null) break;
            }
            firstLine = false;

            if (line.startsWith("EOF")) break;

            // Look for epoch line
            if (line.charAt(0) != '*') continue;
            GTime time = parseEpochLine(line);
            if (time == null) continue;
            if (isUtc) time = utc2gpst(time);

            PrecEph peph = new PrecEph();
            peph.time = time;
            peph.index = index;

            boolean valid = false;

            for (int i = 0; i < n; i++) {
                line = br.readLine();
                if (line == null) break;
                if (line.length() < 4) continue;
                char recType = line.charAt(0);
                if (recType != 'P' && recType != 'V') continue;

                int sys = line.charAt(1) == ' ' ? SYS_GPS : code2sys(line.charAt(1));
                int prn = parseInt(line, 2, 2);
                if (sys == SYS_SBS) prn += 100;
                else if (sys == SYS_QZS) prn += 192;

                int sat = SatelliteUtil.satno(sys, prn);
                if (sat <= 0 || sat > MAXSAT) continue;

                for (int j = 0; j < 4; j++) {
                    double val = parseDouble(line, 4 + j * 14, 14);
                    double stdVal = 0.0;
                    if (j < 3 && line.length() >= 63 + j * 3) {
                        stdVal = parseDouble(line, 61 + j * 3, 2);
                    } else if (j == 3 && line.length() >= 64 + j * 3) {
                        stdVal = parseDouble(line, 61 + j * 3, 3);
                    }

                    if (recType == 'P') {
                        if (val != 0.0 && Math.abs(val - 999999.999999) >= 1E-6) {
                            peph.pos[sat - 1][j] = val * (j < 3 ? 1000.0 : 1E-6);
                            valid = true;
                        }
                        if (bfact[j < 3 ? 0 : 1] > 0.0 && stdVal > 0.0) {
                            peph.std[sat - 1][j] = (float) (Math.pow(bfact[j < 3 ? 0 : 1], stdVal)
                                    * (j < 3 ? 1E-3 : 1E-12));
                        }
                    } else if (valid) { // velocity
                        if (val != 0.0 && Math.abs(val - 999999.999999) >= 1E-6) {
                            peph.vel[sat - 1][j] = val * (j < 3 ? 0.1 : 1E-10);
                        }
                        if (bfact[j < 3 ? 0 : 1] > 0.0 && stdVal > 0.0) {
                            peph.vst[sat - 1][j] = (float) (Math.pow(bfact[j < 3 ? 0 : 1], stdVal)
                                    * (j < 3 ? 1E-7 : 1E-16));
                        }
                    }
                }
            }
            if (valid) {
                nav.peph.add(peph);
            }
        }
    }

    /** Combine same-epoch precise ephemerides. */
    private static void combPeph(Navigation nav) {
        List<PrecEph> combined = new ArrayList<>();
        for (int i = 0; i < nav.peph.size(); i++) {
            PrecEph cur = nav.peph.get(i);
            if (!combined.isEmpty()) {
                PrecEph last = combined.get(combined.size() - 1);
                if (Math.abs(last.time.timediff(cur.time)) < 1E-9) {
                    // Merge: overwrite with non-zero data
                    for (int k = 0; k < MAXSAT; k++) {
                        if (norm4(cur.pos[k]) > 0.0) {
                            System.arraycopy(cur.pos[k], 0, last.pos[k], 0, 4);
                            System.arraycopy(cur.std[k], 0, last.std[k], 0, 4);
                            System.arraycopy(cur.vel[k], 0, last.vel[k], 0, 4);
                            System.arraycopy(cur.vst[k], 0, last.vst[k], 0, 4);
                        }
                    }
                    continue;
                }
            }
            combined.add(cur);
        }
        nav.peph.clear();
        nav.peph.addAll(combined);
    }

    private static double norm4(double[] v) {
        return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]);
    }

    /** Parse epoch line: "* yyyy mm dd hh mm ss.dddddddd" */
    private static GTime parseEpochLine(String line) {
        if (line.length() < 28) return null;
        try {
            int yr = parseInt(line, 3, 4);
            int mo = parseInt(line, 8, 2);
            int dy = parseInt(line, 11, 2);
            int hr = parseInt(line, 14, 2);
            int mn = parseInt(line, 17, 2);
            double sec = parseDouble(line, 20, 10);
            return GTime.epoch2time(new double[]{yr, mo, dy, hr, mn, sec});
        } catch (Exception e) {
            return null;
        }
    }

    /** UTC to GPST conversion (subtract leap seconds). */
    private static GTime utc2gpst(GTime utc) {
        // Use GTime's built-in method if available, else approximate
        return utc.utc2gpst();
    }

    private static int code2sys(char c) {
        switch (c) {
            case 'G': case ' ': return SYS_GPS;
            case 'R': return SYS_GLO;
            case 'E': return SYS_GAL;
            case 'J': return SYS_QZS;
            case 'C': return SYS_CMP;
            case 'I': return SYS_IRN;
            case 'L': return SYS_LEO;
            default:  return SYS_NONE;
        }
    }

    private static char safeChar(String s, int pos) {
        return pos < s.length() ? s.charAt(pos) : ' ';
    }

    private static int parseInt(String s, int pos, int len) {
        if (pos + len > s.length()) return 0;
        return Integer.parseInt(s.substring(pos, pos + len).trim());
    }

    private static double parseDouble(String s, int pos, int len) {
        if (pos + len > s.length()) return 0.0;
        String sub = s.substring(pos, pos + len).trim();
        if (sub.isEmpty()) return 0.0;
        return Double.parseDouble(sub);
    }
}
