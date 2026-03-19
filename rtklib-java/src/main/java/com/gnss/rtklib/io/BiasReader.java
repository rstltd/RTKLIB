package com.gnss.rtklib.io;

import com.gnss.rtklib.core.SatelliteUtil;
import com.gnss.rtklib.core.SignalUtil;
import com.gnss.rtklib.model.Navigation;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

import static com.gnss.rtklib.core.Constants.*;

/**
 * SINEX BIA (Bias-SINEX) file reader for satellite phase biases (OSB).
 * Parses WHU/CODE/CNES phase bias products in nanoseconds, stores in meters.
 * <p>
 * Format example (WHU WUM0MGXFIN):
 * <pre>
 *  OSB  G080 G01           L5Q       2026:059:00000 2026:060:00300 ns    0.0707    0.000000
 * </pre>
 */
public final class BiasReader {

    private BiasReader() {}

    /**
     * Read SINEX BIA file and store satellite phase biases in nav.pbias.
     *
     * @param file path to .BIA file
     * @param nav  navigation data (pbias populated)
     * @throws IOException if file cannot be read
     */
    public static void readBias(String file, Navigation nav) throws IOException {
        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            String line;
            boolean inBlock = false;

            while ((line = br.readLine()) != null) {
                if (line.startsWith("+BIAS/SOLUTION")) {
                    inBlock = true;
                    continue;
                }
                if (line.startsWith("-BIAS/SOLUTION")) {
                    break;
                }
                if (!inBlock) continue;
                if (line.length() < 70) continue;

                // Parse type field (columns 1-4)
                String type = line.substring(1, 4).trim();
                if (!"OSB".equals(type)) continue;

                // Parse PRN (columns 11-14): e.g. "G01", "E05", "C19"
                String prn = line.substring(11, 14).trim();
                if (prn.isEmpty()) continue;

                // Station field (columns 15-24) must be blank for satellite bias
                String station = line.substring(15, 24).trim();
                if (!station.isEmpty()) continue;

                // Parse OBS1 (columns 25-29): e.g. "L1C  ", "L5Q  ", "C1W  "
                String obs1 = line.substring(25, 29).trim();
                if (obs1.isEmpty()) continue;
                char obsType = obs1.charAt(0);
                if (obsType != 'L' && obsType != 'C') continue;

                // Parse unit (columns 65-67): must be "ns"
                String unit = line.substring(65, 67).trim();
                if (!"ns".equals(unit)) continue;

                // Parse value (columns 70-91 approximately)
                double value;
                try {
                    String valStr = line.substring(70).trim().split("\\s+")[0];
                    value = Double.parseDouble(valStr);
                } catch (Exception e) {
                    continue;
                }

                // Convert PRN to satellite number
                int sys = prn2sys(prn.charAt(0));
                if (sys == SYS_NONE) continue;
                int prnNum;
                try {
                    prnNum = Integer.parseInt(prn.substring(1));
                } catch (NumberFormatException e) {
                    continue;
                }
                int sat = SatelliteUtil.satno(sys, prnNum);
                if (sat <= 0 || sat > MAXSAT) continue;

                // Convert observation code: "L1C" -> obs2code("1C")
                int code = SignalUtil.obs2code(obs1.substring(1));
                if (code == CODE_NONE || code > MAXCODE) continue;

                // Convert ns -> meters: bias_m = value * 1e-9 * CLIGHT
                double bias_m = value * 1e-9 * CLIGHT;

                // Store in appropriate array
                if (obsType == 'L') {
                    if (nav.pbias[sat - 1][code] == 0.0) {
                        nav.pbias[sat - 1][code] = bias_m;
                    }
                } else {
                    if (nav.cbias_osb[sat - 1][code] == 0.0) {
                        nav.cbias_osb[sat - 1][code] = bias_m;
                    }
                }
            }
        }
    }

    private static int prn2sys(char c) {
        switch (c) {
            case 'G': return SYS_GPS;
            case 'R': return SYS_GLO;
            case 'E': return SYS_GAL;
            case 'C': return SYS_CMP;
            case 'J': return SYS_QZS;
            case 'I': return SYS_IRN;
            default:  return SYS_NONE;
        }
    }
}
