package com.gnss.rtklib.io;

import com.gnss.rtklib.core.Constants;
import com.gnss.rtklib.core.GTime;
import com.gnss.rtklib.core.SatelliteUtil;
import com.gnss.rtklib.model.Navigation;
import com.gnss.rtklib.model.PrecClk;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

import static com.gnss.rtklib.core.Constants.*;

/**
 * RINEX CLK (precise clock) file reader.
 * Ported from RTKLIB rinex.c readrnxclk().
 */
public final class ClkReader {

    private ClkReader() {}

    /**
     * Read RINEX clock file and add precise clocks to navigation data.
     *
     * @param file CLK file path
     * @param nav  navigation data (pclk list will be appended)
     * @throws IOException on read error
     */
    public static void readClk(String file, Navigation nav) throws IOException {
        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            // Skip header
            String line;
            while ((line = br.readLine()) != null) {
                if (line.contains("END OF HEADER")) break;
            }

            // Parse data records
            GTime prevTime = null;
            PrecClk currentClk = null;

            while ((line = br.readLine()) != null) {
                if (line.length() < 40) continue;

                // Only parse satellite clock ("AS") records
                String recType = line.substring(0, 2);
                if (!"AS".equals(recType)) continue;

                // Parse satellite ID
                char sysChar = line.charAt(3);
                int sys;
                switch (sysChar) {
                    case 'G': sys = SYS_GPS; break;
                    case 'R': sys = SYS_GLO; break;
                    case 'E': sys = SYS_GAL; break;
                    case 'C': sys = SYS_CMP; break;
                    case 'J': sys = SYS_QZS; break;
                    case 'I': sys = SYS_IRN; break;
                    default: continue;
                }

                int prn;
                try {
                    prn = Integer.parseInt(line.substring(4, 7).trim());
                } catch (NumberFormatException e) {
                    continue;
                }

                // Apply PRN offset for systems with large PRN ranges
                if (sys == SYS_QZS) prn += MINPRNQZS - 1; // J02 → prn=194
                else if (sys == SYS_SBS) prn += 100;

                int sat = SatelliteUtil.satno(sys, prn);
                if (sat <= 0 || sat > MAXSAT) continue;

                // Parse epoch — detect column offset for RINEX 2 vs 3 CLK format
                // RINEX 2: name field 4 chars (cols 3-6),  epoch at col 8
                // RINEX 3: name field 10 chars (cols 3-12), epoch at col 13
                GTime time;
                int epochOff;
                try {
                    // Find year field: scan for 4-digit year after satellite ID
                    epochOff = line.indexOf("20", 7);  // year 20xx
                    if (epochOff < 0) epochOff = line.indexOf("19", 7);
                    if (epochOff < 0) continue;
                    // Back up to start of year if needed
                    while (epochOff > 3 && line.charAt(epochOff - 1) != ' ') epochOff--;

                    int yr = Integer.parseInt(line.substring(epochOff, epochOff + 4).trim());
                    int mo = Integer.parseInt(line.substring(epochOff + 5, epochOff + 7).trim());
                    int dy = Integer.parseInt(line.substring(epochOff + 8, epochOff + 10).trim());
                    int hr = Integer.parseInt(line.substring(epochOff + 11, epochOff + 13).trim());
                    int mn = Integer.parseInt(line.substring(epochOff + 14, epochOff + 16).trim());
                    double sec = Double.parseDouble(line.substring(epochOff + 16, epochOff + 26).trim());
                    time = GTime.epoch2time(new double[]{yr, mo, dy, hr, mn, sec});
                } catch (Exception e) {
                    continue;
                }

                // Parse clock value (and optionally std)
                // Data fields start after epoch + nval field
                double clkVal;
                float clkStd = 0.0f;
                try {
                    // Find nval field then clock value: scan from after epoch
                    String afterEpoch = line.substring(epochOff + 26).trim();
                    String[] parts = afterEpoch.split("\\s+");
                    // parts[0] = nval, parts[1] = clock, parts[2] = std (optional)
                    if (parts.length < 2) continue;
                    clkVal = Double.parseDouble(parts[1]);
                    if (parts.length >= 3) {
                        clkStd = (float) Double.parseDouble(parts[2]);
                    }
                } catch (NumberFormatException e) {
                    continue;
                }

                // Group by epoch
                if (currentClk == null || prevTime == null ||
                    Math.abs(time.timediff(prevTime)) > 1E-9) {
                    currentClk = new PrecClk();
                    currentClk.time = time;
                    nav.pclk.add(currentClk);
                    prevTime = time;
                }

                currentClk.clk[sat - 1] = clkVal;
                currentClk.std[sat - 1] = clkStd;
            }
        }

        // Sort by time
        nav.pclk.sort((a, b) -> {
            double dt = a.time.timediff(b.time);
            return dt < -1E-9 ? -1 : (dt > 1E-9 ? 1 : 0);
        });
    }
}
