package com.gnss.rtklib.io;

import com.gnss.rtklib.core.GTime;
import com.gnss.rtklib.core.SatelliteUtil;
import com.gnss.rtklib.model.AntennaModel;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static com.gnss.rtklib.core.Constants.*;

/**
 * ANTEX antenna parameter file reader.
 * Ported from RTKLIB rtkcmn.c readantex().
 */
public final class AntexReader {

    private AntexReader() {}

    /** Frequency indices to map: G01->0, G02->1, G05->2 */
    private static final int[] FREQS = {1, 2, 5};

    /**
     * Read ANTEX file and return list of antenna models.
     */
    public static List<AntennaModel> readAntex(String file) throws IOException {
        List<AntennaModel> pcvs = new ArrayList<>();
        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            String line;
            AntennaModel pcv = null;
            int freq = 0;
            int state = 0;

            while ((line = br.readLine()) != null) {
                if (line.length() < 60) continue;
                String label = line.substring(60).trim();

                if (label.equals("COMMENT")) continue;

                if (label.equals("START OF ANTENNA")) {
                    pcv = new AntennaModel();
                    state = 1;
                    continue;
                }
                if (label.equals("END OF ANTENNA")) {
                    if (pcv != null) pcvs.add(pcv);
                    state = 0;
                    continue;
                }
                if (state == 0 || pcv == null) continue;

                if (label.equals("TYPE / SERIAL NO")) {
                    pcv.type = line.substring(0, Math.min(20, line.length())).trim();
                    pcv.code = line.substring(20, Math.min(40, line.length())).trim();
                    if (pcv.code.length() == 3) {
                        pcv.sat = SatelliteUtil.satid2no(pcv.code);
                    }
                } else if (label.equals("VALID FROM")) {
                    pcv.ts = parseTime(line);
                } else if (label.equals("VALID UNTIL")) {
                    pcv.te = parseTime(line);
                } else if (label.equals("START OF FREQUENCY")) {
                    // Only read receiver ant for GPS, but read all sat frequencies
                    if (pcv.sat == 0 && line.length() >= 4 && line.charAt(3) != 'G') continue;
                    try {
                        int f = Integer.parseInt(line.substring(4, 6).trim());
                        freq = 0;
                        for (int i = 0; i < FREQS.length; i++) {
                            if (FREQS[i] == f) { freq = i + 1; break; }
                        }
                        // For Galileo E5b (f=7): save to freq index 2
                        if (SatelliteUtil.satsys(pcv.sat)[0] == SYS_GAL && f == 7) {
                            freq = 2;
                        }
                    } catch (NumberFormatException e) {
                        freq = 0;
                    }
                } else if (label.equals("END OF FREQUENCY")) {
                    freq = 0;
                } else if (label.contains("NORTH / EAST / UP")) {
                    if (freq < 1 || freq > NFREQ) continue;
                    double[] neu = parseValues(line, 3);
                    if (pcv.sat != 0) {
                        // Satellite: X/Y/Z order
                        pcv.off[freq - 1][0] = neu[0];
                        pcv.off[freq - 1][1] = neu[1];
                    } else {
                        // Receiver: N/E -> E/N swap
                        pcv.off[freq - 1][0] = neu[1]; // E
                        pcv.off[freq - 1][1] = neu[0]; // N
                    }
                    pcv.off[freq - 1][2] = neu[2]; // U or Z
                } else if (line.contains("NOAZI")) {
                    if (freq < 1 || freq > NFREQ) continue;
                    double[] vals = parseValues(line.substring(8), 19);
                    int count = 0;
                    for (int i = 0; i < 19; i++) {
                        if (i < vals.length) {
                            pcv.var[freq - 1][i] = vals[i];
                            count = i;
                        } else {
                            pcv.var[freq - 1][i] = pcv.var[freq - 1][count];
                        }
                    }
                }
            }
        }
        return pcvs;
    }

    /**
     * Parse space-separated numeric values (mm -> m conversion).
     */
    private static double[] parseValues(String s, int maxN) {
        String[] tokens = s.trim().split("\\s+");
        double[] v = new double[maxN];
        for (int i = 0; i < Math.min(tokens.length, maxN); i++) {
            try {
                v[i] = Double.parseDouble(tokens[i]) * 1E-3; // mm -> m
            } catch (NumberFormatException e) {
                v[i] = 0.0;
            }
        }
        return v;
    }

    /**
     * Parse time from ANTEX format "  2020  1  1  0  0  0.0" in first 43 chars.
     */
    private static GTime parseTime(String line) {
        try {
            String[] parts = line.substring(0, Math.min(43, line.length())).trim().split("\\s+");
            if (parts.length < 6) return null;
            double[] ep = new double[6];
            for (int i = 0; i < 6; i++) ep[i] = Double.parseDouble(parts[i]);
            return GTime.epoch2time(ep);
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * Search antenna model for a satellite by sat number and time.
     */
    public static AntennaModel searchSat(List<AntennaModel> pcvs, int sat, GTime time) {
        for (AntennaModel pcv : pcvs) {
            if (pcv.sat != sat) continue;
            if (pcv.ts != null && time != null && time.diff(pcv.ts) < 0.0) continue;
            if (pcv.te != null && time != null && time.diff(pcv.te) >= 0.0) continue;
            return pcv;
        }
        return null;
    }

    /**
     * Search antenna model for a receiver by antenna type.
     */
    public static AntennaModel searchReceiver(List<AntennaModel> pcvs, String type) {
        if (type == null || type.isEmpty()) return null;
        for (AntennaModel pcv : pcvs) {
            if (pcv.sat != 0) continue;
            if (pcv.type.equals(type)) return pcv;
        }
        // Partial match (first 20 chars or prefix)
        for (AntennaModel pcv : pcvs) {
            if (pcv.sat != 0) continue;
            if (type.startsWith(pcv.type) || pcv.type.startsWith(type)) return pcv;
        }
        return null;
    }
}
