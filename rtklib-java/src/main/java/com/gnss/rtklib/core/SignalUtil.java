/*------------------------------------------------------------------------------
 * SignalUtil.java : signal/observation code utility functions ported from rtkcmn.c
 *
 *          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
 *          Java port Copyright (C) 2026
 *
 * Licensed under BSD 2-clause license.
 *-----------------------------------------------------------------------------*/
package com.gnss.rtklib.core;

import static com.gnss.rtklib.core.Constants.*;
import com.gnss.rtklib.model.Navigation;
import com.gnss.rtklib.model.GloEphemeris;

/**
 * Signal and observation code utility functions ported from rtkcmn.c lines 258-840.
 * <p>
 * Provides mapping between observation codes (CODE_xxx), frequency bands,
 * carrier frequencies, and code priority for each GNSS system.
 */
public final class SignalUtil {

    private SignalUtil() {} // utility class

    // -----------------------------------------------------------------------
    // Observation code strings (index = CODE_xxx, value = 2-char RINEX code)
    // -----------------------------------------------------------------------
    private static final String[] OBSCODES = {
        "",  "1C","1P","1W","1Y", "1M","1N","1S","1L","1E",  //  0- 9
        "1A","1B","1X","1Z","2C", "2D","2S","2L","2X","2P",  // 10-19
        "2W","2Y","2M","2N","5I", "5Q","5X","7I","7Q","7X",  // 20-29
        "6A","6B","6C","6X","6Z", "6S","6L","8I","8Q","8X",  // 30-39
        "2I","2Q","6I","6Q","3I", "3Q","3X","1I","1Q","5A",  // 40-49
        "5B","5C","9A","9B","9C", "9X","1D","5D","5P","5Z",  // 50-59
        "6E","7D","7P","7Z","8D", "8P","4A","4B","4X","6D",  // 60-69
        "6P"                                                   // 70
    };

    // -----------------------------------------------------------------------
    // Code priority for each system / frequency index
    // Index: [sysIdx][freqIdx], where sysIdx: 0=GPS,1=GLO,2=GAL,3=QZS,4=SBS,5=BDS,6=IRN
    // -----------------------------------------------------------------------
    private static final String[][] CODEPRIS = {
        /* L1/E1/B1    L2/E5b/B2b   L5/E5a/B2a  E6/LEX/B3   E5(a+b)     (6th) */
        {"CPYWMNSLX", "CPYWMNDLSX", "IQX",       "",          "",          ""},  // GPS
        {"CPABX",     "CPABX",      "IQX",       "",          "",          ""},  // GLO
        {"CABXZ",     "XIQ",        "XIQ",       "ABCXZ",     "IQX",       ""},  // GAL
        {"CLSXZBE",   "LSX",        "IQXDPZ",    "LSXEZ",     "",          ""},  // QZS
        {"C",         "IQX",        "",           "",          "",          ""},  // SBS
        {"IQX",       "IQXDPZ",     "DPX",       "IQXDPZA",   "DPXSLZAN",  "DPX"}, // BDS
        {"ABCX",      "ABCX",       "DPX",       "",          "",          ""}   // IRN
    };

    // -----------------------------------------------------------------------
    // obs2code: convert 2-char observation code string to CODE_xxx
    // -----------------------------------------------------------------------

    /**
     * Convert observation code type string to observation code.
     *
     * @param obs observation code string (e.g. "1C", "2P", "5X")
     * @return obs code (CODE_xxx), or CODE_NONE if not found
     */
    public static int obs2code(String obs) {
        if (obs == null || obs.length() < 2) return CODE_NONE;
        for (int i = 1; i <= MAXCODE; i++) {
            if (OBSCODES[i].equals(obs)) return i;
        }
        return CODE_NONE;
    }

    // -----------------------------------------------------------------------
    // code2obs: convert CODE_xxx to 2-char observation code string
    // -----------------------------------------------------------------------

    /**
     * Convert observation code to observation code string.
     *
     * @param code obs code (CODE_xxx)
     * @return 2-char obs code string (e.g. "1C"), or "" if invalid
     */
    public static String code2obs(int code) {
        if (code <= CODE_NONE || MAXCODE < code) return "";
        return OBSCODES[code];
    }

    // -----------------------------------------------------------------------
    // code2freq_XXX: per-system code to frequency mapping
    // Each returns double[2] = {frequency_Hz, freqIndex} or null on error.
    // -----------------------------------------------------------------------

    /**
     * GPS observation code to frequency.
     *
     * @param code obs code (CODE_xxx)
     * @return {frequency_Hz, freqIndex} or null on error
     */
    public static double[] code2freqGPS(int code) {
        String obs = code2obs(code);
        if (obs.isEmpty()) return null;
        switch (obs.charAt(0)) {
            case '1': return new double[]{FREQL1, 0}; // L1
            case '2': return new double[]{FREQL2, 1}; // L2
            case '5': return new double[]{FREQL5, 2}; // L5
        }
        return null;
    }

    /**
     * GLONASS observation code to frequency.
     *
     * @param code obs code (CODE_xxx)
     * @param fcn  frequency channel number (-7 to 6)
     * @return {frequency_Hz, freqIndex} or null on error
     */
    public static double[] code2freqGLO(int code, int fcn) {
        String obs = code2obs(code);
        if (obs.isEmpty()) return null;
        switch (obs.charAt(0)) {
            case '1': // G1
                if (fcn < -7 || fcn > 6) return null;
                return new double[]{FREQ1_GLO + DFRQ1_GLO * fcn, 0};
            case '2': // G2
                if (fcn < -7 || fcn > 6) return null;
                return new double[]{FREQ2_GLO + DFRQ2_GLO * fcn, 1};
            case '3': return new double[]{FREQ3_GLO,  2}; // G3
            case '4': return new double[]{FREQ1a_GLO, 0}; // G1a
            case '6': return new double[]{FREQ2a_GLO, 1}; // G2a
        }
        return null;
    }

    /**
     * Galileo observation code to frequency.
     *
     * @param code obs code (CODE_xxx)
     * @return {frequency_Hz, freqIndex} or null on error
     */
    public static double[] code2freqGAL(int code) {
        String obs = code2obs(code);
        if (obs.isEmpty()) return null;
        switch (obs.charAt(0)) {
            case '1': return new double[]{FREQL1,   0}; // E1
            case '7': return new double[]{FREQE5b,  1}; // E5b
            case '5': return new double[]{FREQL5,   2}; // E5a
            case '6': return new double[]{FREQL6,   3}; // E6
            case '8': return new double[]{FREQE5ab, 4}; // E5ab
        }
        return null;
    }

    /**
     * QZSS observation code to frequency.
     *
     * @param code obs code (CODE_xxx)
     * @return {frequency_Hz, freqIndex} or null on error
     */
    public static double[] code2freqQZS(int code) {
        String obs = code2obs(code);
        if (obs.isEmpty()) return null;
        switch (obs.charAt(0)) {
            case '1': return new double[]{FREQL1, 0}; // L1
            case '2': return new double[]{FREQL2, 1}; // L2
            case '5': return new double[]{FREQL5, 2}; // L5
            case '6': return new double[]{FREQL6, 3}; // L6
        }
        return null;
    }

    /**
     * SBAS observation code to frequency.
     *
     * @param code obs code (CODE_xxx)
     * @return {frequency_Hz, freqIndex} or null on error
     */
    public static double[] code2freqSBS(int code) {
        String obs = code2obs(code);
        if (obs.isEmpty()) return null;
        switch (obs.charAt(0)) {
            case '1': return new double[]{FREQL1, 0}; // L1
            case '5': return new double[]{FREQL5, 1}; // L5
        }
        return null;
    }

    /**
     * BeiDou observation code to frequency.
     *
     * @param code obs code (CODE_xxx)
     * @return {frequency_Hz, freqIndex} or null on error
     */
    public static double[] code2freqBDS(int code) {
        String obs = code2obs(code);
        if (obs.isEmpty()) return null;
        switch (obs.charAt(0)) {
            case '2': return new double[]{FREQ1_CMP, 0}; // B1I
            case '7': return new double[]{FREQ2_CMP, 1}; // B2,B2b
            case '5': return new double[]{FREQL5,    2}; // B2a
            case '6': return new double[]{FREQ3_CMP, 3}; // B3
            case '1': return new double[]{FREQL1,    4}; // B1C,B1A
            case '8': return new double[]{FREQE5ab,  5}; // B2ab
        }
        return null;
    }

    /**
     * NavIC/IRNSS observation code to frequency.
     *
     * @param code obs code (CODE_xxx)
     * @return {frequency_Hz, freqIndex} or null on error
     */
    public static double[] code2freqIRN(int code) {
        String obs = code2obs(code);
        if (obs.isEmpty()) return null;
        switch (obs.charAt(0)) {
            case '5': return new double[]{FREQL5, 0}; // L5
            case '9': return new double[]{FREQs,  1}; // S
            case '1': return new double[]{FREQL1, 2}; // L1
        }
        return null;
    }

    // -----------------------------------------------------------------------
    // code2idx: system and obs code to frequency index
    // -----------------------------------------------------------------------

    /**
     * Convert system and observation code to frequency index.
     * <pre>
     *              0     1     2     3     4     5
     *  GPS       L1    L2    L5     -     -     -
     *  GLONASS   G1    G2    G3     -     -     -   (G1=G1,G1a; G2=G2,G2a)
     *  Galileo   E1    E5b   E5a   E6   E5ab    -
     *  QZSS      L1    L2    L5    L6     -     -
     *  SBAS      L1     -    L5     -     -     -
     *  BDS       B1    B2b   B2a   B3   B1C   B2ab
     *  NavIC     L5     S    L1     -     -     -
     * </pre>
     *
     * @param sys  satellite system (SYS_xxx)
     * @param code obs code (CODE_xxx)
     * @return frequency index (0 to MAXFREQ-1), or -1 on error
     */
    public static int code2idx(int sys, int code) {
        double[] result;
        switch (sys) {
            case SYS_GPS: result = code2freqGPS(code); break;
            case SYS_GLO: result = code2freqGLO(code, 0); break;
            case SYS_GAL: result = code2freqGAL(code); break;
            case SYS_QZS: result = code2freqQZS(code); break;
            case SYS_SBS: result = code2freqSBS(code); break;
            case SYS_CMP: result = code2freqBDS(code); break;
            case SYS_IRN: result = code2freqIRN(code); break;
            default: return -1;
        }
        return result != null ? (int) result[1] : -1;
    }

    // -----------------------------------------------------------------------
    // code2freq: system and obs code to carrier frequency (Hz)
    // -----------------------------------------------------------------------

    /**
     * Convert system and observation code to carrier frequency.
     *
     * @param sys  satellite system (SYS_xxx)
     * @param code obs code (CODE_xxx)
     * @param fcn  frequency channel number for GLONASS
     * @return carrier frequency in Hz, or 0.0 on error
     */
    public static double code2freq(int sys, int code, int fcn) {
        double[] result;
        switch (sys) {
            case SYS_GPS: result = code2freqGPS(code); break;
            case SYS_GLO: result = code2freqGLO(code, fcn); break;
            case SYS_GAL: result = code2freqGAL(code); break;
            case SYS_QZS: result = code2freqQZS(code); break;
            case SYS_SBS: result = code2freqSBS(code); break;
            case SYS_CMP: result = code2freqBDS(code); break;
            case SYS_IRN: result = code2freqIRN(code); break;
            default: return 0.0;
        }
        return result != null ? result[0] : 0.0;
    }

    // -----------------------------------------------------------------------
    // sat2freq: satellite number and obs code to carrier frequency (Hz)
    // -----------------------------------------------------------------------

    /**
     * Convert satellite number and observation code to carrier frequency.
     * For GLONASS, retrieves the frequency channel number from the navigation
     * data (geph entries or gloFcn array).
     *
     * @param sat  satellite number (1-MAXSAT)
     * @param code obs code (CODE_xxx)
     * @param nav  navigation data (used for GLONASS FCN; may be null)
     * @return carrier frequency in Hz, or 0.0 on error
     */
    public static double sat2freq(int sat, int code, Navigation nav) {
        int[] sp = SatelliteUtil.satsys(sat);
        int sys = sp[0];
        int prn = sp[1];
        int fcn = -8; // invalid default

        if (sys == SYS_GLO && nav != null) {
            // search GLONASS ephemeris for the satellite
            boolean found = false;
            for (GloEphemeris ge : nav.geph) {
                if (ge.sat == sat) {
                    fcn = ge.frq;
                    found = true;
                    break;
                }
            }
            if (!found && prn >= 1 && prn <= nav.gloFcn.length
                    && nav.gloFcn[prn - 1] > 0) {
                fcn = nav.gloFcn[prn - 1] - 8;
            }
        }
        return code2freq(sys, code, fcn);
    }

    // -----------------------------------------------------------------------
    // getcodepri: get code priority (0-15)
    // -----------------------------------------------------------------------

    /**
     * Get code priority for a given system and observation code.
     * Higher value = higher priority (15 = highest, 0 = error/unknown).
     * <p>
     * The opt string can override the default priority table with entries like
     * "-GL1C" (force GPS L1C to priority 15) or "-EL5Q" (force GAL L5Q to 15).
     *
     * @param sys  satellite system (SYS_xxx)
     * @param code obs code (CODE_xxx)
     * @param opt  code options string (may be null)
     * @return priority (15=highest, 1=lowest, 0=error)
     */
    public static int getcodepri(int sys, int code, String opt) {
        int i;
        String optPrefix;

        switch (sys) {
            case SYS_GPS: i = 0; optPrefix = "-GL"; break;
            case SYS_GLO: i = 1; optPrefix = "-RL"; break;
            case SYS_GAL: i = 2; optPrefix = "-EL"; break;
            case SYS_QZS: i = 3; optPrefix = "-JL"; break;
            case SYS_SBS: i = 4; optPrefix = "-SL"; break;
            case SYS_CMP: i = 5; optPrefix = "-CL"; break;
            case SYS_IRN: i = 6; optPrefix = "-IL"; break;
            default: return 0;
        }
        int j = code2idx(sys, code);
        if (j < 0) return 0;

        String obs = code2obs(code);
        if (obs.isEmpty()) return 0;

        // parse code options: look for e.g. "-GL1C" in opt string
        if (opt != null) {
            int pos = 0;
            while ((pos = opt.indexOf('-', pos)) >= 0) {
                // check if this matches our system prefix + freq char
                if (opt.startsWith(optPrefix, pos)
                        && pos + optPrefix.length() + 2 <= opt.length()) {
                    String matched = opt.substring(pos + optPrefix.length(),
                                                   pos + optPrefix.length() + 2);
                    if (matched.charAt(0) == obs.charAt(0)) {
                        return matched.charAt(1) == obs.charAt(1) ? 15 : 0;
                    }
                }
                pos++;
            }
        }

        // search code priority table
        if (j >= CODEPRIS[i].length) return 0;
        String pri = CODEPRIS[i][j];
        int idx = pri.indexOf(obs.charAt(1));
        return idx >= 0 ? 14 - idx : 0;
    }

    // -----------------------------------------------------------------------
    // testsnr: test SNR mask
    // -----------------------------------------------------------------------

    /**
     * Test whether an observation is masked by the SNR mask.
     *
     * @param base 0=rover, 1=base station
     * @param idx  frequency index (0=L1, 1=L2, ...)
     * @param el   elevation angle (rad)
     * @param snr  C/N0 (dBHz)
     * @param mask SNR mask (ena[2] and mask[MAXFREQ][9])
     * @return true if the observation is masked (SNR too low), false if unmasked
     */
    public static boolean testsnr(int base, int idx, double el, double snr,
                                  SnrMask mask) {
        if (mask == null || !mask.ena[base] || idx < 0 || idx >= NFREQ) {
            return false;
        }

        double a = (el * R2D + 5.0) / 10.0;
        int i = (int) Math.floor(a);
        a -= i;

        double minsnr;
        if (i < 1) {
            minsnr = mask.mask[idx][0];
        } else if (i > 8) {
            minsnr = mask.mask[idx][8];
        } else {
            minsnr = (1.0 - a) * mask.mask[idx][i - 1] + a * mask.mask[idx][i];
        }
        return snr < minsnr;
    }

    // -----------------------------------------------------------------------
    // SnrMask: SNR mask type (matches C snrmask_t)
    // -----------------------------------------------------------------------

    /**
     * SNR mask type matching C RTKLIB's snrmask_t.
     */
    public static class SnrMask {
        /** Enable flag {rover, base}. */
        public boolean[] ena = new boolean[2];

        /** Mask (dBHz) at 5, 10, 15, ..., 85 deg elevation. [MAXFREQ][9]. */
        public double[][] mask = new double[MAXFREQ][9];
    }
}
