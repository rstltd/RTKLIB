/*------------------------------------------------------------------------------
 * SatelliteUtil.java : satellite utility functions ported from rtkcmn.c
 *
 *          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
 *          Java port Copyright (C) 2026
 *
 * Licensed under BSD 2-clause license.
 *-----------------------------------------------------------------------------*/
package com.gnss.rtklib.core;

import com.gnss.rtklib.model.ProcessingOptions;
import static com.gnss.rtklib.core.Constants.*;

/**
 * Satellite utility functions ported from rtkcmn.c lines 398-557.
 * All constellations are always enabled (no conditional compilation).
 */
public final class SatelliteUtil {

    /** Max variance of ephemeris to reject satellite (m^2) = 300.0^2. */
    public static final double MAX_VAR_EPH = 300.0 * 300.0;

    private SatelliteUtil() {} // utility class

    /**
     * Convert satellite system and PRN/slot number to satellite number (1-based).
     *
     * @param sys satellite system (SYS_GPS, SYS_GLO, ...)
     * @param prn satellite PRN/slot number
     * @return satellite number (1 to MAXSAT), or 0 on error
     */
    public static int satno(int sys, int prn) {
        if (prn <= 0) return 0;
        switch (sys) {
            case SYS_GPS:
                if (prn < MINPRNGPS || MAXPRNGPS < prn) return 0;
                return prn - MINPRNGPS + 1;
            case SYS_GLO:
                if (prn < MINPRNGLO || MAXPRNGLO < prn) return 0;
                return NSATGPS + prn - MINPRNGLO + 1;
            case SYS_GAL:
                if (prn < MINPRNGAL || MAXPRNGAL < prn) return 0;
                return NSATGPS + NSATGLO + prn - MINPRNGAL + 1;
            case SYS_QZS:
                if (prn < MINPRNQZS || MAXPRNQZS < prn) return 0;
                return NSATGPS + NSATGLO + NSATGAL + prn - MINPRNQZS + 1;
            case SYS_CMP:
                if (prn < MINPRNCMP || MAXPRNCMP < prn) return 0;
                return NSATGPS + NSATGLO + NSATGAL + NSATQZS + prn - MINPRNCMP + 1;
            case SYS_IRN:
                if (prn < MINPRNIRN || MAXPRNIRN < prn) return 0;
                return NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP
                     + prn - MINPRNIRN + 1;
            case SYS_LEO:
                if (prn < MINPRNLEO || MAXPRNLEO < prn) return 0;
                return NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP + NSATIRN
                     + prn - MINPRNLEO + 1;
            case SYS_SBS:
                if (prn < MINPRNSBS || MAXPRNSBS < prn) return 0;
                return NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP + NSATIRN
                     + NSATLEO + prn - MINPRNSBS + 1;
            default:
                return 0;
        }
    }

    /**
     * Convert satellite number to satellite system and PRN.
     * Since Java cannot use output parameters, returns an int[2] array.
     *
     * @param sat satellite number (1 to MAXSAT)
     * @return int[2] where [0]=system code (SYS_xxx), [1]=PRN/slot number.
     *         Returns {SYS_NONE, 0} on error.
     */
    public static int[] satsys(int sat) {
        int sys = SYS_NONE;
        int prn = 0;

        if (sat <= 0 || MAXSAT < sat) {
            return new int[]{SYS_NONE, 0};
        }

        if (sat <= NSATGPS) {
            sys = SYS_GPS;
            prn = sat + MINPRNGPS - 1;
        } else if ((sat -= NSATGPS) <= NSATGLO) {
            sys = SYS_GLO;
            prn = sat + MINPRNGLO - 1;
        } else if ((sat -= NSATGLO) <= NSATGAL) {
            sys = SYS_GAL;
            prn = sat + MINPRNGAL - 1;
        } else if ((sat -= NSATGAL) <= NSATQZS) {
            sys = SYS_QZS;
            prn = sat + MINPRNQZS - 1;
        } else if ((sat -= NSATQZS) <= NSATCMP) {
            sys = SYS_CMP;
            prn = sat + MINPRNCMP - 1;
        } else if ((sat -= NSATCMP) <= NSATIRN) {
            sys = SYS_IRN;
            prn = sat + MINPRNIRN - 1;
        } else if ((sat -= NSATIRN) <= NSATLEO) {
            sys = SYS_LEO;
            prn = sat + MINPRNLEO - 1;
        } else if ((sat -= NSATLEO) <= NSATSBS) {
            sys = SYS_SBS;
            prn = sat + MINPRNSBS - 1;
        } else {
            return new int[]{SYS_NONE, 0};
        }

        return new int[]{sys, prn};
    }

    /**
     * Convert satellite number to satellite ID string.
     *
     * @param sat satellite number (1 to MAXSAT)
     * @return satellite ID string (e.g. "G01", "R15", "E05", "120"),
     *         or "" on error
     */
    public static String satno2id(int sat) {
        int[] sp = satsys(sat);
        int sys = sp[0];
        int prn = sp[1];

        switch (sys) {
            case SYS_GPS: return String.format("G%02d", prn - MINPRNGPS + 1);
            case SYS_GLO: return String.format("R%02d", prn - MINPRNGLO + 1);
            case SYS_GAL: return String.format("E%02d", prn - MINPRNGAL + 1);
            case SYS_QZS: return String.format("J%02d", prn - MINPRNQZS + 1);
            case SYS_CMP: return String.format("C%02d", prn - MINPRNCMP + 1);
            case SYS_IRN: return String.format("I%02d", prn - MINPRNIRN + 1);
            case SYS_LEO: return String.format("L%02d", prn - MINPRNLEO + 1);
            case SYS_SBS: return String.format("%03d",  prn);
            default:      return "";
        }
    }

    /**
     * Convert satellite ID string to satellite number.
     * Accepts formats: "G01", "R15", "E05", "J01", "C01", "I01", "L01", "S20",
     * or bare numeric PRN (e.g. "1" for GPS, "120" for SBAS, "193" for QZSS).
     *
     * @param id satellite ID string
     * @return satellite number (1 to MAXSAT), or 0 on error
     */
    public static int satid2no(String id) {
        if (id == null || id.isEmpty()) return 0;

        String trimmed = id.trim();
        if (trimmed.isEmpty()) return 0;

        char first = trimmed.charAt(0);

        // bare numeric PRN
        if (Character.isDigit(first)) {
            try {
                int prn = Integer.parseInt(trimmed);
                int sys;
                if (MINPRNGPS <= prn && prn <= MAXPRNGPS) {
                    sys = SYS_GPS;
                } else if (MINPRNSBS <= prn && prn <= MAXPRNSBS) {
                    sys = SYS_SBS;
                } else if (MINPRNQZS <= prn && prn <= MAXPRNQZS) {
                    sys = SYS_QZS;
                } else {
                    return 0;
                }
                return satno(sys, prn);
            } catch (NumberFormatException e) {
                return 0;
            }
        }

        // letter + numeric PRN
        if (trimmed.length() < 2) return 0;
        int prn;
        try {
            prn = Integer.parseInt(trimmed.substring(1).trim());
        } catch (NumberFormatException e) {
            return 0;
        }

        int sys;
        switch (first) {
            case 'G': sys = SYS_GPS; prn += MINPRNGPS - 1; break;
            case 'R': sys = SYS_GLO; prn += MINPRNGLO - 1; break;
            case 'E': sys = SYS_GAL; prn += MINPRNGAL - 1; break;
            case 'J': sys = SYS_QZS; prn += MINPRNQZS - 1; break;
            case 'C': sys = SYS_CMP; prn += MINPRNCMP - 1; break;
            case 'I': sys = SYS_IRN; prn += MINPRNIRN - 1; break;
            case 'L': sys = SYS_LEO; prn += MINPRNLEO - 1; break;
            case 'S': sys = SYS_SBS; prn += 100;           break;
            default:  return 0;
        }
        return satno(sys, prn);
    }

    /**
     * Test whether a satellite should be excluded from processing.
     * <p>
     * A satellite is excluded if:
     * <ul>
     *   <li>svh &lt; 0 (ephemeris unavailable)</li>
     *   <li>opt.exsats[sat-1] == 1 (explicitly excluded)</li>
     *   <li>opt.exsats[sat-1] == 2 forces inclusion (returns false immediately)</li>
     *   <li>satellite system not in opt.navsys</li>
     *   <li>satellite is unhealthy (svh != 0, with QZSS LEX mask and GLONASS-specific logic)</li>
     *   <li>ephemeris variance exceeds MAX_VAR_EPH</li>
     * </ul>
     *
     * @param sat satellite number (1 to MAXSAT)
     * @param var variance of ephemeris (m^2)
     * @param svh SV health flag (-1 = ephemeris unavailable)
     * @param opt processing options, or null if not used.
     *            Must provide fields: exsats (byte[MAXSAT]), navsys (int).
     * @return true if the satellite should be excluded
     */
    public static boolean satexclude(int sat, double var, int svh, com.gnss.rtklib.model.ProcessingOptions opt) {
        int[] sp = satsys(sat);
        int sys = sp[0];

        if (svh < 0) return true; // ephemeris unavailable

        if (opt != null) {
            if (opt.exsats[sat - 1] == 1) return true;  // excluded satellite
            if (opt.exsats[sat - 1] == 2) return false;  // included satellite
            if ((sys & opt.navsys) == 0) return true;     // unselected sat sys
        }

        int healthCheck = svh;
        if (sys == SYS_QZS) healthCheck &= 0xFE; // mask QZSS LEX health

        if (sys == SYS_GLO) {
            // GLONASS health: unhealthy if bits 0 or 3 set, or bits 1-2 == 0b10
            if ((healthCheck & 9) != 0 || (healthCheck & 6) == 4) {
                return true;
            }
        } else if (healthCheck != 0) {
            return true;
        }

        if (var > MAX_VAR_EPH) {
            return true;
        }
        return false;
    }

}
