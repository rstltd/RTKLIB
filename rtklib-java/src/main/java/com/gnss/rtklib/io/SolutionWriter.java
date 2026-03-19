package com.gnss.rtklib.io;

import com.gnss.rtklib.core.Coord;
import com.gnss.rtklib.core.GTime;
import com.gnss.rtklib.model.ProcessingOptions;
import com.gnss.rtklib.model.Solution;
import com.gnss.rtklib.model.SolutionOptions;

import java.io.PrintWriter;

import static com.gnss.rtklib.core.Constants.*;

/**
 * Solution output writer for .pos file format.
 * Ported from RTKLIB solution.c (outsolhead, outsol, outpos, outecef).
 */
public final class SolutionWriter {

    /** Solution quality labels */
    private static final String[] SOLQ_LABELS = {
        "none", "fix", "float", "sbas", "dgps", "single", "ppp", "dr"
    };

    /** Positioning mode labels */
    private static final String[] MODE_LABELS = {
        "Single", "DGPS", "Kinematic", "Static", "Static-Start", "Moving-Base",
        "Fixed", "PPP Kinematic", "PPP Static", "PPP Fixed"
    };

    /** Ionosphere option labels */
    private static final String[] ION_LABELS = {
        "OFF", "Broadcast", "SBAS", "Iono-Free LC", "Estimate TEC", "IONEX TEC",
        "QZSS Broadcast"
    };

    /** Troposphere option labels */
    private static final String[] TROP_LABELS = {
        "OFF", "Saastamoinen", "SBAS", "Estimate ZTD", "Estimate ZTD+Grad"
    };

    /** Solution format labels */
    private static final String[] SOLF_LABELS = {
        "Lat/Lon/Height", "X/Y/Z-ECEF", "E/N/U-Baseline", "NMEA", "Stat", "GSIF"
    };

    /** Navigation system labels */
    private static final int[] NAV_SYS = {
        SYS_GPS, SYS_GLO, SYS_GAL, SYS_QZS, SYS_CMP, SYS_IRN, SYS_SBS
    };
    private static final String[] NAV_SYS_LABELS = {
        "GPS", "GLONASS", "Galileo", "QZSS", "BDS", "NavIC", "SBAS"
    };

    /** Time system labels */
    private static final String[] TIME_LABELS = { "GPST", "UTC ", "JST " };

    private SolutionWriter() {}

    /**
     * Write the header of a .pos file including processing options and column headers.
     *
     * @param out  output writer
     * @param popt processing options (may be null for minimal header)
     * @param sopt solution output options
     */
    public static void writeHeader(PrintWriter out, ProcessingOptions popt,
                                   SolutionOptions sopt) {
        String prog = sopt.prog.isEmpty() ? "rtklib-java" : sopt.prog;
        out.printf("%% program          : %s%n", prog);

        if (popt != null) {
            writeProcOptions(out, popt);
        }

        out.printf("%% solution format  : %s%n", safeLabel(SOLF_LABELS, sopt.posf));
        out.println("%");

        writeColumnHeader(out, sopt);
    }

    /**
     * Write one epoch's solution to the output.
     *
     * @param out  output writer
     * @param sol  solution for this epoch
     * @param rb   base station position {x,y,z} ECEF (m), may be null
     * @param sopt solution output options
     */
    public static void writeSolution(PrintWriter out, Solution sol, double[] rb,
                                     SolutionOptions sopt) {
        // Skip if no solution
        if (sol.stat <= SOLQ_NONE) return;

        // Suppress if std exceeds threshold
        if (sopt.maxsolstd > 0.0 && solStd(sol) > sopt.maxsolstd) return;

        String sep = getSeparator(sopt);
        int timeu = Math.max(0, Math.min(sopt.timeu, 20));
        String timeStr = formatTime(sol.time, sopt, timeu);

        switch (sopt.posf) {
            case SOLF_LLH:
                writePos(out, timeStr, sol, sopt, sep, timeu);
                break;
            case SOLF_XYZ:
                writeEcef(out, timeStr, sol, sopt, sep);
                break;
            default:
                writePos(out, timeStr, sol, sopt, sep, timeu);
                break;
        }
    }

    // -----------------------------------------------------------------------
    // Processing options output
    // -----------------------------------------------------------------------

    private static void writeProcOptions(PrintWriter out, ProcessingOptions popt) {
        out.printf("%% pos mode         : %s%n", safeLabel(MODE_LABELS, popt.mode));
        out.printf("%% elev mask        : %.1f deg%n", popt.elmin * R2D);
        out.printf("%% ionos opt        : %s%n", safeLabel(ION_LABELS, popt.ionoopt));
        out.printf("%% tropo opt        : %s%n", safeLabel(TROP_LABELS, popt.tropopt));

        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < NAV_SYS.length; i++) {
            if ((popt.navsys & NAV_SYS[i]) != 0) {
                if (sb.length() > 0) sb.append(' ');
                sb.append(NAV_SYS_LABELS[i]);
            }
        }
        out.printf("%% navi sys         : %s%n", sb.toString());
    }

    // -----------------------------------------------------------------------
    // Column header
    // -----------------------------------------------------------------------

    private static void writeColumnHeader(PrintWriter out, SolutionOptions sopt) {
        String sep = getSeparator(sopt);
        int timeu = Math.max(0, Math.min(sopt.timeu, 20));
        String timeLabel = safeLabel(TIME_LABELS, sopt.times);

        if (sopt.posf == SOLF_LLH) {
            // time width: timef=1 -> 16+timeu+1, timef=0 -> 8+timeu+1
            int tw = (sopt.timef != 0 ? 16 : 8) + timeu + 1;
            out.printf("%% %s", padRight(timeLabel, tw));
            out.printf("%s%14s%s%14s%s%10s%s%3s%s%3s%s%8s%s%8s%s%8s%s%8s%s%8s%s%8s%s%6s%s%6s%n",
                    sep, "latitude(deg)", sep, "longitude(deg)", sep, "height(m)",
                    sep, "Q", sep, "ns",
                    sep, "sdn(m)", sep, "sde(m)", sep, "sdu(m)",
                    sep, "sdne(m)", sep, "sdeu(m)", sep, "sdun(m)",
                    sep, "age(s)", sep, "ratio");
        } else if (sopt.posf == SOLF_XYZ) {
            int tw = (sopt.timef != 0 ? 16 : 8) + timeu + 1;
            out.printf("%% %s", padRight(timeLabel, tw));
            out.printf("%s%14s%s%14s%s%14s%s%3s%s%3s%s%8s%s%8s%s%8s%s%8s%s%8s%s%8s%s%6s%s%6s%n",
                    sep, "x-ecef(m)", sep, "y-ecef(m)", sep, "z-ecef(m)",
                    sep, "Q", sep, "ns",
                    sep, "sdx(m)", sep, "sdy(m)", sep, "sdz(m)",
                    sep, "sdxy(m)", sep, "sdyz(m)", sep, "sdzx(m)",
                    sep, "age(s)", sep, "ratio");
        }
    }

    // -----------------------------------------------------------------------
    // LLH output (outpos)
    // -----------------------------------------------------------------------

    private static void writePos(PrintWriter out, String timeStr, Solution sol,
                                 SolutionOptions sopt, String sep, int timeu) {
        double[] pos = Coord.ecef2pos(sol.rr);

        // Build full 3x3 ECEF covariance from packed sol.qr
        double[] P = soltocov(sol);

        // Rotate to ENU
        double[] Q = new double[9];
        Coord.covenu(pos, P, Q);

        double latDeg = pos[0] * R2D;
        double lonDeg = pos[1] * R2D;
        double height = pos[2];

        // Standard deviations in ENU order: Q[0]=ee, Q[4]=nn, Q[8]=uu
        // off-diag: Q[1]=Q[3]=en, Q[2]=Q[6]=eu, Q[5]=Q[7]=nu
        double sdn  = sqrtSafe(Q[4]);  // north
        double sde  = sqrtSafe(Q[0]);  // east
        double sdu  = sqrtSafe(Q[8]);  // up
        double sdne = sqvar(Q[1]);     // north-east
        double sdeu = sqvar(Q[2]);     // east-up
        double sdun = sqvar(Q[5]);     // up-north

        out.printf("%s%s%14.9f%s%14.9f%s%10.4f%s%3d%s%3d%s%8.4f%s%8.4f%s%8.4f" +
                   "%s%8.4f%s%8.4f%s%8.4f%s%6.2f%s%6.1f%n",
                   timeStr, sep, latDeg, sep, lonDeg, sep, height,
                   sep, sol.stat, sep, sol.ns,
                   sep, sdn, sep, sde, sep, sdu,
                   sep, sdne, sep, sdeu, sep, sdun,
                   sep, (double) sol.age, sep, (double) sol.ratio);
    }

    // -----------------------------------------------------------------------
    // XYZ output (outecef)
    // -----------------------------------------------------------------------

    private static void writeEcef(PrintWriter out, String timeStr, Solution sol,
                                  SolutionOptions sopt, String sep) {
        out.printf("%s%s%14.4f%s%14.4f%s%14.4f%s%3d%s%3d%s%8.4f%s%8.4f%s%8.4f" +
                   "%s%8.4f%s%8.4f%s%8.4f%s%6.2f%s%6.1f%n",
                   timeStr, sep, sol.rr[0], sep, sol.rr[1], sep, sol.rr[2],
                   sep, sol.stat, sep, sol.ns,
                   sep, sqrtSafe(sol.qr[0]), sep, sqrtSafe(sol.qr[1]),
                   sep, sqrtSafe(sol.qr[2]),
                   sep, sqvar(sol.qr[3]), sep, sqvar(sol.qr[4]),
                   sep, sqvar(sol.qr[5]),
                   sep, (double) sol.age, sep, (double) sol.ratio);
    }

    // -----------------------------------------------------------------------
    // Time formatting
    // -----------------------------------------------------------------------

    private static String formatTime(GTime time, SolutionOptions sopt, int timeu) {
        GTime t = time;

        // Convert time system
        if (sopt.times >= TIMES_UTC) t = t.gpst2utc();
        if (sopt.times == TIMES_JST) t = t.add(9 * 3600.0);

        if (sopt.timef != 0) {
            // Calendar format: yyyy/mm/dd hh:mm:ss.sss
            return t.format(timeu);
        } else {
            // GPS week + tow format
            double[] wt = t.time2gpst();
            int week = (int) wt[0];
            double tow = wt[1];
            if (86400.0 * 7 - tow < 0.5 / Math.pow(10.0, timeu)) {
                week++;
                tow = 0.0;
            }
            if (timeu <= 0) {
                return String.format("%4d %6.0f", week, tow);
            } else {
                return String.format("%4d %*.*f", week, 6 + timeu + 1, timeu, tow);
            }
        }
    }

    // -----------------------------------------------------------------------
    // Covariance helpers
    // -----------------------------------------------------------------------

    /**
     * Expand packed covariance sol.qr[6] = {xx,yy,zz,xy,yz,zx} to 3x3 column-major.
     */
    private static double[] soltocov(Solution sol) {
        double[] P = new double[9];
        P[0]     = sol.qr[0]; // xx
        P[4]     = sol.qr[1]; // yy
        P[8]     = sol.qr[2]; // zz
        P[1] = P[3] = sol.qr[3]; // xy
        P[5] = P[7] = sol.qr[4]; // yz
        P[2] = P[6] = sol.qr[5]; // zx
        return P;
    }

    /**
     * Safe square root: returns 0 for negative or NaN values.
     * Matches C RTKLIB's SQRT macro.
     */
    private static double sqrtSafe(double v) {
        return (v < 0.0 || v != v) ? 0.0 : Math.sqrt(v);
    }

    /**
     * Signed square root of covariance: preserves sign.
     * Matches C RTKLIB's sqvar function.
     */
    private static double sqvar(double covar) {
        return covar < 0.0 ? -Math.sqrt(-covar) : Math.sqrt(covar);
    }

    /**
     * Max standard deviation across 3 axes.
     * Matches C RTKLIB's sol_std function.
     */
    private static double solStd(Solution sol) {
        if (sol.qr[0] > sol.qr[1] && sol.qr[0] > sol.qr[2]) return sqrtSafe(sol.qr[0]);
        if (sol.qr[1] > sol.qr[2]) return sqrtSafe(sol.qr[1]);
        return sqrtSafe(sol.qr[2]);
    }

    // -----------------------------------------------------------------------
    // String helpers
    // -----------------------------------------------------------------------

    private static String getSeparator(SolutionOptions sopt) {
        if (sopt.separator == null || sopt.separator.isEmpty()) return " ";
        if (sopt.separator.equals("\\t")) return "\t";
        return sopt.separator;
    }

    private static String safeLabel(String[] labels, int index) {
        if (index < 0 || index >= labels.length) return "unknown";
        return labels[index];
    }

    private static String padRight(String s, int width) {
        if (s.length() >= width) return s;
        StringBuilder sb = new StringBuilder(s);
        while (sb.length() < width) sb.append(' ');
        return sb.toString();
    }
}
