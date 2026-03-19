package com.gnss.rtklib.positioning;

import com.gnss.rtklib.core.GTime;
import com.gnss.rtklib.core.MatrixUtil;
import com.gnss.rtklib.model.Navigation;
import com.gnss.rtklib.model.PrecClk;
import com.gnss.rtklib.model.PrecEph;

import static com.gnss.rtklib.core.Constants.*;

/**
 * Precise ephemeris interpolation and satellite position/clock computation.
 * Ported from RTKLIB preceph.c pephpos()+pephclk()+peph2pos().
 */
public final class PreciseEphemeris {

    private PreciseEphemeris() {}

    /** Polynomial interpolation order */
    private static final int NMAX = 10;

    /** Max time difference to ephemeris epoch (s) */
    private static final double MAXDTE = 900.0;

    /** Extrapolation error for clock (m/s) */
    private static final double EXTERR_CLK = 1E-3;

    /** Extrapolation error for ephemeris (m/s^2) */
    private static final double EXTERR_EPH = 5E-7;

    /**
     * Compute satellite position/clock from precise ephemeris.
     *
     * @param time time (GPST)
     * @param sat  satellite number (1-MAXSAT)
     * @param nav  navigation data with peph/pclk
     * @param opt  0: center of mass, 1: antenna phase center (not implemented)
     * @param rs   output: position/velocity ECEF {x,y,z,vx,vy,vz} (m, m/s)
     * @param dts  output: clock {bias, drift} (s, s/s)
     * @param var  output: variance (m^2), 1-element array (may be null)
     * @return 1 on success, 0 on error
     */
    public static int peph2pos(GTime time, int sat, Navigation nav, int opt,
                                double[] rs, double[] dts, double[] var) {
        if (sat <= 0 || sat > MAXSAT) return 0;

        double[] rss = new double[3], rst = new double[3];
        double[] dtss = new double[1], dtst = new double[1];
        double[] vare = new double[1], varc = new double[1];
        double tt = 1E-3;

        // Position and clock at time
        if (!pephpos(time, sat, nav, rss, dtss, vare, varc)) return 0;
        if (!pephclk(time, sat, nav, dtss, varc)) return 0;

        // Position and clock at time + dt (for velocity/drift)
        GTime time_tt = time.timeadd(tt);
        if (!pephpos(time_tt, sat, nav, rst, dtst, null, null)) return 0;
        if (!pephclk(time_tt, sat, nav, dtst, null)) return 0;

        for (int i = 0; i < 3; i++) {
            rs[i] = rss[i];
            rs[i + 3] = (rst[i] - rss[i]) / tt;
        }

        // Relativistic effect correction: -2*(rs·vs)/c^2
        if (dtss[0] != 0.0) {
            dts[0] = dtss[0] - 2.0 * MatrixUtil.dot(rs, 0, rs, 3, 3) / (CLIGHT * CLIGHT);
            dts[1] = (dtst[0] - dtss[0]) / tt;
        } else {
            dts[0] = dts[1] = 0.0;
        }

        if (var != null) var[0] = vare[0] + varc[0];

        return 1;
    }

    /**
     * Satellite position by precise ephemeris using Neville interpolation.
     */
    static boolean pephpos(GTime time, int sat, Navigation nav,
                            double[] rs, double[] dts,
                            double[] vare, double[] varc) {
        int ne = nav.peph.size();
        rs[0] = rs[1] = rs[2] = 0.0;
        dts[0] = 0.0;

        if (ne < NMAX + 1) return false;
        if (time.timediff(nav.peph.get(0).time) < -MAXDTE) return false;
        if (time.timediff(nav.peph.get(ne - 1).time) > MAXDTE) return false;

        // Binary search
        int lo = 0, hi = ne - 1;
        while (lo < hi) {
            int mid = (lo + hi) / 2;
            if (nav.peph.get(mid).time.timediff(time) < 0.0) lo = mid + 1;
            else hi = mid;
        }
        int index = lo <= 0 ? 0 : lo - 1;

        // Select interpolation window
        int iStart = index - (NMAX + 1) / 2;
        if (iStart < 0) iStart = 0;
        else if (iStart + NMAX >= ne) iStart = ne - NMAX - 1;

        double[] t = new double[NMAX + 1];
        double[][] p = new double[3][NMAX + 1];

        for (int j = 0; j <= NMAX; j++) {
            t[j] = nav.peph.get(iStart + j).time.timediff(time);
            double[] pos = nav.peph.get(iStart + j).pos[sat - 1];
            if (norm3(pos) <= 0.0) return false;
        }

        // Earth rotation correction before interpolation
        for (int j = 0; j <= NMAX; j++) {
            double[] pos = nav.peph.get(iStart + j).pos[sat - 1];
            double sinl = Math.sin(OMGE * t[j]);
            double cosl = Math.cos(OMGE * t[j]);
            p[0][j] = cosl * pos[0] - sinl * pos[1];
            p[1][j] = sinl * pos[0] + cosl * pos[1];
            p[2][j] = pos[2];
        }

        // Neville interpolation
        for (int i = 0; i < 3; i++) {
            rs[i] = interppol(t, p[i], NMAX + 1);
        }

        // Variance
        if (vare != null) {
            double[] s = new double[3];
            for (int i = 0; i < 3; i++) {
                s[i] = nav.peph.get(index).std[sat - 1][i];
            }
            double std = norm3(s);
            if (t[0] > 0.0) std += EXTERR_EPH * t[0] * t[0] / 2.0;
            else if (t[NMAX] < 0.0) std += EXTERR_EPH * t[NMAX] * t[NMAX] / 2.0;
            vare[0] = std * std;
        }

        // Linear interpolation for clock from SP3 (used only if no CLK file)
        double t0 = time.timediff(nav.peph.get(index).time);
        double t1 = time.timediff(nav.peph.get(index + 1).time);
        double c0 = nav.peph.get(index).pos[sat - 1][3];
        double c1 = nav.peph.get(index + 1).pos[sat - 1][3];

        if (t0 <= 0.0) {
            dts[0] = c0;
            if (varc != null && c0 != 0.0) {
                double std = nav.peph.get(index).std[sat - 1][3] * CLIGHT - EXTERR_CLK * t0;
                varc[0] = std * std;
            }
        } else if (t1 >= 0.0) {
            dts[0] = c1;
            if (varc != null && c1 != 0.0) {
                double std = nav.peph.get(index + 1).std[sat - 1][3] * CLIGHT + EXTERR_CLK * t1;
                varc[0] = std * std;
            }
        } else if (c0 != 0.0 && c1 != 0.0) {
            dts[0] = (c1 * t0 - c0 * t1) / (t0 - t1);
            if (varc != null) {
                int ci = t0 < -t1 ? 0 : 1;
                double std = nav.peph.get(index + ci).std[sat - 1][3]
                           + EXTERR_CLK * Math.abs(ci == 0 ? t0 : t1);
                varc[0] = std * std;
            }
        } else {
            dts[0] = 0.0;
        }

        return true;
    }

    /**
     * Satellite clock by precise clock file (overrides SP3 clock).
     */
    static boolean pephclk(GTime time, int sat, Navigation nav,
                            double[] dts, double[] varc) {
        int nc = nav.pclk.size();
        if (nc < 2) return true; // No CLK file → use SP3 clock from pephpos

        if (time.timediff(nav.pclk.get(0).time) < -MAXDTE) return true;
        if (time.timediff(nav.pclk.get(nc - 1).time) > MAXDTE) return true;

        // Binary search
        int lo = 0, hi = nc - 1;
        while (lo < hi) {
            int mid = (lo + hi) / 2;
            if (nav.pclk.get(mid).time.timediff(time) < 0.0) lo = mid + 1;
            else hi = mid;
        }
        int index = lo <= 0 ? 0 : lo - 1;

        // Linear interpolation
        double t0 = time.timediff(nav.pclk.get(index).time);
        double t1 = time.timediff(nav.pclk.get(index + 1).time);
        double c0 = nav.pclk.get(index).clk[sat - 1];
        double c1 = nav.pclk.get(index + 1).clk[sat - 1];

        double std;
        if (t0 <= 0.0) {
            if (c0 == 0.0) return false;
            dts[0] = c0;
            std = nav.pclk.get(index).std[sat - 1] * CLIGHT - EXTERR_CLK * t0;
        } else if (t1 >= 0.0) {
            if (c1 == 0.0) return false;
            dts[0] = c1;
            std = nav.pclk.get(index + 1).std[sat - 1] * CLIGHT + EXTERR_CLK * t1;
        } else if (c0 != 0.0 && c1 != 0.0) {
            dts[0] = (c1 * t0 - c0 * t1) / (t0 - t1);
            int ci = t0 < -t1 ? 0 : 1;
            std = nav.pclk.get(index + ci).std[sat - 1] * CLIGHT
                + EXTERR_CLK * Math.abs(ci == 0 ? t0 : t1);
        } else {
            return false;
        }

        if (varc != null) varc[0] = std * std;
        return true;
    }

    /**
     * Polynomial interpolation by Neville's algorithm.
     */
    static double interppol(double[] x, double[] y, int n) {
        double[] yy = new double[n];
        System.arraycopy(y, 0, yy, 0, n);
        for (int j = 1; j < n; j++) {
            for (int i = 0; i < n - j; i++) {
                yy[i] = (x[i + j] * yy[i] - x[i] * yy[i + 1]) / (x[i + j] - x[i]);
            }
        }
        return yy[0];
    }

    private static double norm3(double[] v) {
        return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }
}
