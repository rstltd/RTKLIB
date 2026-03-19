/*------------------------------------------------------------------------------
 * EphemerisCalc.java : satellite ephemeris and clock functions
 *
 *          Copyright (C) 2010-2020 by T.TAKASU, All rights reserved.
 *          Java port Copyright (C) 2026
 *
 * references :
 *     [1] IS-GPS-200K, Navstar GPS Space Segment/Navigation User Interfaces
 *     [2] GLONASS ICD, Navigational radiosignal In bands L1, L2 (Version 5.1)
 *     [7] Galileo OS SIS ICD, Issue 1.3, December 2016
 *     [8] IS-QZSS-PNT-003, November 5, 2018
 *     [9] BeiDou ICD open service signal B1I (version 3.0)
 *
 * Licensed under BSD 2-clause license.
 *-----------------------------------------------------------------------------*/
package com.gnss.rtklib.positioning;

import com.gnss.rtklib.core.Constants;
import com.gnss.rtklib.core.GTime;
import com.gnss.rtklib.core.SatelliteUtil;
import com.gnss.rtklib.model.Ephemeris;
import com.gnss.rtklib.model.GloEphemeris;
import com.gnss.rtklib.model.Navigation;
import com.gnss.rtklib.model.ObsData;

import static com.gnss.rtklib.core.Constants.*;

/**
 * Satellite position and clock computation from broadcast ephemeris.
 * <p>
 * Ported from RTKLIB ephemeris.c.
 */
public final class EphemerisCalc {

    private EphemerisCalc() {}

    // ---- physical constants ------------------------------------------------

    /** Earth radius for GLONASS (m) ref [2] */
    private static final double RE_GLO = 6378136.0;

    /** Gravitational constant GPS (m^3/s^2) ref [1] */
    private static final double MU_GPS = 3.9860050E14;

    /** Gravitational constant GLONASS (m^3/s^2) ref [2] */
    private static final double MU_GLO = 3.9860044E14;

    /** Gravitational constant Galileo (m^3/s^2) ref [7] */
    private static final double MU_GAL = 3.986004418E14;

    /** Gravitational constant BeiDou (m^3/s^2) ref [9] */
    private static final double MU_CMP = 3.986004418E14;

    /** 2nd zonal harmonic of geopotential ref [2] */
    private static final double J2_GLO = 1.0826257E-3;

    /** Earth angular velocity GLONASS (rad/s) ref [2] */
    private static final double OMGE_GLO = 7.292115E-5;

    /** Earth angular velocity Galileo (rad/s) ref [7] */
    private static final double OMGE_GAL = 7.2921151467E-5;

    /** Earth angular velocity BeiDou (rad/s) ref [9] */
    private static final double OMGE_CMP = 7.292115E-5;

    /** sin(-5.0 deg) for BDS GEO rotation */
    private static final double SIN_5 = -0.0871557427476582;

    /** cos(-5.0 deg) for BDS GEO rotation */
    private static final double COS_5 = 0.9961946980917456;

    /** Error of GLONASS ephemeris (m) */
    private static final double ERREPH_GLO = 5.0;

    /** Integration step for GLONASS ephemeris (s) */
    private static final double TSTEP = 60.0;

    /** Relative tolerance for Kepler equation */
    private static final double RTOL_KEPLER = 1E-13;

    /** Max number of Kepler iterations */
    private static final int MAX_ITER_KEPLER = 30;

    /** Error of broadcast clock (m) */
    private static final double STD_BRDCCLK = 30.0;

    /** Error of Galileo ephemeris for NAPA (m) */
    private static final double STD_GAL_NAPA = 500.0;

    // ---- max time differences to Toe (s) -----------------------------------

    private static final double MAXDTOE     = 7200.0;
    private static final double MAXDTOE_QZS = 7200.0;
    private static final double MAXDTOE_GAL = 14400.0;
    private static final double MAXDTOE_CMP = 21600.0;
    private static final double MAXDTOE_GLO = 1800.0;
    private static final double MAXDTOE_IRN = 7200.0;
    private static final double MAXDTOE_S   = 86400.0;

    /** Max number of observations in an epoch */
    private static final int MAXOBS = 96;

    // ---- ephemeris selection (global state, matches C static array) ---------
    // GPS, GLO, GAL, QZS, BDS, IRN, SBS
    private static final int[] ephSel = {0, 0, 0, 0, 0, 0, 0};

    // ========================================================================
    // URA variance
    // ========================================================================

    /** URA value table for GPS (ref [1] 20.3.3.3.1.1) */
    private static final double[] URA_VALUES = {
        2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0, 96.0, 192.0,
        384.0, 768.0, 1536.0, 3072.0, 6144.0
    };

    /**
     * Variance by URA ephemeris index.
     *
     * @param sys satellite system
     * @param ura URA index
     * @return variance (m^2)
     */
    static double varUraEph(int sys, int ura) {
        if (sys == SYS_GAL) {
            /* Galileo SISA (ref [7] 5.1.11) */
            if (ura <=  49) return sq(ura * 0.01);
            if (ura <=  74) return sq(0.5 + (ura - 50) * 0.02);
            if (ura <=  99) return sq(1.0 + (ura - 75) * 0.04);
            if (ura <= 125) return sq(2.0 + (ura - 100) * 0.16);
            return sq(STD_GAL_NAPA);
        } else {
            /* GPS URA (ref [1] 20.3.3.3.1.1) */
            return (ura < 0 || 14 < ura) ? sq(6144.0) : sq(URA_VALUES[ura]);
        }
    }

    // ========================================================================
    // Clock corrections
    // ========================================================================

    /**
     * Satellite clock correction from broadcast ephemeris (GPS/GAL/QZS/BDS/IRN).
     * <p>
     * Does not include relativity correction or TGD.
     *
     * @param time time by satellite clock (GPST)
     * @param eph  broadcast ephemeris
     * @return satellite clock bias (s)
     */
    public static double eph2clk(GTime time, Ephemeris eph) {
        double ts = time.timediff(eph.toc);
        double t = ts;

        for (int i = 0; i < 2; i++) {
            t = ts - (eph.f0 + eph.f1 * t + eph.f2 * t * t);
        }
        return eph.f0 + eph.f1 * t + eph.f2 * t * t;
    }

    /**
     * GLONASS satellite clock correction from broadcast ephemeris.
     *
     * @param time time by satellite clock (GPST)
     * @param geph GLONASS ephemeris
     * @return satellite clock bias (s)
     */
    public static double geph2clk(GTime time, GloEphemeris geph) {
        double ts = time.timediff(geph.toe);
        double t = ts;

        for (int i = 0; i < 2; i++) {
            t = ts - (-geph.taun + geph.gamn * t);
        }
        return -geph.taun + geph.gamn * t;
    }

    // ========================================================================
    // Position and clock from Kepler orbit (GPS/GAL/QZS/BDS/IRN)
    // ========================================================================

    /**
     * Compute satellite position, clock bias, and variance from broadcast ephemeris.
     * <p>
     * Satellite clock includes relativistic correction but not code bias (TGD/BGD).
     *
     * @param time time (GPST)
     * @param eph  broadcast ephemeris
     * @param rs   output: satellite position and velocity ECEF (6 elements:
     *             x,y,z in m; vx,vy,vz set to 0 here -- caller computes velocity
     *             by numerical differentiation)
     * @param dts  output: satellite clock {bias (s), drift (s/s)} (2 elements)
     * @param var  output: satellite position and clock variance (m^2) (1 element)
     */
    public static void eph2pos(GTime time, Ephemeris eph, double[] rs,
                               double[] dts, double[] var) {
        if (eph.A <= 0.0) {
            rs[0] = rs[1] = rs[2] = 0.0;
            dts[0] = 0.0;
            var[0] = 0.0;
            return;
        }

        double tk = time.timediff(eph.toe);

        int[] sp = SatelliteUtil.satsys(eph.sat);
        int sys = sp[0];
        int prn = sp[1];

        double mu, omge;
        switch (sys) {
            case SYS_GAL: mu = MU_GAL; omge = OMGE_GAL; break;
            case SYS_CMP: mu = MU_CMP; omge = OMGE_CMP; break;
            default:      mu = MU_GPS; omge = OMGE;      break;
        }

        double M = eph.M0 + (Math.sqrt(mu / (eph.A * eph.A * eph.A)) + eph.deln) * tk;

        /* solve Kepler equation M = E - e*sin(E) by Newton's method */
        double E = M, Ek = 0.0;
        int n;
        for (n = 0; Math.abs(E - Ek) > RTOL_KEPLER && n < MAX_ITER_KEPLER; n++) {
            Ek = E;
            E -= (E - eph.e * Math.sin(E) - M) / (1.0 - eph.e * Math.cos(E));
        }

        double sinE = Math.sin(E);
        double cosE = Math.cos(E);

        /* true anomaly + argument of perigee */
        double u = Math.atan2(Math.sqrt(1.0 - eph.e * eph.e) * sinE, cosE - eph.e) + eph.omg;
        double r = eph.A * (1.0 - eph.e * cosE);
        double i = eph.i0 + eph.idot * tk;

        /* perturbation corrections */
        double sin2u = Math.sin(2.0 * u);
        double cos2u = Math.cos(2.0 * u);
        u += eph.cus * sin2u + eph.cuc * cos2u;
        r += eph.crs * sin2u + eph.crc * cos2u;
        i += eph.cis * sin2u + eph.cic * cos2u;

        double x = r * Math.cos(u);
        double y = r * Math.sin(u);
        double cosi = Math.cos(i);

        /* BeiDou GEO satellite (PRN 1-5 or 59-63): ref [9] table 4-1 */
        if (sys == SYS_CMP && (prn <= 5 || prn >= 59)) {
            double O = eph.OMG0 + eph.OMGd * tk - omge * eph.toes;
            double sinO = Math.sin(O), cosO = Math.cos(O);
            double xg = x * cosO - y * cosi * sinO;
            double yg = x * sinO + y * cosi * cosO;
            double zg = y * Math.sin(i);
            double sino = Math.sin(omge * tk), coso = Math.cos(omge * tk);
            rs[0] =  xg * coso + yg * sino * COS_5 + zg * sino * SIN_5;
            rs[1] = -xg * sino + yg * coso * COS_5 + zg * coso * SIN_5;
            rs[2] = -yg * SIN_5 + zg * COS_5;
        } else {
            double O = eph.OMG0 + (eph.OMGd - omge) * tk - omge * eph.toes;
            double sinO = Math.sin(O), cosO = Math.cos(O);
            rs[0] = x * cosO - y * cosi * sinO;
            rs[1] = x * sinO + y * cosi * cosO;
            rs[2] = y * Math.sin(i);
        }

        /* satellite clock bias (with relativity correction) */
        tk = time.timediff(eph.toc);
        dts[0] = eph.f0 + eph.f1 * tk + eph.f2 * tk * tk;

        /* relativity correction: -2*sqrt(mu*A)*e*sin(E) / c^2 */
        dts[0] -= 2.0 * Math.sqrt(mu * eph.A) * eph.e * sinE / (CLIGHT * CLIGHT);

        /* position and clock error variance */
        var[0] = varUraEph(sys, eph.sva);
    }

    // ========================================================================
    // GLONASS orbit (Runge-Kutta integration)
    // ========================================================================

    /**
     * GLONASS orbit differential equations.
     * <p>
     * Computes state derivatives for position/velocity with J2 perturbation
     * and luni-solar acceleration.
     *
     * @param x    state vector {x,y,z,vx,vy,vz} (m, m/s)
     * @param xdot output derivative {vx,vy,vz,ax,ay,az}
     * @param acc  luni-solar acceleration {ax,ay,az} (m/s^2)
     */
    static void deq(double[] x, double[] xdot, double[] acc) {
        double r2 = x[0] * x[0] + x[1] * x[1] + x[2] * x[2];
        double r3 = r2 * Math.sqrt(r2);
        double omg2 = OMGE_GLO * OMGE_GLO;

        if (r2 <= 0.0) {
            for (int i = 0; i < 6; i++) xdot[i] = 0.0;
            return;
        }

        /* 3/2 * J2 * mu * Ae^2 / r^5 */
        double a = 1.5 * J2_GLO * MU_GLO * RE_GLO * RE_GLO / r2 / r3;
        double b = 5.0 * x[2] * x[2] / r2;         /* 5*z^2/r^2 */
        double c = -MU_GLO / r3 - a * (1.0 - b);    /* -mu/r^3 - a(1-b) */

        xdot[0] = x[3];
        xdot[1] = x[4];
        xdot[2] = x[5];
        xdot[3] = (c + omg2) * x[0] + 2.0 * OMGE_GLO * x[4] + acc[0];
        xdot[4] = (c + omg2) * x[1] - 2.0 * OMGE_GLO * x[3] + acc[1];
        xdot[5] = (c - 2.0 * a) * x[2] + acc[2];
    }

    /**
     * GLONASS orbit propagation by 4th-order Runge-Kutta.
     *
     * @param t   integration step (s), signed
     * @param x   state vector {x,y,z,vx,vy,vz}, updated in place
     * @param acc luni-solar acceleration {ax,ay,az} (m/s^2)
     */
    static void glorbit(double t, double[] x, double[] acc) {
        double[] k1 = new double[6];
        double[] k2 = new double[6];
        double[] k3 = new double[6];
        double[] k4 = new double[6];
        double[] w  = new double[6];

        deq(x, k1, acc);
        for (int i = 0; i < 6; i++) w[i] = x[i] + k1[i] * t / 2.0;

        deq(w, k2, acc);
        for (int i = 0; i < 6; i++) w[i] = x[i] + k2[i] * t / 2.0;

        deq(w, k3, acc);
        for (int i = 0; i < 6; i++) w[i] = x[i] + k3[i] * t;

        deq(w, k4, acc);

        for (int i = 0; i < 6; i++) {
            x[i] += (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) * t / 6.0;
        }
    }

    /**
     * Compute GLONASS satellite position, clock bias, and variance.
     *
     * @param time time (GPST)
     * @param geph GLONASS ephemeris
     * @param rs   output: satellite position ECEF (m), 3+ elements
     *             (only rs[0..2] set; velocity computed by caller)
     * @param dts  output: satellite clock bias (s), 1+ elements
     * @param var  output: satellite position variance (m^2), 1 element
     */
    public static void geph2pos(GTime time, GloEphemeris geph, double[] rs,
                                double[] dts, double[] var) {
        double t = time.timediff(geph.toe);

        dts[0] = -geph.taun + geph.gamn * t;

        double[] x = new double[6];
        for (int i = 0; i < 3; i++) {
            x[i]     = geph.pos[i];
            x[i + 3] = geph.vel[i];
        }

        double tt = t < 0.0 ? -TSTEP : TSTEP;
        while (Math.abs(t) > 1E-9) {
            if (Math.abs(t) < TSTEP) tt = t;
            glorbit(tt, x, geph.acc);
            t -= tt;
        }

        for (int i = 0; i < 3; i++) rs[i] = x[i];

        var[0] = ERREPH_GLO * ERREPH_GLO;
    }

    // ========================================================================
    // Ephemeris selection
    // ========================================================================

    /**
     * Set ephemeris selection for a satellite system.
     *
     * @param sys satellite system (SYS_xxx)
     * @param sel selection index (system-dependent)
     */
    public static void setSelEph(int sys, int sel) {
        switch (sys) {
            case SYS_GPS: ephSel[0] = sel; break;
            case SYS_GLO: ephSel[1] = sel; break;
            case SYS_GAL: ephSel[2] = sel; break;
            case SYS_QZS: ephSel[3] = sel; break;
            case SYS_CMP: ephSel[4] = sel; break;
            case SYS_IRN: ephSel[5] = sel; break;
            case SYS_SBS: ephSel[6] = sel; break;
        }
    }

    /**
     * Get ephemeris selection for a satellite system.
     *
     * @param sys satellite system (SYS_xxx)
     * @return selection index
     */
    public static int getSelEph(int sys) {
        switch (sys) {
            case SYS_GPS: return ephSel[0];
            case SYS_GLO: return ephSel[1];
            case SYS_GAL: return ephSel[2];
            case SYS_QZS: return ephSel[3];
            case SYS_CMP: return ephSel[4];
            case SYS_IRN: return ephSel[5];
            case SYS_SBS: return ephSel[6];
            default:      return 0;
        }
    }

    /**
     * Select GPS/GAL/QZS/BDS/IRN broadcast ephemeris closest to time.
     *
     * @param time time (GPST)
     * @param sat  satellite number
     * @param iode IODE to match, or -1 for any
     * @param nav  navigation data
     * @return selected ephemeris, or null if not found
     */
    public static Ephemeris seleph(GTime time, int sat, int iode, Navigation nav) {
        int[] sp = SatelliteUtil.satsys(sat);
        int sys = sp[0];

        double tmax;
        switch (sys) {
            case SYS_GPS: tmax = MAXDTOE + 1.0;     break;
            case SYS_GAL: tmax = MAXDTOE_GAL;        break;
            case SYS_QZS: tmax = MAXDTOE_QZS + 1.0;  break;
            case SYS_CMP: tmax = MAXDTOE_CMP + 1.0;  break;
            case SYS_IRN: tmax = MAXDTOE_IRN + 1.0;  break;
            default:      tmax = MAXDTOE + 1.0;      break;
        }
        double tmin = tmax + 1.0;

        int bestIdx = -1;

        for (int i = 0; i < nav.eph.size(); i++) {
            Ephemeris e = nav.eph.get(i);
            if (e.sat != sat) continue;
            if (iode >= 0 && e.iode != iode) continue;

            if (sys == SYS_GAL) {
                int sel = getSelEph(SYS_GAL);
                if (sel == 1 && (e.code & (1 << 9)) == 0) continue; /* I/NAV */
                if (sel == 2 && (e.code & (1 << 8)) == 0) continue; /* F/NAV */
                if (time.timediff(e.toe) <= 0.0) continue; /* AOD <= 0 */
            }

            double t = Math.abs(e.toe.timediff(time));
            if (t > tmax) continue;
            if (iode >= 0) return e;
            if (t <= tmin) {
                bestIdx = i;
                tmin = t;
            }
        }

        if (iode >= 0 || bestIdx < 0) {
            return null;
        }
        return nav.eph.get(bestIdx);
    }

    /**
     * Select GLONASS broadcast ephemeris closest to time.
     *
     * @param time time (GPST)
     * @param sat  satellite number
     * @param iode IODE to match, or -1 for any
     * @param nav  navigation data
     * @return selected GLONASS ephemeris, or null if not found
     */
    public static GloEphemeris selgeph(GTime time, int sat, int iode, Navigation nav) {
        double tmax = MAXDTOE_GLO;
        double tmin = tmax + 1.0;
        int bestIdx = -1;

        for (int i = 0; i < nav.geph.size(); i++) {
            GloEphemeris ge = nav.geph.get(i);
            if (ge.sat != sat) continue;
            if (iode >= 0 && ge.iode != iode) continue;

            double t = Math.abs(ge.toe.timediff(time));
            if (t > tmax) continue;
            if (iode >= 0) return ge;
            if (t <= tmin) {
                bestIdx = i;
                tmin = t;
            }
        }

        if (iode >= 0 || bestIdx < 0) {
            return null;
        }
        return nav.geph.get(bestIdx);
    }

    // ========================================================================
    // Internal helpers: ephclk and ephpos
    // ========================================================================

    /**
     * Get satellite clock bias from broadcast ephemeris.
     *
     * @param time satellite time (GPST)
     * @param teph time for ephemeris selection (GPST)
     * @param sat  satellite number
     * @param nav  navigation data
     * @return clock bias (s), or {@code Double.NaN} if unavailable
     */
    private static double ephclk(GTime time, GTime teph, int sat, Navigation nav) {
        int sys = SatelliteUtil.satsys(sat)[0];

        if (sys == SYS_GPS || sys == SYS_GAL || sys == SYS_QZS ||
            sys == SYS_CMP || sys == SYS_IRN) {
            Ephemeris eph = seleph(teph, sat, -1, nav);
            if (eph == null) return Double.NaN;
            return eph2clk(time, eph);
        } else if (sys == SYS_GLO) {
            GloEphemeris geph = selgeph(teph, sat, -1, nav);
            if (geph == null) return Double.NaN;
            if (Math.abs(geph.taun) > 1) return Double.NaN; /* reject invalid data */
            return geph2clk(time, geph);
        }
        return Double.NaN;
    }

    /**
     * Compute satellite position and clock from broadcast ephemeris.
     * <p>
     * Velocity is computed by numerical differentiation (dt = 1 ms).
     *
     * @param time time (GPST)
     * @param teph time for ephemeris selection (GPST)
     * @param sat  satellite number
     * @param iode IODE to match, or -1 for any
     * @param nav  navigation data
     * @param rs   output: position and velocity ECEF {x,y,z,vx,vy,vz} (m, m/s)
     * @param dts  output: clock {bias, drift} (s, s/s)
     * @param var  output: variance (m^2)
     * @param svh  output: SV health [0] (-1 if unavailable)
     * @return true on success
     */
    private static boolean ephpos(GTime time, GTime teph, int sat, int iode,
                                  Navigation nav, double[] rs, double[] dts,
                                  double[] var, int[] svh) {
        int sys = SatelliteUtil.satsys(sat)[0];
        double tt = 1E-3;
        double[] rst = new double[3];
        double[] dtst = new double[1];

        svh[0] = -1;

        if (sys == SYS_GPS || sys == SYS_GAL || sys == SYS_QZS ||
            sys == SYS_CMP || sys == SYS_IRN) {
            Ephemeris eph = seleph(teph, sat, iode, nav);
            if (eph == null) return false;
            eph2pos(time, eph, rs, dts, var);
            GTime time2 = time.timeadd(tt);
            eph2pos(time2, eph, rst, dtst, var);
            svh[0] = eph.svh;
        } else if (sys == SYS_GLO) {
            GloEphemeris geph = selgeph(teph, sat, iode, nav);
            if (geph == null) return false;
            geph2pos(time, geph, rs, dts, var);
            GTime time2 = time.timeadd(tt);
            geph2pos(time2, geph, rst, dtst, var);
            svh[0] = geph.svh;
        } else {
            return false;
        }

        /* satellite velocity and clock drift by differential approximation */
        for (int i = 0; i < 3; i++) {
            rs[i + 3] = (rst[i] - rs[i]) / tt;
        }
        dts[1] = (dtst[0] - dts[0]) / tt;

        return true;
    }

    // ========================================================================
    // High-level entry point
    // ========================================================================

    /** Ephemeris option: broadcast */
    public static final int EPHOPT_BRDC = 0;

    /**
     * Compute satellite positions, velocities and clocks for all observations.
     * <p>
     * Uses broadcast ephemeris only (EPHOPT_BRDC). For each observation, the
     * transmission time is estimated from pseudorange, then the satellite
     * position and clock are computed at that time.
     *
     * @param teph   time for ephemeris selection (GPST)
     * @param obs    observation data array
     * @param n      number of observations to process
     * @param nav    navigation data
     * @param ephopt ephemeris option (only EPHOPT_BRDC=0 supported)
     * @param rs     output: satellite positions and velocities, n*6 elements
     *               {x,y,z,vx,vy,vz} per satellite (m, m/s)
     * @param dts    output: satellite clocks, n*2 elements {bias, drift} per
     *               satellite (s, s/s)
     * @param var    output: satellite position/clock variance, n elements (m^2)
     * @param svh    output: satellite health flags, n elements
     */
    public static void satposs(GTime teph, ObsData[] obs, int n, Navigation nav,
                               int ephopt, double[] rs, double[] dts,
                               double[] var, int[] svh) {
        int limit = Math.min(n, 2 * MAXOBS);

        for (int i = 0; i < limit; i++) {
            /* initialize outputs to zero */
            for (int j = 0; j < 6; j++) rs[j + i * 6] = 0.0;
            for (int j = 0; j < 2; j++) dts[j + i * 2] = 0.0;
            var[i] = 0.0;
            svh[i] = 0;

            /* search any pseudorange */
            double pr = 0.0;
            int j;
            for (j = 0; j < NFREQ; j++) {
                pr = obs[i].P[j];
                if (pr != 0.0) break;
            }
            if (j >= NFREQ) {
                continue; /* no pseudorange */
            }

            /* transmission time by satellite clock */
            GTime time = obs[i].time.timeadd(-pr / CLIGHT);

            /* satellite clock bias by broadcast ephemeris */
            double dt = ephclk(time, teph, obs[i].sat, nav);
            if (Double.isNaN(dt)) {
                continue;
            }

            time = time.timeadd(-dt);

            /* satellite position and clock at transmission time */
            double[] rsi  = new double[6];
            double[] dtsi = new double[2];
            double[] vari = new double[1];
            int[]    svhi = new int[1];

            boolean ok;
            if (ephopt == Constants.EPHOPT_PREC) {
                int ret = PreciseEphemeris.peph2pos(time, obs[i].sat, nav, 0,
                                                     rsi, dtsi, vari);
                svhi[0] = ret != 0 ? 0 : -1;
                ok = ret != 0;
            } else {
                ok = ephpos(time, teph, obs[i].sat, -1, nav, rsi, dtsi, vari, svhi);
            }

            if (!ok) {
                continue;
            }

            /* copy results to output arrays */
            System.arraycopy(rsi, 0, rs, i * 6, 6);
            dts[i * 2]     = dtsi[0];
            dts[i * 2 + 1] = dtsi[1];
            var[i]         = vari[0];
            svh[i]         = svhi[0];

            /* if no clock available, use broadcast clock */
            if (dts[i * 2] == 0.0) {
                double clk = ephclk(time, teph, obs[i].sat, nav);
                if (Double.isNaN(clk)) continue;
                dts[i * 2] = clk;
                dts[i * 2 + 1] = 0.0;
                var[i] = STD_BRDCCLK * STD_BRDCCLK;
            }
        }
    }

    // ---- utility ------------------------------------------------------------

    private static double sq(double x) {
        return x * x;
    }
}
