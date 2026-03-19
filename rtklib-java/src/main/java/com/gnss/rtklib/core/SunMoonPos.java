package com.gnss.rtklib.core;

import static com.gnss.rtklib.core.Constants.*;

/**
 * Sun and moon position computation.
 * Ported from RTKLIB rtkcmn.c sunmoonpos(), sunpos_eci(), moonpos_eci(), eci2ecef().
 * Uses RTKLIB's original analytical model (not SOFA).
 */
public final class SunMoonPos {

    private SunMoonPos() {}

    // IAU 1980 nutation coefficients [106][10]
    // {l, l', F, D, OMG, period, dpsi_s, dpsi_t, deps_s, deps_t}
    private static final double[][] NUT = {
        {   0,   0,   0,   0,   1, -6798.4, -171996, -174.2, 92025,   8.9},
        {   0,   0,   2,  -2,   2,   182.6,  -13187,   -1.6,  5736,  -3.1},
        {   0,   0,   2,   0,   2,    13.7,   -2274,   -0.2,   977,  -0.5},
        {   0,   0,   0,   0,   2, -3399.2,    2062,    0.2,  -895,   0.5},
        {   0,  -1,   0,   0,   0,  -365.3,   -1426,    3.4,    54,  -0.1},
        {   1,   0,   0,   0,   0,    27.6,     712,    0.1,    -7,   0.0},
        {   0,   1,   2,  -2,   2,   121.7,    -517,    1.2,   224,  -0.6},
        {   0,   0,   2,   0,   1,    13.6,    -386,   -0.4,   200,   0.0},
        {   1,   0,   2,   0,   2,     9.1,    -301,    0.0,   129,  -0.1},
        {   0,  -1,   2,  -2,   2,   365.2,     217,   -0.5,   -95,   0.3},
        {  -1,   0,   0,   2,   0,    31.8,     158,    0.0,    -1,   0.0},
        {   0,   0,   2,  -2,   1,   177.8,     129,    0.1,   -70,   0.0},
        {  -1,   0,   2,   0,   2,    27.1,     123,    0.0,   -53,   0.0},
        {   1,   0,   0,   0,   1,    27.7,      63,    0.1,   -33,   0.0},
        {   0,   0,   0,   2,   0,    14.8,      63,    0.0,    -2,   0.0},
        {  -1,   0,   2,   2,   2,     9.6,     -59,    0.0,    26,   0.0},
        {  -1,   0,   0,   0,   1,   -27.4,     -58,   -0.1,    32,   0.0},
        {   1,   0,   2,   0,   1,     9.1,     -51,    0.0,    27,   0.0},
        {  -2,   0,   0,   2,   0,  -205.9,     -48,    0.0,     1,   0.0},
        {  -2,   0,   2,   0,   1,  1305.5,      46,    0.0,   -24,   0.0},
        {   0,   0,   2,   2,   2,     7.1,     -38,    0.0,    16,   0.0},
        {   2,   0,   2,   0,   2,     6.9,     -31,    0.0,    13,   0.0},
        {   2,   0,   0,   0,   0,    13.8,      29,    0.0,    -1,   0.0},
        {   1,   0,   2,  -2,   2,    23.9,      29,    0.0,   -12,   0.0},
        {   0,   0,   2,   0,   0,    13.6,      26,    0.0,    -1,   0.0},
        {   0,   0,   2,  -2,   0,   173.3,     -22,    0.0,     0,   0.0},
        {  -1,   0,   2,   0,   1,    27.0,      21,    0.0,   -10,   0.0},
        {   0,   2,   0,   0,   0,   182.6,      17,   -0.1,     0,   0.0},
        {   0,   2,   2,  -2,   2,    91.3,     -16,    0.1,     7,   0.0},
        {  -1,   0,   0,   2,   1,    32.0,      16,    0.0,    -8,   0.0},
        {   0,   1,   0,   0,   1,   386.0,     -15,    0.0,     9,   0.0},
        {   1,   0,   0,  -2,   1,   -31.7,     -13,    0.0,     7,   0.0},
        {   0,  -1,   0,   0,   1,  -346.6,     -12,    0.0,     6,   0.0},
        {   2,   0,  -2,   0,   0, -1095.2,      11,    0.0,     0,   0.0},
        {  -1,   0,   2,   2,   1,     9.5,     -10,    0.0,     5,   0.0},
        {   1,   0,   2,   2,   2,     5.6,      -8,    0.0,     3,   0.0},
        {   0,  -1,   2,   0,   2,    14.2,      -7,    0.0,     3,   0.0},
        {   0,   0,   2,   2,   1,     7.1,      -7,    0.0,     3,   0.0},
        {   1,   1,   0,  -2,   0,   -34.8,      -7,    0.0,     0,   0.0},
        {   0,   1,   2,   0,   2,    13.2,       7,    0.0,    -3,   0.0},
        {  -2,   0,   0,   2,   1,  -199.8,      -6,    0.0,     3,   0.0},
        {   0,   0,   0,   2,   1,    14.8,      -6,    0.0,     3,   0.0},
        {   2,   0,   2,  -2,   2,    12.8,       6,    0.0,    -3,   0.0},
        {   1,   0,   0,   2,   0,     9.6,       6,    0.0,     0,   0.0},
        {   1,   0,   2,  -2,   1,    23.9,       6,    0.0,    -3,   0.0},
        {   0,   0,   0,  -2,   1,   -14.7,      -5,    0.0,     3,   0.0},
        {   0,  -1,   2,  -2,   1,   346.6,      -5,    0.0,     3,   0.0},
        {   2,   0,   2,   0,   1,     6.9,      -5,    0.0,     3,   0.0},
        {   1,  -1,   0,   0,   0,    29.8,       5,    0.0,     0,   0.0},
        {   1,   0,   0,  -1,   0,   411.8,      -4,    0.0,     0,   0.0},
        {   0,   0,   0,   1,   0,    29.5,      -4,    0.0,     0,   0.0},
        {   0,   1,   0,  -2,   0,   -15.4,      -4,    0.0,     0,   0.0},
        {   1,   0,  -2,   0,   0,   -26.9,       4,    0.0,     0,   0.0},
        {   2,   0,   0,  -2,   1,   212.3,       4,    0.0,    -2,   0.0},
        {   0,   1,   2,  -2,   1,   119.6,       4,    0.0,    -2,   0.0},
        {   1,   1,   0,   0,   0,    25.6,      -3,    0.0,     0,   0.0},
        {   1,  -1,   0,  -1,   0, -3232.9,      -3,    0.0,     0,   0.0},
        {  -1,  -1,   2,   2,   2,     9.8,      -3,    0.0,     1,   0.0},
        {   0,  -1,   2,   2,   2,     7.2,      -3,    0.0,     1,   0.0},
        {   1,  -1,   2,   0,   2,     9.4,      -3,    0.0,     1,   0.0},
        {   3,   0,   2,   0,   2,     5.5,      -3,    0.0,     1,   0.0},
        {  -2,   0,   2,   0,   2,  1615.7,      -3,    0.0,     1,   0.0},
        {   1,   0,   2,   0,   0,     9.1,       3,    0.0,     0,   0.0},
        {  -1,   0,   2,   4,   2,     5.8,      -2,    0.0,     1,   0.0},
        {   1,   0,   0,   0,   2,    27.8,      -2,    0.0,     1,   0.0},
        {  -1,   0,   2,  -2,   1,   -32.6,      -2,    0.0,     1,   0.0},
        {   0,  -2,   2,  -2,   1,  6786.3,      -2,    0.0,     1,   0.0},
        {  -2,   0,   0,   0,   1,   -13.7,      -2,    0.0,     1,   0.0},
        {   2,   0,   0,   0,   1,    13.8,       2,    0.0,    -1,   0.0},
        {   3,   0,   0,   0,   0,     9.2,       2,    0.0,     0,   0.0},
        {   1,   1,   2,   0,   2,     8.9,       2,    0.0,    -1,   0.0},
        {   0,   0,   2,   1,   2,     9.3,       2,    0.0,    -1,   0.0},
        {   1,   0,   0,   2,   1,     9.6,      -1,    0.0,     0,   0.0},
        {   1,   0,   2,   2,   1,     5.6,      -1,    0.0,     1,   0.0},
        {   1,   1,   0,  -2,   1,   -34.7,      -1,    0.0,     0,   0.0},
        {   0,   1,   0,   2,   0,    14.2,      -1,    0.0,     0,   0.0},
        {   0,   1,   2,  -2,   0,   117.5,      -1,    0.0,     0,   0.0},
        {   0,   1,  -2,   2,   0,  -329.8,      -1,    0.0,     0,   0.0},
        {   1,   0,  -2,   2,   0,    23.8,      -1,    0.0,     0,   0.0},
        {   1,   0,  -2,  -2,   0,    -9.5,      -1,    0.0,     0,   0.0},
        {   1,   0,   2,  -2,   0,    32.8,      -1,    0.0,     0,   0.0},
        {   1,   0,   0,  -4,   0,   -10.1,      -1,    0.0,     0,   0.0},
        {   2,   0,   0,  -4,   0,   -15.9,      -1,    0.0,     0,   0.0},
        {   0,   0,   2,   4,   2,     4.8,      -1,    0.0,     0,   0.0},
        {   0,   0,   2,  -1,   2,    25.4,      -1,    0.0,     0,   0.0},
        {  -2,   0,   2,   4,   2,     7.3,      -1,    0.0,     1,   0.0},
        {   2,   0,   2,   2,   2,     4.7,      -1,    0.0,     0,   0.0},
        {   0,  -1,   2,   0,   1,    14.2,      -1,    0.0,     0,   0.0},
        {   0,   0,  -2,   0,   1,   -13.6,      -1,    0.0,     0,   0.0},
        {   0,   0,   4,  -2,   2,    12.7,       1,    0.0,     0,   0.0},
        {   0,   1,   0,   0,   2,   409.2,       1,    0.0,     0,   0.0},
        {   1,   1,   2,  -2,   2,    22.5,       1,    0.0,    -1,   0.0},
        {   3,   0,   2,  -2,   2,     8.7,       1,    0.0,     0,   0.0},
        {  -2,   0,   2,   2,   2,    14.6,       1,    0.0,    -1,   0.0},
        {  -1,   0,   0,   0,   2,   -27.3,       1,    0.0,    -1,   0.0},
        {   0,   0,  -2,   2,   1,  -169.0,       1,    0.0,     0,   0.0},
        {   0,   1,   2,   0,   1,    13.1,       1,    0.0,     0,   0.0},
        {  -1,   0,   4,   0,   2,     9.1,       1,    0.0,     0,   0.0},
        {   2,   1,   0,  -2,   0,   131.7,       1,    0.0,     0,   0.0},
        {   2,   0,   0,   2,   0,     7.1,       1,    0.0,     0,   0.0},
        {   2,   0,   2,  -2,   1,    12.8,       1,    0.0,    -1,   0.0},
        {   2,   0,  -2,   0,   1,  -943.2,       1,    0.0,     0,   0.0},
        {   1,  -1,   0,  -2,   0,   -29.3,       1,    0.0,     0,   0.0},
        {  -1,   0,   0,   1,   1,  -388.3,       1,    0.0,     0,   0.0},
        {  -1,  -1,   0,   2,   1,    35.0,       1,    0.0,     0,   0.0},
        {   0,   1,   0,   1,   0,    27.3,       1,    0.0,     0,   0.0}
    };

    // Astronomical arguments coefficients for IAU 1980
    private static final double[][] FC = {
        { 134.96340251, 1717915923.2178,  31.8792,  0.051635, -0.00024470},
        { 357.52910918,  129596581.0481,  -0.5532,  0.000136, -0.00001149},
        {  93.27209062, 1739527262.8478, -12.7512, -0.001037,  0.00000417},
        { 297.85019547, 1602961601.2090,  -6.3706,  0.006593, -0.00003169},
        { 125.04455501,   -6962890.2665,   7.4722,  0.007702, -0.00005939}
    };

    /**
     * Compute astronomical arguments f={l, l', F, D, OMG} in radians.
     */
    static void astArgs(double t, double[] f) {
        double[] tt = new double[4];
        tt[0] = t;
        for (int i = 1; i < 4; i++) tt[i] = tt[i - 1] * t;

        for (int i = 0; i < 5; i++) {
            f[i] = FC[i][0] * 3600.0;
            for (int j = 0; j < 4; j++) f[i] += FC[i][j + 1] * tt[j];
            f[i] = (f[i] * AS2R) % (2.0 * PI);
        }
    }

    /**
     * IAU 1980 nutation: compute dpsi and deps.
     * @return double[2] = {dpsi, deps} in radians
     */
    static double[] nutIau1980(double t, double[] f) {
        double dpsi = 0.0, deps = 0.0;

        for (int i = 0; i < 106; i++) {
            double ang = 0.0;
            for (int j = 0; j < 5; j++) ang += NUT[i][j] * f[j];
            dpsi += (NUT[i][6] + NUT[i][7] * t) * Math.sin(ang);
            deps += (NUT[i][8] + NUT[i][9] * t) * Math.cos(ang);
        }
        dpsi *= 1E-4 * AS2R; // 0.1 mas -> rad
        deps *= 1E-4 * AS2R;
        return new double[]{dpsi, deps};
    }

    /**
     * Compute UTC to GMST (Greenwich Mean Sidereal Time).
     * @param t  UTC time
     * @param ut1_utc  UT1-UTC correction (s)
     * @return GMST in radians
     */
    static double utc2gmst(GTime t, double ut1_utc) {
        double[] ep2000 = {2000, 1, 1, 12, 0, 0};
        GTime tut = t.add(ut1_utc);

        // time2sec: separate day and second-of-day
        double[] ep = tut.time2epoch();
        double secOfDay = ep[3] * 3600 + ep[4] * 60 + ep[5];
        GTime tut0 = tut.add(-secOfDay);

        double t1 = tut0.diff(GTime.epoch2time(ep2000)) / 86400.0 / 36525.0;
        double t2 = t1 * t1, t3 = t2 * t1;
        double gmst0 = 24110.54841 + 8640184.812866 * t1 + 0.093104 * t2 - 6.2E-6 * t3;
        double gmst = gmst0 + 1.002737909350795 * secOfDay;

        return (gmst % 86400.0) * PI / 43200.0;
    }

    // Rotation matrices (column-major 3x3)
    private static void Rx(double t, double[] X) {
        X[0] = 1.0; X[1] = 0.0; X[2] = 0.0;
        X[3] = 0.0; X[4] = Math.cos(t); X[5] = -Math.sin(t);
        X[6] = 0.0; X[7] = Math.sin(t); X[8] = Math.cos(t);
    }

    private static void Ry(double t, double[] X) {
        X[0] = Math.cos(t); X[1] = 0.0; X[2] = Math.sin(t);
        X[3] = 0.0; X[4] = 1.0; X[5] = 0.0;
        X[6] = -Math.sin(t); X[7] = 0.0; X[8] = Math.cos(t);
    }

    private static void Rz(double t, double[] X) {
        X[0] = Math.cos(t); X[1] = -Math.sin(t); X[2] = 0.0;
        X[3] = Math.sin(t); X[4] = Math.cos(t); X[5] = 0.0;
        X[6] = 0.0; X[7] = 0.0; X[8] = 1.0;
    }

    /**
     * ECI to ECEF transformation matrix.
     * @param tutc  time in UTC
     * @param erpv  ERP values {xp, yp, ut1_utc, lod} (rad, rad, s, s/d)
     * @param U     output: 3x3 transformation matrix (column-major)
     * @return GMST in radians
     */
    public static double eci2ecef(GTime tutc, double[] erpv, double[] U) {
        double[] ep2000 = {2000, 1, 1, 12, 0, 0};

        // Terrestrial time
        GTime tgps = tutc.utc2gpst();
        double t = (tgps.diff(GTime.epoch2time(ep2000)) + 19.0 + 32.184) / 86400.0 / 36525.0;
        double t2 = t * t, t3 = t2 * t;

        // Astronomical arguments
        double[] f = new double[5];
        astArgs(t, f);

        // IAU 1976 precession
        double ze = (2306.2181 * t + 0.30188 * t2 + 0.017998 * t3) * AS2R;
        double th = (2004.3109 * t - 0.42665 * t2 - 0.041833 * t3) * AS2R;
        double z  = (2306.2181 * t + 1.09468 * t2 + 0.018203 * t3) * AS2R;
        double eps = (84381.448 - 46.8150 * t - 0.00059 * t2 + 0.001813 * t3) * AS2R;

        double[] R1 = new double[9], R2 = new double[9], R3 = new double[9];
        double[] R = new double[9], P = new double[9];
        Rz(-z, R1); Ry(th, R2); Rz(-ze, R3);
        matmul33(R1, R2, R);
        matmul33(R, R3, P);

        // IAU 1980 nutation
        double[] nutResult = nutIau1980(t, f);
        double dpsi = nutResult[0], deps = nutResult[1];
        double[] N = new double[9];
        Rx(-eps - deps, R1); Rz(-dpsi, R2); Rx(eps, R3);
        matmul33(R1, R2, R);
        matmul33(R, R3, N);

        // Greenwich apparent sidereal time
        double gmst = utc2gmst(tutc, erpv[2]);
        double gast = gmst + dpsi * Math.cos(eps);
        gast += (0.00264 * Math.sin(f[4]) + 0.000063 * Math.sin(2.0 * f[4])) * AS2R;

        // ECI to ECEF: U = W * Rz(gast) * N * P
        double[] W = new double[9], NP = new double[9];
        Ry(-erpv[0], R1); Rx(-erpv[1], R2); Rz(gast, R3);
        matmul33(R1, R2, W);
        matmul33(W, R3, R);
        matmul33(N, P, NP);
        matmul33(R, NP, U);

        return gmst;
    }

    /**
     * Sun position in ECI (Keplerian approximation).
     */
    public static void sunposEci(GTime tutc, double[] erpv, double[] rsun) {
        double[] ep2000 = {2000, 1, 1, 12, 0, 0};
        GTime tut = tutc.add(erpv[2]);
        double t = tut.diff(GTime.epoch2time(ep2000)) / 86400.0 / 36525.0;

        double[] f = new double[5];
        astArgs(t, f);

        double eps = 23.439291 - 0.0130042 * t;
        double sine = Math.sin(eps * D2R);
        double cose = Math.cos(eps * D2R);

        double Ms = 357.5277233 + 35999.05034 * t;
        double ls = 280.460 + 36000.770 * t + 1.914666471 * Math.sin(Ms * D2R)
                    + 0.019994643 * Math.sin(2.0 * Ms * D2R);
        double rs = AU * (1.000140612 - 0.016708617 * Math.cos(Ms * D2R)
                    - 0.000139589 * Math.cos(2.0 * Ms * D2R));
        double sinl = Math.sin(ls * D2R);
        double cosl = Math.cos(ls * D2R);
        rsun[0] = rs * cosl;
        rsun[1] = rs * cose * sinl;
        rsun[2] = rs * sine * sinl;
    }

    /**
     * Moon position in ECI (analytical model).
     */
    public static void moonposEci(GTime tutc, double[] erpv, double[] rmoon) {
        double[] ep2000 = {2000, 1, 1, 12, 0, 0};
        GTime tut = tutc.add(erpv[2]);
        double t = tut.diff(GTime.epoch2time(ep2000)) / 86400.0 / 36525.0;

        double[] f = new double[5];
        astArgs(t, f);

        double eps = 23.439291 - 0.0130042 * t;
        double sine = Math.sin(eps * D2R);
        double cose = Math.cos(eps * D2R);

        double lm = 218.32 + 481267.883 * t + 6.29 * Math.sin(f[0])
                    - 1.27 * Math.sin(f[0] - 2.0 * f[3])
                    + 0.66 * Math.sin(2.0 * f[3]) + 0.21 * Math.sin(2.0 * f[0])
                    - 0.19 * Math.sin(f[1]) - 0.11 * Math.sin(2.0 * f[2]);
        double pm = 5.13 * Math.sin(f[2]) + 0.28 * Math.sin(f[0] + f[2])
                    - 0.28 * Math.sin(f[2] - f[0]) - 0.17 * Math.sin(f[2] - 2.0 * f[3]);
        double rm = RE_WGS84 / Math.sin((0.9508 + 0.0518 * Math.cos(f[0])
                    + 0.0095 * Math.cos(f[0] - 2.0 * f[3])
                    + 0.0078 * Math.cos(2.0 * f[3]) + 0.0028 * Math.cos(2.0 * f[0])) * D2R);
        double sinl = Math.sin(lm * D2R);
        double cosl = Math.cos(lm * D2R);
        double sinp = Math.sin(pm * D2R);
        double cosp = Math.cos(pm * D2R);
        rmoon[0] = rm * cosp * cosl;
        rmoon[1] = rm * (cose * cosp * sinl - sine * sinp);
        rmoon[2] = rm * (sine * cosp * sinl + cose * sinp);
    }

    /**
     * Sun and moon position in ECEF.
     * @param tutc   time in UTC
     * @param erpv   ERP values {xp, yp, ut1_utc, lod} (rad, rad, s, s/d)
     * @param rsun   output sun position in ECEF (m), may be null
     * @param rmoon  output moon position in ECEF (m), may be null
     * @return GMST in radians
     */
    public static double sunmoonpos(GTime tutc, double[] erpv,
                                     double[] rsun, double[] rmoon) {
        double[] U = new double[9];
        double gmst = eci2ecef(tutc, erpv, U);

        if (rsun != null) {
            double[] rs = new double[3];
            sunposEci(tutc, erpv, rs);
            matmulVec(U, rs, rsun);
        }
        if (rmoon != null) {
            double[] rm = new double[3];
            moonposEci(tutc, erpv, rm);
            matmulVec(U, rm, rmoon);
        }
        return gmst;
    }

    // 3x3 matrix multiply: C = A * B (column-major)
    private static void matmul33(double[] A, double[] B, double[] C) {
        double[] T = new double[9];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                T[i + j * 3] = 0.0;
                for (int k = 0; k < 3; k++) {
                    T[i + j * 3] += A[i + k * 3] * B[k + j * 3];
                }
            }
        }
        System.arraycopy(T, 0, C, 0, 9);
    }

    // 3x3 matrix * 3x1 vector: y = A * x (column-major)
    private static void matmulVec(double[] A, double[] x, double[] y) {
        y[0] = A[0] * x[0] + A[3] * x[1] + A[6] * x[2];
        y[1] = A[1] * x[0] + A[4] * x[1] + A[7] * x[2];
        y[2] = A[2] * x[0] + A[5] * x[1] + A[8] * x[2];
    }

    /**
     * Normalize a 3-vector in place. Returns the original norm, or 0 if zero vector.
     */
    public static double normv3(double[] a, double[] b) {
        double r = Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        if (r <= 0.0) return 0.0;
        b[0] = a[0] / r;
        b[1] = a[1] / r;
        b[2] = a[2] / r;
        return r;
    }
}
