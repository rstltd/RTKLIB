package com.gnss.rtklib.correction;

import com.gnss.rtklib.core.Coord;
import com.gnss.rtklib.core.GTime;
import com.gnss.rtklib.core.MatrixUtil;
import com.gnss.rtklib.core.SunMoonPos;

import static com.gnss.rtklib.core.Constants.*;

/**
 * Tidal displacement corrections.
 * Ported from RTKLIB tides.c: dehanttideinel(), step2diu_(), step2lon_(),
 * st1idiu_(), st1isem_(), st1l1_(), tide_pole(), tidedisp().
 */
public final class TideCorrection {

    private TideCorrection() {}

    /**
     * Compute tidal displacement in ECEF (m).
     * @param tutc  time in UTC
     * @param rr    site position in ECEF (m)
     * @param opt   tide options: bit0=solid, bit2=pole
     * @param erpv  ERP values {xp, yp, ut1_utc, lod} (rad, rad, s, s/d), may be null
     * @param dr    output displacement in ECEF (m), length 3
     */
    public static void tidedisp(GTime tutc, double[] rr, int opt,
                                 double[] erpv, double[] dr) {
        dr[0] = dr[1] = dr[2] = 0.0;

        if (MatrixUtil.norm(rr, 3) <= 0.0) return;

        double[] erp = (erpv != null) ? erpv : new double[5];

        // Solid earth tides (bit 0)
        if ((opt & 1) != 0) {
            double[] rsun = new double[3], rmoon = new double[3];
            SunMoonPos.sunmoonpos(tutc, erp, rsun, rmoon);
            double[] drt = new double[3];
            dehanttideinel(tutc, rr, rsun, rmoon, drt);
            for (int i = 0; i < 3; i++) dr[i] += drt[i];
        }

        // Pole tide (bit 2)
        if ((opt & 4) != 0 && erpv != null) {
            double[] pos = Coord.ecef2pos(rr);
            double[] denu = new double[3];
            tidePole(tutc, pos, erp, denu);
            // ENU to ECEF
            double[] drt = enu2ecef(pos, denu);
            for (int i = 0; i < 3; i++) dr[i] += drt[i];
        }
    }

    /**
     * Solid earth tide displacement (IERS/Dehant model).
     */
    static void dehanttideinel(GTime tutc, double[] xsta, double[] xsun,
                                double[] xmon, double[] dxtide) {
        double h20 = 0.6078, l20 = 0.0847, h3 = 0.292, l3 = 0.015;

        double rsta = Math.sqrt(sq(xsta[0]) + sq(xsta[1]) + sq(xsta[2]));
        double rsun = Math.sqrt(sq(xsun[0]) + sq(xsun[1]) + sq(xsun[2]));
        double rmon = Math.sqrt(sq(xmon[0]) + sq(xmon[1]) + sq(xmon[2]));
        double scs = dot3(xsta, xsun);
        double scm = dot3(xsta, xmon);
        double scsun = scs / rsta / rsun;
        double scmon = scm / rsta / rmon;

        double cosphi = Math.sqrt(sq(xsta[0]) + sq(xsta[1])) / rsta;
        double h2 = h20 - (1.0 - 3.0 / 2.0 * sq(cosphi)) * 6e-4;
        double l2 = l20 + (1.0 - 3.0 / 2.0 * sq(cosphi)) * 2e-4;

        // P2 term
        double p2sun = 3.0 * (h2 / 2.0 - l2) * sq(scsun) - h2 / 2.0;
        double p2mon = 3.0 * (h2 / 2.0 - l2) * sq(scmon) - h2 / 2.0;

        // P3 term
        double p3sun = 5.0 / 2.0 * (h3 - l3 * 3.0) * (sq(scsun) * scsun) + 3.0 / 2.0 * (l3 - h3) * scsun;
        double p3mon = 5.0 / 2.0 * (h3 - l3 * 3.0) * (sq(scmon) * scmon) + 3.0 / 2.0 * (l3 - h3) * scmon;

        double x2sun = 3.0 * l2 * scsun;
        double x2mon = 3.0 * l2 * scmon;
        double x3sun = 3.0 / 2.0 * l3 * (sq(scsun) * 5.0 - 1.0);
        double x3mon = 3.0 / 2.0 * l3 * (sq(scmon) * 5.0 - 1.0);

        double mass_ratio_sun = 332946.0482;
        double mass_ratio_moon = 0.0123000371;
        double re = 6378136.6;
        double resun = re / rsun;
        double fac2sun = mass_ratio_sun * re * sq(resun) * resun;
        double remon = re / rmon;
        double fac2mon = mass_ratio_moon * re * sq(remon) * remon;
        double fac3sun = fac2sun * resun;
        double fac3mon = fac2mon * remon;

        for (int i = 0; i < 3; i++) {
            dxtide[i] = fac2sun * (x2sun * xsun[i] / rsun + p2sun * xsta[i] / rsta) +
                         fac2mon * (x2mon * xmon[i] / rmon + p2mon * xsta[i] / rsta) +
                         fac3sun * (x3sun * xsun[i] / rsun + p3sun * xsta[i] / rsta) +
                         fac3mon * (x3mon * xmon[i] / rmon + p3mon * xsta[i] / rsta);
        }

        // Step 1 corrections
        double[] xcorsta = new double[3];
        st1idiu(xsta, xsun, xmon, fac2sun, fac2mon, xcorsta);
        for (int i = 0; i < 3; i++) dxtide[i] += xcorsta[i];

        st1isem(xsta, xsun, xmon, fac2sun, fac2mon, xcorsta);
        for (int i = 0; i < 3; i++) dxtide[i] += xcorsta[i];

        st1l1(xsta, xsun, xmon, fac2sun, fac2mon, xcorsta);
        for (int i = 0; i < 3; i++) dxtide[i] += xcorsta[i];

        // Step 2 corrections
        double[] ep = tutc.time2epoch();
        double fhr = ep[3] + ep[4] / 60.0 + ep[5] / 3600.0;

        double[] ep2000 = {2000, 1, 1, 11, 59, 8.816};
        GTime tgps = tutc.utc2gpst();
        double t = tgps.diff(GTime.epoch2time(ep2000)) / 86400.0 / 36525.0;

        step2diu(xsta, fhr, t, xcorsta);
        for (int i = 0; i < 3; i++) dxtide[i] += xcorsta[i];

        step2lon(xsta, t, xcorsta);
        for (int i = 0; i < 3; i++) dxtide[i] += xcorsta[i];
    }

    /** Out-of-phase diurnal (Step 1). */
    private static void st1idiu(double[] xsta, double[] xsun, double[] xmon,
                                 double fac2sun, double fac2mon, double[] xcorsta) {
        double dhi = -0.0025, dli = -7e-4;
        double rsta = norm3(xsta);
        double sinphi = xsta[2] / rsta;
        double cosphi = Math.sqrt(sq(xsta[0]) + sq(xsta[1])) / rsta;
        double cos2phi = sq(cosphi) - sq(sinphi);
        double sinla = xsta[1] / cosphi / rsta;
        double cosla = xsta[0] / cosphi / rsta;
        double rmon = norm3(xmon);
        double rsun = norm3(xsun);

        double drsun = dhi * -3.0 * sinphi * cosphi * fac2sun * xsun[2] *
                        (xsun[0] * sinla - xsun[1] * cosla) / sq(rsun);
        double drmon = dhi * -3.0 * sinphi * cosphi * fac2mon * xmon[2] *
                        (xmon[0] * sinla - xmon[1] * cosla) / sq(rmon);
        double dnsun = dli * -3.0 * cos2phi * fac2sun * xsun[2] *
                        (xsun[0] * sinla - xsun[1] * cosla) / sq(rsun);
        double dnmon = dli * -3.0 * cos2phi * fac2mon * xmon[2] *
                        (xmon[0] * sinla - xmon[1] * cosla) / sq(rmon);
        double desun = dli * -3.0 * sinphi * fac2sun * xsun[2] *
                        (xsun[0] * cosla + xsun[1] * sinla) / sq(rsun);
        double demon = dli * -3.0 * sinphi * fac2mon * xmon[2] *
                        (xmon[0] * cosla + xmon[1] * sinla) / sq(rmon);
        double dr = drsun + drmon;
        double dn = dnsun + dnmon;
        double de = desun + demon;
        xcorsta[0] = dr * cosla * cosphi - de * sinla - dn * sinphi * cosla;
        xcorsta[1] = dr * sinla * cosphi + de * cosla - dn * sinphi * sinla;
        xcorsta[2] = dr * sinphi + dn * cosphi;
    }

    /** Out-of-phase semi-diurnal (Step 1). */
    private static void st1isem(double[] xsta, double[] xsun, double[] xmon,
                                  double fac2sun, double fac2mon, double[] xcorsta) {
        double dhi = -0.0022, dli = -7e-4;
        double rsta = norm3(xsta);
        double sinphi = xsta[2] / rsta;
        double cosphi = Math.sqrt(sq(xsta[0]) + sq(xsta[1])) / rsta;
        double sinla = xsta[1] / cosphi / rsta;
        double cosla = xsta[0] / cosphi / rsta;
        double costwola = sq(cosla) - sq(sinla);
        double sintwola = cosla * 2.0 * sinla;
        double rmon = norm3(xmon);
        double rsun = norm3(xsun);

        double drsun = -3.0 / 4.0 * dhi * sq(cosphi) * fac2sun *
                        ((sq(xsun[0]) - sq(xsun[1])) * sintwola - xsun[0] * 2.0 * xsun[1] * costwola) / sq(rsun);
        double drmon = -3.0 / 4.0 * dhi * sq(cosphi) * fac2mon *
                        ((sq(xmon[0]) - sq(xmon[1])) * sintwola - xmon[0] * 2.0 * xmon[1] * costwola) / sq(rmon);
        double dnsun = 3.0 / 2.0 * dli * sinphi * cosphi * fac2sun *
                        ((sq(xsun[0]) - sq(xsun[1])) * sintwola - xsun[0] * 2.0 * xsun[1] * costwola) / sq(rsun);
        double dnmon = 3.0 / 2.0 * dli * sinphi * cosphi * fac2mon *
                        ((sq(xmon[0]) - sq(xmon[1])) * sintwola - xmon[0] * 2.0 * xmon[1] * costwola) / sq(rmon);
        double desun = -3.0 / 2.0 * dli * cosphi * fac2sun *
                        ((sq(xsun[0]) - sq(xsun[1])) * costwola + xsun[0] * 2.0 * xsun[1] * sintwola) / sq(rsun);
        double demon = -3.0 / 2.0 * dli * cosphi * fac2mon *
                        ((sq(xmon[0]) - sq(xmon[1])) * costwola + xmon[0] * 2.0 * xmon[1] * sintwola) / sq(rmon);
        double dr = drsun + drmon;
        double dn = dnsun + dnmon;
        double de = desun + demon;
        xcorsta[0] = dr * cosla * cosphi - de * sinla - dn * sinphi * cosla;
        xcorsta[1] = dr * sinla * cosphi + de * cosla - dn * sinphi * sinla;
        xcorsta[2] = dr * sinphi + dn * cosphi;
    }

    /** Latitude dependence correction (Step 1). */
    private static void st1l1(double[] xsta, double[] xsun, double[] xmon,
                                double fac2sun, double fac2mon, double[] xcorsta) {
        double l1d = 0.0012, l1sd = 0.0024;
        double rsta = norm3(xsta);
        double sinphi = xsta[2] / rsta;
        double cosphi = Math.sqrt(sq(xsta[0]) + sq(xsta[1])) / rsta;
        double sinla = xsta[1] / cosphi / rsta;
        double cosla = xsta[0] / cosphi / rsta;
        double rmon = norm3(xmon);
        double rsun = norm3(xsun);

        // Diurnal band
        double l1 = l1d;
        double dnsun = -l1 * sq(sinphi) * fac2sun * xsun[2] *
                        (xsun[0] * cosla + xsun[1] * sinla) / sq(rsun);
        double dnmon = -l1 * sq(sinphi) * fac2mon * xmon[2] *
                        (xmon[0] * cosla + xmon[1] * sinla) / sq(rmon);
        double desun = l1 * sinphi * (sq(cosphi) - sq(sinphi)) * fac2sun * xsun[2] *
                        (xsun[0] * sinla - xsun[1] * cosla) / sq(rsun);
        double demon = l1 * sinphi * (sq(cosphi) - sq(sinphi)) * fac2mon * xmon[2] *
                        (xmon[0] * sinla - xmon[1] * cosla) / sq(rmon);
        double de = 3.0 * (desun + demon);
        double dn = 3.0 * (dnsun + dnmon);
        xcorsta[0] = -de * sinla - dn * sinphi * cosla;
        xcorsta[1] = de * cosla - dn * sinphi * sinla;
        xcorsta[2] = dn * cosphi;

        // Semi-diurnal band
        l1 = l1sd;
        double costwola = sq(cosla) - sq(sinla);
        double sintwola = 2.0 * cosla * sinla;
        dnsun = -l1 / 2.0 * sinphi * cosphi * fac2sun *
                ((sq(xsun[0]) - sq(xsun[1])) * costwola + xsun[0] * 2.0 * xsun[1] * sintwola) / sq(rsun);
        dnmon = -l1 / 2.0 * sinphi * cosphi * fac2mon *
                ((sq(xmon[0]) - sq(xmon[1])) * costwola + xmon[0] * 2.0 * xmon[1] * sintwola) / sq(rmon);
        desun = -l1 / 2.0 * sq(sinphi) * cosphi * fac2sun *
                ((sq(xsun[0]) - sq(xsun[1])) * sintwola - xsun[0] * 2.0 * xsun[1] * costwola) / sq(rsun);
        demon = -l1 / 2.0 * sq(sinphi) * cosphi * fac2mon *
                ((sq(xmon[0]) - sq(xmon[1])) * sintwola - xmon[0] * 2.0 * xmon[1] * costwola) / sq(rmon);
        de = 3.0 * (desun + demon);
        dn = 3.0 * (dnsun + dnmon);
        xcorsta[0] += -de * sinla - dn * sinphi * cosla;
        xcorsta[1] += de * cosla - dn * sinphi * sinla;
        xcorsta[2] += dn * cosphi;
    }

    // Step 2 diurnal correction coefficients [31][9]
    private static final double[][] DATDI_DIU = {
        {-3, 0, 2, 0, 0, -0.01, 0, 0, 0},
        {-3, 2, 0, 0, 0, -0.01, 0, 0, 0},
        {-2, 0, 1, -1, 0, -0.02, 0, 0, 0},
        {-2, 0, 1, 0, 0, -0.08, 0, -0.01, 0.01},
        {-2, 2, -1, 0, 0, -0.02, 0, 0, 0},
        {-1, 0, 0, -1, 0, -0.10, 0, 0, 0},
        {-1, 0, 0, 0, 0, -0.51, 0, -0.02, 0.03},
        {-1, 2, 0, 0, 0, 0.01, 0, 0, 0},
        {0, -2, 1, 0, 0, 0.01, 0, 0, 0},
        {0, 0, -1, 0, 0, 0.02, 0, 0, 0},
        {0, 0, 1, 0, 0, 0.06, 0, 0, 0},
        {0, 0, 1, 1, 0, 0.01, 0, 0, 0},
        {0, 2, -1, 0, 0, 0.01, 0, 0, 0},
        {1, -3, 0, 0, 1, -0.06, 0, 0, 0},
        {1, -2, 0, -1, 0, 0.01, 0, 0, 0},
        {1, -2, 0, 0, 0, -1.23, -0.07, 0.06, 0.01},
        {1, -1, 0, 0, -1, 0.02, 0, 0, 0},
        {1, -1, 0, 0, 1, 0.04, 0, 0, 0},
        {1, 0, 0, -1, 0, -0.22, 0.01, 0.01, 0},
        {1, 0, 0, 0, 0, 12.00, -0.80, -0.67, -0.03},
        {1, 0, 0, 1, 0, 1.73, -0.12, -0.10, 0},
        {1, 0, 0, 2, 0, -0.04, 0, 0, 0},
        {1, 1, 0, 0, -1, -0.50, -0.01, 0.03, 0},
        {1, 1, 0, 0, 1, 0.01, 0, 0, 0},
        {0, 1, 0, 1, -1, -0.01, 0, 0, 0},
        {1, 2, -2, 0, 0, -0.01, 0, 0, 0},
        {1, 2, 0, 0, 0, -0.11, 0.01, 0.01, 0},
        {2, -2, 1, 0, 0, -0.01, 0, 0, 0},
        {2, 0, -1, 0, 0, -0.02, 0, 0, 0},
        {3, 0, 0, 0, 0, 0, 0, 0, 0},
        {3, 0, 0, 1, 0, 0, 0, 0, 0}
    };

    /** Step 2 diurnal correction. */
    private static void step2diu(double[] xsta, double fhr, double t, double[] xcorsta) {
        double s = ((t * 1.85139e-6 - .0014663889) * t + 481267.88194) * t + 218.31664563;
        double tau = fhr * 15.0 + 280.4606184 + ((t * -2.58e-8 + 3.8793e-4) * t + 36000.7700536) * t + (-s);
        double pr = (((t * 7e-9 + 2.1e-8) * t + 3.08889e-4) * t + 1.396971278) * t;
        s += pr;
        double h = (((t * -6.54e-9 + 2e-8) * t + 3.0322222e-4) * t + 36000.7697489) * t + 280.46645;
        double p = (((t * 5.263e-8 - 1.24991e-5) * t - 0.01032172222) * t + 4069.01363525) * t + 83.35324312;
        double zns = (((t * 1.65e-8 - 2.13944e-6) * t - 0.00207561111) * t + 1934.13626197) * t + 234.95544499;
        double ps = (((t * -3.34e-9 - 1.778e-8) * t + 4.5688889e-4) * t + 1.71945766667) * t + 282.93734098;
        s %= 360.0; tau %= 360.0; h %= 360.0; p %= 360.0; zns %= 360.0; ps %= 360.0;

        double rsta = norm3(xsta);
        double sinphi = xsta[2] / rsta;
        double cosphi = Math.sqrt(sq(xsta[0]) + sq(xsta[1])) / rsta;
        double cosla = xsta[0] / cosphi / rsta;
        double sinla = xsta[1] / cosphi / rsta;
        double zla = Math.atan2(xsta[1], xsta[0]);

        for (int i = 0; i < 3; i++) xcorsta[i] = 0.0;
        for (int j = 0; j < 31; j++) {
            double thetaf = (tau + DATDI_DIU[j][0] * s + DATDI_DIU[j][1] * h + DATDI_DIU[j][2] * p +
                             DATDI_DIU[j][3] * zns + DATDI_DIU[j][4] * ps) * D2R;
            double dr = DATDI_DIU[j][5] * 2.0 * sinphi * cosphi * Math.sin(thetaf + zla) +
                        DATDI_DIU[j][6] * 2.0 * sinphi * cosphi * Math.cos(thetaf + zla);
            double dn = DATDI_DIU[j][7] * (sq(cosphi) - sq(sinphi)) * Math.sin(thetaf + zla) +
                        DATDI_DIU[j][8] * (sq(cosphi) - sq(sinphi)) * Math.cos(thetaf + zla);
            double de = DATDI_DIU[j][7] * sinphi * Math.cos(thetaf + zla) -
                        DATDI_DIU[j][8] * sinphi * Math.sin(thetaf + zla);
            xcorsta[0] += dr * cosla * cosphi - de * sinla - dn * sinphi * cosla;
            xcorsta[1] += dr * sinla * cosphi + de * cosla - dn * sinphi * sinla;
            xcorsta[2] += dr * sinphi + dn * cosphi;
        }
        for (int i = 0; i < 3; i++) xcorsta[i] /= 1e3;
    }

    // Step 2 long-period correction coefficients [5][9]
    private static final double[][] DATDI_LON = {
        {0., 0., 0., 1., 0., .47, .23, .16, .07},
        {0., 2., 0., 0., 0., -.2, -.12, -.11, -.05},
        {1., 0., -1., 0., 0., -.11, -.08, -.09, -.04},
        {2., 0., 0., 0., 0., -.13, -.11, -.15, -.07},
        {2., 0., 0., 1., 0., -.05, -.05, -.06, -.03}
    };

    /** Step 2 long-period correction. */
    private static void step2lon(double[] xsta, double t, double[] xcorsta) {
        double s = ((t * 1.85139e-6 - .0014663889) * t + 481267.88194) * t + 218.31664563;
        double pr = (((t * 7e-9 + 2.1e-8) * t + 3.08889e-4) * t + 1.396971278) * t;
        s += pr;
        double h = (((t * -6.54e-9 + 2e-8) * t + 3.0322222e-4) * t + 36000.7697489) * t + 280.46645;
        double p = (((t * 5.263e-8 - 1.24991e-5) * t - 0.01032172222) * t + 4069.01363525) * t + 83.35324312;
        double zns = (((t * 1.65e-8 - 2.13944e-6) * t - 0.00207561111) * t + 1934.13626197) * t + 234.95544499;
        double ps = (((t * -3.34e-9 - 1.778e-8) * t + 4.5688889e-4) * t + 1.71945766667) * t + 282.93734098;
        s %= 360.0; h %= 360.0; p %= 360.0; zns %= 360.0; ps %= 360.0;

        double rsta = norm3(xsta);
        double sinphi = xsta[2] / rsta;
        double cosphi = Math.sqrt(sq(xsta[0]) + sq(xsta[1])) / rsta;
        double cosla = xsta[0] / cosphi / rsta;
        double sinla = xsta[1] / cosphi / rsta;

        for (int i = 0; i < 3; i++) xcorsta[i] = 0.0;
        for (int j = 0; j < 5; j++) {
            double thetaf = (DATDI_LON[j][0] * s + DATDI_LON[j][1] * h + DATDI_LON[j][2] * p +
                             DATDI_LON[j][3] * zns + DATDI_LON[j][4] * ps) * D2R;
            double dr = DATDI_LON[j][5] * (sq(sinphi) * 3.0 - 1.0) / 2.0 * Math.cos(thetaf) +
                        DATDI_LON[j][7] * (sq(sinphi) * 3.0 - 1.0) / 2.0 * Math.sin(thetaf);
            double dn = DATDI_LON[j][6] * (cosphi * sinphi * 2.0) * Math.cos(thetaf) +
                        DATDI_LON[j][8] * (cosphi * sinphi * 2.0) * Math.sin(thetaf);
            xcorsta[0] += dr * cosla * cosphi - dn * sinphi * cosla;
            xcorsta[1] += dr * sinla * cosphi - dn * sinphi * sinla;
            xcorsta[2] += dr * sinphi + dn * cosphi;
        }
        for (int i = 0; i < 3; i++) xcorsta[i] /= 1e3;
    }

    /**
     * IERS secular pole position (mas).
     */
    private static double[] iersSecularPole(GTime tutc) {
        double[] ep2000 = {2000, 1, 1, 11, 59, 8.816};
        GTime tgps = tutc.utc2gpst();
        double y = tgps.diff(GTime.epoch2time(ep2000)) / 86400.0 / 365.25;
        return new double[]{55.0 + 1.677 * y, 320.5 + 3.460 * y};
    }

    /**
     * Displacement by pole tide (IERS eq.7.26).
     */
    static void tidePole(GTime tutc, double[] pos, double[] erpv, double[] denu) {
        double[] pole = iersSecularPole(tutc);
        double xp_bar = pole[0], yp_bar = pole[1];

        double m1 = erpv[0] / AS2R - xp_bar * 1E-3; // (as)
        double m2 = -erpv[1] / AS2R + yp_bar * 1E-3;

        double cosl = Math.cos(pos[1]);
        double sinl = Math.sin(pos[1]);
        denu[0] = 9E-3 * Math.sin(pos[0]) * (m1 * sinl - m2 * cosl);
        denu[1] = -9E-3 * Math.cos(2.0 * pos[0]) * (m1 * cosl + m2 * sinl);
        denu[2] = -33E-3 * Math.sin(2.0 * pos[0]) * (m1 * cosl + m2 * sinl);
    }

    /** Convert ENU displacement to ECEF. */
    private static double[] enu2ecef(double[] pos, double[] enu) {
        double sinp = Math.sin(pos[0]), cosp = Math.cos(pos[0]);
        double sinl = Math.sin(pos[1]), cosl = Math.cos(pos[1]);
        // E^T * enu (transpose of xyz2enu matrix)
        double[] r = new double[3];
        r[0] = -sinl * enu[0] - sinp * cosl * enu[1] + cosp * cosl * enu[2];
        r[1] = cosl * enu[0] - sinp * sinl * enu[1] + cosp * sinl * enu[2];
        r[2] = cosp * enu[1] + sinp * enu[2];
        return r;
    }

    private static double sq(double x) { return x * x; }
    private static double norm3(double[] a) { return Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); }
    private static double dot3(double[] a, double[] b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
}
