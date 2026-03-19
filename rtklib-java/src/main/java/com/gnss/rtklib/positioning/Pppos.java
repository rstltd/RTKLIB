package com.gnss.rtklib.positioning;

import com.gnss.rtklib.core.*;
import com.gnss.rtklib.correction.TideCorrection;
import com.gnss.rtklib.correction.Troposphere;
import com.gnss.rtklib.io.AntexReader;
import com.gnss.rtklib.model.*;

import java.util.Arrays;
import java.util.logging.Logger;

import static com.gnss.rtklib.core.Constants.*;
import static com.gnss.rtklib.model.PppState.*;

/**
 * Precise Point Positioning (PPP) engine — float-only.
 * Ported from RTKLIB ppp.c pppos().
 */
public final class Pppos {

    private static final Logger trace = Logger.getLogger(Pppos.class.getName());

    private Pppos() {}

    // Constants
    private static final int MAX_ITER = 8;
    private static final int MIN_NSAT_SOL = 4;
    private static final double THRES_REJECT = 4.0;
    private static final double THRES_MW_JUMP = 10.0;

    // Diagnostic counters (for testing)
    public static int diagNoObs, diagFilterErr, diagIterOverflow, diagOk, diagNsLow;
    public static int[] diagNsHist = new int[20];

    private static final double VAR_POS  = 3600.0;   // 60^2
    private static final double VAR_VEL  = 100.0;     // 10^2
    private static final double VAR_ACC  = 100.0;     // 10^2
    private static final double VAR_CLK  = 3600.0;    // 60^2
    private static final double VAR_ZTD  = 0.36;      // 0.6^2
    private static final double VAR_GRA  = 0.0001;    // 0.01^2
    private static final double VAR_BIAS = 3600.0;    // 60^2

    private static final double ERR_SAAS  = 0.3;
    private static final double ERR_BRDCI = 0.5;
    private static final double ERR_CBIAS = 0.3;
    private static final double REL_HUMI  = 0.7;
    private static final double EFACT_GPS_L5 = 10.0;
    private static final double VAR_GLO_IFB = 0.36; // 0.6^2

    /**
     * PPP positioning for one epoch.
     *
     * @param rtk  PPP state
     * @param obs  observations
     * @param n    number of observations
     * @param nav  navigation data (with peph/pclk)
     */
    public static void pppos(PppState rtk, ObsData[] obs, int n, Navigation nav) {
        ProcessingOptions opt = rtk.opt;
        int nf = opt.nf;

        // Clear fix flags
        for (int i = 0; i < MAXSAT; i++) {
            for (int j = 0; j < nf; j++) rtk.ssat[i].fix[j] = 0;
        }
        for (int i = 0; i < n && i < MAXOBS; i++) {
            for (int j = 0; j < nf; j++) {
                rtk.ssat[obs[i].sat - 1].snr_rover[j] = obs[i].SNR[j];
            }
        }

        // Time update of EKF states
        udstatePpp(rtk, obs, n, nav);

        // Satellite positions and clocks
        double[] rs = new double[6 * n];
        double[] dts = new double[2 * n];
        double[] varRs = new double[n];
        int[] svh = new int[n];
        double[] azel = new double[2 * n];

        EphemerisCalc.satposs(obs[0].time, obs, n, nav, opt.sateph, rs, dts, varRs, svh);

        // Eclipse exclusion (posopt[3])
        if (opt.posopt[3] != 0) {
            testEclipse(obs, n, nav, rs);
        }

        // EKF iteration
        int nv = n * opt.nf * 2 + MAXSAT + 3;
        double[] xp = new double[rtk.nx];
        double[] Pp = new double[rtk.nx * rtk.nx];
        double[] v = new double[nv];
        double[] H = new double[rtk.nx * nv];
        double[] R = new double[nv * nv];
        int[] exc = new int[n];
        int stat = SOLQ_SINGLE;

        // Compute tidal displacement once per epoch
        double[] dr = new double[3];
        if (opt.tidecorr != 0 && MatrixUtil.norm(rtk.x, 3) > 0.0) {
            double[] rrTide = new double[3];
            System.arraycopy(rtk.x, 0, rrTide, 0, 3);
            GTime tutc = obs[0].time.gpst2utc();
            TideCorrection.tidedisp(tutc, rrTide, opt.tidecorr, nav.erpv, dr);
        }

        int iterFinal = -1;
        for (int iter = 0; iter < MAX_ITER; iter++) {
            System.arraycopy(rtk.x, 0, xp, 0, rtk.nx);
            System.arraycopy(rtk.P, 0, Pp, 0, rtk.nx * rtk.nx);

            // Pre-fit residuals
            int nvp = pppRes(0, obs, n, rs, dts, varRs, svh, dr, exc, nav, xp, rtk, v, H, R, azel);
            if (nvp == 0) { iterFinal = -2; break; }

            // EKF measurement update
            int info = rtk.filter.update(xp, Pp, H, v, R, rtk.nx, nvp);
            if (info != 0) { iterFinal = -3; break; }

            // Post-fit residuals
            if (pppRes(iter + 1, obs, n, rs, dts, varRs, svh, dr, exc, nav, xp, rtk,
                       null, null, null, azel) != 0) {
                System.arraycopy(xp, 0, rtk.x, 0, rtk.nx);
                System.arraycopy(Pp, 0, rtk.P, 0, rtk.nx * rtk.nx);
                stat = SOLQ_PPP;
                break;
            }
            iterFinal = iter;
        }

        if (iterFinal == -2) diagNoObs++;
        else if (iterFinal == -3) diagFilterErr++;
        else if (stat != SOLQ_PPP) diagIterOverflow++;

        if (stat == SOLQ_PPP) {
            updateStat(rtk, obs, n, stat);
            int ns = rtk.sol.ns;
            if (ns < diagNsHist.length) diagNsHist[ns]++;
            if (rtk.sol.stat == SOLQ_NONE) diagNsLow++;
            else diagOk++;
        }
    }

    // ---------------------------------------------------------------
    // Eclipse exclusion (Step 3)
    // ---------------------------------------------------------------

    /**
     * Exclude eclipsing satellites by zeroing their positions.
     * Ported from ppp.c testeclipse().
     */
    private static void testEclipse(ObsData[] obs, int n, Navigation nav, double[] rs) {
        double[] rsun = new double[3], esun = new double[3];
        double[] erpv = new double[5];

        // Sun direction in ECEF
        SunMoonPos.sunmoonpos(obs[0].time.gpst2utc(), erpv, rsun, null);
        if (SunMoonPos.normv3(rsun, esun) <= 0.0) return;

        for (int i = 0; i < n; i++) {
            double r = Math.sqrt(sq(rs[i * 6]) + sq(rs[i * 6 + 1]) + sq(rs[i * 6 + 2]));
            if (r <= 0.0) continue;

            // Sun-earth-satellite angle
            double cosa = (rs[i * 6] * esun[0] + rs[i * 6 + 1] * esun[1] + rs[i * 6 + 2] * esun[2]) / r;
            cosa = Math.max(-1.0, Math.min(1.0, cosa));
            double ang = Math.acos(cosa);

            // Test eclipse: satellite is in earth shadow
            if (ang < PI / 2.0 || r * Math.sin(ang) > RE_WGS84) continue;

            for (int j = 0; j < 3; j++) rs[j + i * 6] = 0.0;
        }
    }

    // ---------------------------------------------------------------
    // Phase wind-up model (Step 4)
    // ---------------------------------------------------------------

    /**
     * Nominal yaw angle.
     */
    private static double yawNominal(double beta, double mu) {
        if (Math.abs(beta) < 1E-12 && Math.abs(mu) < 1E-12) return PI;
        return Math.atan2(-Math.tan(beta), Math.sin(mu)) + PI;
    }

    /**
     * Satellite attitude model — compute satellite fixed x,y unit vectors.
     * Ported from ppp.c sat_yaw() with nominal yaw only.
     */
    private static boolean satYaw(GTime time, int sat, double[] rs, double[] exs, double[] eys) {
        double[] rsun = new double[3], es = new double[3], esun = new double[3];
        double[] n = new double[3], p = new double[3], en = new double[3], ep = new double[3], ex = new double[3];
        double[] ri = new double[6];
        double[] erpv = new double[5];

        SunMoonPos.sunmoonpos(time.gpst2utc(), erpv, rsun, null);

        // beta and orbit angle
        System.arraycopy(rs, 0, ri, 0, 6);
        ri[3] -= OMGE * ri[1];
        ri[4] += OMGE * ri[0];
        MatrixUtil.cross3(ri, new double[]{ri[3], ri[4], ri[5]}, n);
        MatrixUtil.cross3(rsun, n, p);

        if (SunMoonPos.normv3(rs, es) <= 0.0) return false;
        if (SunMoonPos.normv3(rsun, esun) <= 0.0) return false;
        if (SunMoonPos.normv3(n, en) <= 0.0) return false;
        if (SunMoonPos.normv3(p, ep) <= 0.0) return false;

        double beta = PI / 2.0 - Math.acos(MatrixUtil.dot3(esun, en));
        double E = Math.acos(Math.max(-1.0, Math.min(1.0, MatrixUtil.dot3(es, ep))));
        double mu = PI / 2.0 + (MatrixUtil.dot3(es, esun) <= 0 ? -E : E);
        if (mu < -PI / 2.0) mu += 2.0 * PI;
        else if (mu >= PI / 2.0) mu -= 2.0 * PI;

        double yaw = yawNominal(beta, mu);

        // Satellite fixed x,y-vector
        MatrixUtil.cross3(en, es, ex);
        double cosy = Math.cos(yaw);
        double siny = Math.sin(yaw);
        for (int i = 0; i < 3; i++) {
            exs[i] = -siny * en[i] + cosy * ex[i];
            eys[i] = -cosy * en[i] - siny * ex[i];
        }
        return true;
    }

    /**
     * Phase wind-up correction model.
     * Ported from ppp.c model_phw().
     * @return true on success
     */
    static boolean modelPhw(GTime time, int sat, double[] rs,
                              double[] rr, PppState.SatState ssat) {
        double[] exs = new double[3], eys = new double[3], ek = new double[3];
        double[] exr = new double[3], eyr = new double[3];
        double[] eks = new double[3], ekr = new double[3];
        double[] ds = new double[3], dr = new double[3], drs = new double[3];
        double[] r = new double[3];

        // Satellite yaw attitude
        if (!satYaw(time, sat, rs, exs, eys)) return false;

        // Unit vector satellite to receiver
        for (int i = 0; i < 3; i++) r[i] = rr[i] - rs[i];
        if (SunMoonPos.normv3(r, ek) <= 0.0) return false;

        // Unit vectors of receiver antenna
        double[] pos = Coord.ecef2pos(rr);
        double[][] E = Coord.xyz2enu(pos);
        // x = north, y = west (sign-flipped east)
        exr[0] = E[1][0]; exr[1] = E[1][1]; exr[2] = E[1][2]; // North row
        eyr[0] = -E[0][0]; eyr[1] = -E[0][1]; eyr[2] = -E[0][2]; // -East = West

        // Phase wind-up effect
        MatrixUtil.cross3(ek, eys, eks);
        MatrixUtil.cross3(ek, eyr, ekr);
        for (int i = 0; i < 3; i++) {
            ds[i] = exs[i] - ek[i] * MatrixUtil.dot3(ek, exs) - eks[i];
            dr[i] = exr[i] - ek[i] * MatrixUtil.dot3(ek, exr) + ekr[i];
        }
        double nds = MatrixUtil.norm(ds, 3);
        double ndr = MatrixUtil.norm(dr, 3);
        if (nds <= 0.0 || ndr <= 0.0) return false;

        double cosp = MatrixUtil.dot3(ds, dr) / nds / ndr;
        cosp = Math.max(-1.0, Math.min(1.0, cosp));
        double ph = Math.acos(cosp) / 2.0 / PI;
        MatrixUtil.cross3(ds, dr, drs);
        if (MatrixUtil.dot3(ek, drs) < 0.0) ph = -ph;

        ssat.phw = ph + Math.floor(ssat.phw - ph + 0.5); // in cycle
        return true;
    }

    // ---------------------------------------------------------------
    // Satellite antenna offset (Step 6)
    // ---------------------------------------------------------------

    /**
     * Satellite antenna phase center offset correction.
     * Ported from preceph.c satantoff().
     * @param dant output: ECEF offset [3] (m)
     */
    static void satantoff(GTime time, double[] rs, int sat, Navigation nav, double[] dant) {
        dant[0] = dant[1] = dant[2] = 0.0;

        // Find antenna model for this satellite
        AntennaModel pcv = AntexReader.searchSat(nav.pcvs, sat, time);
        if (pcv == null) return;

        // Sun position in ECEF
        double[] rsun = new double[3], erpv = new double[5];
        SunMoonPos.sunmoonpos(time.gpst2utc(), erpv, rsun, null);

        // Unit vectors of satellite fixed coordinates
        double[] r = new double[3], ez = new double[3], es = new double[3];
        double[] ey = new double[3], ex = new double[3];
        for (int i = 0; i < 3; i++) r[i] = -rs[i];
        if (SunMoonPos.normv3(r, ez) <= 0.0) return;
        for (int i = 0; i < 3; i++) r[i] = rsun[i] - rs[i];
        if (SunMoonPos.normv3(r, es) <= 0.0) return;
        MatrixUtil.cross3(ez, es, r);
        if (SunMoonPos.normv3(r, ey) <= 0.0) return;
        MatrixUtil.cross3(ey, ez, ex);

        // IFLC coefficients
        int sys = SatelliteUtil.satsys(sat)[0];
        double[] freq = new double[2];
        if (sys == SYS_GPS || sys == SYS_QZS) { freq[0] = FREQL1; freq[1] = FREQL2; }
        else if (sys == SYS_GLO) { freq[0] = SignalUtil.sat2freq(sat, CODE_L1C, nav); freq[1] = SignalUtil.sat2freq(sat, CODE_L2C, nav); }
        else if (sys == SYS_GAL) { freq[0] = FREQL1; freq[1] = FREQE5b; }
        else if (sys == SYS_CMP) { freq[0] = FREQ1_CMP; freq[1] = FREQ2_CMP; }
        else if (sys == SYS_IRN) { freq[0] = FREQL5; freq[1] = FREQs; }
        else return;

        if (freq[0] == 0.0 || freq[1] == 0.0) return;
        double C1 = sq(freq[0]) / (sq(freq[0]) - sq(freq[1]));
        double C2 = -sq(freq[1]) / (sq(freq[0]) - sq(freq[1]));

        // IFLC antenna offset
        for (int i = 0; i < 3; i++) {
            double dant1 = pcv.off[0][0] * ex[i] + pcv.off[0][1] * ey[i] + pcv.off[0][2] * ez[i];
            double dant2 = (NFREQ > 1) ? pcv.off[1][0] * ex[i] + pcv.off[1][1] * ey[i] + pcv.off[1][2] * ez[i] : 0.0;
            dant[i] = C1 * dant1 + C2 * dant2;
        }
    }

    // ---------------------------------------------------------------
    // Receiver antenna model (Step 7)
    // ---------------------------------------------------------------

    /**
     * Receiver antenna model — compute range offsets per frequency.
     * Ported from rtkcmn.c antmodel().
     */
    static void antmodel(AntennaModel pcv, double[] del, double[] azel,
                           int opt, double[] dant) {
        if (pcv == null) { Arrays.fill(dant, 0, NFREQ, 0.0); return; }

        double cosel = Math.cos(azel[1]);
        double[] e = new double[3];
        e[0] = Math.sin(azel[0]) * cosel;
        e[1] = Math.cos(azel[0]) * cosel;
        e[2] = Math.sin(azel[1]);

        for (int i = 0; i < NFREQ; i++) {
            double[] off = new double[3];
            for (int j = 0; j < 3; j++) off[j] = pcv.off[i][j] + del[j];
            dant[i] = -(off[0] * e[0] + off[1] * e[1] + off[2] * e[2])
                       + (opt != 0 ? interpvar(90.0 - azel[1] * R2D, pcv.var[i]) : 0.0);
        }
    }

    /** Interpolate PCV variation by elevation angle. */
    private static double interpvar(double ang, double[] var) {
        double a = ang / 5.0;
        int i = (int) a;
        if (i < 0) return var[0];
        if (i >= 18) return var[18];
        return var[i] * (1.0 - a + i) + var[i + 1] * (a - i);
    }

    // ---------------------------------------------------------------
    // Time update functions
    // ---------------------------------------------------------------

    private static void udstatePpp(PppState rtk, ObsData[] obs, int n, Navigation nav) {
        udposPpp(rtk);
        udclkPpp(rtk);
        if (rtk.opt.tropopt == TROPOPT_EST || rtk.opt.tropopt == TROPOPT_ESTG) {
            udtropPpp(rtk);
        }
        udbiasPpp(rtk, obs, n, nav);
    }

    /** Temporal update of position. */
    private static void udposPpp(PppState rtk) {
        // Fixed mode
        if (rtk.opt.mode == PMODE_PPP_FIXED) {
            for (int i = 0; i < 3; i++) rtk.initx(rtk.opt.ru[i], 1E-8, i);
            return;
        }
        // Initialize position for first epoch
        if (MatrixUtil.norm(rtk.x, 3) <= 0.0) {
            for (int i = 0; i < 3; i++) rtk.initx(rtk.sol.rr[i], VAR_POS, i);
            if (rtk.opt.dynamics != 0) {
                for (int i = 3; i < 6; i++) rtk.initx(rtk.sol.rr[i], VAR_VEL, i);
                for (int i = 6; i < 9; i++) rtk.initx(1E-6, VAR_ACC, i);
            }
        }
        // Static PPP mode: add process noise
        if (rtk.opt.mode == PMODE_PPP_STATIC) {
            for (int i = 0; i < 3; i++) {
                rtk.P[i * (1 + rtk.nx)] += rtk.opt.prn[5] * rtk.opt.prn[5] * Math.abs(rtk.tt);
            }
            return;
        }
        // Kinematic without dynamics: reset each epoch
        if (rtk.opt.dynamics == 0) {
            for (int i = 0; i < 3; i++) rtk.initx(rtk.sol.rr[i], VAR_POS, i);
        }
    }

    /** Temporal update of clock — white noise reset every epoch. */
    private static void udclkPpp(PppState rtk) {
        for (int i = 0; i < NSYS; i++) {
            double dtr;
            if (rtk.opt.sateph == EPHOPT_PREC) {
                dtr = rtk.sol.dtr[0];
            } else {
                dtr = i == 0 ? rtk.sol.dtr[0] : rtk.sol.dtr[0] + rtk.sol.dtr[i];
            }
            rtk.initx(CLIGHT * dtr, VAR_CLK, IC(i, rtk.opt));
        }
    }

    /** Temporal update of troposphere. */
    private static void udtropPpp(PppState rtk) {
        int i = IT(rtk.opt);

        if (rtk.x[i] == 0.0) {
            // Initialize from SBAS MOPS model (matches C's udtrop_ppp using sbstropcorr)
            double[] pos = Coord.ecef2pos(rtk.sol.rr);
            double[] zazel = {0.0, Math.PI / 2.0};
            double[] varOut = new double[1];
            double ztd = Troposphere.sbstropcorr(rtk.sol.time, pos, zazel, varOut);
            double var = varOut[0];
            rtk.initx(ztd, var, i);

            if (rtk.opt.tropopt >= TROPOPT_ESTG) {
                for (int j = i + 1; j < i + 3; j++) rtk.initx(1E-6, VAR_GRA, j);
            }
        } else {
            rtk.P[i + i * rtk.nx] += rtk.opt.prn[2] * rtk.opt.prn[2] * Math.abs(rtk.tt);

            if (rtk.opt.tropopt >= TROPOPT_ESTG) {
                for (int j = i + 1; j < i + 3; j++) {
                    rtk.P[j + j * rtk.nx] += (rtk.opt.prn[2] * 0.1) * (rtk.opt.prn[2] * 0.1)
                                              * Math.abs(rtk.tt);
                }
            }
        }
    }

    /** Temporal update of phase biases. */
    private static void udbiasPpp(PppState rtk, ObsData[] obs, int n, Navigation nav) {
        // Reset slip flags
        for (int i = 0; i < MAXSAT; i++) {
            for (int j = 0; j < rtk.opt.nf; j++) rtk.ssat[i].slip[j] = 0;
        }

        // Detect cycle slips
        detslpLl(rtk, obs, n);
        detslpGf(rtk, obs, n, nav);
        detslpMw(rtk, obs, n, nav);

        for (int f = 0; f < NF(rtk.opt); f++) {
            // Reset phase-bias if outage counter exceeded
            for (int i = 0; i < MAXSAT; i++) {
                if (++rtk.ssat[i].outc[f] > rtk.opt.maxout) {
                    rtk.initx(0.0, 0.0, IB(i + 1, f, rtk.opt));
                }
            }

            double[] L = new double[NFREQ], P = new double[NFREQ];
            double[] Lc = new double[1], Pc = new double[1];
            double[] bias = new double[n];
            int[] slip = new int[n];
            int k = 0;
            double offset = 0.0;

            for (int i = 0; i < n && i < MAXOBS; i++) {
                int sat = obs[i].sat;
                int j = IB(sat, f, rtk.opt);
                corrMeas(obs[i], nav, rtk.ssat[sat - 1].azel, rtk.opt, null, null,
                         L, P, Lc, Pc);

                bias[i] = 0.0;
                if (rtk.opt.ionoopt == IONOOPT_IFLC) {
                    bias[i] = Lc[0] - Pc[0];
                    slip[i] = (rtk.ssat[sat - 1].slip[0] | rtk.ssat[sat - 1].slip[1]) != 0 ? 1 : 0;
                } else if (L[f] != 0.0 && P[f] != 0.0) {
                    double freq1 = SignalUtil.sat2freq(sat, obs[i].code[0], nav);
                    double freq2 = SignalUtil.sat2freq(sat, obs[i].code[f], nav);
                    slip[i] = rtk.ssat[sat - 1].slip[f];
                    double ion = 0.0;
                    if (f != 0 && obs[i].P[0] != 0.0 && obs[i].P[f] != 0.0
                        && freq1 != 0.0 && freq2 != 0.0) {
                        ion = (obs[i].P[0] - obs[i].P[f]) / (1.0 - sq(freq1 / freq2));
                    }
                    bias[i] = L[f] - P[f] + 2.0 * ion * sq(freq1 / freq2);
                }

                if (rtk.x[j] == 0.0 || slip[i] != 0 || bias[i] == 0.0) continue;
                offset += bias[i] - rtk.x[j];
                k++;
            }

            // Correct phase-code jump
            if (k >= 2 && Math.abs(offset / k) > 0.0005 * CLIGHT) {
                for (int i = 0; i < MAXSAT; i++) {
                    int j = IB(i + 1, f, rtk.opt);
                    if (rtk.x[j] != 0.0) rtk.x[j] += offset / k;
                }
            }

            for (int i = 0; i < n && i < MAXOBS; i++) {
                int sat = obs[i].sat;
                int j = IB(sat, f, rtk.opt);

                rtk.P[j + j * rtk.nx] += rtk.opt.prn[0] * rtk.opt.prn[0] * Math.abs(rtk.tt);

                if (bias[i] == 0.0 || (rtk.x[j] != 0.0 && slip[i] == 0)) continue;
                rtk.initx(bias[i], VAR_BIAS, IB(sat, f, rtk.opt));
            }
        }
    }

    // ---------------------------------------------------------------
    // Cycle slip detection
    // ---------------------------------------------------------------

    private static void detslpLl(PppState rtk, ObsData[] obs, int n) {
        int nf = Math.min(rtk.opt.nf, NFREQ);
        for (int i = 0; i < n && i < MAXOBS; i++) {
            for (int j = 0; j < nf; j++) {
                if (obs[i].L[j] == 0.0 || (obs[i].LLI[j] & (LLI_SLIP | LLI_HALFC)) == 0) continue;
                rtk.ssat[obs[i].sat - 1].slip[j] = LLI_SLIP;
            }
        }
    }

    private static void detslpGf(PppState rtk, ObsData[] obs, int n, Navigation nav) {
        for (int i = 0; i < n && i < MAXOBS; i++) {
            double g1 = gfmeas(obs[i], nav);
            if (g1 == 0.0) continue;

            double g0 = rtk.ssat[obs[i].sat - 1].gf[0];
            rtk.ssat[obs[i].sat - 1].gf[0] = g1;

            if (g0 != 0.0 && Math.abs(g1 - g0) > rtk.opt.thresslip) {
                for (int j = 0; j < rtk.opt.nf; j++) {
                    rtk.ssat[obs[i].sat - 1].slip[j] |= LLI_SLIP;
                }
            }
        }
    }

    private static void detslpMw(PppState rtk, ObsData[] obs, int n, Navigation nav) {
        for (int i = 0; i < n && i < MAXOBS; i++) {
            double w1 = mwmeas(obs[i], nav);
            if (w1 == 0.0) continue;

            double w0 = rtk.ssat[obs[i].sat - 1].mw[0];
            rtk.ssat[obs[i].sat - 1].mw[0] = w1;

            if (w0 != 0.0 && Math.abs(w1 - w0) > THRES_MW_JUMP) {
                for (int j = 0; j < rtk.opt.nf; j++) {
                    rtk.ssat[obs[i].sat - 1].slip[j] |= LLI_SLIP;
                }
            }
        }
    }

    /** Geometry-free phase measurement. */
    private static double gfmeas(ObsData obs, Navigation nav) {
        double freq1 = SignalUtil.sat2freq(obs.sat, obs.code[0], nav);
        double freq2 = SignalUtil.sat2freq(obs.sat, obs.code[1], nav);
        if (freq1 == 0.0 || freq2 == 0.0 || obs.L[0] == 0.0 || obs.L[1] == 0.0) return 0.0;
        return (obs.L[0] / freq1 - obs.L[1] / freq2) * CLIGHT;
    }

    /** Melbourne-Wubbena linear combination. */
    private static double mwmeas(ObsData obs, Navigation nav) {
        double freq1 = SignalUtil.sat2freq(obs.sat, obs.code[0], nav);
        double freq2 = SignalUtil.sat2freq(obs.sat, obs.code[1], nav);
        if (freq1 == 0.0 || freq2 == 0.0 || obs.L[0] == 0.0 || obs.L[1] == 0.0
            || obs.P[0] == 0.0 || obs.P[1] == 0.0) return 0.0;
        return (obs.L[0] - obs.L[1]) * CLIGHT / (freq1 - freq2)
             - (freq1 * obs.P[0] + freq2 * obs.P[1]) / (freq1 + freq2);
    }

    // ---------------------------------------------------------------
    // Corrected measurements (with antenna + wind-up support)
    // ---------------------------------------------------------------

    /**
     * Compute corrected phase and code measurements with iono-free LC.
     * Now accepts antenna corrections and phase wind-up.
     *
     * @param dantr receiver antenna corrections per freq (m), may be null
     * @param dants satellite antenna corrections per freq (m), may be null
     */
    static void corrMeas(ObsData obs, Navigation nav, double[] azel,
                          ProcessingOptions opt,
                          double[] dantr, double[] dants,
                          double[] L, double[] P,
                          double[] Lc, double[] Pc) {
        double[] freq = new double[NFREQ];
        int sys = SatelliteUtil.satsys(obs.sat)[0];

        for (int i = 0; i < opt.nf; i++) {
            L[i] = P[i] = 0.0;
            freq[i] = SignalUtil.sat2freq(obs.sat, obs.code[i], nav);
            if (freq[i] == 0.0 || obs.L[i] == 0.0 || obs.P[i] == 0.0) continue;

            // Phase and code with antenna + wind-up corrections
            L[i] = obs.L[i] * CLIGHT / freq[i];
            P[i] = obs.P[i];

            // Receiver antenna correction
            if (dantr != null && i < dantr.length) {
                L[i] -= dantr[i];
                P[i] -= dantr[i];
            }
            // Satellite antenna correction
            if (dants != null && i < dants.length) {
                L[i] -= dants[i];
                P[i] -= dants[i];
            }

            // Apply code bias corrections from file
            int frq;
            if (sys == SYS_GAL && (i == 1 || i == 2)) frq = 3 - i;
            else frq = i;
            if (frq >= MAX_CODE_BIAS_FREQS) continue;
            int biasIx = Spp.code2biasIx(sys, obs.code[i]);
            if (biasIx > 0) {
                P[i] += nav.cbias[obs.sat - 1][frq][biasIx - 1];
            }
        }

        // Iono-free LC
        Lc[0] = Pc[0] = 0.0;
        int frq2 = L[1] == 0.0 ? 2 : 1;
        if (frq2 >= opt.nf) frq2 = 1;
        if (freq[0] == 0.0 || freq[frq2] == 0.0) return;
        double C1 = sq(freq[0]) / (sq(freq[0]) - sq(freq[frq2]));
        double C2 = -sq(freq[frq2]) / (sq(freq[0]) - sq(freq[frq2]));

        if (L[0] != 0.0 && L[frq2] != 0.0) Lc[0] = C1 * L[0] + C2 * L[frq2];
        if (P[0] != 0.0 && P[frq2] != 0.0) Pc[0] = C1 * P[0] + C2 * P[frq2];
    }

    // ---------------------------------------------------------------
    // Residuals
    // ---------------------------------------------------------------

    /**
     * PPP phase and code residuals.
     *
     * @return post mode: 1 if converged (no large outlier), 0 otherwise.
     *         pre mode: number of valid measurements.
     */
    private static int pppRes(int post, ObsData[] obs, int n,
                               double[] rs, double[] dts, double[] varRs,
                               int[] svh, double[] dr, int[] exc,
                               Navigation nav, double[] x, PppState rtk,
                               double[] v, double[] H, double[] R,
                               double[] azel) {
        ProcessingOptions opt = rtk.opt;
        int nx = rtk.nx;
        int nv = 0;
        int stat = 1;
        int ne = 0;
        double[] var = new double[MAXOBS * 2 * NFREQ];
        double[] ve = new double[MAXOBS * 2 * NFREQ];
        int[] obsi = new int[MAXOBS * 2 * NFREQ];
        int[] frqi = new int[MAXOBS * 2 * NFREQ];

        for (int i = 0; i < MAXSAT; i++) {
            for (int j = 0; j < opt.nf; j++) rtk.ssat[i].vsat[j] = 0;
        }

        double[] rr = new double[3];
        for (int i = 0; i < 3; i++) rr[i] = x[i] + dr[i];
        double[] pos = new double[3];
        { double[] tmp = Coord.ecef2pos(rr); pos[0]=tmp[0]; pos[1]=tmp[1]; pos[2]=tmp[2]; }

        // Search receiver antenna model
        AntennaModel rcvPcv = null;
        if (!nav.pcvs.isEmpty() && opt.anttype[0] != null && !opt.anttype[0].isEmpty()) {
            rcvPcv = AntexReader.searchReceiver(nav.pcvs, opt.anttype[0]);
        }

        double[] e = new double[3];
        double[] L = new double[NFREQ], P = new double[NFREQ];
        double[] Lc = new double[1], Pc = new double[1];
        double[] dtdx = new double[3];

        for (int i = 0; i < n && i < MAXOBS; i++) {
            int sat = obs[i].sat;

            double r = Geometry.geodist(rs, i * 6, rr, e);
            if (r <= 0.0) { exc[i] = 1; continue; }

            double el = Geometry.satazel(pos, e, azel, i * 2);
            if (el < opt.elmin) { exc[i] = 1; continue; }

            int sys = SatelliteUtil.satsys(sat)[0];
            if (sys == SYS_NONE || rtk.ssat[sat - 1].vs == 0 ||
                SatelliteUtil.satexclude(sat, varRs[i], svh[i], opt) ||
                exc[i] != 0) { exc[i] = 1; continue; }

            // SNR mask check
            if (opt.snrmask.ena[0] != 0) {
                if (Spp.testsnr(0, 0, azel[i * 2 + 1], obs[i].SNR[0], opt.snrmask)) {
                    exc[i] = 1; continue;
                }
                if (opt.ionoopt == IONOOPT_IFLC) {
                    int f2 = Spp.seliflc(opt.nf, sys);
                    if (Spp.testsnr(0, 0, azel[i * 2 + 1], obs[i].SNR[f2], opt.snrmask)) {
                        exc[i] = 1; continue;
                    }
                }
            }

            // Satellite antenna offset (Step 6)
            double[] dants = new double[NFREQ];
            if (opt.posopt[0] != 0 && !nav.pcvs.isEmpty()) {
                satantoff(obs[i].time, rs, sat, nav, dants);
                // Apply ECEF offset to geometric range
                r += dants[0] * e[0] + dants[1] * e[1] + dants[2] * e[2];
                // Reset dants for corrMeas (already applied to range)
                Arrays.fill(dants, 0.0);
            }

            // Receiver antenna model (Step 7)
            double[] dantr = new double[NFREQ];
            if (opt.posopt[1] != 0 && rcvPcv != null) {
                antmodel(rcvPcv, opt.antdel[0],
                         new double[]{azel[i * 2], azel[i * 2 + 1]}, 1, dantr);
            }

            // Phase wind-up (Step 4)
            double phw = 0.0;
            if (opt.posopt[2] != 0) {
                double[] rsArr = new double[]{rs[i * 6], rs[i * 6 + 1], rs[i * 6 + 2],
                                              rs[i * 6 + 3], rs[i * 6 + 4], rs[i * 6 + 5]};
                modelPhw(obs[i].time, sat, rsArr, rr, rtk.ssat[sat - 1]);
                phw = rtk.ssat[sat - 1].phw;
            }

            // Troposphere model
            double dtrp = 0.0, vart = 0.0;
            if (!modelTrop(obs[i].time, pos, azel, i * 2, opt, x, dtdx, rtk, nav)) continue;
            dtrp = tropResult[0];
            vart = tropResult[1];

            // Corrected measurements (with antenna corrections)
            corrMeas(obs[i], nav, new double[]{azel[i * 2], azel[i * 2 + 1]},
                     opt, dantr, null, L, P, Lc, Pc);

            // Apply phase wind-up to phase measurements
            if (phw != 0.0) {
                for (int fi = 0; fi < opt.nf; fi++) {
                    if (L[fi] != 0.0) {
                        double freq = SignalUtil.sat2freq(sat, obs[i].code[fi], nav);
                        if (freq != 0.0) L[fi] -= phw * CLIGHT / freq;
                    }
                }
                // For IFLC, recompute Lc after wind-up correction
                if (opt.ionoopt == IONOOPT_IFLC) {
                    double[] freqArr = new double[NFREQ];
                    for (int fi = 0; fi < opt.nf; fi++)
                        freqArr[fi] = SignalUtil.sat2freq(sat, obs[i].code[fi], nav);
                    int frq2 = L[1] == 0.0 ? 2 : 1;
                    if (frq2 >= opt.nf) frq2 = 1;
                    if (freqArr[0] != 0.0 && freqArr[frq2] != 0.0) {
                        double C1 = sq(freqArr[0]) / (sq(freqArr[0]) - sq(freqArr[frq2]));
                        double C2 = -sq(freqArr[frq2]) / (sq(freqArr[0]) - sq(freqArr[frq2]));
                        if (L[0] != 0.0 && L[frq2] != 0.0) Lc[0] = C1 * L[0] + C2 * L[frq2];
                    }
                }
            }

            // Stack phase and code residuals
            for (int j = 0; j < 2 * NF(opt); j++) {
                int code = j % 2;  // 0=phase, 1=code
                int frq = j / 2;
                double y, C = 0.0, bias = 0.0, dcb = 0.0;

                if (opt.ionoopt == IONOOPT_IFLC) {
                    y = code == 0 ? Lc[0] : Pc[0];
                } else {
                    y = code == 0 ? L[frq] : P[frq];
                    if (y == 0.0) continue;
                    double freq = SignalUtil.sat2freq(sat, obs[i].code[frq], nav);
                    if (freq == 0.0) continue;
                    C = sq(FREQL1 / freq) * (code == 0 ? -1.0 : 1.0);
                }
                if (y == 0.0) continue;

                if (H != null) {
                    for (int k = 0; k < nx; k++) H[k + nx * nv] = 0.0;
                    for (int k = 0; k < 3; k++) H[k + nx * nv] = -e[k];
                }

                // Receiver clock
                int ck;
                switch (sys) {
                    case SYS_GLO: ck = 1; break;
                    case SYS_GAL: ck = 2; break;
                    case SYS_CMP: ck = 3; break;
                    case SYS_IRN: ck = 4; break;
                    default:      ck = 0; break;
                }
                double cdtr = x[IC(ck, opt)];
                if (H != null) {
                    H[IC(ck, opt) + nx * nv] = 1.0;

                    // Troposphere partials
                    if (opt.tropopt == TROPOPT_EST || opt.tropopt == TROPOPT_ESTG) {
                        int ntrop = opt.tropopt >= TROPOPT_ESTG ? 3 : 1;
                        for (int k = 0; k < ntrop; k++) {
                            H[IT(opt) + k + nx * nv] = tropDtdx[k];
                        }
                    }
                }

                // Phase bias
                if (code == 0) {
                    bias = x[IB(sat, frq, opt)];
                    if (bias == 0.0) continue;
                    if (H != null) H[IB(sat, frq, opt) + nx * nv] = 1.0;
                }

                // Residual
                double res = y - (r + cdtr - CLIGHT * dts[i * 2] + dtrp + C * 0.0 + dcb + bias);

                if (v != null) v[nv] = res;

                if (code == 0) rtk.ssat[sat - 1].resc[frq] = res;
                else           rtk.ssat[sat - 1].resp[frq] = res;

                // Variance
                double vv = varerr(sys, azel[1 + i * 2], rtk.ssat[sat - 1].snr_rover[frq],
                                   j, opt);
                vv += vart + varRs[i];
                if (sys == SYS_GLO && code == 1) vv += VAR_GLO_IFB;
                var[nv] = vv;

                // Pre-fit residual rejection
                if (post == 0 && opt.maxinno[code] > 0.0 && Math.abs(res) > opt.maxinno[code]) {
                    exc[i] = 1;
                    rtk.ssat[sat - 1].rejc[frq]++;
                    continue;
                }

                // Post-fit residual check
                if (post > 0 && Math.abs(res) > Math.sqrt(vv) * THRES_REJECT) {
                    obsi[ne] = i; frqi[ne] = j; ve[ne] = res; ne++;
                }

                if (code == 0) rtk.ssat[sat - 1].vsat[frq] = 1;
                nv++;
            }
        }

        // Reject satellite with largest post-fit residual
        if (post > 0 && ne > 0) {
            double vmax = ve[0]; int maxobs = obsi[0]; int maxfrq = frqi[0];
            for (int j = 1; j < ne; j++) {
                if (Math.abs(ve[j]) > Math.abs(vmax)) {
                    vmax = ve[j]; maxobs = obsi[j]; maxfrq = frqi[j];
                }
            }
            int sat = obs[maxobs].sat;
            exc[maxobs] = 1;
            rtk.ssat[sat - 1].rejc[maxfrq % 2]++;
            stat = 0;
        }

        if (R != null) {
            Arrays.fill(R, 0, nv * nv, 0.0);
            for (int i = 0; i < nv; i++) R[i + i * nv] = var[i];
        }

        return post > 0 ? stat : nv;
    }

    // Thread-local storage for trop model results (avoids extra arrays)
    private static final double[] tropResult = new double[2]; // [dtrp, vart]
    private static final double[] tropDtdx = new double[3];

    /**
     * Troposphere model for PPP.
     */
    private static boolean modelTrop(GTime time, double[] pos, double[] azel, int azelOff,
                                      ProcessingOptions opt, double[] x, double[] dtdx,
                                      PppState rtk, Navigation nav) {
        double el = azel[azelOff + 1];
        if (el <= 0.0) return false;

        Arrays.fill(tropDtdx, 0.0);

        if (opt.tropopt == TROPOPT_SAAS) {
            double dtrp = Troposphere.tropmodel(time, pos,
                    new double[]{azel[azelOff], azel[azelOff + 1]}, REL_HUMI);
            tropResult[0] = dtrp;
            tropResult[1] = ERR_SAAS * ERR_SAAS;
            return true;
        }
        if (opt.tropopt == TROPOPT_EST || opt.tropopt == TROPOPT_ESTG) {
            // Precise troposphere model
            double[] zazel = {0.0, Math.PI / 2.0};
            double zhd = Troposphere.tropmodel(time, pos, zazel, 0.0);

            double[] mapfw = new double[1];
            double m_h = Troposphere.tropmapf(time, pos,
                    new double[]{azel[azelOff], azel[azelOff + 1]}, mapfw);
            double m_w = mapfw[0];

            double[] trp = new double[3];
            int it = IT(opt);
            trp[0] = x[it];
            if (opt.tropopt >= TROPOPT_ESTG && it + 2 < x.length) {
                trp[1] = x[it + 1];
                trp[2] = x[it + 2];
            }

            // Gradient correction
            double grad_n = 0.0, grad_e = 0.0;
            if (azel[azelOff + 1] > 0.0) {
                double cotz = 1.0 / Math.tan(azel[azelOff + 1]);
                grad_n = m_w * cotz * Math.cos(azel[azelOff]);
                grad_e = m_w * cotz * Math.sin(azel[azelOff]);
                m_w += grad_n * trp[1] + grad_e * trp[2];
                tropDtdx[1] = grad_n * (trp[0] - zhd);
                tropDtdx[2] = grad_e * (trp[0] - zhd);
            }
            tropDtdx[0] = m_w;

            tropResult[0] = m_h * zhd + m_w * (trp[0] - zhd);
            tropResult[1] = 0.01 * 0.01; // 0.01^2
            return true;
        }
        return false;
    }

    // ---------------------------------------------------------------
    // Measurement error variance
    // ---------------------------------------------------------------

    /**
     * Measurement error variance model.
     */
    static double varerr(int sys, double el, double snrRover, int f,
                          ProcessingOptions opt) {
        int frq = f / 2;
        int code = f % 2;
        double fact = 1.0;

        if (code != 0) fact = opt.eratio[frq];
        if (fact <= 0.0) fact = opt.eratio[0];

        switch (sys) {
            case SYS_GPS: fact *= EFACT_GPS; break;
            case SYS_GLO: fact *= EFACT_GLO; break;
            case SYS_GAL: fact *= EFACT_GAL; break;
            case SYS_SBS: fact *= EFACT_SBS; break;
            case SYS_QZS: fact *= EFACT_QZS; break;
            case SYS_CMP: fact *= EFACT_CMP; break;
            case SYS_IRN: fact *= EFACT_IRN; break;
            default:      fact *= EFACT_GPS; break;
        }

        if ((sys == SYS_GPS || sys == SYS_QZS) && frq == 2) {
            fact *= EFACT_GPS_L5;
        }

        double a = fact * opt.err[1];
        double b = fact * opt.err[2];
        double sinel = Math.sin(el);
        double v = a * a + b * b / (sinel * sinel);

        // IFLC scaling factor
        if (opt.ionoopt == IONOOPT_IFLC) v *= 9.0; // 3^2

        return v;
    }

    // ---------------------------------------------------------------
    // Solution status update
    // ---------------------------------------------------------------

    private static void updateStat(PppState rtk, ObsData[] obs, int n, int stat) {
        ProcessingOptions opt = rtk.opt;

        rtk.sol.ns = 0;
        for (int i = 0; i < n && i < MAXOBS; i++) {
            for (int j = 0; j < opt.nf; j++) {
                if (rtk.ssat[obs[i].sat - 1].vsat[j] == 0) continue;
                rtk.ssat[obs[i].sat - 1].lock[j]++;
                rtk.ssat[obs[i].sat - 1].outc[j] = 0;
                if (j == 0) rtk.sol.ns++;
            }
        }
        rtk.sol.stat = rtk.sol.ns < MIN_NSAT_SOL ? SOLQ_NONE : stat;

        // Copy state to solution
        for (int i = 0; i < 3; i++) {
            rtk.sol.rr[i] = rtk.x[i];
            rtk.sol.qr[i] = (float) rtk.P[i + i * rtk.nx];
        }
        rtk.sol.qr[3] = (float) rtk.P[1];
        rtk.sol.qr[4] = (float) rtk.P[2 + rtk.nx];
        rtk.sol.qr[5] = (float) rtk.P[2];

        rtk.sol.dtr[0] = rtk.x[IC(0, opt)] / CLIGHT;
        rtk.sol.dtr[1] = (rtk.x[IC(1, opt)] - rtk.x[IC(0, opt)]) / CLIGHT;
        rtk.sol.dtr[2] = (rtk.x[IC(2, opt)] - rtk.x[IC(0, opt)]) / CLIGHT;
        rtk.sol.dtr[3] = (rtk.x[IC(3, opt)] - rtk.x[IC(0, opt)]) / CLIGHT;

        // Update SNR
        for (int i = 0; i < n && i < MAXOBS; i++) {
            for (int j = 0; j < opt.nf; j++) {
                rtk.ssat[obs[i].sat - 1].snr_rover[j] = obs[i].SNR[j];
            }
        }

        // Update slip counts
        for (int i = 0; i < MAXSAT; i++) {
            for (int j = 0; j < opt.nf; j++) {
                if ((rtk.ssat[i].slip[j] & (LLI_SLIP | LLI_HALFC)) != 0) {
                    rtk.ssat[i].slipc[j]++;
                }
            }
        }
    }

    private static double sq(double x) { return x * x; }

    private static final int MAXOBS = 96;
}
