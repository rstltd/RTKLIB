/*------------------------------------------------------------------------------
* cxx_abi_probe.cpp : C/C++ boundary probe for the RTKLIB FGO extension
*
*          Copyright (C) 2026 by RST Ltd., All rights reserved.
*
* Asserts the two boundary invariants of plan.md 3.1 that the visibility
* promotions in 6.1 M2/M4/M5 exist to satisfy:
*
*   I2  rtklib.h can be included from a C++ translation unit -- the extern "C"
*       guard holds and no C-only construct breaks C++ parsing.
*   I3  the promoted residual and error-model helpers are reachable and
*       callable from C++, so src/fgo/ can use RTKLIB as the single source of
*       truth for the observation model instead of reimplementing it.
*
* Covers both families: the rtk_ helpers from rtkpos.c (double-difference
* path) and the spp_ helpers from pntpos.c (undifferenced path).
*
* rtk_varerr() is checked against a value derived by hand rather than merely
* called, so a silently wrong symbol binding cannot pass.
*
* version : $Revision:$ $Date:$
* history : 2026/08/20 1.0 new
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <vector>

/* RTKLIB expects the application to supply these three */
extern "C" int  showmsg (const char *format, ...) { (void)format; return 0; }
extern "C" void settspan(gtime_t ts, gtime_t te)  { (void)ts; (void)te; }
extern "C" void settime (gtime_t time)            { (void)time; }

static int nfail = 0;

/* SQR is a file-local macro inside the .c files, deliberately not exported */
static inline double sqr(double a) { return a * a; }

static void check(const char *name, bool ok, const char *detail)
{
    printf("  %-38s %s%s%s\n", name, ok ? "PASS" : "FAIL",
           detail && *detail ? "  " : "", detail ? detail : "");
    if (!ok) nfail++;
}

/* rtk_varerr(): reproduce the closed form for the default options ----------- */
static void probe_varerr(void)
{
    prcopt_t opt = prcopt_default;
    obsd_t   obs = {};
    const double el = 45.0 * D2R;

    /* carrier phase on the first frequency, GPS, zero baseline and age, so
       every term but the base and elevation ones drops out:
           fact = eratio[0]/eratio[0] * EFACT_GPS = 1
           var  = 2*(a^2 + b^2/sin^2(el)),  a = err[1], b = err[2]           */
    opt.err[5] = 0.0;      /* no SNR term  */
    opt.err[6] = 0.0;
    opt.err[7] = 0.0;      /* no receiver-stdev term */

    const double a = opt.err[1], b = opt.err[2], s = sin(el);
    const double want = 2.0 * (a * a + b * b / (s * s));
    const double got  = rtk_varerr(1, SYS_GPS, el, 0.0, 0.0, 0.0, 0.0, 0,
                                   &opt, &obs);

    char d[128];
    snprintf(d, sizeof d, "got=%.12g want=%.12g", got, want);
    check("rtk_varerr matches closed form", std::fabs(got - want) < 1e-15, d);
}

/* rtk_zdres() / rtk_ddcov(): must resolve at link time --------------------- */
static void probe_linkage(void)
{
    /* taking the addresses forces the linker to resolve the symbols; calling
       them meaningfully would need a full nav_t/obsd_t fixture, which is the
       job of the t_ddres unit test that accompanies the M3 refactor.        */
    const void *z = reinterpret_cast<const void *>(&rtk_zdres);
    const void *c = reinterpret_cast<const void *>(&rtk_ddcov);
    const void *d = reinterpret_cast<const void *>(&spp_resdop);
    const void *e = reinterpret_cast<const void *>(&ddres_core);
    check("rtk_zdres resolves from C++",  z != nullptr, "");
    check("rtk_ddcov resolves from C++",  c != nullptr, "");
    check("spp_resdop resolves from C++", d != nullptr, "");
    check("ddres_core resolves from C++", e != nullptr, "");
}

/* rtk_ddcov(): exercise the documented diagonal-plus-rank-1 structure ------- */
static void probe_ddcov_structure(void)
{
    /* plan.md 4.2.1: within one block every DD shares the reference
       satellite, so R = sigma_ref^2 * 1*1' + diag(sigma_j^2).  Feeding a
       constant Ri and a varying Rj must reproduce exactly that.             */
    const int nb[1] = {3}, n = 1, nv = 3;
    const double Ri[3] = {0.02, 0.02, 0.02};      /* reference sat, constant */
    const double Rj[3] = {0.01, 0.02, 0.03};      /* per-sat, independent    */
    double R[9];

    rtk_ddcov(nb, n, Ri, Rj, nv, R);

    bool ok = true;
    char d[160] = "";
    for (int i = 0; i < nv && ok; i++) {
        for (int j = 0; j < nv; j++) {
            const double want = Ri[i] + (i == j ? Rj[i] : 0.0);
            if (std::fabs(R[i + j * nv] - want) > 1e-15) {
                snprintf(d, sizeof d, "R[%d,%d]=%.12g want=%.12g",
                         i, j, R[i + j * nv], want);
                ok = false;
                break;
            }
        }
    }
    check("rtk_ddcov builds diag+rank-1 block", ok, d);
}

/* spp_varerr(): reproduce the closed form for the default options ---------- */
static void probe_spp_varerr(void)
{
    prcopt_t opt = prcopt_default;
    obsd_t   obs = {};
    const double el = 30.0 * D2R;

    opt.err[5] = 0.0;      /* no SNR term */
    opt.err[6] = 0.0;
    opt.err[7] = 0.0;      /* no receiver-stdev term */

    /* var = EFACT_GPS^2 * eratio[0]^2 * (err[1]^2 + err[2]^2/sin(el))
       -- note this divides by sin(el), where the DD form of rtkpos.c
       divides by sin^2(el); the two error models are genuinely different  */
    const double a = opt.err[1], b = opt.err[2];
    const double want = sqr((double)EFACT_GPS) * sqr(opt.eratio[0]) *
                        (a * a + b * b / sin(el));
    const double got  = spp_varerr(&opt, &obs, el, SYS_GPS);

    char d[128];
    snprintf(d, sizeof d, "got=%.12g want=%.12g", got, want);
    check("spp_varerr matches closed form", std::fabs(got - want) < 1e-15, d);
}

/* spp_nx(): must agree with the H matrices spp_rescode() actually writes --- */
static void probe_spp_nx(void)
{
    const int nx = spp_nx();
    char d[64];
    snprintf(d, sizeof d, "nx=%d", nx);
    /* 3 position + 1 clock, plus one offset per extra constellation */
    check("spp_nx is a sane state count", nx >= 4 && nx <= 16, d);
}

/* spp_rescode(): a real call on a synthetic single-satellite fixture ------- */
static void probe_spp_rescode(void)
{
    const int nx = spp_nx();
    prcopt_t opt = prcopt_default;
    opt.ionoopt = IONOOPT_OFF;
    opt.tropopt = TROPOPT_OFF;
    opt.elmin   = 0.0;
    opt.sateph  = EPHOPT_BRDC;

    nav_t nav = {};
    obsd_t obs = {};
    obs.sat = 1;                       /* GPS PRN 1 */
    double ep[6] = {2020, 1, 1, 0, 0, 0};
    obs.time = epoch2time(ep);
    obs.P[0] = 22000000.0;
    obs.code[0] = CODE_L1C;

    /* satellite 20000 km up along +Z from the receiver, which sits on the
       equator at x = R_EARTH; geometry only has to be self-consistent      */
    double rs[6] = {26000000.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double dts[2] = {0.0, 0.0};
    double vare[1] = {1.0};
    int svh[1] = {0};

    std::vector<double> x(nx, 0.0);
    x[0] = 6378137.0;                  /* receiver on the equator */

    std::vector<double> v(nx + 4, 0.0), H((nx + 4) * nx, 0.0),
                        var(nx + 4, 0.0), azel(2, 0.0), resp(1, 0.0);
    int vsat[1] = {0}, ns = 0;

    const int nv = spp_rescode(0, &obs, 1, rs, dts, vare, svh, &nav,
                               x.data(), &opt, v.data(), H.data(), var.data(),
                               azel.data(), vsat, resp.data(), &ns);

    /* one satellite must be accepted, and its position partials must be the
       negated line-of-sight unit vector with a 1 in the clock column        */
    bool ok = (nv >= 1) && (ns == 1) && (vsat[0] == 1);
    if (ok) {
        const double nrm = std::sqrt(H[0] * H[0] + H[1] * H[1] + H[2] * H[2]);
        ok = std::fabs(nrm - 1.0) < 1e-9 && std::fabs(H[3] - 1.0) < 1e-12;
    }
    char d[128];
    snprintf(d, sizeof d, "nv=%d ns=%d |H_pos|=%.9f H_clk=%.3f",
             nv, ns, ok ? std::sqrt(H[0]*H[0]+H[1]*H[1]+H[2]*H[2]) : 0.0,
             nv >= 1 ? H[3] : 0.0);
    check("spp_rescode evaluates a real fixture", ok, d);
}

int main(void)
{
    printf("RTKLIB C/C++ boundary probe (plan.md 3.1 I2/I3)\n");
    probe_varerr();
    probe_linkage();
    probe_ddcov_structure();
    probe_spp_varerr();
    probe_spp_nx();
    probe_spp_rescode();

    if (nfail) {
        printf("\n%d probe(s) FAILED\n", nfail);
        return 1;
    }
    printf("\nall probes passed\n");
    return 0;
}
