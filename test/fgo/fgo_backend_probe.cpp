/*------------------------------------------------------------------------------
* fgo_backend_probe.cpp : the GTSAM backend, when ENABLE_FGO is on
*
*          Copyright (C) 2026 by RST Ltd., All rights reserved.
*
* Checks what an ENABLE_FGO=ON build must do that the stub does not, and what
* it must do IDENTICALLY to the stub.  The second half matters as much as the
* first: rtkpos.c calls these without knowing which is linked, so anything
* that differs beyond "a backend exists" is a trap.
*
* version : $Revision:$ $Date:$
* history : 2026/08/21 1.0 new
*-----------------------------------------------------------------------------*/
#include "fgo_rtklib.h"
#include "fgo_config.h"
#include "fgo_factor.h"

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <string>

static int nfail = 0;

static void check(const char *name, bool ok, const char *fmt = nullptr, ...)
{
    char d[192] = "";
    if (fmt) { va_list ap; va_start(ap, fmt); vsnprintf(d, sizeof d, fmt, ap); va_end(ap); }
    std::printf("  %-46s %s%s%s\n", name, ok ? "PASS" : "FAIL",
                d[0] ? "  " : "", d);
    if (!ok) nfail++;
}

int main(int argc, char **argv)
{
    std::printf("FGO backend (ENABLE_FGO=ON)\n");

    check("fgo_enabled reports a real backend", fgo_enabled() == 1,
          "fgo_enabled()=%d", fgo_enabled());

    rtk_t rtk;
    prcopt_t opt = prcopt_default;
    opt.fgo_solver = FGO_SOLVER_ISAM2;
    opt.fgo_maxiter = 7;
    rtkinit(&rtk, &opt);

    check("rtkinit leaves rtk->fgo NULL", rtk.fgo == nullptr);

    int rc = fgo_init(&rtk, &opt);
    check("fgo_init succeeds and binds a context",
          rc == FGO_OK && rtk.fgo != nullptr, "rc=%d", rc);

    void *first = rtk.fgo;
    rc = fgo_init(&rtk, &opt);
    check("fgo_init is idempotent",
          rc == FGO_OK && rtk.fgo == first, "rc=%d", rc);

    /* Not yet a solver.  It must SAY so rather than claim success, because
       rtkpos.c reads anything but FGO_OK as "fall back to the EKF". */
    rc = fgo_process_epoch(&rtk, nullptr, 0, 0, nullptr);
    check("fgo_process_epoch reports non-convergence",
          rc == FGO_ERR_NOTCONV, "rc=%d", rc);

    /* Same shape as the stub: buffer terminated even when declining. */
    char buff[64];
    std::memset(buff, 'x', sizeof buff);
    rc = fgo_insight_json(&rtk, buff, sizeof buff);
    check("fgo_insight_json declines and terminates the buffer",
          rc < 0 && buff[0] == '\0', "rc=%d", rc);

    fgo_reset(&rtk);
    check("fgo_reset keeps the context alive", rtk.fgo != nullptr);

    fgo_free(&rtk);
    check("fgo_free releases and clears", rtk.fgo == nullptr);
    fgo_free(&rtk);
    fgo_free(nullptr);
    fgo_reset(nullptr);
    check("free/reset tolerate NULL and repeat calls", rtk.fgo == nullptr);

    /* An unbound context must be refused, not dereferenced. */
    rc = fgo_process_epoch(&rtk, nullptr, 0, 0, nullptr);
    check("process_epoch without init is refused",
          rc == FGO_ERR_DISABLED, "rc=%d", rc);

    rtkfree(&rtk);

    /* Configuration: clamped, not trusted (plan.md 6.4 M14 and the option
       file being editable by hand). */
    {
        prcopt_t bad = prcopt_default;
        bad.fgo_maxiter = 0;
        bad.fgo_window = -1.0;
        bad.fgo_scaleclamp[0] = 9.0;   /* reversed */
        bad.fgo_scaleclamp[1] = 0.1;
        fgo::Config c = fgo::Config::fromPrcopt(bad);
        check("nonsense options are clamped, not propagated",
              c.maxIter == FGO_DEF_MAXITER && c.windowSec == FGO_DEF_WINDOW &&
              c.robust.scaleMin < c.robust.scaleMax,
              "maxiter=%d window=%.1f clamp=[%.2f,%.2f]",
              c.maxIter, c.windowSec, c.robust.scaleMin, c.robust.scaleMax);

        prcopt_t good = prcopt_default;
        good.fgo_solver = FGO_SOLVER_SLIDING;
        good.fgo_window = 120.5;
        good.fgo_maxiter = 7;
        fgo::Config g = fgo::Config::fromPrcopt(good);
        check("valid options survive conversion",
              g.solver == FGO_SOLVER_SLIDING && g.windowSec == 120.5 &&
              g.maxIter == 7, "%s", g.describe().c_str());
    }

    /* Key schema: an overflow must be reported, not silently corrupt the
       character byte and alias a different variable. */
    {
        bool threw = false;
        try { (void)fgo::keyPos(fgo::kMaxKeyIndex + 1); }
        catch (const std::exception &) { threw = true; }
        check("key index overflow is reported", threw);

        bool ok = true;
        try {
            gtsam::Symbol s(fgo::keyPos(12345));
            ok = (s.chr() == 'p' && s.index() == 12345);
            gtsam::Symbol b(fgo::keyBias(999));
            ok = ok && (b.chr() == 'b' && b.index() == 999);
            /* distinct schemas must not collide */
            ok = ok && (fgo::keyPos(7) != fgo::keyVel(7)) &&
                       (fgo::keyTrop(3, 0) != fgo::keyTrop(3, 1)) &&
                       (fgo::keyIono(2, 5) != fgo::keyIono(2, 6));
        } catch (const std::exception &) { ok = false; }
        check("keys round-trip and do not collide", ok);

        /* A composite key packs two numbers into one index, so an out-of-range
           component aliases rather than overflows: keyTrop(k,2) would be
           keyTrop(k+1,0).  Tying two unrelated quantities to one variable is
           far worse than a crash and would be almost undiagnosable from the
           solution, so each component is range-checked. */
        struct { const char *what; bool threw; } aliasing[] = {
            {"keyTrop rcv=2",     false},
            {"keyIono sat=0",     false},
            {"keyIono sat=MAXSAT+1", false},
            {"keyGloIcb f=NFREQ", false},
            {"keyIono epoch overflow", false},
        };
        try { (void)fgo::keyTrop(5, 2); }        catch (const std::exception &) { aliasing[0].threw = true; }
        try { (void)fgo::keyIono(5, 0); }        catch (const std::exception &) { aliasing[1].threw = true; }
        try { (void)fgo::keyIono(5, MAXSAT + 1); } catch (const std::exception &) { aliasing[2].threw = true; }
        try { (void)fgo::keyGloIcb(NFREQ); }     catch (const std::exception &) { aliasing[3].threw = true; }
        /* the multiplication itself must not wrap before the check */
        try { (void)fgo::keyIono(fgo::kMaxKeyIndex, 1); } catch (const std::exception &) { aliasing[4].threw = true; }

        std::string bad2;
        for (const auto &a : aliasing) if (!a.threw) { bad2 += a.what; bad2 += " "; }
        check("out-of-range key components are rejected", bad2.empty(),
              "%s", bad2.empty() ? "" : ("accepted: " + bad2).c_str());

        /* and the valid boundary values must still work */
        bool edges = true;
        try {
            (void)fgo::keyTrop(0, 1);
            (void)fgo::keyIono(0, MAXSAT);
            (void)fgo::keyGloIcb(NFREQ - 1);
        } catch (const std::exception &) { edges = false; }
        check("valid boundary components are accepted", edges);
    }

    /* ---- the factor, on real observations -------------------------------
       This is gate G4's central check (plan.md 6.11): the analytic Jacobian
       against GTSAM's own numerical derivative.  A sign error here produces a
       Jacobian of the right magnitude pointing the wrong way, which converges
       to the wrong answer without ever failing loudly. */
    if (argc > 3) {
        obs_t obs = {};
        nav_t nav = {};
        sta_t sta[2];
        gtime_t t0 = {};
        std::memset(sta, 0, sizeof sta);

        if (readrnxt(argv[1], 1, t0, t0, 0.0, "", &obs, &nav, sta + 0) < 0 ||
            readrnxt(argv[2], 2, t0, t0, 0.0, "", &obs, &nav, sta + 1) < 0 ||
            readrnxt(argv[3], 1, t0, t0, 0.0, "", nullptr, &nav, nullptr) < 0) {
            check("factor: sample observations readable", false);
        }
        else {
            sortobs(&obs);
            uniqnav(&nav);
            int nu = 0, nr = 0;
            for (int i = 0; i < obs.n &&
                 timediff(obs.data[i].time, obs.data[0].time) == 0.0; i++) {
                if (obs.data[i].rcv == 1) nu++; else nr++;
            }

            rtk_t r2;
            prcopt_t o2 = prcopt_default;
            /* Code only.  A carrier phase mode would need bias variables the
               factor does not yet provide, and with the baseline biases at
               zero the phase residuals are of order 1e7 m -- the optimizer
               then moves the position thousands of kilometres to fit them.
               DGPS is the configuration this factor is currently complete
               for, and it is a real one. */
            o2.mode = PMODE_DGPS; o2.nf = 2; o2.navsys = SYS_GPS;
            o2.elmin = 15.0 * D2R; o2.ionoopt = IONOOPT_BRDC;
            o2.tropopt = TROPOPT_SAAS; o2.modear = ARMODE_CONT;
            rtkinit(&r2, &o2);
            for (int i = 0; i < 3; i++) {
                r2.rb[i] = sta[1].pos[i];
                r2.x[i]  = sta[0].pos[i];
            }
            r2.sol.time = obs.data[0].time;

            fgo_dd_ctx_t *ctx = nullptr;
            int rc2 = fgo_dd_ctx_create(&ctx, &r2, obs.data, nu, nr, &nav);
            if (rc2 == FGO_OK) rc2 = fgo_dd_freeze_pairs(ctx, r2.x);
            check("factor: frozen context built", rc2 == FGO_OK, "rc=%d", rc2);

            if (rc2 == FGO_OK) {
                fgo::Config cfg = fgo::Config::fromPrcopt(o2);
                auto f = fgo::GnssDDFactor::create(fgo::keyPos(0), ctx,
                                                   r2.x, r2.nx, cfg);
                check("factor: constructed with rows", f && f->rows() > 0,
                      "rows=%d rejected=%d", f ? f->rows() : 0,
                      f ? f->rejectedRows() : 0);

                /* A configuration whose residual uses states this factor does
                   not connect must be refused.  Holding them at their baseline
                   puts their whole error into the position, which is the one
                   thing the optimizer can move. */
                {
                    std::string why;
                    const bool sup = fgo::GnssDDFactor::supports(ctx, &why);
                    check("factor: this configuration is supported", sup,
                          "%s", why.c_str());

                    /* and each unsupported one is actually refused, checked by
                       building a context for it rather than by inspection */
                    static const struct { const char *name; int mode, iono,
                                          tropo, glo; } unsup[] = {
                        {"carrier phase", PMODE_STATIC, IONOOPT_BRDC,
                         TROPOPT_SAAS, GLO_ARMODE_OFF},
                        {"estimated iono", PMODE_DGPS, IONOOPT_EST,
                         TROPOPT_SAAS, GLO_ARMODE_OFF},
                        {"estimated tropo", PMODE_DGPS, IONOOPT_BRDC,
                         TROPOPT_EST, GLO_ARMODE_OFF},
                        {"GLONASS autocal", PMODE_DGPS, IONOOPT_BRDC,
                         TROPOPT_SAAS, GLO_ARMODE_AUTOCAL},
                    };
                    std::string accepted;
                    for (const auto &u : unsup) {
                        rtk_t r3;
                        prcopt_t o3 = o2;
                        o3.mode = u.mode; o3.ionoopt = u.iono;
                        o3.tropopt = u.tropo; o3.glomodear = u.glo;
                        rtkinit(&r3, &o3);
                        for (int i = 0; i < 3; i++) {
                            r3.rb[i] = sta[1].pos[i];
                            r3.x[i]  = sta[0].pos[i];
                        }
                        r3.sol.time = obs.data[0].time;
                        fgo_dd_ctx_t *c3 = nullptr;
                        if (fgo_dd_ctx_create(&c3, &r3, obs.data, nu, nr, &nav)
                                == FGO_OK) {
                            std::string w3;
                            if (fgo::GnssDDFactor::supports(c3, &w3)) {
                                accepted += u.name; accepted += " ";
                            }
                            fgo_dd_ctx_destroy(c3);
                        }
                        rtkfree(&r3);
                    }
                    check("factor: unsupported configurations are refused",
                          accepted.empty(), "%s",
                          accepted.empty() ? "" :
                          ("wrongly accepted: " + accepted).c_str());
                }

                if (f) {
                    const gtsam::Point3 p0(r2.x[0], r2.x[1], r2.x[2]);

                    /* analytic */
                    gtsam::Matrix Ja;
                    gtsam::Vector e0 = f->evaluateError(p0, Ja);

                    /* numerical, through the same callback */
                    std::function<gtsam::Vector(const gtsam::Point3 &)> h =
                        [&f](const gtsam::Point3 &p) {
                            return f->evaluateError(p);
                        };
                    /* Step size is not arbitrary.  The residual is a small
                       difference of ranges of order 2e7 m, so double
                       precision leaves about 1e-8 m of noise in it; a finite
                       difference divides that by the step, giving 1e-4
                       relative error at a 1e-4 m step.  The truncation term
                       is meanwhile only about h/r, so a LARGER step is more
                       accurate here, the opposite of the usual intuition.
                       At 1 m: rounding ~1e-8, truncation ~5e-8. */
                    gtsam::Matrix Jn =
                        gtsam::numericalDerivative11<gtsam::Vector, gtsam::Point3>(
                            h, p0, 1.0);

                    const double scale = Jn.cwiseAbs().maxCoeff();
                    const double err = (Ja - Jn).cwiseAbs().maxCoeff();
                    /* Tolerance is 2e-3, not the 1e-6 one would want, and
                       the reason is a property of RTKLIB rather than of this
                       factor.  RTKLIB's analytic Jacobian is purely
                       geometric: -e_i + e_j.  Two terms in zdres() that do
                       depend on the receiver position are left out of it.
                       Measured by removing each and re-running:

                         tropospheric hydrostatic delay   4.1e-4 relative
                         Sagnac term inside geodist()     6.0e-6 relative

                       The troposphere dominates.  zdres() applies
                       mapfh * zhd unconditionally -- opt->tropopt governs only
                       whether ddres() ESTIMATES a troposphere state, not
                       whether the model delay is applied -- and zhd falls by
                       about 2.9e-4 m per metre of height, which a double
                       difference between a high and a low satellite does not
                       cancel.

                       This is a long-standing approximation, and a harmless
                       one for Gauss-Newton: an approximate Jacobian costs
                       convergence rate, not correctness, since the residual
                       itself is exact.  It bounds how tight this check can
                       be, and it is worth revisiting if millimetre-level work
                       ever needs the extra rate. */
                    std::printf("      [info] Jacobian rel err %.2e "
                                "(RTKLIB omits d(trop)/dr, ~4e-4)\n",
                                scale > 0 ? err / scale : 0.0);
                    check("factor: Jacobian matches numericalDerivative",
                          Ja.rows() == Jn.rows() && Ja.cols() == 3 &&
                          scale > 0.1 && err / scale < 2e-3,
                          "max|Ja-Jn|=%.3e scale=%.3f rel=%.2e",
                          err, scale, scale > 0 ? err / scale : 0.0);

                    /* A sign-flipped Jacobian would have the right magnitude,
                       so magnitude alone proves nothing -- compare against the
                       negation explicitly. */
                    const double errneg = (Ja + Jn).cwiseAbs().maxCoeff();
                    check("factor: Jacobian sign is not inverted",
                          err < errneg, "|Ja-Jn|=%.3e |Ja+Jn|=%.3e", err, errneg);

                    /* The whole point of A': moving the state must move the
                       error, through a fresh call into RTKLIB. */
                    gtsam::Vector e1 = f->evaluateError(
                        gtsam::Point3(p0.x() + 1.0, p0.y(), p0.z()));
                    check("factor: relinearises rather than replaying",
                          (e1 - e0).cwiseAbs().maxCoeff() > 1e-3,
                          "max|de|=%.4f m", (e1 - e0).cwiseAbs().maxCoeff());

                    /* clone() must work: GTSAM calls it whenever a graph is
                       cloned or rekeyed, and the base implementation throws.
                       The copy must also have its own scratch, so evaluating
                       one does not disturb the other. */
                    {
                        bool cloned = false, agrees = false, independent = false;
                        try {
                            auto c = f->clone();
                            cloned = (c != nullptr);
                            if (cloned) {
                                auto *cd = dynamic_cast<fgo::GnssDDFactor *>(c.get());
                                if (cd) {
                                    gtsam::Vector ec = cd->evaluateError(p0);
                                    agrees = (ec.size() == e0.size()) &&
                                             ((ec - e0).cwiseAbs().maxCoeff() < 1e-12);
                                    /* move the clone, original must not shift */
                                    (void)cd->evaluateError(
                                        gtsam::Point3(p0.x()+50.0, p0.y(), p0.z()));
                                    gtsam::Vector again = f->evaluateError(p0);
                                    independent =
                                        ((again - e0).cwiseAbs().maxCoeff() < 1e-12);
                                }
                            }
                        } catch (const std::exception &) { cloned = false; }
                        check("factor: clone works and has its own scratch",
                              cloned && agrees && independent,
                              "cloned=%d agrees=%d independent=%d",
                              (int)cloned, (int)agrees, (int)independent);
                    }

                    /* And it must solve: a graph of this factor plus a loose
                       prior should move toward the true position. */
                    gtsam::NonlinearFactorGraph g;
                    g.add(f);
                    g.addPrior(fgo::keyPos(0), p0,
                               gtsam::noiseModel::Isotropic::Sigma(3, 100.0));
                    gtsam::Values init;
                    init.insert(fgo::keyPos(0),
                                gtsam::Point3(p0.x() + 2.0, p0.y() - 2.0,
                                              p0.z() + 2.0));
                    gtsam::LevenbergMarquardtParams lm;
                    lm.setMaxIterations(cfg.maxIter);
                    gtsam::Values res =
                        gtsam::LevenbergMarquardtOptimizer(g, init, lm).optimize();
                    const double moved =
                        (res.at<gtsam::Point3>(fgo::keyPos(0)) -
                         init.at<gtsam::Point3>(fgo::keyPos(0))).norm();
                    const double d0 =
                        (init.at<gtsam::Point3>(fgo::keyPos(0)) - p0).norm();
                    const double d1 =
                        (res.at<gtsam::Point3>(fgo::keyPos(0)) - p0).norm();
                    check("factor: optimizes toward the observations",
                          moved > 0.1 && d1 < d0,
                          "start %.3f m off, ended %.3f m off, moved %.3f m",
                          d0, d1, moved);
                }
                fgo_dd_ctx_destroy(ctx);
            }
            rtkfree(&r2);
            freeobs(&obs); freenav(&nav, 0xFF);
        }
    }

    if (nfail) { std::printf("\n%d check(s) FAILED\n", nfail); return 1; }
    std::printf("\nall FGO backend checks passed\n");
    return 0;
}
