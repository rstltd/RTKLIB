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
#include "rtklib_fgo_api.h"
#include "fgo_config.h"

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

int main()
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

    if (nfail) { std::printf("\n%d check(s) FAILED\n", nfail); return 1; }
    std::printf("\nall FGO backend checks passed\n");
    return 0;
}
