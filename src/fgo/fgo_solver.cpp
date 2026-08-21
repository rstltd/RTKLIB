/*------------------------------------------------------------------------------
* fgo_solver.cpp : FGO lifecycle and the C ABI boundary
*
*          Copyright (C) 2026 by RST Ltd., All rights reserved.
*
* Implements the C -> C++ half of rtklib_fgo_api.h when ENABLE_FGO is on.
* src/fgo/fgo_stub.c implements the same symbols when it is off, which is what
* keeps rtkpos.c free of #ifdef (plan.md 3.1 I5).
*
* EVERY definition below wraps its body in FGO_BOUNDARY.  A C++ exception
* crossing into C is undefined behaviour, not untidiness, and this is not a
* theoretical concern here: gtsam::Symbol throws std::invalid_argument on key
* overflow, GTSAM throws IndeterminantLinearSystemException on a rank-deficient
* graph, and std::bad_alloc is always possible.  plan.md 6.10 RC-9 makes the
* template a review checklist item; test/fgo/check_fgo_solver.sh enforces it.
*
* version : $Revision:$ $Date:$
* history : 2026/08/21 1.0 new
*-----------------------------------------------------------------------------*/
#include "fgo_rtklib.h"
#include "fgo_config.h"

#include <exception>
#include <memory>
#include <new>
#include <cstdio>
#include <cstring>

/* The mandatory boundary.  Written as a macro so that a missing catch is a
   visible omission rather than a subtly different hand-written sequence. */
#define FGO_BOUNDARY_BEGIN try {
#define FGO_BOUNDARY_END(retexpr)                                              \
    } catch (const std::bad_alloc &) {                                         \
        fgo_trace(1, "fgo: out of memory\n");                                      \
        return retexpr(FGO_ERR_NOMEM);                                         \
    } catch (const std::exception &e) {                                        \
        fgo_trace(1, "fgo: %s\n", e.what());                                       \
        return retexpr(FGO_ERR_INTERNAL);                                      \
    } catch (...) {                                                            \
        fgo_trace(1, "fgo: unknown exception\n");                                  \
        return retexpr(FGO_ERR_INTERNAL);                                      \
    }
#define FGO_RET_INT(code)  (code)
#define FGO_RET_VOID(code) ((void)(code), (void)0)

namespace fgo {

Config Config::fromPrcopt(const prcopt_t &opt)
{
    Config c;
    c.solver     = opt.fgo_solver;
    c.ddCovMode  = opt.fgo_ddcov;
    c.elwModel   = opt.fgo_elwmodel;
    c.enableTdcp = opt.fgo_tdcp != 0;
    c.mpAdaptive = opt.fgo_mpadapt != 0;
    c.arcStitch  = opt.fgo_stitch != 0;
    c.jerkConst  = opt.fgo_jerk != 0;
    c.async      = opt.fgo_async != 0;
    c.siteFile   = opt.fgo_sitefile;
    c.insightFile= opt.fgo_insightfile;
    c.mpMapFile  = opt.fgo_mpmapfile;

    /* Clamped, not trusted.  An option file written before these fields
       existed leaves them at the resetsysopts() defaults, but one written by
       hand can carry anything, and a zero iteration count or a negative window
       would surface much later as a mysterious non-convergence. */
    c.windowSec  = opt.fgo_window     > 0.0 ? opt.fgo_window     : FGO_DEF_WINDOW;
    c.maxIter    = opt.fgo_maxiter    > 0   ? opt.fgo_maxiter    : FGO_DEF_MAXITER;
    c.timeoutMs  = opt.fgo_timeout    > 0   ? opt.fgo_timeout    : FGO_DEF_TIMEOUT;
    c.innoScale  = opt.fgo_innoscale  > 0.0 ? opt.fgo_innoscale  : FGO_DEF_INNOSCALE;
    c.relinThres = opt.fgo_relinthres > 0.0 ? opt.fgo_relinthres : FGO_DEF_RELINTHRES;

    c.robust.type     = opt.fgo_robust;
    c.robust.scaleEst = opt.fgo_scaleest != 0;
    c.robust.huberD   = opt.fgo_kparam[0] > 0.0 ? opt.fgo_kparam[0] : FGO_DEF_HUBER_D;
    c.robust.cauchyC  = opt.fgo_kparam[1] > 0.0 ? opt.fgo_kparam[1] : FGO_DEF_CAUCHY_C;
    c.robust.tukeyC   = opt.fgo_kparam[2] > 0.0 ? opt.fgo_kparam[2] : FGO_DEF_TUKEY_C;

    /* The clamp must stay ordered, or the scale estimator would be handed an
       empty interval and silently pin every scale to one endpoint. */
    c.robust.scaleMin = opt.fgo_scaleclamp[0] > 0.0 ? opt.fgo_scaleclamp[0]
                                                    : FGO_DEF_SCALEMIN;
    c.robust.scaleMax = opt.fgo_scaleclamp[1] > 0.0 ? opt.fgo_scaleclamp[1]
                                                    : FGO_DEF_SCALEMAX;
    if (c.robust.scaleMax < c.robust.scaleMin) {
        fgo_trace(2, "fgo: scale clamp reversed (%.3f > %.3f), using defaults\n",
              c.robust.scaleMin, c.robust.scaleMax);
        c.robust.scaleMin = FGO_DEF_SCALEMIN;
        c.robust.scaleMax = FGO_DEF_SCALEMAX;
    }
    return c;
}

std::string Config::describe() const
{
    static const char *kSolver[] = {"ekf", "fgo-batch", "fgo-sliding", "fgo-isam2"};
    static const char *kRobust[] = {"none", "huber", "cauchy", "tukey", "gnc"};
    char buf[320];
    std::snprintf(buf, sizeof buf,
                  "solver=%s robust=%s window=%.1fs maxiter=%d ddcov=%d "
                  "tdcp=%d scale-est=%d",
                  (solver >= 0 && solver <= 3) ? kSolver[solver] : "?",
                  (robust.type >= 0 && robust.type <= 4) ? kRobust[robust.type] : "?",
                  windowSec, maxIter, ddCovMode,
                  enableTdcp ? 1 : 0, robust.scaleEst ? 1 : 0);
    return std::string(buf);
}

/* The per-rtk_t context.  rtk->fgo points at one of these, following the
   solstat precedent of an opaque pointer the C side never inspects. */
class Solver {
public:
    explicit Solver(const prcopt_t &opt) : cfg_(Config::fromPrcopt(opt)) {}

    const Config &config() const { return cfg_; }
    void reset() { epoch_ = 0; }

    int processEpoch(rtk_t *rtk, const obsd_t *obs, int nu, int nr,
                     const nav_t *nav);

    static Solver *from(rtk_t *rtk)
    {
        return rtk ? static_cast<Solver *>(rtk->fgo) : nullptr;
    }

private:
    Config cfg_;
    std::uint64_t epoch_ = 0;
};

int Solver::processEpoch(rtk_t *rtk, const obsd_t *obs, int nu, int nr,
                         const nav_t *nav)
{
    (void)rtk; (void)obs; (void)nu; (void)nr; (void)nav;
    /* The graph, the factors and the optimizer land in the commits that follow
       (plan.md 11.3 P3.9-P3.11).  Reporting non-convergence rather than success
       keeps rtkpos.c on its EKF fallback until there is something real to fall
       back FROM. */
    ++epoch_;
    return FGO_ERR_NOTCONV;
}

}  // namespace fgo

/*---------------------------------------------------------------------------
 * C ABI
 *-------------------------------------------------------------------------*/
extern "C" int fgo_enabled(void)
{
    return 1;
}

extern "C" int fgo_init(rtk_t *rtk, const prcopt_t *opt)
{
    if (!rtk || !opt) return FGO_ERR_BADARG;
    FGO_BOUNDARY_BEGIN
        if (rtk->fgo) return FGO_OK;          /* idempotent */
        /* Everything that can throw happens before rtk->fgo is assigned, and
           the solver is owned by a unique_ptr until then.  Otherwise a failure
           in, say, describe() building its string would return an error while
           leaving a live context bound: the caller would see initialisation
           fail, and a retry would then return FGO_OK for a context it never
           successfully created. */
        std::unique_ptr<fgo::Solver> s(new fgo::Solver(*opt));
        const std::string summary = s->config().describe();
        fgo_trace(2, "fgo: init %s\n", summary.c_str());
        rtk->fgo = s.release();
        return FGO_OK;
    FGO_BOUNDARY_END(FGO_RET_INT)
}

extern "C" void fgo_free(rtk_t *rtk)
{
    if (!rtk) return;
    FGO_BOUNDARY_BEGIN
        delete fgo::Solver::from(rtk);
        rtk->fgo = nullptr;
        return;
    FGO_BOUNDARY_END(FGO_RET_VOID)
}

extern "C" void fgo_reset(rtk_t *rtk)
{
    if (!rtk) return;
    FGO_BOUNDARY_BEGIN
        if (fgo::Solver *s = fgo::Solver::from(rtk)) s->reset();
        return;
    FGO_BOUNDARY_END(FGO_RET_VOID)
}

extern "C" int fgo_process_epoch(rtk_t *rtk, const obsd_t *obs, int nu, int nr,
                                 const nav_t *nav)
{
    if (!rtk) return FGO_ERR_BADARG;
    FGO_BOUNDARY_BEGIN
        fgo::Solver *s = fgo::Solver::from(rtk);
        if (!s) return FGO_ERR_DISABLED;      /* fgo_init() was never called */
        return s->processEpoch(rtk, obs, nu, nr, nav);
    FGO_BOUNDARY_END(FGO_RET_INT)
}

extern "C" int fgo_insight_json(const rtk_t *rtk, char *buff, int size)
{
    if (buff && size > 0) buff[0] = '\0';
    if (!rtk) return FGO_ERR_BADARG;
    FGO_BOUNDARY_BEGIN
        return FGO_ERR_NOTCONV;               /* plan.md 11.3 P3.13 */
    FGO_BOUNDARY_END(FGO_RET_INT)
}
