/*------------------------------------------------------------------------------
* rtklib_fgo_api.h : C ABI between RTKLIB and the Factor Graph Optimization
*                    backend
*
*          Copyright (C) 2026 by RST Ltd., All rights reserved.
*
* This is the ONLY contract between the C sources in src/ and the C++ sources in src/fgo/ (plan.md 3.6).
* It is a pure C header: no C++ syntax, no GTSAM, Eigen or Boost, no C++ types.
* Anything that would leak the backend into the core belongs on the other side
* of this file.
*
* Traffic crosses in both directions, and the two halves behave differently:
*
*   C -> C++   lifecycle and the per-epoch entry point.  Implemented in
*              src/fgo/ when ENABLE_FGO is on, and by src/fgo/fgo_stub.c when
*              it is off.  The stub is what lets rtkpos.c call these without
*              any #ifdef (plan.md 3.1 I5, 7.2).
*
*   C++ -> C   residual evaluation callbacks.  Implemented in RTKLIB itself,
*              and always present regardless of ENABLE_FGO.  These are what
*              make architecture A' work: a GTSAM factor re-evaluates the
*              observation model through RTKLIB at every linearisation point,
*              so RTKLIB stays the single source of truth for the model
*              (plan.md 3.4, invariant I3).
*
* version : $Revision:$ $Date:$
* history : 2026/08/21 1.0 new
*-----------------------------------------------------------------------------*/
#ifndef RTKLIB_FGO_API_H
#define RTKLIB_FGO_API_H

/* Included OUTSIDE the linkage specification below.  rtklib.h pulls in system
   headers (math.h, pthread.h, ...) and opens its own extern "C"; wrapping
   another linkage specification around all of that is at best redundant and
   is not guaranteed portable.  plan.md 4.6 sketches the include inside the
   block -- it happens to compile with glibc, whose C headers guard their own
   linkage, but there is no reason to depend on that. */
#include "rtklib.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- error codes ---------------------------------------------------------*/
/* Every entry point returns FGO_OK or one of these.  They are part of the ABI:
   changing a value requires an ABI bump (see docs/fgo/abi.md).              */
#define FGO_OK              0
#define FGO_ERR_DISABLED   -1   /* built without ENABLE_FGO */
#define FGO_ERR_NOMEM      -2   /* allocation failed */
#define FGO_ERR_SINGULAR   -3   /* factorization failed (rank deficient?) */
#define FGO_ERR_NOTCONV    -4   /* optimizer did not converge */
#define FGO_ERR_INTERNAL   -5   /* a C++ exception was caught at the boundary */
#define FGO_ERR_TIMEOUT    -6   /* exceeded the per-epoch time budget */
#define FGO_ERR_BADARG     -7   /* caller passed something unusable */

/*=============================================================================
 * C -> C++ : lifecycle and per-epoch entry
 *
 * IMPLEMENTATION RULE for src/fgo/ (plan.md 4.6, risk RC-9):
 * a C++ exception must never cross this boundary -- that is undefined
 * behaviour, not merely untidy.  EVERY definition of the functions below must
 * wrap its body in
 *
 *     try { ... }
 *     catch (const std::bad_alloc&) { return FGO_ERR_NOMEM; }
 *     catch (const std::exception& e) { trace(...); return FGO_ERR_INTERNAL; }
 *     catch (...) { return FGO_ERR_INTERNAL; }
 *
 * This is a review checklist item, not a suggestion.
 *===========================================================================*/

/* Whether this build has a real FGO backend.  Returns 0 for the stub, so a
   caller can say why a solver request was refused instead of reporting a bare
   error code.  Never fails.                                                 */
EXPORT int  fgo_enabled(void);

/* Bind an FGO context to rtk->fgo.  Returns FGO_ERR_DISABLED when the backend
   is absent, leaving rtk->fgo NULL.  Safe to call twice; the second call is a
   no-op returning the same status as the first.                             */
EXPORT int  fgo_init (rtk_t *rtk, const prcopt_t *opt);

/* Release rtk->fgo and set it to NULL.  Safe on a NULL or already-freed
   context, so rtkfree() may call it unconditionally.                        */
EXPORT void fgo_free (rtk_t *rtk);

/* Discard the graph but keep the configuration, e.g. after a solution reset.
   Safe on a context that was never initialised.                             */
EXPORT void fgo_reset(rtk_t *rtk);

/* Per-epoch entry, called from rtkpos.c in place of relpos() when a solver
   other than FGO_SOLVER_EKF is selected.  obs holds nu rover observations
   followed by nr base observations, as relpos() receives them.  On anything
   other than FGO_OK the caller is expected to fall back to the EKF, so this
   must not leave rtk in a partially updated state when it fails.            */
EXPORT int  fgo_process_epoch(rtk_t *rtk, const obsd_t *obs, int nu, int nr,
                              const nav_t *nav);

/* Serialise the current AI Insight record (plan.md 10.2) into buff.  Returns
   the number of bytes written, or a negative error code.  Writes nothing and
   returns FGO_ERR_DISABLED without a backend.                               */
EXPORT int  fgo_insight_json(const rtk_t *rtk, char *buff, int size);

/*=============================================================================
 * C++ -> C : residual evaluation callbacks
 *
 * Implemented in RTKLIB (rtkpos.c, pntpos.c), so they exist in every build.
 * The contexts are opaque: src/fgo/ holds a pointer and never inspects it.
 *
 * The eval functions are PURE.  They must not modify the context or any
 * RTKLIB state, so that distinct contexts can be evaluated concurrently and
 * an optimizer can call them freely while re-linearising.
 *
 * NOT YET IMPLEMENTED: these land with the FGO body (plan.md 11.3 PR-6).
 * They are declared here so the ABI is fixed before anything is written
 * against it.
 *===========================================================================*/

typedef struct fgo_dd_ctx_tag fgo_dd_ctx_t;  /* double-difference context */
typedef struct fgo_pr_ctx_tag fgo_pr_ctx_t;  /* undifferenced-range context */

/* Build a context for one epoch: satellite positions, the common satellite
   set and the undifferenced residuals -- everything that does not depend on
   the state vector.  *ctx is set to NULL on failure.

   The context COPIES every epoch-dependent input it needs, so it stays valid
   after rtk and the observation buffer have moved on.  That is what lets a
   sliding-window or incremental solver re-linearise an old factor.  The only
   borrowed input is nav, whose ephemerides accumulate rather than being
   rewritten in place, and which must therefore outlive the context.

   This call does update rtk->ssat -- the system id, the valid flags and the
   SNR fields -- exactly as relpos() does, because the ambiguity resolution
   that follows a solve reads them.  fgo_dd_eval() by contrast touches nothing
   outside its own outputs.                                                  */
EXPORT int  fgo_dd_ctx_create (fgo_dd_ctx_t **ctx, rtk_t *rtk,
                               const obsd_t *obs, int nu, int nr,
                               const nav_t *nav);
EXPORT void fgo_dd_ctx_destroy(fgo_dd_ctx_t *ctx);

/* Fix the observation set, so that the number and meaning of the rows
   fgo_dd_eval() returns stop depending on the state.  A factor's dimension is
   fixed when it is built, so this must be called before the first evaluation
   (plan.md 4.2.1).  x is the state the selection is made at, normally the
   current best estimate.

   Two things are frozen, and both are needed.  The reference satellite of each
   block would otherwise be re-chosen by minimum variance; and zdres() masks on
   elevation and SNR, both of which move with the state, so an iterate that
   carries a satellite across the mask would silently drop its rows.  After
   this call the masks are not re-applied.                                   */
EXPORT int  fgo_dd_freeze_pairs(fgo_dd_ctx_t *ctx, const double *x);

/* Re-evaluate the double-differenced residuals and their Jacobian at state x.
   Returns the number of rows written, or a negative error code.

   This is the hot path: an optimizer calls it once per factor per
   linearisation, so it allocates nothing.  All scratch comes from ws, which
   must hold at least fgo_dd_ws_size(ctx) doubles, and st receives the
   per-satellite results -- including rowrej, which flags rows whose residual
   exceeded the innovation threshold so the caller can down-weight rather than
   drop them.  Both are caller-owned and neither may be NULL.

   Size the outputs with fgo_dd_nvmax(): v and vflg need that many entries, H
   that many rows of nx columns, R that many squared.

   Pure: ctx is not modified, so contexts may be evaluated concurrently.     */
EXPORT int  fgo_dd_eval(const fgo_dd_ctx_t *ctx, const double *x, int nx,
                        double *ws, double *v, double *H, double *R,
                        int *vflg, ddres_stat_t *st);

/* Upper bound on the rows fgo_dd_eval() writes for this context.            */
EXPORT int  fgo_dd_nvmax(const fgo_dd_ctx_t *ctx);

/* Doubles of scratch fgo_dd_eval() needs for this context.                  */
EXPORT int  fgo_dd_ws_size(const fgo_dd_ctx_t *ctx);

/* The options the context was built with.  Callers need these to know which
   states the residual actually depends on -- the positioning mode decides
   whether carrier phase rows appear, and ionoopt/tropopt/glomodear decide
   whether atmospheric and inter-channel states enter it.  The returned
   pointer belongs to the context and is valid for its lifetime.            */
EXPORT const prcopt_t *fgo_dd_opt(const fgo_dd_ctx_t *ctx);

/* Undifferenced pseudorange equivalents (plan.md 4.2.2).                    */
EXPORT int  fgo_pr_ctx_create (fgo_pr_ctx_t **ctx, rtk_t *rtk,
                               const obsd_t *obs, int n, const nav_t *nav);
EXPORT void fgo_pr_ctx_destroy(fgo_pr_ctx_t *ctx);
EXPORT int  fgo_pr_eval(const fgo_pr_ctx_t *ctx, const double *x, int nx,
                        double *ws, double *v, double *H, double *R);
EXPORT int  fgo_pr_nvmax(const fgo_pr_ctx_t *ctx);
EXPORT int  fgo_pr_ws_size(const fgo_pr_ctx_t *ctx);

/* The error model is deliberately NOT duplicated here.  plan.md 4.6 lists
   fgo_obsvar_rtk() and fgo_obsvar_spp(), but those would be exact aliases of
   rtk_varerr() and spp_varerr(), which rtklib_api.h already exports and which
   src/fgo/ sees through the rtklib.h include above.  Two names for one
   function is a maintenance liability, not an abstraction.                  */

#ifdef __cplusplus
}
#endif
#endif /* RTKLIB_FGO_API_H */
