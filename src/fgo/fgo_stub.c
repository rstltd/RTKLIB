/*------------------------------------------------------------------------------
* fgo_stub.c : FGO entry points for builds without the GTSAM backend
*
*          Copyright (C) 2026 by RST Ltd., All rights reserved.
*
* Compiled instead of the C++ backend in src/fgo/ when ENABLE_FGO is off (plan.md 7.2).
*
* Its whole purpose is that rtkpos.c can call fgo_process_epoch() with no
* #ifdef around it: the call resolves either to the real backend or to the
* refusal below, and the core stays free of build-configuration branching
* (plan.md 3.1 I5).  A build without GTSAM therefore differs from one with it
* only in what these symbols do, never in what the core looks like.
*
* Pure C99.  Linking this must not pull in libstdc++.
*
* version : $Revision:$ $Date:$
* history : 2026/08/21 1.0 new
*-----------------------------------------------------------------------------*/
#include "rtklib_fgo_api.h"

extern int fgo_enabled(void)
{
    return 0;
}
extern int fgo_init(rtk_t *rtk, const prcopt_t *opt)
{
    (void)opt;
    /* leave rtk->fgo NULL so fgo_free() stays a no-op and nothing downstream
       can mistake an absent backend for an initialised one */
    if (rtk) rtk->fgo = NULL;
    trace(2, "fgo: built without ENABLE_FGO, solver unavailable\n");
    return FGO_ERR_DISABLED;
}
extern void fgo_free(rtk_t *rtk)
{
    if (rtk) rtk->fgo = NULL;
}
extern void fgo_reset(rtk_t *rtk)
{
    (void)rtk;
}
extern int fgo_process_epoch(rtk_t *rtk, const obsd_t *obs, int nu, int nr,
                             const nav_t *nav)
{
    (void)rtk; (void)obs; (void)nu; (void)nr; (void)nav;
    return FGO_ERR_DISABLED;
}
extern int fgo_insight_json(const rtk_t *rtk, char *buff, int size)
{
    (void)rtk;
    /* terminate the caller's buffer rather than leaving it undefined: a
       caller that ignores the status should still find an empty string */
    if (buff && size > 0) buff[0] = '\0';
    return FGO_ERR_DISABLED;
}
