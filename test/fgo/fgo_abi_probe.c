/*------------------------------------------------------------------------------
* fgo_abi_probe.c : C-side checks of the FGO ABI and its disabled-build stub
*
*          Copyright (C) 2026 by RST Ltd., All rights reserved.
*
* rtkpos.c will call these entry points with no #ifdef around them, so a build
* without the GTSAM backend has to refuse cleanly rather than misbehave.  What
* matters is not that the stub returns an error -- it is that it leaves rtk in
* a state the caller can keep using, since the caller's next move is to fall
* back to the EKF.
*
* version : $Revision:$ $Date:$
* history : 2026/08/21 1.0 new
*-----------------------------------------------------------------------------*/
#include "rtklib_fgo_api.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

extern int  showmsg (const char *format, ...) { (void)format; return 0; }
extern void settspan(gtime_t ts, gtime_t te)  { (void)ts; (void)te; }
extern void settime (gtime_t time)            { (void)time; }

static int nfail=0;

static void check(const char *name, int ok, const char *fmt, ...)
{
    char d[160]="";
    va_list ap;
    if (fmt) { va_start(ap,fmt); vsnprintf(d,sizeof d,fmt,ap); va_end(ap); }
    printf("  %-42s %s%s%s\n",name,ok?"PASS":"FAIL",d[0]?"  ":"",d);
    if (!ok) nfail++;
}

int main(void)
{
    rtk_t rtk;
    prcopt_t opt=prcopt_default;
    char buff[64];
    int rc;

    printf("FGO C ABI probe (plan.md 4.6, 7.2)\n");

    check("fgo_enabled reports the stub build", fgo_enabled()==0,
          "fgo_enabled()=%d",fgo_enabled());

    rtkinit(&rtk,&opt);
    check("rtkinit leaves rtk->fgo NULL", rtk.fgo==NULL, NULL);

    rc=fgo_init(&rtk,&opt);
    check("fgo_init refuses without a backend", rc==FGO_ERR_DISABLED,
          "rc=%d",rc);
    check("fgo_init leaves rtk->fgo NULL on refusal", rtk.fgo==NULL, NULL);

    /* calling it twice must not change the answer or the state */
    rc=fgo_init(&rtk,&opt);
    check("fgo_init is idempotent", rc==FGO_ERR_DISABLED && rtk.fgo==NULL,
          "rc=%d",rc);

    rc=fgo_process_epoch(&rtk,NULL,0,0,NULL);
    check("fgo_process_epoch refuses", rc==FGO_ERR_DISABLED,"rc=%d",rc);

    memset(buff,'x',sizeof buff);
    rc=fgo_insight_json(&rtk,buff,sizeof buff);
    check("fgo_insight_json refuses and terminates the buffer",
          rc==FGO_ERR_DISABLED && buff[0]=='\0',"rc=%d",rc);

    /* the buffer-less form must not write through a NULL pointer */
    rc=fgo_insight_json(&rtk,NULL,0);
    check("fgo_insight_json tolerates a NULL buffer", rc==FGO_ERR_DISABLED,
          "rc=%d",rc);

    /* free and reset must be safe on a context that was never created, so
       rtkfree() can call them unconditionally */
    fgo_reset(&rtk);
    fgo_free(&rtk);
    fgo_free(&rtk);
    check("fgo_reset/fgo_free are safe on an uninitialised context",
          rtk.fgo==NULL, NULL);

    fgo_free(NULL);
    fgo_reset(NULL);
    check("fgo_free/fgo_reset tolerate a NULL rtk", 1, NULL);

    /* error codes are part of the ABI; a silent renumbering would make an
       old caller misread a refusal as success */
    check("error codes hold their documented values",
          FGO_OK==0 && FGO_ERR_DISABLED==-1 && FGO_ERR_NOMEM==-2 &&
          FGO_ERR_SINGULAR==-3 && FGO_ERR_NOTCONV==-4 &&
          FGO_ERR_INTERNAL==-5 && FGO_ERR_TIMEOUT==-6 &&
          FGO_ERR_BADARG==-7, NULL);

    rtkfree(&rtk);

    if (nfail) { printf("\n%d check(s) FAILED\n",nfail); return 1; }
    printf("\nall FGO ABI checks passed\n");
    return 0;
}
