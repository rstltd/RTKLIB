/*------------------------------------------------------------------------------
* opts_compat.c : configuration compatibility checks for the FGO options
*
*          Copyright (C) 2026 by RST Ltd., All rights reserved.
*
* Gate G3 of plan.md 6.11, guarding risk RC-3: if the FGO fields were not
* given defaults, a configuration file written before they existed would leave
* them holding whatever happened to be in memory -- and could silently select
* a solver other than the EKF.  That is the one failure mode in PR-4 that
* would be both severe and easy to miss, since every existing config file in
* the wild predates these options.
*
* Checks, in order:
*   1. a real pre-FGO config loads without error and leaves the EKF selected
*   2. every FGO default matches the value Appendix B.1 documents
*   3. a config that does set fgo- options actually takes effect
*   4. saveopts -> loadopts round-trips the FGO fields unchanged
*   5. unknown option names are ignored rather than rejected, which is what
*      makes a new config readable by an older binary
*
* version : $Revision:$ $Date:$
* history : 2026/08/20 1.0 new
*-----------------------------------------------------------------------------*/
#include "rtklib.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* RTKLIB expects the application to supply these */
extern int  showmsg (const char *format, ...) { (void)format; return 0; }
extern void settspan(gtime_t ts, gtime_t te)  { (void)ts; (void)te; }
extern void settime (gtime_t time)            { (void)time; }

static int nfail=0;

static void check(const char *name, int ok, const char *fmt, ...)
{
    char d[192]="";
    va_list ap;
    if (fmt) { va_start(ap,fmt); vsnprintf(d,sizeof d,fmt,ap); va_end(ap); }
    printf("  %-44s %s%s%s\n",name,ok?"PASS":"FAIL",d[0]?"  ":"",d);
    if (!ok) nfail++;
}

static int eq(double a, double b) { return fabs(a-b)<1e-12; }

int main(int argc, char **argv)
{
    prcopt_t p; solopt_t s; filopt_t f;
    const char *legacy = argc>1?argv[1]:"../../data/config/f9p_ppk.conf";
    const char *tmpdir = argc>2?argv[2]:".";
    char path[1024];

    printf("FGO option compatibility (plan.md 6.11 G3, risk RC-3)\n");

    /* ---- 1. a real pre-FGO config must not enable FGO ---------------------*/
    resetsysopts();
    if (!loadopts(legacy,sysopts)) {
        printf("  cannot read legacy config '%s'\n",legacy);
        return 2;
    }
    getsysopts(&p,&s,&f);
    check("legacy config leaves the EKF selected",
          p.fgo_solver==FGO_SOLVER_EKF,"fgo_solver=%d",p.fgo_solver);

    /* ---- 2. defaults must match Appendix B.1 -----------------------------*/
    check("fgo defaults match the documented values",
          p.fgo_robust    ==FGO_ROBUST_HUBER &&
          p.fgo_ddcov     ==FGO_DDCOV_BLOCK &&
          p.fgo_elwmodel  ==FGO_ELW_RTKLIB &&
          p.fgo_maxiter   ==FGO_DEF_MAXITER &&
          p.fgo_tdcp      ==1 &&
          p.fgo_mpadapt   ==0 &&
          p.fgo_scaleest  ==1 &&
          p.fgo_jerk      ==0 &&
          p.fgo_stitch    ==0 &&
          p.fgo_async     ==0 &&
          p.fgo_timeout   ==FGO_DEF_TIMEOUT &&
          eq(p.fgo_window,      FGO_DEF_WINDOW) &&
          eq(p.fgo_kparam[0],   FGO_DEF_HUBER_D) &&
          eq(p.fgo_kparam[1],   FGO_DEF_CAUCHY_C) &&
          eq(p.fgo_kparam[2],   FGO_DEF_TUKEY_C) &&
          eq(p.fgo_innoscale,   FGO_DEF_INNOSCALE) &&
          eq(p.fgo_scaleclamp[0],FGO_DEF_SCALEMIN) &&
          eq(p.fgo_scaleclamp[1],FGO_DEF_SCALEMAX) &&
          eq(p.fgo_relinthres,  FGO_DEF_RELINTHRES) &&
          p.fgo_sitefile[0]=='\0' && p.fgo_insightfile[0]=='\0' &&
          p.fgo_mpmapfile[0]=='\0',
          "window=%.1f maxiter=%d robust=%d",
          p.fgo_window,p.fgo_maxiter,p.fgo_robust);

    /* ---- 3. an FGO config must take effect -------------------------------*/
    snprintf(path,sizeof path,"%s/fgo_on.conf",tmpdir);
    {
        FILE *fp=fopen(path,"w");
        if (!fp) { printf("  cannot write %s\n",path); return 2; }
        fprintf(fp,"pos1-solver        =fgo-isam2\n"
                   "fgo-window         =120.5\n"
                   "fgo-maxiter        =7\n"
                   "fgo-robust         =cauchy\n"
                   "fgo-tdcp           =off\n"
                   "fgo-insight-out    =/tmp/insight.ndjson\n");
        fclose(fp);
    }
    resetsysopts();
    loadopts(path,sysopts);
    getsysopts(&p,&s,&f);
    check("fgo options take effect when set",
          p.fgo_solver==FGO_SOLVER_ISAM2 && eq(p.fgo_window,120.5) &&
          p.fgo_maxiter==7 && p.fgo_robust==FGO_ROBUST_CAUCHY &&
          p.fgo_tdcp==0 &&
          !strcmp(p.fgo_insightfile,"/tmp/insight.ndjson"),
          "solver=%d window=%.1f maxiter=%d robust=%d tdcp=%d",
          p.fgo_solver,p.fgo_window,p.fgo_maxiter,p.fgo_robust,p.fgo_tdcp);

    /* ---- 4. save/load round-trip -----------------------------------------*/
    {
        prcopt_t q; solopt_t s2; filopt_t f2;
        char out[1024];
        snprintf(out,sizeof out,"%s/fgo_rt.conf",tmpdir);
        setsysopts(&p,&s,&f);
        if (!saveopts(out,"w",NULL,sysopts)) {
            check("saveopts/loadopts round-trip",0,"saveopts failed");
        }
        else {
            resetsysopts();
            loadopts(out,sysopts);
            getsysopts(&q,&s2,&f2);
            check("saveopts/loadopts round-trip",
                  q.fgo_solver==p.fgo_solver && eq(q.fgo_window,p.fgo_window) &&
                  q.fgo_maxiter==p.fgo_maxiter && q.fgo_robust==p.fgo_robust &&
                  q.fgo_tdcp==p.fgo_tdcp &&
                  !strcmp(q.fgo_insightfile,p.fgo_insightfile) &&
                  eq(q.fgo_relinthres,p.fgo_relinthres) &&
                  eq(q.fgo_scaleclamp[0],p.fgo_scaleclamp[0]) &&
                  eq(q.fgo_scaleclamp[1],p.fgo_scaleclamp[1]),
                  "solver=%d window=%.1f",q.fgo_solver,q.fgo_window);
        }
    }

    /* ---- 5. unknown names are ignored, so new configs stay readable -------*/
    snprintf(path,sizeof path,"%s/fgo_future.conf",tmpdir);
    {
        FILE *fp=fopen(path,"w");
        if (!fp) return 2;
        fprintf(fp,"pos1-posmode       =static\n"
                   "fgo-not-invented-yet =42\n"
                   "some-future-option =hello\n");
        fclose(fp);
    }
    resetsysopts();
    {
        int rc=loadopts(path,sysopts);
        getsysopts(&p,&s,&f);
        check("unknown option names are ignored",
              rc && p.mode==PMODE_STATIC && p.fgo_solver==FGO_SOLVER_EKF,
              "rc=%d mode=%d",rc,p.mode);
    }

    if (nfail) { printf("\n%d check(s) FAILED\n",nfail); return 1; }
    printf("\nall option compatibility checks passed\n");
    return 0;
}
