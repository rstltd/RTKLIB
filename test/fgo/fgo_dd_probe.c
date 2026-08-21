/*------------------------------------------------------------------------------
* fgo_dd_probe.c : the FGO double-difference callback, on real observations
*
*          Copyright (C) 2026 by RST Ltd., All rights reserved.
*
* Gate G4 for the RTKLIB half of architecture A' (plan.md 3.4, 6.11).
*
* The property that matters is not that fgo_dd_eval() produces numbers -- it is
* that it produces DIFFERENT, CORRECT numbers when the state moves.  A cached
* Jacobian would pass any check that only calls it once, and would then quietly
* reduce the whole extension to the linearised architecture A that 1.2 rejects.
* So the central check here is that the analytic Jacobian agrees with a finite
* difference taken through the callback itself.
*
* plan.md 6.11 calls this the most important single check in the project:
* "解析 Jacobian 寫錯是 FGO 最常見且最難察覺的 bug".
*
* Checks:
*   1. a context builds from one epoch of the in-repo sample pair
*   2. eval produces rows, within the bound fgo_dd_nvmax() advertises
*   3. eval is pure: two calls with the same state give identical output
*   4. the Jacobian agrees with a finite difference in the position states
*   5. freezing makes the row count independent of the state
*
* version : $Revision:$ $Date:$
* history : 2026/08/21 1.0 new
*-----------------------------------------------------------------------------*/
#include "rtklib_fgo_api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

extern int  showmsg (const char *format, ...) { (void)format; return 0; }
extern void settspan(gtime_t ts, gtime_t te)  { (void)ts; (void)te; }
extern void settime (gtime_t time)            { (void)time; }

static int nfail=0;

static void check(const char *name, int ok, const char *fmt, ...)
{
    char d[192]="";
    va_list ap;
    if (fmt) { va_start(ap,fmt); vsnprintf(d,sizeof d,fmt,ap); va_end(ap); }
    printf("  %-46s %s%s%s\n",name,ok?"PASS":"FAIL",d[0]?"  ":"",d);
    if (!ok) nfail++;
}

int main(int argc, char **argv)
{
    const char *frov = argc>1?argv[1]:"../data/rinex/07590920.05o";
    const char *fbas = argc>2?argv[2]:"../data/rinex/30400920.05o";
    const char *fnav = argc>3?argv[3]:"../data/rinex/30400920.05n";

    obs_t obs={0};
    nav_t nav={0};
    sta_t sta[2];
    gtime_t t0={0};
    rtk_t rtk;
    prcopt_t opt=prcopt_default;
    fgo_dd_ctx_t *ctx=NULL;
    ddres_stat_t *st=NULL,*st2=NULL;
    double *ws=NULL,*v=NULL,*H=NULL,*R=NULL,*v2=NULL,*H2=NULL,*R2=NULL,*x=NULL;
    int *vflg=NULL,*vflg2=NULL;
    int nu=0,nr=0,i,nv,nv2,nvmax,rc;

    memset(sta,0,sizeof(sta));
    printf("FGO double-difference callback (plan.md 3.4, gate G4)\n");

    if (readrnxt(frov,1,t0,t0,0.0,"",&obs,&nav,sta+0)<0 ||
        readrnxt(fbas,2,t0,t0,0.0,"",&obs,&nav,sta+1)<0 ||
        readrnxt(fnav,1,t0,t0,0.0,"",NULL,&nav,NULL)<0) {
        printf("  cannot read the sample observations\n");
        return 2;
    }
    sortobs(&obs);
    uniqnav(&nav);
    if (obs.n<=0||nav.n<=0) { printf("  no data (n=%d nav=%d)\n",obs.n,nav.n); return 2; }

    /* sortobs orders by time, then receiver, then satellite, which is exactly
       the layout relpos() and the FGO context expect: rover first, base next */
    for (i=0;i<obs.n&&timediff(obs.data[i].time,obs.data[0].time)==0.0;i++) {
        if (obs.data[i].rcv==1) nu++; else nr++;
    }
    if (nu<=0||nr<=0) { printf("  first epoch lacks a pair (nu=%d nr=%d)\n",nu,nr); return 2; }

    /* static short-baseline RTK, matching the regression configuration */
    opt.mode=PMODE_STATIC; opt.nf=2; opt.navsys=SYS_GPS;
    opt.elmin=15.0*D2R; opt.ionoopt=IONOOPT_BRDC; opt.tropopt=TROPOPT_SAAS;
    opt.modear=ARMODE_CONT; opt.dynamics=0;

    rtkinit(&rtk,&opt);
    for (i=0;i<3;i++) {
        rtk.rb[i]=sta[1].pos[i];
        rtk.x[i] =sta[0].pos[i];
    }
    rtk.sol.time=obs.data[0].time;

    rc=fgo_dd_ctx_create(&ctx,&rtk,obs.data,nu,nr,&nav);
    check("context builds from one epoch",rc==FGO_OK&&ctx!=NULL,
          "rc=%d nu=%d nr=%d",rc,nu,nr);
    if (rc!=FGO_OK) { rtkfree(&rtk); return 1; }

    nvmax=fgo_dd_nvmax(ctx);
    ws=mat(fgo_dd_ws_size(ctx),1);
    v =mat(nvmax,1); H =zeros(rtk.nx,nvmax); R =mat(nvmax,nvmax);
    v2=mat(nvmax,1); H2=zeros(rtk.nx,nvmax); R2=mat(nvmax,nvmax);
    vflg=imat(nvmax,1); vflg2=imat(nvmax,1); x=mat(rtk.nx,1);
    st =(ddres_stat_t *)malloc(sizeof(ddres_stat_t));
    st2=(ddres_stat_t *)malloc(sizeof(ddres_stat_t));
    if (!ws||!v||!H||!R||!v2||!H2||!R2||!vflg||!vflg2||!x||!st||!st2) {
        printf("  allocation failed\n"); return 2;
    }
    matcpy(x,rtk.x,rtk.nx,1);

    nv=fgo_dd_eval(ctx,x,rtk.nx,ws,v,H,R,vflg,st);
    check("eval produces rows within the advertised bound",
          nv>0&&nv<=nvmax,"nv=%d nvmax=%d",nv,nvmax);
    if (nv<=0) { rtkfree(&rtk); return 1; }

    /* 3. purity: the same state must give the same answer */
    nv2=fgo_dd_eval(ctx,x,rtk.nx,ws,v2,H2,R2,vflg2,st2);
    check("eval is pure",
          nv2==nv && !memcmp(v,v2,nv*sizeof(double)) &&
          !memcmp(H,H2,(size_t)nv*rtk.nx*sizeof(double)) &&
          !memcmp(R,R2,(size_t)nv*nv*sizeof(double)) &&
          !memcmp(vflg,vflg2,nv*sizeof(int)),"nv=%d",nv);

    /* 4. the Jacobian, against a finite difference through the callback.
       RTKLIB stores H as dh/dx with v = z - h(x), so dv = -H*dx (plan.md 3.4
       "sign convention").  A factor that assumes +H gets a sign-flipped
       Jacobian, which is the failure mode 6.11 warns is hardest to see. */
    {
        const double d=1e-4;            /* 0.1 mm */
        double worst=0.0,maxpred=0.0;
        int q,k;
        matcpy(x,rtk.x,rtk.nx,1);
        x[0]+=d; x[1]-=d; x[2]+=d;
        nv2=fgo_dd_eval(ctx,x,rtk.nx,ws,v2,H2,R2,vflg2,st2);
        if (nv2!=nv) {
            check("Jacobian matches a finite difference",0,
                  "row count changed %d -> %d",nv,nv2);
        }
        else {
            for (q=0;q<nv;q++) {
                double pred=0.0,act=v2[q]-v[q];
                for (k=0;k<3;k++) pred+=H[k+q*rtk.nx]*(x[k]-rtk.x[k]);
                pred=-pred;
                if (fabs(pred)>maxpred) maxpred=fabs(pred);
                if (fabs(act-pred)>worst) worst=fabs(act-pred);
            }
            /* The floor here is not truncation but cancellation: y is a small
               residual formed from ranges of order 2e7 m, so double precision
               leaves roughly 1e-8 m of noise.  Relative to the predicted
               change this must still be small. */
            check("Jacobian matches a finite difference",
                  maxpred>1e-6 && worst<1e-6 && worst/maxpred<1e-2,
                  "worst=%.3e max_pred=%.3e rel=%.2e",worst,maxpred,
                  maxpred>0?worst/maxpred:0.0);
            check("the state actually moved the residuals",maxpred>1e-6,
                  "max_pred=%.3e m",maxpred);
        }
    }

    /* 5. freezing must decouple the row count from the state.  The
       displacement is deliberately large: a few metres only moves elevations
       by microdegrees, so it would not exercise the elevation and SNR masks
       inside zdres() that are the harder half of the problem. */
    rc=fgo_dd_freeze_pairs(ctx,rtk.x);
    if (rc!=FGO_OK) {
        check("freezing fixes the row count",0,"freeze rc=%d",rc);
    }
    else {
        static const double disp[]={5.0,1.0e3,1.0e5,3.0e6};
        int nva,nvb,k,ok=1;
        char d[128];
        matcpy(x,rtk.x,rtk.nx,1);
        nva=fgo_dd_eval(ctx,x,rtk.nx,ws,v,H,R,vflg,st);
        snprintf(d,sizeof d,"nv=%d",nva);
        for (k=0;k<(int)(sizeof(disp)/sizeof(disp[0]))&&ok;k++) {
            matcpy(x,rtk.x,rtk.nx,1);
            x[0]+=disp[k]; x[1]-=disp[k]; x[2]+=disp[k];
            nvb=fgo_dd_eval(ctx,x,rtk.nx,ws,v2,H2,R2,vflg2,st2);
            if (nvb!=nva||memcmp(vflg,vflg2,nva*sizeof(int))) {
                ok=0;
                snprintf(d,sizeof d,"nv=%d at x, %d at +%.0e m",nva,nvb,disp[k]);
            }
        }
        check("freezing fixes the row set, even across the masks",
              nva>0&&ok,"%s",d);
    }

    /* 6. the context must own its inputs.  Scribble over the caller's
       observation buffer and status, exactly as the next epoch would, and the
       results must not move -- this is what lets a sliding-window solver
       re-linearise a factor from an epoch that has long since passed. */
    {
        int nvc;
        matcpy(x,rtk.x,rtk.nx,1);
        nv=fgo_dd_eval(ctx,x,rtk.nx,ws,v,H,R,vflg,st);
        memset(obs.data,0,sizeof(obsd_t)*(size_t)(nu+nr));
        memset(rtk.ssat,0,sizeof(rtk.ssat));
        for (i=0;i<3;i++) rtk.rb[i]=0.0;
        rtk.sol.time=t0;
        nvc=fgo_dd_eval(ctx,x,rtk.nx,ws,v2,H2,R2,vflg2,st2);
        check("context survives its epoch being overwritten",
              nvc==nv && !memcmp(v,v2,nv*sizeof(double)) &&
              !memcmp(H,H2,(size_t)nv*rtk.nx*sizeof(double)) &&
              !memcmp(R,R2,(size_t)nv*nv*sizeof(double)),
              "nv=%d then %d",nv,nvc);
    }

    free(ws);free(v);free(H);free(R);free(v2);free(H2);free(R2);
    free(vflg);free(vflg2);free(x);free(st);free(st2);
    fgo_dd_ctx_destroy(ctx);
    rtkfree(&rtk);
    freeobs(&obs); freenav(&nav,0xFF);

    if (nfail) { printf("\n%d check(s) FAILED\n",nfail); return 1; }
    printf("\nall FGO double-difference checks passed\n");
    return 0;
}
