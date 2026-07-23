/*------------------------------------------------------------------------------
* ppp_ar.c : ppp ambiguity resolution
*
* WL/NL two-step on the ionosphere-free (IFLC) float PPP (Fork A proper; see
* doc/research/ppp_ar_design.md + est_ar_research.md). Running AR on the est-stec
* per-frequency ambiguities failed: those states absorb the ionosphere
* (rank deficiency), so the IF combination c1*IB0+c2*IB1 carried a systematic
* datum bias -> LAMBDA fixed a shifted integer grid (confident ratio, wrong
* position). IONOOPT_IFLC has ONE clean ambiguity state per satellite,
*   IB(s,0) = B_IF = lam_N*N1 + lam_N*(f2/(f1-f2))*N_WL   (metres, = Lc-Pc),
* with no ionosphere states to contaminate it, so the cascade is exact:
*   1. Wide-lane: round the time-averaged, OSB-corrected Melbourne-Wubbena SD
*      ambiguity (geometry-and-ionosphere-free, lambda_W ~0.86 m -> safe).
*   2. Narrow-lane: N1_SD = B_IF_SD/lam_N - (f2/(f1-f2))*N_WL is integer; fix it
*      jointly by LAMBDA + ratio test over all accepted satellites. The float N1
*      is the linear map a = D*x - (f2/(f1-f2))*N_WL, with D carrying +1/lam_N on
*      the satellite IF state and -1/lam_N on the reference IF state.
*   3. Back-substitute (Teunissen remove-restore, in place) by conditioning the
*      float state on a==N1_fixed: x -= P*D'*(D*P*D')^-1*(a-N1); one IF state/sat.
* Between-satellite SD against a reference cancels the receiver IF phase bias
* (RTKLIB has no receiver phase-bias state). First cut: GPS, nf>=2, IONOOPT_IFLC.
*
*          Copyright (C) 2012-2015 by T.TAKASU, All rights reserved.
* history : 2013/03/11  1.0  new
*           2016/05/10  1.1  delete codes
*           2026/07/23  1.2  revive as WL/NL two-step (Fork A, IFLC)
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

/* state-index macros -- MUST MATCH src/ppp.c (ppp_ar operates on ppp.c's state).
 * ppp.c and rtkpos.c deliberately use different NR layouts; ppp_ar follows ppp.c. */
#define NF(opt)     ((opt)->ionoopt==IONOOPT_IFLC?1:(opt)->nf)
#define NP(opt)     ((opt)->dynamics?9:3)
#define NC(opt)     (NSYS)
#define NT(opt)     ((opt)->tropopt<TROPOPT_EST?0:((opt)->tropopt==TROPOPT_EST?1:3))
#define NI(opt)     ((opt)->ionoopt==IONOOPT_EST?MAXSAT:0)
#define ND(opt)     ((opt)->nf>=3?1:0)
#define NR(opt)     (NP(opt)+NC(opt)+NT(opt)+NI(opt)+ND(opt))
#define IB(s,f,opt) (NR(opt)+MAXSAT*(f)+(s)-1)

#define MW_NMIN      40     /* min Melbourne-Wubbena epochs before a sat is usable */
#define WL_MARGIN    0.20   /* max |WL fractional| to accept a wide-lane round */
#define AR_THRES_NL  2.5    /* narrow-lane LAMBDA ratio-test threshold */
#define MIN_FIX_SATS 4      /* min satellites fixed to promote a solution */
#define AR_ELMASK    (15.0*D2R) /* fallback AR elevation mask when elmaskar unset */

/* is GPS sat s (1-based) eligible for IFLC AR (single IF ambiguity) ---------*/
static int ar_elig(const rtk_t *rtk, int s)
{
    const ssat_t *ss=&rtk->ssat[s-1];
    double elmask=rtk->opt.elmaskar>0.0?rtk->opt.elmaskar:AR_ELMASK;
    return satsys(s,NULL)==SYS_GPS && rtk->x[IB(s,0,&rtk->opt)]!=0.0 &&
           ss->vsat[0] && !(ss->slip[0]&LLI_HALFC) &&
           ss->azel[1]>=elmask;
}
/* clear all PPP fix flags --------------------------------------------------*/
static void ar_clearfix(rtk_t *rtk)
{
    int s,f;
    for (s=0;s<MAXSAT;s++) for (f=0;f<NFREQ;f++) rtk->ssat[s].fix[f]=0;
}
/* accumulate OSB-corrected Melbourne-Wubbena wide-lane per GPS sat ----------
 * obs.L are already phase-OSB corrected (corr_phase_bias_file); obs.P are raw,
 * so code OSB is applied here. Time-averaged into the (otherwise dead)
 * ambc.LC[0]/n[0]; reset on cycle slip.                                      */
static void ar_accum_mw(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    double g1,g2,L1,L2,P1,P2,mw;
    int i,s;
    for (i=0;i<n;i++) {
        s=obs[i].sat;
        if (satsys(s,NULL)!=SYS_GPS) continue;
        if (rtk->ssat[s-1].slip[0]||rtk->ssat[s-1].slip[1]) {  /* reset on slip */
            rtk->ambc[s-1].LC[0]=0.0; rtk->ambc[s-1].n[0]=0;
        }
        L1=obs[i].L[0]; L2=obs[i].L[1]; P1=obs[i].P[0]; P2=obs[i].P[1];
        if (L1==0.0||L2==0.0||P1==0.0||P2==0.0) continue;
        g1=sat2freq(s,obs[i].code[0],nav); g2=sat2freq(s,obs[i].code[1],nav);
        if (g1==0.0||g2==0.0) continue;
        P1-=code2bias(nav,SYS_GPS,s,obs[i].code[0],1);  /* code OSB (m) */
        P2-=code2bias(nav,SYS_GPS,s,obs[i].code[1],1);
        mw=(L1-L2)*CLIGHT/(g1-g2)-(g1*P1+g2*P2)/(g1+g2);
        rtk->ambc[s-1].n[0]++;
        rtk->ambc[s-1].LC[0]+=(mw-rtk->ambc[s-1].LC[0])/rtk->ambc[s-1].n[0];
    }
}
/* ambiguity resolution in ppp ----------------------------------------------*/
extern int ppp_ar(rtk_t *rtk, const obsd_t *obs, int n, int *exc,
                  const nav_t *nav, const double *azel, double *x, double *P)
{
    const prcopt_t *opt=&rtk->opt;
    int i,k,nx=rtk->nx,ref,s,nfx,fsat[MAXSAT];
    double lamW,lamN,f1=FREQL1,f2=FREQL2,elmax,wc;
    double fNwl[MAXSAT];
    double *D,*DP,*QNL,*aNL,*Fnl,s2[2];
    double *Qab,*db,*QQ,*resid;
    int info,ok;

    (void)exc; (void)azel;

    rtk->sol.ratio=0.0f;

    /* self-gate: pppos() calls ppp_ar() unconditionally on float convergence */
    if (opt->mode<PMODE_PPP_KINEMA) return 0;
    if (opt->modear==ARMODE_OFF) return 0;
    if (opt->ionoopt!=IONOOPT_IFLC||opt->nf<2) return 0;

    ar_accum_mw(rtk,obs,n,nav);   /* every epoch */

    lamW=CLIGHT/(f1-f2); lamN=CLIGHT/(f1+f2); wc=f2/(f1-f2);

    /* reference GPS sat: HELD across epochs for SD-datum continuity (design);
       only re-pick (highest elevation) when the held reference drops out. */
    static int held_ref=0;   /* persistent; postpos is single-session */
    ar_clearfix(rtk);
    if (held_ref>0&&ar_elig(rtk,held_ref)&&rtk->ambc[held_ref-1].n[0]>=MW_NMIN) {
        ref=held_ref;
    }
    else {
        ref=-1; elmax=-1.0;
        for (s=1;s<=MAXSAT;s++) {
            if (!ar_elig(rtk,s)||rtk->ambc[s-1].n[0]<MW_NMIN) continue;
            if (rtk->ssat[s-1].azel[1]>elmax) { elmax=rtk->ssat[s-1].azel[1]; ref=s; }
        }
        held_ref=ref;
    }
    if (ref<0) return 0;

    /* --- step 1: wide-lane round per satellite (iono+geometry-free, lambda_W
       large -> safe rounding); collect accepted sats + fixed N_WL --- */
    nfx=0;
    for (s=1;s<=MAXSAT;s++) {
        double mwsd,nwlf;
        if (s==ref||!ar_elig(rtk,s)||rtk->ambc[s-1].n[0]<MW_NMIN) continue;
        mwsd=rtk->ambc[s-1].LC[0]-rtk->ambc[ref-1].LC[0];
        nwlf=mwsd/lamW;
        if (fabs(nwlf-floor(nwlf+0.5))>WL_MARGIN) continue;
        fsat[nfx]=s; fNwl[nfx]=floor(nwlf+0.5); nfx++;
    }
    if (nfx<MIN_FIX_SATS) return 0;  /* fix flags already cleared above */

    /* --- step 2: narrow-lane by JOINT LAMBDA + ratio test on the clean single
       IF ambiguity state. The NL float ambiguity is
         N1_SD(s) = B_IF_SD(s)/lam_N - (f2/(f1-f2))*N_WL(s),
       and B_IF_SD is just the SD of the (metre) IF state IB(s,0), so D has
       +1/lam_N on the sat IF state and -1/lam_N on the ref IF state. --- */
    D=mat(nfx,nx); DP=mat(nfx,nx); QNL=mat(nfx,nfx); aNL=mat(nfx,1); Fnl=mat(nfx,2);
    for (i=0;i<nfx*nx;i++) D[i]=0.0;
    for (k=0;k<nfx;k++) {
        D[k+IB(fsat[k],0,opt)*nfx]= 1.0/lamN;
        D[k+IB(ref,    0,opt)*nfx]=-1.0/lamN;
    }
    matmul("NN",nfx,1,nx,D,x,aNL);           /* aNL = B_IF_SD/lam_N       */
    for (k=0;k<nfx;k++) aNL[k]-=wc*fNwl[k];  /* - WL term -> float N1     */
    matmul("NN",nfx,nx,nx,D,P,DP);           /* DP = D*P                  */
    matmul("NT",nfx,nfx,nx,DP,D,QNL);        /* Q_NL = D*P*D'             */
    info=lambda(nfx,2,aNL,QNL,Fnl,s2);
    rtk->sol.ratio=(!info&&s2[0]>0.0)?(float)(s2[1]/s2[0]):0.0f;
    if (rtk->sol.ratio>999.9f) rtk->sol.ratio=999.9f;
    ok=(!info&&s2[0]>0.0&&s2[1]/s2[0]>=AR_THRES_NL);
    if (!ok) {
        free(D);free(DP);free(QNL);free(aNL);free(Fnl);
        ar_clearfix(rtk); return 0;
    }

    /* --- step 3: Teunissen remove-restore, conditioning float on a==N1_fixed.
       Mirrors rtkpos resamb_LAMBDA (NN/NT only): the "ambiguity" is D*x, so
       Qab = P*D' (nx x nfx), Qgg = D*P*D' = QNL, resid = aNL - N1_fixed.       */
    Qab=mat(nx,nfx); db=mat(nfx,1); QQ=mat(nx,nfx); resid=mat(nfx,1);
    for (k=0;k<nfx;k++) resid[k]=aNL[k]-Fnl[k];
    matmul("NT",nx,nfx,nx,P,D,Qab);          /* Qab = P*D'                */
    if (matinv(QNL,nfx)) {
        trace(2,"ppp_ar: QNL singular nfx=%d\n",nfx);
        free(D);free(DP);free(QNL);free(aNL);free(Fnl);
        free(Qab);free(db);free(QQ);free(resid);
        ar_clearfix(rtk); return 0;
    }
    matmul ("NN",nfx,1,nfx,QNL,resid,db);    /* db = QNL^-1 (aNL - N1)    */
    matmulm("NN",nx,1,nfx,Qab,db,x );         /* x  = x - Qab*db           */
    matmul ("NN",nx,nfx,nfx,Qab,QNL,QQ);      /* QQ = Qab*QNL^-1           */
    matmulm("NT",nx,nx,nfx,QQ,Qab,P);         /* P  = P - QQ*Qab'          */

    for (k=0;k<nfx;k++) rtk->ssat[fsat[k]-1].fix[0]=2;
    rtk->ssat[ref-1].fix[0]=2;

    trace(3,"ppp_ar(IFLC): WL/NL fixed nsat=%d ref=%d ratio=%.2f\n",nfx,ref,rtk->sol.ratio);

    free(D);free(DP);free(QNL);free(aNL);free(Fnl);
    free(Qab);free(db);free(QQ);free(resid);
    return nfx;
}
