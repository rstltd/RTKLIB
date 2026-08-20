/* rtklib_api.h : global variable & EXPORT function declarations
 * Fragment of rtklib.h -- include "rtklib.h", never this file directly. */
#ifndef RTKLIB_H
#error "include rtklib.h, not rtklib_api.h"
#endif

/* global variables ----------------------------------------------------------*/
EXPORT extern const double chisqr[];        /* chi-sqr(n) table (alpha=0.001) */
EXPORT extern const prcopt_t prcopt_default; /* default positioning options */
EXPORT extern const solopt_t solopt_default; /* default solution output options */
EXPORT extern const sbsigpband_t igpband1[9][8]; /* SBAS IGP band 0-8 */
EXPORT extern const sbsigpband_t igpband2[2][5]; /* SBAS IGP band 9-10 */
EXPORT extern const char *formatstrs[];     /* stream format strings */
EXPORT extern opt_t sysopts[];              /* system options table */

/* satellites, systems, codes functions --------------------------------------*/
EXPORT int  satno   (int sys, int prn);
EXPORT int  satsys  (int sat, int *prn);
EXPORT int  satid2no(const char *id);
EXPORT void satno2id(int sat, char id[8]);
EXPORT uint8_t obs2code(const char *obs);
EXPORT const char *code2obs(uint8_t code);
EXPORT double code2freq(int sys, uint8_t code, int fcn);
EXPORT double sat2freq(int sat, uint8_t code, const nav_t *nav);
EXPORT int  code2idx(int sys, uint8_t code);
EXPORT int  satexclude(int sat, double var, int svh, const prcopt_t *opt);
EXPORT int  testsnr(int base, int freq, double el, double snr,
                    const snrmask_t *mask);
EXPORT void setcodepri(int sys, int idx, const char *pri);
EXPORT int  getcodepri(int sys, uint8_t code, const char *opt);

/* matrix and vector functions -----------------------------------------------*/
EXPORT double *mat  (int n, int m);
EXPORT int    *imat (int n, int m);
EXPORT double *zeros(int n, int m);
EXPORT double *eye  (int n);
EXPORT double dot2(const double *a, const double *b);
EXPORT double dot3(const double *a, const double *b);
EXPORT double dot (const double *a, const double *b, int n);
EXPORT double norm(const double *a, int n);
EXPORT void cross3(const double *a, const double *b, double *c);
EXPORT int  normv3(const double *a, double *b);
EXPORT void matcpy(double *A, const double *B, int n, int m);
EXPORT void matmul(const char *tr, int n, int k, int m,
                   const double *A, const double *B, double *C);
EXPORT void matmulp(const char *tr, int n, int k, int m,
                    const double *A, const double *B, double *C);
EXPORT void matmulm(const char *tr, int n, int k, int m,
                    const double *A, const double *B, double *C);
EXPORT int  matinv(double *A, int n);
EXPORT int  solve (const char *tr, const double *A, const double *Y, int n,
                   int m, double *X);
EXPORT int  lsq   (const double *A, const double *y, int n, int m, double *x,
                   double *Q);
EXPORT int  filter(double *x, double *P, const double *H, const double *v,
                   const double *R, int n, int m);
EXPORT int  smoother(const double *xf, const double *Qf, const double *xb,
                     const double *Qb, int n, double *xs, double *Qs);
EXPORT void matprint (const double *A, int n, int m, int p, int q);
EXPORT void matfprint(const double *A, int n, int m, int p, int q, FILE *fp);

EXPORT void add_fatal(fatalfunc_t *func);

/* time and string functions -------------------------------------------------*/
EXPORT void    setstr(char *dst, const char *src, int n);
EXPORT double  str2num(const char *s, int i, int n);
EXPORT int     str2time(const char *s, int i, int n, gtime_t *t);
EXPORT char    *time2str(gtime_t t, char str[40], int n);
EXPORT gtime_t epoch2time(const double *ep);
EXPORT void    time2epoch(gtime_t t, double *ep);
EXPORT void    time2epoch_n(gtime_t t, double *ep, int n);
EXPORT gtime_t gpst2time(int week, double sec);
EXPORT double  time2gpst(gtime_t t, int *week);
EXPORT gtime_t gst2time(int week, double sec);
EXPORT double  time2gst(gtime_t t, int *week);
EXPORT gtime_t bdt2time(int week, double sec);
EXPORT double  time2bdt(gtime_t t, int *week);

EXPORT gtime_t timeadd  (gtime_t t, double sec);
EXPORT double  timediff (gtime_t t1, gtime_t t2);
EXPORT gtime_t gpst2utc (gtime_t t);
EXPORT gtime_t utc2gpst (gtime_t t);
EXPORT gtime_t gpst2bdt (gtime_t t);
EXPORT gtime_t bdt2gpst (gtime_t t);
EXPORT gtime_t timeget  (void);
EXPORT void    timeset  (gtime_t t);
EXPORT void    timereset(void);
EXPORT double  time2doy (gtime_t t);
EXPORT double  utc2gmst (gtime_t t, double ut1_utc);
EXPORT int read_leaps(const char *file);

EXPORT int adjgpsweek(int week);
EXPORT uint32_t tickget(void);
EXPORT void sleepms(int ms);

EXPORT int reppath(const char *path, char *rpath, gtime_t time, const char *rov,
                   const char *base);
EXPORT int reppaths(const char *path, char *rpaths[], int nmax, gtime_t ts,
                    gtime_t te, const char *rov, const char *base);

/* coordinates transformation ------------------------------------------------*/
EXPORT void ecef2pos(const double *r, double *pos);
EXPORT void pos2ecef(const double *pos, double *r);
EXPORT void ecef2enu(const double *pos, const double *r, double *e);
EXPORT void enu2ecef(const double *pos, const double *e, double *r);
EXPORT void covenu  (const double *pos, const double *P, double *Q);
EXPORT void covecef (const double *pos, const double *Q, double *P);
EXPORT void xyz2enu (const double *pos, double *E);
EXPORT void eci2ecef(gtime_t tutc, const double *erpv, double *U, double *gmst);
EXPORT void deg2dms (double deg, double *dms, int ndec);
EXPORT double dms2deg(const double *dms);

/* input and output functions ------------------------------------------------*/
EXPORT void readpos(const char *file, const char *rcv, double *pos);
EXPORT int  sortobs(obs_t *obs);
EXPORT void uniqnav(nav_t *nav);
EXPORT int  screent(gtime_t time, gtime_t ts, gtime_t te, double tint);
EXPORT int  readnav(const char *file, nav_t *nav);
EXPORT int  savenav(const char *file, const nav_t *nav);
EXPORT void freeobs(obs_t *obs);
EXPORT void freenav(nav_t *nav, int opt);
EXPORT int  readblq(const char *file, const char *sta, double odisp[2][11][3]);
EXPORT int  readerp(const char *file, erp_t *erp);
EXPORT int  geterp (const erp_t *erp, gtime_t time, double *val);

/* debug trace functions -----------------------------------------------------*/
#ifdef TRACE
#define trace(level, ...) do { if (level <= gettracelevel()) trace_impl(level, __VA_ARGS__); } while (0)
#define tracet(level, ...) do { if (level <= gettracelevel()) tracet_impl(level, __VA_ARGS__); } while (0)
#define tracemat(level, ...) do { if (level <= gettracelevel()) tracemat_impl(level, __VA_ARGS__); } while (0)
#define traceobs(level, ...) do { if (level <= gettracelevel()) traceobs_impl(level, __VA_ARGS__); } while (0)
#define tracenav(level, ...) do { if (level <= gettracelevel()) tracenav_impl(level, __VA_ARGS__); } while (0)
#define tracegnav(level, ...) do { if (level <= gettracelevel()) tracegnav_impl(level, __VA_ARGS__); } while (0)
#define tracehnav(level, ...) do { if (level <= gettracelevel()) tracehnav_impl(level, __VA_ARGS__); } while (0)
#define tracepeph(level, ...) do { if (level <= gettracelevel()) tracepeph_impl(level, __VA_ARGS__); } while (0)
#define tracepclk(level, ...) do { if (level <= gettracelevel()) tracepclk_impl(level, __VA_ARGS__); } while (0)
#define traceb(level, ...) do { if (level <= gettracelevel()) traceb_impl(level, __VA_ARGS__); } while (0)

EXPORT void traceopen(const char *file);
EXPORT void traceclose(void);
EXPORT void tracelevel(int level);
EXPORT int gettracelevel(void);

EXPORT void trace_impl    (int level, const char *format, ...);
EXPORT void tracet_impl   (int level, const char *format, ...);
EXPORT void tracemat_impl (int level, const double *A, int n, int m, int p, int q);
EXPORT void traceobs_impl (int level, const obsd_t *obs, int n);
EXPORT void tracenav_impl (int level, const nav_t *nav);
EXPORT void tracegnav_impl(int level, const nav_t *nav);
EXPORT void tracehnav_impl(int level, const nav_t *nav);
EXPORT void tracepeph_impl(int level, const nav_t *nav);
EXPORT void tracepclk_impl(int level, const nav_t *nav);
EXPORT void traceb_impl   (int level, const uint8_t *p, int n);

#else

#define traceopen(file)       ((void)0)
#define traceclose()          ((void)0)
#define tracelevel(level)     ((void)0)
#define gettracelevel() 0

#define trace(level, ...)     ((void)0)
#define tracet(level, ...)    ((void)0)
#define tracemat(level, ...)  ((void)0)
#define traceobs(level, ...)  ((void)0)
#define tracenav(level, ...)  ((void)0)
#define tracegnav(level, ...) ((void)0)
#define tracehnav(level, ...) ((void)0)
#define tracepeph(level, ...) ((void)0)
#define tracepclk(level, ...) ((void)0)
#define traceb(level, ...)    ((void)0)

#endif /* TRACE */

/* platform dependent functions ----------------------------------------------*/
EXPORT int execcmd(const char *cmd);
EXPORT int expath (const char *path, char *paths[], int nmax);
EXPORT void createdir(const char *path);

/* positioning models --------------------------------------------------------*/
EXPORT double satazel(const double *pos, const double *e, double *azel);
EXPORT double geodist(const double *rs, const double *rr, double *e);
EXPORT void dops(int ns, const double *azel, double elmin, double *dop);

/* atmosphere models ---------------------------------------------------------*/
EXPORT double ionmodel(gtime_t t, const double *ion, const double *pos,
                       const double *azel);
EXPORT double ionmapf(const double *pos, const double *azel);
EXPORT double ionppp(const double *pos, const double *azel, double re,
                     double hion, double *pppos);
EXPORT double tropmodel(gtime_t time, const double *pos, const double *azel,
                        double humi);
EXPORT double tropmapf(gtime_t time, const double *pos, const double *azel,
                       double *mapfw);
EXPORT int iontec(gtime_t time, const nav_t *nav, const double *pos,
                  const double *azel, int opt, double *delay, double *var);
EXPORT void readtec(const char *file, nav_t *nav, int opt);
EXPORT int ionocorr(gtime_t time, const nav_t *nav, int sat, const double *pos,
                    const double *azel, int ionoopt, double *ion, double *var);
EXPORT int ionvtec(gtime_t time, const nav_t *nav, const double *pos,
                   const double *azel, double freq, double *delay, double *var);
EXPORT int tropcorr(gtime_t time, const nav_t *nav, const double *pos,
                    const double *azel, int tropopt, double *trp, double *var);
EXPORT int seliflc(int optnf, int sys);

/* antenna models ------------------------------------------------------------*/
EXPORT int  readpcv(const char *file, pcvs_t *pcvs);
EXPORT pcv_t *searchpcv(int sat, const char *type, gtime_t time,
                        const pcvs_t *pcvs);
EXPORT void antmodel(const pcv_t *pcv, const double *del, const double *azel,
                     int opt, double *dant);
EXPORT void antmodel_s(const pcv_t *pcv, double nadir, double *dant);
EXPORT void free_pcvs(pcvs_t *pcvs);

/* earth tide models ---------------------------------------------------------*/
EXPORT void sunmoonpos(gtime_t tutc, const double *erpv, double *rsun,
                       double *rmoon, double *gmst);
EXPORT void tidedisp(gtime_t tutc, const double *rr, int opt, const erp_t *erp,
                     const double odisp[2][11][3], double *dr);

/* geoid models --------------------------------------------------------------*/
EXPORT int opengeoid(int model, const char *file);
EXPORT void closegeoid(void);
EXPORT double geoidh(const double *pos);

/* datum transformation ------------------------------------------------------*/
EXPORT int loaddatump(const char *file);
EXPORT int tokyo2jgd(double *pos);
EXPORT int jgd2tokyo(double *pos);

/* rinex functions -----------------------------------------------------------*/
EXPORT int readrnx (const char *file, int rcv, const char *opt, obs_t *obs,
                    nav_t *nav, sta_t *sta);
EXPORT int readrnxt(const char *file, int rcv, gtime_t ts, gtime_t te,
                    double tint, const char *opt, obs_t *obs, nav_t *nav,
                    sta_t *sta);
EXPORT int readrnxc(const char *file, nav_t *nav);
EXPORT int outrnxobsh(FILE *fp, const rnxopt_t *opt, const nav_t *nav);
EXPORT int outrnxobsb(FILE *fp, const rnxopt_t *opt, const obsd_t *obs, int n,
                      int epflag);
EXPORT int outrnxnavh (FILE *fp, const rnxopt_t *opt, const nav_t *nav);
EXPORT int outrnxgnavh(FILE *fp, const rnxopt_t *opt, const nav_t *nav);
EXPORT int outrnxhnavh(FILE *fp, const rnxopt_t *opt, const nav_t *nav);
EXPORT int outrnxlnavh(FILE *fp, const rnxopt_t *opt, const nav_t *nav);
EXPORT int outrnxqnavh(FILE *fp, const rnxopt_t *opt, const nav_t *nav);
EXPORT int outrnxcnavh(FILE *fp, const rnxopt_t *opt, const nav_t *nav);
EXPORT int outrnxinavh(FILE *fp, const rnxopt_t *opt, const nav_t *nav);
EXPORT int outrnxnavb (FILE *fp, const rnxopt_t *opt, const eph_t *eph);
EXPORT int outrnxgnavb(FILE *fp, const rnxopt_t *opt, const geph_t *geph);
EXPORT int outrnxhnavb(FILE *fp, const rnxopt_t *opt, const seph_t *seph);
EXPORT int rnxcomment(rnxopt_t *opt, const char *format, ...);
EXPORT int rtk_uncompress(const char *file, char *uncfile);
EXPORT int convrnx(int format, rnxopt_t *opt, const char *file, char **ofile);
EXPORT int  init_rnxctr (rnxctr_t *rnx);
EXPORT void free_rnxctr (rnxctr_t *rnx);
EXPORT int  open_rnxctr (rnxctr_t *rnx, FILE *fp);
EXPORT int  input_rnxctr(rnxctr_t *rnx, FILE *fp);

/* ephemeris and clock functions ---------------------------------------------*/
EXPORT int pephclk(gtime_t time, int sat, const nav_t *nav, double *dts,
                   double *varc);
EXPORT double eph2clk (gtime_t time, const eph_t  *eph);
EXPORT double geph2clk(gtime_t time, const geph_t *geph);
EXPORT double seph2clk(gtime_t time, const seph_t *seph);
EXPORT void eph2pos (gtime_t time, const eph_t  *eph,  double *rs, double *dts,
                     double *var);
EXPORT void geph2pos(gtime_t time, const geph_t *geph, double *rs, double *dts,
                     double *var);
EXPORT void seph2pos(gtime_t time, const seph_t *seph, double *rs, double *dts,
                     double *var);
EXPORT int  peph2pos(gtime_t time, int sat, const nav_t *nav, int opt,
                     double *rs, double *dts, double *var);
EXPORT void satantoff(gtime_t time, const double *rs, int sat, const nav_t *nav,
                      double *dant);
EXPORT int  satpos(gtime_t time, gtime_t teph, int sat, int ephopt,
                   const nav_t *nav, double *rs, double *dts, double *var,
                   int *svh);
EXPORT void satposs(gtime_t time, const obsd_t *obs, int n, const nav_t *nav,
                    int sateph, double *rs, double *dts, double *var, int *svh);
EXPORT void setseleph(int sys, int sel);
EXPORT int  getseleph(int sys);
EXPORT void readsp3(const char *file, nav_t *nav, int opt);
EXPORT int  readsap(const char *file, gtime_t time, nav_t *nav);
EXPORT int  readdcb(const char *file, nav_t *nav, const sta_t *sta);
EXPORT double code2bias(const nav_t *nav, int sys, int sat, int code, int mode);
EXPORT double phase2bias(const nav_t *nav, int sys, int sat, int code, int mode);
/*EXPORT int  readfcb(const char *file, nav_t *nav);*/
EXPORT void alm2pos(gtime_t time, const alm_t *alm, double *rs, double *dts);

EXPORT int tle_read(const char *file, tle_t *tle);
EXPORT int tle_name_read(const char *file, tle_t *tle);
EXPORT int tle_pos(gtime_t time, const char *name, const char *satno,
                   const char *desig, const tle_t *tle, const erp_t *erp,
                   double *rs);

/* receiver raw data functions -----------------------------------------------*/
EXPORT uint32_t getbitu(const uint8_t *buff, int pos, int len);
EXPORT int32_t  getbits(const uint8_t *buff, int pos, int len);
EXPORT void setbitu(uint8_t *buff, int pos, int len, uint32_t data);
EXPORT void setbits(uint8_t *buff, int pos, int len, int32_t  data);
EXPORT uint32_t rtk_crc32 (const uint8_t *buff, int len);
EXPORT uint32_t rtk_crc24q(const uint8_t *buff, int len);
EXPORT uint16_t rtk_crc16 (const uint8_t *buff, int len);
EXPORT int decode_word (uint32_t word, uint8_t *data);
EXPORT int decode_frame(const uint8_t *buff, int sys, eph_t *eph, alm_t *alm,
                        double *ion, double *utc);
EXPORT int test_glostr(const uint8_t *buff);
EXPORT int decode_glostr(const uint8_t *buff, geph_t *geph, double *utc);
EXPORT int decode_bds_d1(const uint8_t *buff, eph_t *eph, double *ion,
                         double *utc);
EXPORT int decode_bds_d2(const uint8_t *buff, eph_t *eph, double *utc);
EXPORT int decode_gal_inav(const uint8_t *buff, eph_t *eph, double *ion,
                           double *utc);
EXPORT int decode_gal_fnav(const uint8_t *buff, eph_t *eph, double *ion,
                           double *utc);
EXPORT int decode_irn_nav(const uint8_t *buff, eph_t *eph, double *ion,
                          double *utc);

EXPORT int init_raw   (raw_t *raw, int format);
EXPORT void free_raw  (raw_t *raw);
EXPORT int input_raw  (raw_t *raw, int format, uint8_t data);
EXPORT int input_rawf (raw_t *raw, int format, FILE *fp);

EXPORT int init_rt17  (raw_t *raw);
EXPORT int init_sbf   (raw_t *raw);
EXPORT void free_rt17 (raw_t *raw);
EXPORT void free_sbf  (raw_t *raw);

EXPORT int input_oem4  (raw_t *raw, uint8_t data);
EXPORT int input_cnav  (raw_t *raw, uint8_t data);
EXPORT int input_ubx   (raw_t *raw, uint8_t data);
EXPORT int input_sbp   (raw_t *raw, uint8_t data);
EXPORT int input_cres  (raw_t *raw, uint8_t data);
EXPORT int input_stq   (raw_t *raw, uint8_t data);
EXPORT int input_javad (raw_t *raw, uint8_t data);
EXPORT int input_nvs   (raw_t *raw, uint8_t data);
EXPORT int input_bnx   (raw_t *raw, uint8_t data);
EXPORT int input_rt17  (raw_t *raw, uint8_t data);
EXPORT int input_sbf   (raw_t *raw, uint8_t data);
EXPORT int input_tersus(raw_t *raw, uint8_t data);
EXPORT int input_unicore(raw_t *raw, uint8_t data);
EXPORT int input_oem4f (raw_t *raw, FILE *fp);
EXPORT int input_cnavf (raw_t *raw, FILE *fp);
EXPORT int input_ubxf  (raw_t *raw, FILE *fp);
EXPORT int input_sbpf  (raw_t *raw, FILE *fp);
EXPORT int input_cresf (raw_t *raw, FILE *fp);
EXPORT int input_stqf  (raw_t *raw, FILE *fp);
EXPORT int input_javadf(raw_t *raw, FILE *fp);
EXPORT int input_nvsf  (raw_t *raw, FILE *fp);
EXPORT int input_bnxf  (raw_t *raw, FILE *fp);
EXPORT int input_rt17f (raw_t *raw, FILE *fp);
EXPORT int input_sbff  (raw_t *raw, FILE *fp);
EXPORT int input_tersusf(raw_t *raw, FILE *fp);
EXPORT int input_unicoref(raw_t *raw, FILE *fp);

EXPORT int gen_ubx (const char *msg, uint8_t *buff);
EXPORT int gen_stq (const char *msg, uint8_t *buff);
EXPORT int gen_nvs (const char *msg, uint8_t *buff);

/* rtcm functions ------------------------------------------------------------*/
EXPORT int init_rtcm   (rtcm_t *rtcm);
EXPORT void free_rtcm  (rtcm_t *rtcm);
EXPORT int input_rtcm2 (rtcm_t *rtcm, uint8_t data);
EXPORT int input_rtcm3 (rtcm_t *rtcm, uint8_t data);
EXPORT int input_rtcm2f(rtcm_t *rtcm, FILE *fp);
EXPORT int input_rtcm3f(rtcm_t *rtcm, FILE *fp);
EXPORT int gen_rtcm2   (rtcm_t *rtcm, int type, int sync);
EXPORT int gen_rtcm3   (rtcm_t *rtcm, int type, int subtype, int sync);

/* solution functions --------------------------------------------------------*/
EXPORT void initsolbuf(solbuf_t *solbuf, int cyclic, int nmax);
EXPORT void freesolbuf(solbuf_t *solbuf);
EXPORT void freesolstatbuf(solstatbuf_t *solstatbuf);
EXPORT sol_t *getsol(solbuf_t *solbuf, int index);
EXPORT int addsol(solbuf_t *solbuf, const sol_t *sol);
EXPORT int readsol (const char *files[], int nfile, solbuf_t *sol);
EXPORT int readsolt(const char *files[], int nfile, gtime_t ts, gtime_t te,
                    double tint, int qflag, int mean, solbuf_t *sol);
EXPORT int readsolstat(const char *files[], int nfile, solstatbuf_t *statbuf);
EXPORT int readsolstatt(const char *files[], int nfile, gtime_t ts, gtime_t te,
                        double tint, solstatbuf_t *statbuf);
EXPORT int inputsol(uint8_t data, gtime_t ts, gtime_t te, double tint,
                    int qflag, const solopt_t *opt, solbuf_t *solbuf);

EXPORT int outprcopts(uint8_t *buff, const prcopt_t *opt);
EXPORT int outsolheads(uint8_t *buff, const solopt_t *opt);
EXPORT int outsols  (uint8_t *buff, const sol_t *sol, const double *rb,
                     const solopt_t *opt);
EXPORT int outsolexs(uint8_t *buff, const sol_t *sol, const ssat_t *ssat,
                     const solopt_t *opt);
EXPORT void outprcopt(FILE *fp, const prcopt_t *opt);
EXPORT void outsolhead(FILE *fp, const solopt_t *opt);
EXPORT void outsol  (FILE *fp, const sol_t *sol, const double *rb,
                     const solopt_t *opt);
EXPORT void outsolex(FILE *fp, const sol_t *sol, const ssat_t *ssat,
                     const solopt_t *opt);
EXPORT int outnmea_rmc(uint8_t *buff, const sol_t *sol);
EXPORT int outnmea_gga(uint8_t *buff, const sol_t *sol);
EXPORT int outnmea_gsa(uint8_t *buff, const sol_t *sol,
                       const ssat_t *ssat);
EXPORT int outnmea_gsv(uint8_t *buff, const sol_t *sol,
                       const ssat_t *ssat);

/* google earth kml converter ------------------------------------------------*/
EXPORT int convkml(const char *infile, const char *outfile, gtime_t ts,
                   gtime_t te, double tint, int qflg, int mean, const char *name,
                   double *offset, int tcolor, int pcolor, int outalt, int outtime);

/* gpx converter -------------------------------------------------------------*/
EXPORT int convgpx(const char *infile, const char *outfile, gtime_t ts,
                   gtime_t te, double tint, int qflg, int mean, const char *name,
                   double *offset, int outtrk, int outpnt, int outalt, int outtime);

// CSV converter -------------------------------------------------------------
EXPORT int convcsv(const char *infile, const char *outfile, gtime_t ts,
                   gtime_t te, double tint, int qflg, int mean, const char *name,
                   double *offset, int outalt, int outtime, int outorder);

/* sbas functions ------------------------------------------------------------*/
EXPORT int  sbsreadmsg (const char *file, int sel, sbs_t *sbs);
EXPORT int  sbsreadmsgt(const char *file, int sel, gtime_t ts, gtime_t te,
                        sbs_t *sbs);
EXPORT void sbsoutmsg(FILE *fp, sbsmsg_t *sbsmsg);
EXPORT int  sbsdecodemsg(gtime_t time, int prn, const uint32_t *words,
                         sbsmsg_t *sbsmsg);
EXPORT int sbsupdatecorr(const sbsmsg_t *msg, nav_t *nav);
EXPORT int sbssatcorr(gtime_t time, int sat, const nav_t *nav, double *rs,
                      double *dts, double *var);
EXPORT int sbsioncorr(gtime_t time, const nav_t *nav, const double *pos,
                      const double *azel, double *delay, double *var);
EXPORT double sbstropcorr(gtime_t time, const double *pos, const double *azel,
                          double *var);

/* options functions ---------------------------------------------------------*/
EXPORT opt_t *searchopt(const char *name, const opt_t *opts);
EXPORT int str2opt(opt_t *opt, const char *str);
EXPORT int opt2str(const opt_t *opt, char *str);
EXPORT int opt2buf(const opt_t *opt, char *buff);
EXPORT int loadopts(const char *file, opt_t *opts);
EXPORT int saveopts(const char *file, const char *mode, const char *comment,
                    const opt_t *opts);
EXPORT void resetsysopts(void);
EXPORT void getsysopts(prcopt_t *popt, solopt_t *sopt, filopt_t *fopt);
EXPORT void setsysopts(const prcopt_t *popt, const solopt_t *sopt,
                       const filopt_t *fopt);

/* stream data input and output functions ------------------------------------*/
EXPORT void strinitcom(void);
EXPORT void strinit  (stream_t *stream);
EXPORT void strlock  (stream_t *stream);
EXPORT void strunlock(stream_t *stream);
EXPORT int  stropen  (stream_t *stream, int type, int mode, const char *path);
EXPORT void strclose (stream_t *stream);
EXPORT int  strread  (stream_t *stream, uint8_t *buff, int n);
EXPORT int  strwrite (stream_t *stream, uint8_t *buff, int n);
EXPORT void strsync  (stream_t *stream1, stream_t *stream2);
EXPORT int  strstat  (stream_t *stream, char *msg);
EXPORT int  strstatx (stream_t *stream, char *msg);
EXPORT void strsum   (stream_t *stream, int *inb, int *inr, int *outb, int *outr);
EXPORT void strsetopt(const int *opt);
EXPORT gtime_t strgettime(stream_t *stream);
EXPORT void strsendnmea(stream_t *stream, const sol_t *sol);
EXPORT void strsendcmd(stream_t *stream, const char *cmd);
EXPORT void strsettimeout(stream_t *stream, int toinact, int tirecon);
EXPORT void strsetdir(const char *dir);
EXPORT void strsetproxy(const char *addr);

/* integer ambiguity resolution ----------------------------------------------*/
EXPORT int lambda(int n, int m, const double *a, const double *Q, double *F,
                  double *s);
EXPORT int lambda_reduction(int n, const double *Q, double *Z);
EXPORT int lambda_search(int n, int m, const double *a, const double *Q,
                         double *F, double *s);

/* standard positioning ------------------------------------------------------*/
EXPORT int pntpos(const obsd_t *obs, int n, const nav_t *nav,
                  const prcopt_t *opt, sol_t *sol, double *azel,
                  ssat_t *ssat, char *msg);

/* precise positioning -------------------------------------------------------*/
EXPORT void rtkinit(rtk_t *rtk, const prcopt_t *opt);
EXPORT void rtkfree(rtk_t *rtk);
EXPORT int  rtkpos (rtk_t *rtk, const obsd_t *obs, int nobs, const nav_t *nav);
EXPORT int  rtkopenstat(const char *file, int level);
EXPORT void rtkclosestat(void);
EXPORT int  rtkoutstat(rtk_t *rtk, int level, char *buff);

/* FGO support: residual and error-model helpers exported from rtkpos.c -------
 * Used by src/fgo/ to evaluate the observation model at an arbitrary state,
 * so that RTKLIB remains the single source of truth for the model while the
 * graph optimiser owns only the graph (plan.md 3.4, invariant I3).
 * The rtk_ prefix avoids clashing with the same-named static in pntpos.c and
 * with host application symbols (plan.md 6.10 RC-4).  Behaviour is identical
 * to the previously static versions.                                        */
EXPORT int    rtk_zdres(int base, const obsd_t *obs, int n, const double *rs,
                        const double *dts, const double *var, const int *svh,
                        const nav_t *nav, const double *rr, const prcopt_t *opt,
                        double *y, double *e, double *azel, double *freq);
EXPORT double rtk_varerr(int sat, int sys, double el, double snr_rover,
                         double snr_base, double bl, double dt, int f,
                         const prcopt_t *opt, const obsd_t *obs);
EXPORT void   rtk_ddcov(const int *nb, int n, const double *Ri,
                        const double *Rj, int nv, double *R);
/* Re-entrant core of ddres(): computes the double-differenced residuals v,
   their Jacobian H and their covariance R for an arbitrary state x, writing
   nothing to rtk_t.  Side effects the original produced are returned in *st
   for the caller to apply.  Set ctx->frozen_ref to hold the reference
   satellites fixed across the iterations of one epoch.  Returns the number
   of double differences.                                                    */
EXPORT void   ddres_ctx_init(ddres_ctx_t *ctx);
EXPORT int    ddres_core(const ddres_ctx_t *ctx, const double *x,
                         const double *P, double *ws, double *v, double *H,
                         double *R, int *vflg, ddres_stat_t *st);
/* doubles of scratch ddres_core() needs for ns satellites and nf frequencies.
   Pass a buffer of at least that size as its ws argument to keep an optimiser's
   hot path allocation-free, or NULL to let it allocate per call.             */
EXPORT int    ddres_ws_size(int ns, int nf);

/* FGO support: residual and error-model helpers exported from pntpos.c ------
 * Used by the undifferenced pseudorange and Doppler factors (plan.md 4.2.2,
 * 4.2.5).  rescode() is already a pure function of its arguments: every
 * output goes through a caller-provided array, and it reads no file-scope or
 * rtk_t state.  The spp_ prefix keeps these distinct from the rtk_ helpers
 * above, which have different signatures (plan.md 6.10 RC-4).
 *
 * The two functions solve DIFFERENT problems with DIFFERENT state vectors and
 * Jacobian strides.  Do not share a size between them.
 *
 *   spp_rescode()  x has spp_nx() elements: x[0..2] receiver position (ECEF,
 *                  m), x[3] GPS receiver clock bias (m), then one inter-system
 *                  time offset per additional constellation.  H rows have
 *                  spp_nx() columns.
 *
 *                  It writes MORE rows than there are observations: after the
 *                  per-observation residuals it appends a rank-deficiency
 *                  constraint row for every constellation clock offset no
 *                  observation activated.  One GPS observation already yields
 *                  six rows.  Size v, var and the rows of H by
 *                  spp_rescode_nvmax(n), never by n.  azel (2 per obs), vsat
 *                  and resp are written only for the first min(n,MAXOBS)
 *                  observations.
 *
 *   spp_resdop()   x has spp_nx_dop() elements, currently 4: x[0..2] receiver
 *                  VELOCITY (ECEF, m/s) and x[3] receiver clock DRIFT (m/s).
 *                  H rows have spp_nx_dop() columns.  It returns at most
 *                  min(n,MAXOBS) rows and appends no constraints.
 *
 * Both counts are functions rather than macros because NX depends on QZSDT,
 * which is defined inside pntpos.c: a header macro could silently disagree
 * with the library that was actually built.                                 */
EXPORT int    spp_nx(void);
EXPORT int    spp_rescode_nvmax(int n);
EXPORT int    spp_nx_dop(void);
EXPORT double spp_varerr(const prcopt_t *opt, const obsd_t *obs, double el,
                         int sys);
EXPORT int    spp_rescode(int iter, const obsd_t *obs, int n, const double *rs,
                          const double *dts, const double *vare,
                          const int *svh, const nav_t *nav, const double *x,
                          const prcopt_t *opt, double *v, double *H,
                          double *var, double *azel, int *vsat, double *resp,
                          int *ns);
EXPORT int    spp_resdop(const obsd_t *obs, int n, const double *rs,
                         const double *dts, const nav_t *nav, const double *rr,
                         const double *x, const double *azel, const int *vsat,
                         double err, double *v, double *H);

/* precise point positioning -------------------------------------------------*/
EXPORT void pppos(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav);
EXPORT int pppnx(const prcopt_t *opt);
EXPORT int pppoutstat(rtk_t *rtk, char *buff, int level);

EXPORT int ppp_ar(rtk_t *rtk, const obsd_t *obs, int n, int *exc,
                  const nav_t *nav, const double *azel, double *x, double *P);

/* post-processing positioning -----------------------------------------------*/
EXPORT int postpos(gtime_t ts, gtime_t te, double ti, double tu,
                   const prcopt_t *popt, const solopt_t *sopt,
                   const filopt_t *fopt, const char **infile, int n, const char *outfile,
                   const char *rov, const char *base);
EXPORT int getstapos(const char *file, const char *name, double *r);

/* stream server functions ---------------------------------------------------*/
EXPORT void strsvrinit (strsvr_t *svr, int nout);
EXPORT int  strsvrstart(strsvr_t *svr, int *opts, int *strs, const char **paths,
                        const char **logs, strconv_t **conv, const char **cmds,
                        const char **cmds_periodic, const double *nmeapos);
EXPORT void strsvrstop (strsvr_t *svr, const char **cmds);
EXPORT void strsvrstat (strsvr_t *svr, int *stat, int *log_stat, int *byte,
                        int *bps, char *msg);
EXPORT strconv_t *strconvnew(int itype, int otype, const char *msgs, int staid,
                             int stasel, const char *opt);
EXPORT void strconvfree(strconv_t *conv);

/* rtk server functions ------------------------------------------------------*/
EXPORT int  rtksvrinit  (rtksvr_t *svr);
EXPORT void rtksvrfree  (rtksvr_t *svr);
EXPORT int  rtksvrstart (rtksvr_t *svr, int cycle, int buffsize, int *strs,
                         const char **paths, int *formats, int navsel, const char **cmds,
                         const char **cmds_periodic, const char **rcvopts, int nmeacycle,
                         int nmeareq, const double *nmeapos, prcopt_t *prcopt,
                         solopt_t *solopt, stream_t *moni, char *errmsg);
EXPORT void rtksvrstop  (rtksvr_t *svr, const char **cmds);
EXPORT int  rtksvropenstr(rtksvr_t *svr, int index, int str, const char *path,
                          const solopt_t *solopt, const prcopt_t *prcopt);
EXPORT void rtksvrclosestr(rtksvr_t *svr, int index);
EXPORT void rtksvrlock  (rtksvr_t *svr);
EXPORT void rtksvrunlock(rtksvr_t *svr);
EXPORT int  rtksvrostat (rtksvr_t *svr, int type, gtime_t *time, int sat[MAXSAT],
                         double *az, double *el, int snr[MAXSAT][NFREQ], int vsat[MAXSAT][NFREQ]);
EXPORT void rtksvrsstat (rtksvr_t *svr, int *sstat, char *msg);
EXPORT int  rtksvrmark(rtksvr_t *svr, const char *name, const char *comment);

/* downloader functions ------------------------------------------------------*/
EXPORT int dl_readurls(const char *file, const char **types, int ntype, url_t *urls,
                       int nmax);
EXPORT int dl_readstas(const char *file, char **stas, int nmax);
EXPORT int dl_exec(gtime_t ts, gtime_t te, double ti, int seqnos, int seqnoe,
                   const url_t *urls, int nurl, const char **stas, int nsta,
                   const char *dir, const char *usr, const char *pwd,
                   const char *proxy, int opts, char *msg, FILE *fp);
EXPORT void dl_test(gtime_t ts, gtime_t te, double ti, const url_t *urls,
                    int nurl, const char **stas, int nsta, const char *dir,
                    int ncol, int datefmt, FILE *fp);

/* GIS data functions --------------------------------------------------------*/
EXPORT int gis_read(const char *file, gis_t *gis, int layer);
EXPORT void gis_free(gis_t *gis);

/* application defined functions ---------------------------------------------*/
extern int showmsg(const char *format,...);
extern void settspan(gtime_t ts, gtime_t te);
extern void settime(gtime_t time);

