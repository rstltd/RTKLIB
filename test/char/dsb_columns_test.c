/* Regression guard for Bias-SINEX fixed-column parsing in readbiaf().
 *
 * The SINEX_BIAS header fixes OBS1 at column 25 and OBS2 at column 30:
 *   *BIAS SVN_ PRN STATION__ OBS1 OBS2 BIAS_START____ ...
 *   0     6    11  15        25   30
 * Reading OBS2 from column 29 yields " C1" instead of "C1W", so the
 * same-frequency check (obs1[1]!=obs2[1]) never matches and every DSB
 * record is silently dropped -- the file loads, and every bias is zero.
 *
 * data/synthetic_dsb.bia holds one GPS G01 DSB record, C1C-C1W = 5 ns.
 * code_bias_ix[GPS][CODE_L1W]=0 (reference), [CODE_L1C]=1, so the loader
 * must land it in nav->cbias[G01-1][0][1].
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "rtklib.h"

int main(int argc, char **argv)
{
    const char *biafile = argc > 1 ? argv[1] : "data/synthetic_dsb.bia";
    nav_t *nav = (nav_t *)calloc(1, sizeof(nav_t));
    int sat = satid2no("G01");
    assert(sat > 0);

    assert(readdcb(biafile, nav, NULL) == 1);

    double expect = 5.0e-9 * CLIGHT;
    double cb = code2bias(nav, SYS_GPS, sat, CODE_L1C, 1);
    if (fabs(cb - expect) > 1e-9) {
        printf("FAIL  code2bias(C1C)=%.9f m, expected %.9f m"
               " -- DSB record dropped (OBS2 column offset?)\n", cb, expect);
        return 1;
    }
    printf("OK  DSB C1C-C1W loaded: code2bias(C1C)=%.9f m\n", cb);
    free(nav);
    return 0;
}
