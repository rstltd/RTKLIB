/* TDD unit test for the file-based phase-OSB loader (Bias-SINEX phase biases).
 *
 * Starts RED: readbiaf() skips phase lines (`if (obs1[0]!='C') continue`),
 * nav_t has no file phase-OSB table, and there is no phase2bias() getter — so
 * this TU does not even compile. It turns GREEN once three seams are added:
 *   (1) nav_t.pbias[MAXSAT][NFREQ][MAX_CODE_BIASES]   (rtklib_types.h)
 *   (2) readbiaf() loads 'L' OSB lines into nav->pbias (preceph.c), and
 *       readdcb() zeroes that table alongside cbias
 *   (3) extern phase2bias() getter mirroring code2bias() (preceph.c)
 *
 * The synthetic .bia (test/char/data/synthetic_osb.bia) holds, for GPS G01:
 *   C1C = 5 ns  (code  OSB)  -> nav->cbias[G01-1][0][1] = 5e-9*CLIGHT
 *   L1C = 9 ns  (phase OSB)  -> nav->pbias[G01-1][0][1] = 9e-9*CLIGHT
 * Both C1C and L1C map to CODE_L1C (obs2code ignores the C/L prefix), GPS L1
 * freq index 0, bias column code_bias_ix[0][CODE_L1C]=1.
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "rtklib.h"

int main(int argc, char **argv)
{
    const char *biafile = argc > 1 ? argv[1] : "data/synthetic_osb.bia";
    nav_t *nav = (nav_t *)calloc(1, sizeof(nav_t));
    int sat = satid2no("G01");
    assert(sat > 0);

    int r = readdcb(biafile, nav, NULL);
    assert(r == 1);

    /* regression guard: adding phase support must not break code-OSB loading */
    double cb = code2bias(nav, SYS_GPS, sat, CODE_L1C, 1);
    assert(fabs(cb - 5.0e-9 * CLIGHT) < 1e-9);

    /* (1)+(2): the phase OSB must now be loaded into the new pbias table */
    double expect = 9.0e-9 * CLIGHT;
    assert(fabs(nav->pbias[sat - 1][0][1] - expect) < 1e-9);

    /* (3): phase2bias() returns the absolute phase bias (mode 1), like code2bias() */
    double pb = phase2bias(nav, SYS_GPS, sat, CODE_L1C, 1);
    assert(fabs(pb - expect) < 1e-9);

    printf("OK  code2bias(C1C)=%.9f m  phase2bias(L1C)=%.9f m\n", cb, pb);
    free(nav);
    return 0;
}
