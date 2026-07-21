/* Regression test for the RINEX "SYS / # / OBS TYPES" out-of-bounds row write.
 *
 * decode_obsh() does  p = strchr(syscodes, buff[0]); i = p - syscodes;  and then
 * writes tobs[i][...]. syscodes is "GREJSCI" (RNX_NUMSYS=7 chars); a header line
 * whose system-code byte is 0x00 makes strchr match syscodes' own NUL terminator,
 * so i == RNX_NUMSYS == 7 -- one row past tobs[RNX_NUMSYS][MAXOBSTYPE][4]. The
 * loop then writes a full obs-type row out of bounds.
 *
 * tobs here is a stack array, so the OOB row write is a plain stack-buffer
 * overflow that AddressSanitizer sees directly. Built twice by
 * check_rinex_fix.sh (patched rinex.c vs a copy with the guard reverted).
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "rtklib.h"
#include RINEX_UNDER_TEST   /* the rinex.c under test (brings in static decode_obsh) */

int main(void)
{
    char tobs[RNX_NUMSYS][MAXOBSTYPE][4] = {{{0}}};
    char buff[MAXRNXLEN];
    nav_t nav; sta_t sta;
    int tsys = TSYS_GPS;

    memset(&nav, 0, sizeof(nav));
    memset(&sta, 0, sizeof(sta));
    memset(buff, ' ', sizeof(buff));
    buff[0] = '\0';                       /* empty system code -> strchr hits NUL */
    memcpy(buff + 3,  "  1", 3);          /* # of obs types = 1 (str2num col 3..5) */
    memcpy(buff + 7,  "C1C", 3);          /* one obs type                          */
    memcpy(buff + 60, "SYS / # / OBS TYPES", 19); /* the header label at col 60    */
    buff[79] = '\0';

    /* Patched: decode_obsh rejects buff[0]==0 and writes nothing.
       Unpatched: writes tobs[7][0] -> ASan stack-buffer-overflow. */
    decode_obsh(NULL, buff, 3.04, &tsys, tobs, &nav, &sta);

    printf("OK  empty system code rejected, no out-of-bounds tobs row  [RNX_NUMSYS=%d]\n",
           RNX_NUMSYS);
    return 0;
}
