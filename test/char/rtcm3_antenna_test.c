/* Regression test for the RTCM3 type 1007/1008 antenna-descriptor overflow.
 *
 * The descriptor/serial length fields (n,m) are 8-bit (0..255) but sta.antdes /
 * sta.antsno are only MAXANT(64) bytes, so `strncpy(sta.antdes,des,n)` and
 * `sta.antdes[n]='\0'` with n>=64 write past the field into the ADJACENT
 * members of the same sta_t (antsno -> rectype -> ...). Because that is an
 * intra-struct-member overflow contained within one stack object,
 * AddressSanitizer cannot see it, so this test uses an explicit sentinel:
 *
 *   sta is pre-filled with 0x7E; a valid type-1008 decode must never touch
 *   sta.rectype (that is written only by type 1033). The buggy decoder's
 *   over-long strncpy null-pads across antsno into rectype, wiping the
 *   sentinel; the patched decoder clamps n,m and leaves rectype intact.
 *
 * Built twice by check_rtcm3_fix.sh (patched src/rtcm3.c vs a clamp-stripped
 * copy) which supplies the file to test via -DRTCM3_UNDER_TEST.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "rtklib.h"
#include RTCM3_UNDER_TEST   /* the rtcm3.c under test (brings in static decoders) */

int main(void)
{
    rtcm_t rtcm;
    int i, j, k, n = 100, m = 100, ret;

    /* decode_type1008 only touches buff/len/staid/opt/outtype/sta, so a zeroed
       stack rtcm_t is sufficient (avoids init_rtcm and its heap deps). */
    memset(&rtcm, 0, sizeof(rtcm));

    i = 24;
    setbitu(rtcm.buff, i, 12, 1008); i += 12;   /* message type 1008        */
    setbitu(rtcm.buff, i, 12, 123);  i += 12;   /* station id               */
    setbitu(rtcm.buff, i, 8, n);     i += 8;    /* descriptor length = 100  */
    for (j = 0; j < n; j++) { setbitu(rtcm.buff, i, 8, 'A'); i += 8; }
    setbitu(rtcm.buff, i, 8, 7);     i += 8;    /* antenna setup id         */
    setbitu(rtcm.buff, i, 8, m);     i += 8;    /* serial length = 100      */
    for (j = 0; j < m; j++) { setbitu(rtcm.buff, i, 8, 'B'); i += 8; }
    rtcm.len = (i + 7) / 8;                      /* payload length in bytes  */

    /* Sentinel the whole station struct; rectype must survive a 1008 decode. */
    memset(&rtcm.sta, 0x7E, sizeof(rtcm.sta));

    ret = decode_type1008(&rtcm);
    assert(ret == 5);

    /* Primary oracle: rectype (immediately after antsno) is never written by a
       1008 decode, so any change proves an out-of-bounds write into it. */
    for (k = 0; k < MAXANT; k++) {
        if (rtcm.sta.rectype[k] != 0x7E) {
            fprintf(stderr,
                "OVERFLOW: sta.rectype[%d]=0x%02X corrupted by 1008 decode\n",
                k, (unsigned char)rtcm.sta.rectype[k]);
            return 1;
        }
    }
    /* Secondary: descriptors stay NUL-terminated within their own buffers. */
    assert(strlen(rtcm.sta.antdes) < MAXANT);
    assert(strlen(rtcm.sta.antsno) < MAXANT);

    printf("OK  antdes(len=%zu) antsno(len=%zu) rectype sentinel intact  [MAXANT=%d]\n",
           strlen(rtcm.sta.antdes), strlen(rtcm.sta.antsno), MAXANT);
    return 0;
}
