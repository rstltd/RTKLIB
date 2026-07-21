/* Regression test for the RTCM3 type 1007/1008/1033 antenna-descriptor overflow.
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
    int i, j, k, n = 100, m = 100, ret, bad = 0;

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
            bad = 1; break;
        }
    }
    /* Secondary: descriptors stay NUL-terminated within their own buffers. */
    assert(strlen(rtcm.sta.antdes) < MAXANT);
    assert(strlen(rtcm.sta.antsno) < MAXANT);

    /* --- type 1033 (combined receiver + antenna descriptors): same class, five
       8-bit length fields n,m,n1,n2,n3 into antdes/antsno/rectype/recver/recsno.
       Sentinel is sta.itrf, which a 1033 decode never writes but the buggy
       recsno overflow (n3 bytes past recsno[64]) null-pads over. --- */
    memset(&rtcm, 0, sizeof(rtcm));
    setbitu(rtcm.buff, 24, 12, 1033);          /* message type 1033           */
    setbitu(rtcm.buff, 36, 12, 123);           /* station id                  */
    setbitu(rtcm.buff, 48, 8, 100);            /* n  = antenna descriptor len */
    setbitu(rtcm.buff, 36 + 28 + 8 * 100, 8, 100); /* m  = antenna serial len  */
    setbitu(rtcm.buff, 36 + 36 + 8 * 200, 8, 100); /* n1 = receiver type len   */
    setbitu(rtcm.buff, 36 + 44 + 8 * 300, 8, 100); /* n2 = receiver version len*/
    setbitu(rtcm.buff, 36 + 52 + 8 * 400, 8, 100); /* n3 = receiver serial len */
    rtcm.len = 550;                            /* >= (36+60+8*500)/8 = 512     */

    memset(&rtcm.sta, 0x7E, sizeof(rtcm.sta));
    ret = decode_type1033(&rtcm);
    assert(ret == 5);
    if (rtcm.sta.itrf != 0x7E7E7E7E) {
        fprintf(stderr, "OVERFLOW: sta.itrf=0x%08X corrupted by 1033 decode\n",
                (unsigned)rtcm.sta.itrf);
        bad = 1;
    }
    assert(strlen(rtcm.sta.rectype) < MAXANT);
    assert(strlen(rtcm.sta.recsno) < MAXANT);

    if (bad) return 1;
    printf("OK  1008 rectype sentinel intact; 1033 itrf sentinel intact  [MAXANT=%d]\n",
           MAXANT);
    return 0;
}
