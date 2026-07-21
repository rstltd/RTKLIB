/* Regression test for the u-blox BDS GEO (D2) subframe-5 subfrm-buffer overrun.
 *
 * decode_cnav() stores BDS D2 subframe 1 (10 pages * 38 = 380 bytes) filling a
 * whole subfrm[sat-1] row, then also writes subframe 5 page 102 at offset
 * 10*38 = 380 (bytes 380-417). With 380-byte rows that 38-byte write spills into
 * the NEXT satellite's row (subfrm[sat][0..37]); with 418-byte rows it stays
 * within the satellite's own row.
 *
 * The overrun is intra-array (subfrm[sat-1]+380 == subfrm[sat][0] for a 380-byte
 * stride), so AddressSanitizer cannot see it; the oracle is a sentinel filled
 * into the neighbor row, which a 380-byte decode wipes and a 418-byte decode
 * leaves intact.
 *
 * Built twice by check_ublox_bds_subfrm.sh (real 418-byte subfrm vs a copy of
 * rtklib_types.h with the row back at 380).
 */
#define _POSIX_C_SOURCE 200112L  /* strtok_r, used by ublox.c gen_ubx (before any include) */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "rtklib.h"
#include "rcv/ublox.c"   /* brings in static decode_cnav + U4/setU4 */

int main(void)
{
    static raw_t raw;                 /* static: large, zero-initialized */
    int i, k, sat = satno(SYS_CMP, 3);/* BDS GEO PRN 3 (D2) */
    uint8_t frame[38] = {0};

    if (sat <= 0) { printf("SKIP: BDS not enabled\n"); return 0; }
    memset(&raw, 0, sizeof(raw));

    /* Craft the 10 subframe words so decode_cnav's packed buff carries
       subframe ID = 5 (bits 15-17) and page number = 102 (bits 43-49),
       which is exactly what reaches the subframe-5 memcpy for a GEO sat. */
    setbitu(frame, 15, 3, 5);
    setbitu(frame, 43, 7, 102);
    for (i = 0; i < 10; i++) setU4(raw.buff + 6 + 4 * i, getbitu(frame, 30 * i, 30));
    raw.len = 48;

    /* Sentinel the neighbor row that a 380-byte overrun would land in. */
    memset(raw.subfrm[sat], 0x7E, sizeof(raw.subfrm[sat]));

    (void)decode_cnav(&raw, sat, 0);  /* the subframe-5 memcpy happens here */

    for (k = 0; k < 38; k++) {
        if ((unsigned char)raw.subfrm[sat][k] != 0x7E) {
            fprintf(stderr,
                "OVERRUN: subfrm[%d][%d]=0x%02X -- BDS sf5 decode wrote past the row\n",
                sat, k, (unsigned char)raw.subfrm[sat][k]);
            return 1;
        }
    }
    printf("OK  BDS GEO sf5 decode stayed within its own %zu-byte subfrm row; neighbor intact\n",
           sizeof(raw.subfrm[0]));
    return 0;
}
