# Upstream issue draft — readbiaf() drops every DSB record (OBS2 column off by one)

Target: https://github.com/rtklibexplorer/RTKLIB/issues
Status: not yet filed. Found 2026-08-20 while merging upstream main into the rstltd fork.

Local state:
- Fix carried in commit a8062992 on `chore/sync-upstream-20260820` (`buff+29` -> `buff+30`),
  guarded by `test/char/check_dsb_columns.sh` / `test/char/dsb_columns_test.c`.
- Both the bug and the fix below were reproduced against a **pristine upstream worktree**
  at `3aedf054` (not against our fork), so the repro is valid as written.
- If you are submitting this as a pull request rather than an issue, use
  `upstream-patches/` instead — it holds the same two fixes as a `git am`-able series
  plus a PR body. This file remains the standalone issue-report form.

Everything below the line is the issue body, ready to paste.

---

## `readbiaf()` silently drops every DSB record — OBS2 read from column 29, should be 30

### Summary

In `src/preceph.c`, `readbiaf()` reads the OBS2 field of a Bias-SINEX record starting at
column 29. The SINEX_BIAS layout puts OBS2 at column 30; column 29 is the separator
space. The parsed value is therefore shifted by one character, and the same-frequency
check in the DSB branch can never match, so **every DSB record in the file is skipped**.

This fails silently: `readbiaf()` still returns 1, `readdcb()` reports success, the trace
log shows nothing unusual, and every bias that should have come from a DSB record simply
reads back as 0.

OSB-only products are unaffected — `obs2` is not used on that path — which is likely why
this has gone unnoticed.

### Affected code

`src/preceph.c` at `3aedf054` (2.5.1):

```c
478:    while (fgets(buff,sizeof(buff),fp)) {
479:        if ((int)strlen(buff)<91) continue;
480:        strncpy(bias, buff+1,  3); bias[3] ='\0';
481:        strncpy(prn,  buff+11, 3); prn[3]  ='\0';
482:        strncpy(obs1, buff+25, 3); obs1[3] ='\0';
483:        strncpy(obs2, buff+29, 3); obs2[3] ='\0';   /* <-- should be buff+30 */
...
496:        else if (strcmp(bias,"DSB")==0) {
497:            /* differential signal bias */
498:            if (obs1[1]!=obs2[1]) continue; /* skip biases between freqs for now */
```

Introduced by 5b59da32 ("Some .BIA bias files leave SVN field blank, modify file parsing
to handle this case."), which replaced the `sscanf` whitespace parse with fixed-column
`strncpy`. The move to fixed columns is the right call — the whitespace parse genuinely
mis-binds records with a blank SVN field — the offset for OBS2 is just one short.

### Column layout

From the SINEX_BIAS header line, confirmed against a real CODE `.BIA` product:

```
*BIAS SVN_ PRN STATION__ OBS1 OBS2 BIAS_START____ BIAS_END______ UNIT __ESTIMATED_VALUE____ _STD_DEV___
0     6    11  15        25   30
```

`OBS1` occupies columns 25-28 and `OBS2` starts at 30, with column 29 as the separator.
The three other offsets in the function (1, 11, 25) are correct.

### Effect

For a record with `OBS1 = C1C` and `OBS2 = C1W`:

| read from | `obs2` | `obs1[1]!=obs2[1]` | outcome |
|-----------|--------|--------------------|---------|
| `buff+29` | `" C1"` | `'1' != 'C'` -> true  | `continue`, record dropped |
| `buff+30` | `"C1W"` | `'1' != '1'` -> false | record parsed |

Because `obs2[1]` is always the *first* character of the observation code (`C` or `L`)
rather than the band digit, the comparison against `obs1[1]` (a digit) can never be
equal, for any code pair. So the DSB branch is unreachable in its entirety, not just for
particular signals.

### Reproduction

Minimal self-contained case — one GPS DSB record, `C1C`-`C1W` = 5 ns, laid out on the
SINEX_BIAS columns (record is 103 characters, so it clears the length guard on line 479
and isolates the OBS2 offset):

`synthetic_dsb.bia`
```
%=BIA 1.00 TST 2023:152:00000 TST 2023:152:00000 2023:153:00000 R 00000000000
+BIAS/SOLUTION
*BIAS SVN_ PRN STATION__ OBS1 OBS2 BIAS_START____ BIAS_END______ UNIT __ESTIMATED_VALUE____ _STD_DEV___
 DSB  G001 G01           C1C  C1W  2023:152:00000 2023:153:00000 ns                  5.0000      0.0038
-BIAS/SOLUTION
%=ENDBIA
```

`dsb_test.c`
```c
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "rtklib.h"

int main(int argc, char **argv)
{
    nav_t *nav = (nav_t *)calloc(1, sizeof(nav_t));
    int sat = satid2no("G01");

    assert(readdcb(argv[1], nav, NULL) == 1);   /* reports success either way */

    /* code_bias_ix[GPS][CODE_L1W]=0 (reference), [CODE_L1C]=1,
       so the DSB must land in nav->cbias[G01-1][0][1] */
    double cb = code2bias(nav, SYS_GPS, sat, CODE_L1C, 1);
    printf("code2bias(C1C) = %.9f m (expected %.9f)\n", cb, 5.0e-9 * CLIGHT);
    return fabs(cb - 5.0e-9 * CLIGHT) > 1e-9;
}
```

```sh
gcc -std=c99 -Isrc -DTRACE -DENAGLO -DENAGAL -DENACMP -DENAQZS -DENAIRN -DNFREQ=3 \
    dsb_test.c src/rtkcmn.c src/trace.c src/preceph.c src/rinex.c src/ephemeris.c \
    src/sbas.c src/geoid.c src/ionex.c -lm -o dsb_test
./dsb_test synthetic_dsb.bia
```

Observed on `3aedf054`:
```
code2bias(C1C) = 0.000000000 m (expected 1.498962290)     # exit 1
```

With the one-character fix below:
```
code2bias(C1C) = 1.498962290 m (expected 1.498962290)     # exit 0
```

### Suggested fix

```diff
-        strncpy(obs2, buff+29, 3); obs2[3] ='\0';
+        strncpy(obs2, buff+30, 3); obs2[3] ='\0';
```

Verified against the full `test/utest` suite (13/13 pass) and a clean build of
`rnx2rtkp`, `rtkrcv`, `str2str` and `convbin`.

### Secondary observation (separate, lower confidence)

The `if ((int)strlen(buff)<91) continue;` guard on line 479 requires a record to be at
least 91 characters, i.e. to carry a trailing `_STD_DEV___` column. Records that stop
after `__ESTIMATED_VALUE____` are shorter than that and get dropped, even though
everything the parser indexes ends at the value field starting at column 70, and
`str2num()` already clamps to the end of the line (returning 0, which line 485 skips).

Same failure mode as above — the file loads "successfully" and every bias reads back as
zero. Reproducible with an 88-character OSB record:

```
%=BIA 1.00 TST 2010:001:00000 TST 2010:001:00000 2010:002:00000 R 00000000000
+BIAS/SOLUTION
*BIAS Svn_ Prn Station__ Obs1 Obs2 Bias_Start____ Bias_End______ Unit __Estimated_Value____ _Std_Dev___
 OSB  G001 G01           C1C       2010:001:00000 2010:002:00000 ns   5.000000000000E+00
-BIAS/SOLUTION
%=ENDBIA
```

Whether that matters depends on whether you consider the std-dev column mandatory for
this reader — flagging it rather than asserting it is wrong. Relaxing the guard to `<71`
preserves the blank-SVN fix that motivated 5b59da32 while accepting those records.
