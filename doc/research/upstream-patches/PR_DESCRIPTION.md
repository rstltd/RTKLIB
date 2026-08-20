# Upstream PR description — Bias-SINEX parsing fixes

Target: https://github.com/rtklibexplorer/RTKLIB
Base: `main` @ `3aedf054`
Local branch: `upstream-fix/bia-dsb-columns` (worktree, not pushed)
Patches: `0001-*.patch`, `0002-*.patch` — verified to `git am` cleanly onto `3aedf054`.

Paste everything below the line as the PR body.

---

## Fix Bias-SINEX parsing: DSB records are silently dropped

Two fixes to `readbiaf()` in `src/preceph.c`, both cases where a `.BIA`/`.BSX`
file loads "successfully" — `readbiaf()` returns 1, `readdcb()` reports success,
nothing in the trace log — while the biases it should have supplied all read back
as zero.

The second commit is independent of the first; drop it if you disagree with it.

### 1. OBS2 is read from column 29, should be 30

The SINEX_BIAS layout puts OBS2 at column 30. Column 29 is the separator space:

```
*BIAS SVN_ PRN STATION__ OBS1 OBS2 BIAS_START____ BIAS_END______ UNIT __ESTIMATED_VALUE____ _STD_DEV___
0     6    11  15        25   30
```

So `obs2` comes back shifted by one — `" C1"` instead of `"C1W"` — which makes
`obs2[1]` the leading `C`/`L` of the observation code rather than the band digit.
The same-frequency test in the DSB branch then compares a letter against a digit:

```c
if (obs1[1]!=obs2[1]) continue; /* skip biases between freqs for now */
```

That can never be equal, for any code pair, so the DSB branch is unreachable in
its entirety and **every DSB record in the file is skipped**. OSB-only products
are unaffected because `obs2` is unused on that path, which is probably why this
has not surfaced before.

The other three offsets in the function (1, 11, 25) are correct.

This came in with 5b59da3, when the `sscanf` whitespace parse was replaced by
fixed-column `strncpy`. Moving to fixed columns is the right call — the whitespace
parse genuinely mis-binds the blank-SVN files that motivated the change — the OBS2
offset is just one short.

### 2. Records without a trailing std-dev column are skipped

`if ((int)strlen(buff)<91) continue;` requires each record to carry a
`_STD_DEV___` field. Records that end after `__ESTIMATED_VALUE____` are shorter
and get dropped.

Nothing the parser indexes needs the line to be that long: the last field it
touches is the value at column 70, and `str2num()` already clamps to the end of
the line, returning 0 for a short or absent value — which the existing
`if ((cbias=str2num(buff,70,21))==0.0) continue;` check skips. Relaxing the guard
to 71 keeps the blank-SVN handling intact and only stops requiring an optional
trailing column.

Flagging this one as the weaker of the two — if you consider the std-dev field
mandatory for this reader, the first commit stands on its own.

## Reproduction

`dsb_test.c`:

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
       so the record must land in nav->cbias[G01-1][0][1] */
    double cb = code2bias(nav, SYS_GPS, sat, CODE_L1C, 1);
    printf("code2bias(C1C) = %.9f m (expected %.9f)\n", cb, 5.0e-9 * CLIGHT);
    return fabs(cb - 5.0e-9 * CLIGHT) > 1e-9;
}
```

```sh
gcc -std=c99 -Isrc -DTRACE -DENAGLO -DENAGAL -DENACMP -DENAQZS -DENAIRN -DNFREQ=3 \
    dsb_test.c src/rtkcmn.c src/trace.c src/preceph.c src/rinex.c src/ephemeris.c \
    src/sbas.c src/geoid.c src/ionex.c -lm -o dsb_test
```

`dsb.bia` — one GPS DSB record, C1C-C1W = 5 ns, 103 characters (clears the length
guard, so it isolates the OBS2 offset):

```
%=BIA 1.00 TST 2023:152:00000 TST 2023:152:00000 2023:153:00000 R 00000000000
+BIAS/SOLUTION
*BIAS SVN_ PRN STATION__ OBS1 OBS2 BIAS_START____ BIAS_END______ UNIT __ESTIMATED_VALUE____ _STD_DEV___
 DSB  G001 G01           C1C  C1W  2023:152:00000 2023:153:00000 ns                  5.0000      0.0038
-BIAS/SOLUTION
%=ENDBIA
```

`osb.bia` — one GPS OSB record, C1C = 5 ns, 88 characters (no std-dev column):

```
%=BIA 1.00 TST 2010:001:00000 TST 2010:001:00000 2010:002:00000 R 00000000000
+BIAS/SOLUTION
*BIAS Svn_ Prn Station__ Obs1 Obs2 Bias_Start____ Bias_End______ Unit __Estimated_Value____ _Std_Dev___
 OSB  G001 G01           C1C       2010:001:00000 2010:002:00000 ns   5.000000000000E+00
-BIAS/SOLUTION
%=ENDBIA
```

Per-commit results (`./dsb_test <file>`, expected value 1.498962290 m):

| commit                      | dsb.bia (103 ch) | osb.bia (88 ch) |
|-----------------------------|------------------|-----------------|
| `3aedf054` (base)           | 0.000000000 FAIL | 0.000000000 FAIL |
| + commit 1 (OBS2 column)    | 1.498962290 PASS | 0.000000000 FAIL |
| + commit 2 (length guard)   | 1.498962290 PASS | 1.498962290 PASS |

## Testing

- `test/utest`: 13/13 pass on the final state.
- `rnx2rtkp` builds and links clean.

No test is included in the patch: `test/utest/t_preceph.c` currently covers only
SP3/clock reading and there is no bias test data in `test/data`, so adding one
would mean introducing both a new fixture and a new test shape. Happy to add it if
you would like it in the same PR.
