#!/usr/bin/env python3
# Generate a minimal synthetic Bias-SINEX (.bia) for the phase-OSB loader test.
# Field columns follow SINEX_BIAS v1.00; the ESTIMATED_VALUE starts at column 71
# (0-indexed 70), which is exactly where readbiaf() reads it via str2num(buff,70,21).
# One CODE OSB (C1C) and one PHASE OSB (L1C) for GPS G01, both in nanoseconds.
import os

def datline(bias, svn, prn, obs1, obs2, unit, value):
    s = [' '] * 92
    def put(col1, text):            # col1 is 1-indexed
        for k, ch in enumerate(text):
            s[col1 - 1 + k] = ch
    put(2,  bias)                   # 2-5   BIAS type
    put(7,  svn)                    # 7-10  SVN
    put(12, prn)                    # 12-14 PRN
    # 16-24 STATION left blank (satellite bias)
    put(26, obs1)                   # 26-29 OBS1
    put(31, obs2)                   # 31-34 OBS2
    put(36, "2010:001:00000")      # 36-49 BIAS_START
    put(51, "2010:002:00000")      # 51-64 BIAS_END
    put(66, unit)                   # 66-69 UNIT
    put(71, value)                  # 71-91 ESTIMATED_VALUE
    return ''.join(s).rstrip()

lines = [
    "%=BIA 1.00 TST 2010:001:00000 TST 2010:001:00000 2010:002:00000 R 00000000000",
    "+BIAS/SOLUTION",
    "*BIAS Svn_ Prn Station__ Obs1 Obs2 Bias_Start____ Bias_End______ Unit __Estimated_Value____ _Std_Dev___",
    datline("OSB", "G001", "G01", "C1C", "", "ns", "5.000000000000E+00"),
    datline("OSB", "G001", "G01", "L1C", "", "ns", "9.000000000000E+00"),
    "-BIAS/SOLUTION",
    "%=ENDBIA",
]

here = os.path.dirname(os.path.abspath(__file__))
outdir = os.path.join(here, "data")
os.makedirs(outdir, exist_ok=True)
out = os.path.join(outdir, "synthetic_osb.bia")
with open(out, "w") as f:
    f.write("\n".join(lines) + "\n")
print("wrote", out)
# Self-check: confirm the value token really lands at column 71.
for ln in lines:
    if ln.startswith(" OSB"):
        print("  col71+ ->", repr(ln[70:91]))
