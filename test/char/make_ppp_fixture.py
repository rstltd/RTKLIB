#!/usr/bin/env python3
# Build the real precise-PPP + phase-OSB characterization fixture (test/char/data/).
# Outputs are gzip-compressed to keep the repo small; check_realppp_osb.sh gunzips
# them into its work dir before running rnx2rtkp.
#
# Source (WTZR, Wettzell; 2023-06-01 = DOY 152, GPS week 2264; CODE rapid IAR
# products carry BOTH code and phase OSBs for G/R/E):
#   obs  : https://igs.bkg.bund.de/root_ftp/IGS/obs/2023/152/WTZR00DEU_R_20231520000_01D_30S_MO.crx.gz
#          (Hatanaka; expand with `crx2rnx` or the python `hatanaka` package)
#   nav  : https://igs.bkg.bund.de/root_ftp/IGS/obs/2023/152/WTZR00DEU_R_20231520000_01D_MN.rnx.gz
#   sp3  : https://garner.ucsd.edu/pub/products/2264/COD0OPSRAP_20231520000_01D_05M_ORB.SP3.gz
#   clk  : https://garner.ucsd.edu/pub/products/2264/COD0OPSRAP_20231520000_01D_30S_CLK.CLK.gz
#   bia  : https://garner.ucsd.edu/pub/products/2264/COD0OPSRAP_20231520000_01D_01D_OSB.BIA.gz
#
# obs and clk are trimmed to a 1-hour window (00:00-01:00) to keep the fixture
# small; SP3 (5-min, whole day — PPP needs orbit outside the window for Neville
# interpolation) and BIA (daily OSB, incl. the 356 phase-OSB lines) are kept whole.
#
# Reads the decompressed originals from the scratchpad (override with REALDATA=)
# and writes the gzipped fixture into test/char/data/. Re-run after re-fetching.
import os, sys, gzip

MAX_HOUR = 1  # keep epochs/records with hour < MAX_HOUR (00:00..00:59:30)

def trim_obs(src, dst):
    """Trim a RINEX3 OBS file to hour < MAX_HOUR (epoch lines start with '>')."""
    keep = True
    with open(src) as fi, gzip.open(dst, "wt") as fo:
        in_hdr = True
        for line in fi:
            if in_hdr:
                fo.write(line)
                if "END OF HEADER" in line:
                    in_hdr = False
                continue
            if line.startswith(">"):
                p = line.split()          # > 2023 06 01 HH MM SS.sss  0 nn
                keep = int(p[4]) < MAX_HOUR
            if keep:
                fo.write(line)

def trim_clk(src, dst):
    """Trim a RINEX CLK file; keep header + AS/AR records with hour < MAX_HOUR."""
    with open(src) as fi, gzip.open(dst, "wt") as fo:
        in_hdr = True
        for line in fi:
            if in_hdr:
                fo.write(line)
                if "END OF HEADER" in line:
                    in_hdr = False
                continue
            p = line.split()              # AS G01 2023 06 01 HH MM SS.s N ...
            if len(p) >= 7 and p[0] in ("AS", "AR"):
                if int(p[5]) < MAX_HOUR:
                    fo.write(line)
            else:
                fo.write(line)

def gzcopy(src, dst):
    with open(src, "rb") as fi, gzip.open(dst, "wb") as fo:
        fo.write(fi.read())

def main():
    here = os.path.dirname(os.path.abspath(__file__))
    rd = os.environ.get("REALDATA", os.path.join(
        here, "..", "..", "..", "..", "scratchpad", "realdata", "decompressed"))
    rd = os.path.abspath(rd)
    out = os.path.join(here, "data")
    os.makedirs(out, exist_ok=True)
    jobs = [
        (trim_obs, "WTZR00DEU_R_20231520000_01D_30S_MO.rnx", "wtzr_2264_1h.obs.gz"),
        (trim_clk, "COD0OPSRAP_20231520000_01D_30S_CLK.CLK", "cod_2264_1h.clk.gz"),
        (gzcopy,   "COD0OPSRAP_20231520000_01D_05M_ORB.SP3", "cod_2264.sp3.gz"),
        (gzcopy,   "COD0OPSRAP_20231520000_01D_01D_OSB.BIA", "cod_2264.bia.gz"),
        (gzcopy,   "WTZR00DEU_R_20231520000_01D_MN.rnx",     "wtzr_2264.nav.gz"),
    ]
    for fn, s, d in jobs:
        sp = os.path.join(rd, s)
        if not os.path.exists(sp):
            print("MISSING source:", sp); sys.exit(1)
        fn(sp, os.path.join(out, d))
        print("  %-8s -> data/%s  (%d bytes gz)" % (
            fn.__name__, d, os.path.getsize(os.path.join(out, d))))

if __name__ == "__main__":
    main()
