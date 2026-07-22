#!/usr/bin/env bash
# Characterization of the file phase-OSB consumption path on a REAL precise-PPP
# dataset (WTZR 2023-152, CODE rapid products; fixture in data/*.gz built by
# make_ppp_fixture.py). This is property-based, not a bit-golden pin, because the
# meaningful, robust invariant of a file phase-OSB is:
#
#   a constant per-satellite/per-frequency phase bias is fully ABSORBED by the
#   float ambiguity states, so it MUST move the $SAT ambiguity records but MUST
#   NOT change the $POS position solution (its position benefit only appears once
#   integer AR is wired — ppp_ar is still a stub).
#
# The same precise PPP is run with the full Bias-SINEX (code+phase OSB) vs a
# code-only copy (phase 'L' OSB lines stripped); the two runs must differ in
# $SAT and be identical in $POS. If corr_phase_bias_file() were a no-op, $SAT
# would match; if phase OSB leaked into position, $POS would differ.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
BIN="$REPO/app/consapp/rnx2rtkp/gcc/rnx2rtkp"
DATA="$HERE/data"
W="$(mktemp -d)"

[ -x "$BIN" ] || make -C "$REPO/app/consapp/rnx2rtkp/gcc" -j"$(nproc)" LDLIBS=-lm >/dev/null 2>&1

echo "== gunzip fixture =="
for f in wtzr_2264_1h.obs cod_2264_1h.clk cod_2264.sp3 cod_2264.bia wtzr_2264.nav; do
    zcat "$DATA/$f.gz" > "$W/$f" || { echo "gunzip $f failed"; rm -rf "$W"; exit 2; }
done
# code-only Bias-SINEX: drop phase (L in OBS1, col 26) OSB lines, keep code (C)
awk '!(substr($0,1,5)==" OSB " && substr($0,26,1)=="L")' "$W/cod_2264.bia" > "$W/cod_codeonly.bia"
echo "   phase OSB lines: full=$(grep -cE '^ OSB .{20}L' "$W/cod_2264.bia") code-only=$(grep -cE '^ OSB .{20}L' "$W/cod_codeonly.bia")"

cat > "$W/ppp.conf" <<'EOF'
pos1-posmode   =ppp-static
pos1-frequency =l1+l2
pos1-elmask    =10
pos1-ionoopt   =dual-freq
pos1-tropopt   =est-ztd
pos1-sateph    =precise
pos1-navsys    =9
pos2-armode    =off
out-solformat  =xyz
out-outstat    =residual
EOF

run() {  # $1=bia file  $2=output tag
    ( cd "$W" && "$BIN" -t -k ppp.conf -o "$2.pos" \
        wtzr_2264_1h.obs wtzr_2264.nav cod_2264.sp3 cod_2264_1h.clk "$1" >/dev/null 2>&1 )
}
echo "== run precise PPP: full bia vs code-only bia =="
run "cod_2264.bia" full
run "cod_codeonly.bia" co

nq6=$(grep -v '^%' "$W/full.pos" | grep -c ' 6 ')
dpos=$(diff <(grep '^\$POS' "$W/full.pos.stat") <(grep '^\$POS' "$W/co.pos.stat") | grep -c '^[<>]')
damb=$(diff <(grep '^\$SAT' "$W/full.pos.stat") <(grep '^\$SAT' "$W/co.pos.stat") | grep -c '^[<>]')
rm -rf "$W"

echo "   Q=6 rows=$nq6 ; \$POS diff=$dpos ; \$SAT diff=$damb"
fail=0
[ "$nq6" -ge 100 ] || { echo "FAIL: too few Q=6 rows ($nq6) — dataset/products not driving PPP"; fail=1; }
[ "$dpos" -eq 0 ]  || { echo "FAIL: phase OSB changed \$POS ($dpos lines) — should be absorbed by float ambiguities, not leak into position"; fail=1; }
[ "$damb" -gt 0 ]  || { echo "FAIL: phase OSB left \$SAT unchanged — corr_phase_bias_file() is a no-op"; fail=1; }
if [ "$fail" -eq 0 ]; then
    echo "RESULT: PASS (phase OSB moves \$SAT ambiguities [$damb lines], leaves \$POS unchanged — consumption wired, absorbed by float as expected)"
else
    echo "RESULT: FAIL"
fi
exit $fail
