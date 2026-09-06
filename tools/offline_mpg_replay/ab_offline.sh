#!/bin/bash
# Offline MPG/WGN decode A/B: one deterministic channel realization per seed fed
# IDENTICALLY to two iris binaries (BASE vs FIX), isolating a demod change from
# connect/ARQ variance and card wedges. See README.md.
#   usage: ab_offline.sh <level> <snr3k_db> <nseeds>   [profile default mpg]
#   env:   BASE, FIX      iris binaries (A/B arms)
#          WORK           scratch dir (default /dev/shm/scratch)
#          PROFILE        channel profile (default mpg; wgn = flat control)
#          IRIS_OFDM_PB_LOW/HIGH=300/5000  -> 87-carrier live config (BW 4150 Hz)
#          IRIS_OFDM_PAYLOAD=94            -> ~21-symbol O1 frame (matches live)
set -u
BASE=${BASE:?set BASE}; FIX=${FIX:?set FIX}
L=$1; SNR=$2; N=$3; PROFILE=${PROFILE:-mpg}
WORK=${WORK:-/dev/shm/scratch}; cd "$WORK"
HERE=$(cd "$(dirname "$0")" && pwd)
TX=$WORK/tx_L$L.s16
"$BASE" --tx-ofdm "$L" "$TX" >/dev/null 2>&1
bp=0; fp=0
for s in $(seq 1 "$N"); do
  RX=$WORK/rx_L${L}_s${s}.s16
  python3 "$HERE/chan_apply.py" "$TX" "$RX" "$PROFILE" "$SNR" "$s"
  br=$("$BASE" --rx-ofdm "$L" "$RX" 2>&1 | grep -oE 'RESULT: (PASS|FAIL|NO DETECTION)' | head -1)
  fr=$("$FIX"  --rx-ofdm "$L" "$RX" 2>&1 | grep -oE 'RESULT: (PASS|FAIL|NO DETECTION)' | head -1)
  [ "$br" = 'RESULT: PASS' ] && bp=$((bp+1))
  [ "$fr" = 'RESULT: PASS' ] && fp=$((fp+1))
  echo "L$L seed $s: base=${br#RESULT: } fix=${fr#RESULT: }"
done
echo "=== L$L $PROFILE:$SNR N=$N : BASE_PASS=$bp/$N  FIX_PASS=$fp/$N ==="
