#!/bin/bash
# GENIE-H fork A/B (snd-aloop FREE, deterministic per seed).
# For each seed, decode the SAME noisy MPG frame two ways with the SAME binary:
#   BASE  : the preamble-derived channel estimate (production path).
#   GENIE : the TRUE per-carrier channel, captured from a NOISELESS same-seed
#           companion (identical Watterson taps, negligible AWGN) and injected
#           via IRIS_GENIE_H_FILE -> demodulate(genie_H) override.
# Isolates channel-ESTIMATION error from the EQ/diversity/FEC on the MPG
# frequency-selective decode boss:
#   GENIE >> BASE  -> the ESTIMATOR is the binding term (port Mercury 616fcd13).
#   GENIE ~= BASE (still failing) -> the notch exceeds FEC capacity regardless
#                                    of H accuracy -> DIVERSITY/architectural.
#
#   usage: genie_ab.sh <level> <snr3k_db> <nseeds>   [profile default mpg]
#   env:   BIN            iris binary (single arm; base & genie share it)
#          WORK           scratch dir (default /dev/shm/scratch/genie)
#          PROFILE        channel profile (default mpg; wgn = flat control)
#          GENIE_SNR      companion SNR (default 200 -> noiseless, same taps)
#          IRIS_GENIE_NVFRAME=1  also broadcast guard-bin AWGN nv to all carriers
#          IRIS_OFDM_PB_LOW/HIGH=300/5000  -> 87-carrier live config (BW 4150 Hz)
#          IRIS_OFDM_PAYLOAD=94            -> ~21-symbol O1 frame (matches live)
set -u
BIN=${BIN:?set BIN}
L=$1; SNR=$2; N=$3; PROFILE=${PROFILE:-mpg}
GENIE_SNR=${GENIE_SNR:-200}
HERE=$(cd "$(dirname "$0")" && pwd)
WORK=${WORK:-/dev/shm/scratch/genie}; mkdir -p "$WORK"; cd "$WORK"
TX=$WORK/tx_L$L.s16
"$BIN" --tx-ofdm "$L" "$TX" >/dev/null 2>&1
bp=0; gp=0; both=0
for s in $(seq 1 "$N"); do
  RX=$WORK/rx_L${L}_s${s}.s16
  CLEAN=$WORK/clean_L${L}_s${s}.s16
  python3 "$HERE/chan_apply.py" "$TX" "$RX"    "$PROFILE" "$SNR"       "$s"
  python3 "$HERE/chan_apply.py" "$TX" "$CLEAN" "$PROFILE" "$GENIE_SNR" "$s"
  br=$(                           "$BIN" --rx-ofdm "$L" "$RX" 2>&1 | grep -oE 'RESULT: (PASS|FAIL|NO DETECTION)' | head -1)
  gr=$(IRIS_GENIE_H_FILE="$CLEAN" "$BIN" --rx-ofdm "$L" "$RX" 2>&1 | grep -oE 'RESULT: (PASS|FAIL|NO DETECTION)' | head -1)
  [ "$br" = 'RESULT: PASS' ] && bp=$((bp+1))
  [ "$gr" = 'RESULT: PASS' ] && gp=$((gp+1))
  echo "L$L seed $s: base=${br#RESULT: } genie=${gr#RESULT: }"
done
echo "=== L$L $PROFILE:$SNR N=$N : BASE_PASS=$bp/$N  GENIE_PASS=$gp/$N (companion $PROFILE:$GENIE_SNR nvframe=${IRIS_GENIE_NVFRAME:-0}) ==="
