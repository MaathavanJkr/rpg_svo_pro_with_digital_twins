#!/usr/bin/env bash
# FYP Phase 10 initial-alignment robustness batch driver (pre-registered in docs/PLAN.md, "Phase 10 alignment
# robustness pre-registration"). Based on analysis/run_phase10_d4.sh. Sequences V1_02, V2_02; injected D1 heading
# offset (svo_ros d1_yaw_offset_deg, about the body at the end of the D1 window); heuristic information; gates
# 30 deg / 2.0 m (configs euroc_<room>_align*.yaml). Arms (refine_T_WL on, gpose_settings.yaml):
#   M2F05 (correspondence 0.5 m) and M3 at delta 0, 10, 20 deg; M2F20 (correspondence 2.0 m, descriptive) at 10, 20 deg.
# Refine-off block (gpose_settings_m2.yaml, descriptive): M2F05-roff, M3-roff at delta 10 deg in rounds 1-5 only.
# Run ids ..._<arm>-d<delta>_p10al_rNN; freeze file runs/phase10_align/.
# Usage (WITHOUT the analysis venv active; no other ROS session may run):
#   analysis/run_phase10_align.sh freeze                 write runs/phase10_align/build_freeze.txt (once)
#   analysis/run_phase10_align.sh smoke                  technical smoke on V1_02 (not in the statistics): M2F05, M3, M2F20 at
#                                                        20 deg and M2F05-roff, M3-roff at 10 deg
#   analysis/run_phase10_align.sh batch [first] [last]   rounds first..last (default 1..10)
# Before every run the frozen build is verified (checksums); any difference aborts. Runs are never repeated.
# Progress: runs/phase10_align/progress.log.
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
WS="$REPO/../catkin_ws"
P9="$REPO/runs/phase10_align"
FREEZE="$P9/build_freeze.txt"
SEQS=(V1_02_medium V2_02_medium)
D="${PHASE9_DATE:-$(date +%F)}"   # one date prefix for all run ids of a batch invocation
mkdir -p "$P9"

frozen_files() {
  echo "$(readlink -f "$WS/devel/lib/svo_ros/svo_node")"
  echo "$WS/devel/.private/svo_ros/lib/libsvo_ros.so"
  ls "$REPO"/svo_registration/scripts/fyp_df.cpython-38-*.so
  echo "$REPO/svo_registration/scripts/online_registration_fyp.py"
  echo "$REPO/svo_registration/scripts/online_registration_m3.py"
  echo "$REPO/svo_registration/scripts/online_registration_m2fast.py"
  echo "$REPO/svo_registration/scripts/p2p_icp_persistent.py"
  ls "$REPO"/svo_registration/config/euroc_v1_d4.yaml "$REPO"/svo_registration/config/euroc_v2_d4.yaml \
     "$REPO"/svo_registration/config/euroc_v1_m3_d4.yaml "$REPO"/svo_registration/config/euroc_v2_m3_d4.yaml
  echo "$REPO/runs/phase10_d4/calibration.json"
  ls "$REPO"/svo_registration/config/euroc_v1_align.yaml "$REPO"/svo_registration/config/euroc_v2_align.yaml \
     "$REPO"/svo_registration/config/euroc_v1_align_r2.yaml "$REPO"/svo_registration/config/euroc_v2_align_r2.yaml \
     "$REPO"/svo_registration/config/euroc_v1_m3_align.yaml "$REPO"/svo_registration/config/euroc_v2_m3_align.yaml
  ls "$REPO"/svo_registration/config/euroc_v1.yaml "$REPO"/svo_registration/config/euroc_v1_m3.yaml \
     "$REPO"/svo_registration/config/euroc_v2.yaml "$REPO"/svo_registration/config/euroc_v2_m3.yaml
  ls "$REPO"/svo_ros/launch/fyp/euroc_m1.launch "$REPO"/svo_ros/launch/fyp/euroc_m2.launch "$REPO"/svo_ros/launch/fyp/euroc_m3.launch "$REPO"/svo_ros/launch/fyp/euroc_m2fast.launch
  ls "$REPO"/svo_ros/param/fyp/euroc/*.yaml
  echo "$REPO/analysis/run_euroc.sh"
  echo "$REPO/analysis/run_phase10_align.sh"
  ls "$REPO"/dataset/derived/euroc_twin/V1_leica_v30mm.ply "$REPO"/dataset/derived/euroc_twin/V2_leica_v30mm.ply
  ls "$REPO"/dataset/derived/euroc_twin/df/V1_leica_v30mm_udf_v50mm.bin* "$REPO"/dataset/derived/euroc_twin/df/V2_leica_v30mm_udf_v50mm.bin*
}

checksums() { frozen_files | while read -r f; do md5sum "$f"; done; }

verify() {
  [ -f "$FREEZE" ] || { echo "no $FREEZE; run 'freeze' first" >&2; exit 1; }
  if ! diff <(grep -v '^#' "$FREEZE") <(checksums) > "$P9/verify.diff"; then
    echo "$(date -Iseconds) ABORT: build differs from the frozen build (see $P9/verify.diff)" | tee -a "$P9/progress.log" >&2; exit 1
  fi
}

evaluate() {  # run_dir sequence method
  bash -c "source /opt/ros/noetic/setup.bash && source '$WS/devel/setup.bash' && source '$HOME/venvs/fyp/bin/activate' && cd '$REPO' &&
    python analysis/eval_vio.py '$1' '$2' > '$1/eval.log' 2> '$1/eval.err';
    python analysis/initial_alignment.py '$1' '$2' --kf-log '$1/frame_events.csv' > '$1/align.log' 2> '$1/align.err';
    if [ '$3' != M1 ]; then python analysis/phase6_events.py '$1' '$2' > '$1/events.log' 2> '$1/events.err'; fi;
    python analysis/phase9_run_metrics.py '$1' '$2' '$3' > '$1/phase9_metrics.log' 2> '$1/phase9_metrics.err';
    python analysis/phase10_align_analysis.py run '$1' > '$1/align_outcome.log' 2> '$1/align_outcome.err'" || true
}

one_run() {  # run_id sequence arm(M2F05|M2F20|M3) delta_deg refine(on|off) label
  local id="$1" seq="$2" arm="$3" delta="$4" refine="$5" label="$6" room="${2:0:2}" args
  local cfg="$REPO/svo_registration/config/euroc_${room,,}"
  local twin="$REPO/dataset/derived/euroc_twin/${room}_leica_v30mm.ply"
  local grid="$REPO/dataset/derived/euroc_twin/df/${room}_leica_v30mm_udf_v50mm.bin"
  local gp=gpose_settings.yaml
  [ "$refine" = off ] && gp=gpose_settings_m2.yaml
  case "$arm" in
    M2F05) args=(euroc_m2fast.launch registration_config:="${cfg}_align.yaml" reference_map:="$twin") ;;
    M2F20) args=(euroc_m2fast.launch registration_config:="${cfg}_align_r2.yaml" reference_map:="$twin") ;;
    M3) args=(euroc_m3.launch registration_config:="${cfg}_m3_align.yaml" reference_map:="$twin" df_grid:="$grid") ;;
  esac
  args+=(log_frame_events:=true information:=heuristic gpose_settings:="$gp" d1_yaw_offset_deg:="$delta")
  verify
  [ -e "$REPO/runs/$id" ] && { echo "$(date -Iseconds) ABORT: $id exists (runs are never repeated)" | tee -a "$P9/progress.log" >&2; exit 1; }
  echo "$(date -Iseconds) start $id" >> "$P9/progress.log"
  local rc=0
  "$REPO/analysis/run_euroc.sh" "$id" "$seq" "${args[@]}" > "$P9/last_harness.log" 2>&1 || rc=$?
  evaluate "$REPO/runs/$id" "$seq" "$label"
  echo "$(date -Iseconds) end   $id harness_rc=$rc $(tail -1 "$REPO/runs/$id/align_outcome.log" 2>/dev/null | python3 -c 'import json,sys; d=json.loads(sys.stdin.read() or "{}"); print("refined=%s recovered=%s" % (d.get("refined"), d.get("recovered")))' 2>/dev/null)" >> "$P9/progress.log"
  sleep 10   # let the machine settle between runs
}

case "${1:-}" in
  freeze)
    [ -f "$FREEZE" ] && { echo "$FREEZE exists; the build is frozen once" >&2; exit 1; }
    git -C "$REPO" diff --quiet HEAD -- svo_ros svo_registration analysis/run_euroc.sh analysis/run_phase10_align.sh || { echo "uncommitted changes in the build" >&2; exit 1; }
    { echo "# Phase 10 alignment-robustness frozen build: git $(git -C "$REPO" rev-parse HEAD), $(date -Iseconds)"; checksums; } > "$FREEZE"
    cat "$FREEZE" ;;
  smoke)
    for arm in M2F05 M3 M2F20; do
      one_run "${D}_V1_02_medium_${arm}-d20_p10al_smoke" V1_02_medium "$arm" 20.0 on "${arm}-d20"
    done
    for arm in M2F05 M3; do
      one_run "${D}_V1_02_medium_${arm}-roff-d10_p10al_smoke" V1_02_medium "$arm" 10.0 off "${arm}-roff-d10"
    done ;;
  batch)
    first="${2:-1}"; last="${3:-10}"
    for r in $(seq "$first" "$last"); do
      rr=$(printf %02d "$r")
      for seq in "${SEQS[@]}"; do
        for delta in 0 10 20; do
          arms="M2F05 M3"
          [ "$delta" != 0 ] && arms="M2F05 M3 M2F20"
          for arm in $arms; do
            one_run "${D}_${seq}_${arm}-d${delta}_p10al_r${rr}" "$seq" "$arm" "${delta}.0" on "${arm}-d${delta}"
          done
        done
        if [ "$r" -le 5 ]; then
          for arm in M2F05 M3; do
            one_run "${D}_${seq}_${arm}-roff-d10_p10al_r${rr}" "$seq" "$arm" 10.0 off "${arm}-roff-d10"
          done
        fi
      done
    done
    echo "$(date -Iseconds) batch rounds $first..$last complete" >> "$P9/progress.log" ;;
  *) sed -n 2,10p "$0"; exit 1 ;;
esac
