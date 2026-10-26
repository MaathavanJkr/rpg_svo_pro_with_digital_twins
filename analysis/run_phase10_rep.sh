#!/usr/bin/env bash
# FYP Phase 10 confirmatory V2_02 replication driver (pre-registered in docs/PLAN.md, "Phase 10 V2_02 replication
# pre-registration"). Copy of analysis/run_phase10_d4.sh with: sequence V2_02_medium only; arms M2F-heur and M3-heur
# (the D4 heuristic arms, build 0ed4c47 unchanged); 20 rounds; run ids ..._p10rep_rNN; freeze file runs/phase10_rep/.
# Usage (WITHOUT the analysis venv active; no other ROS session may run):
#   analysis/run_phase10_rep.sh freeze                 write runs/phase10_rep/build_freeze.txt (once)
#   analysis/run_phase10_rep.sh batch [first] [last]   rounds first..last (default 1..20); per round: M2F-heur -> M3-heur
# Before every run the frozen build is verified (checksums); any difference aborts. Runs are never repeated.
# Progress: runs/phase10_rep/progress.log.
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
WS="$REPO/../catkin_ws"
P9="$REPO/runs/phase10_rep"
FREEZE="$P9/build_freeze.txt"
SEQS=(V2_02_medium)
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
  ls "$REPO"/svo_registration/config/euroc_v1.yaml "$REPO"/svo_registration/config/euroc_v1_m3.yaml \
     "$REPO"/svo_registration/config/euroc_v2.yaml "$REPO"/svo_registration/config/euroc_v2_m3.yaml
  ls "$REPO"/svo_ros/launch/fyp/euroc_m1.launch "$REPO"/svo_ros/launch/fyp/euroc_m2.launch "$REPO"/svo_ros/launch/fyp/euroc_m3.launch "$REPO"/svo_ros/launch/fyp/euroc_m2fast.launch
  ls "$REPO"/svo_ros/param/fyp/euroc/*.yaml
  echo "$REPO/analysis/run_euroc.sh"
  echo "$REPO/analysis/run_phase10_d4.sh"
  echo "$REPO/analysis/run_phase10_rep.sh"
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
    python analysis/phase9_run_metrics.py '$1' '$2' '$3' > '$1/phase9_metrics.log' 2> '$1/phase9_metrics.err'" || true
}

one_run() {  # run_id sequence method(M2F|M3) information(heuristic|hessian|hessian_calibrated) label
  local id="$1" seq="$2" m="$3" info="$4" label="$5" room="${2:0:2}" args
  local cfg="$REPO/svo_registration/config/euroc_${room,,}"
  local twin="$REPO/dataset/derived/euroc_twin/${room}_leica_v30mm.ply"
  local grid="$REPO/dataset/derived/euroc_twin/df/${room}_leica_v30mm_udf_v50mm.bin"
  case "$m" in
    M1) args=(euroc_m1.launch log_frame_events:=true) ;;
    M2) args=(euroc_m2.launch log_frame_events:=true registration_config:="$cfg.yaml" reference_map:="$twin") ;;
    M2F) args=(euroc_m2fast.launch log_frame_events:=true registration_config:="${cfg}_d4.yaml" reference_map:="$twin" information:="$info") ;;
    M3) args=(euroc_m3.launch log_frame_events:=true registration_config:="${cfg}_m3_d4.yaml" reference_map:="$twin" df_grid:="$grid" information:="$info") ;;
  esac
  verify
  [ -e "$REPO/runs/$id" ] && { echo "$(date -Iseconds) ABORT: $id exists (runs are never repeated)" | tee -a "$P9/progress.log" >&2; exit 1; }
  echo "$(date -Iseconds) start $id" >> "$P9/progress.log"
  local rc=0
  "$REPO/analysis/run_euroc.sh" "$id" "$seq" "${args[@]}" > "$P9/last_harness.log" 2>&1 || rc=$?
  evaluate "$REPO/runs/$id" "$seq" "$label"
  echo "$(date -Iseconds) end   $id harness_rc=$rc $(tail -1 "$REPO/runs/$id/phase9_metrics.log" 2>/dev/null | python3 -c 'import json,sys; d=json.loads(sys.stdin.read() or "{}"); print("failed=%s reason=%s map=%s 4dof=%s" % (d.get("failed"), d.get("failure_reason"), d.get("map_frame_ate_m"), d.get("ate_4dof_m")))' 2>/dev/null)" >> "$P9/progress.log"
  sleep 10   # let the machine settle between runs
}

case "${1:-}" in
  freeze)
    [ -f "$FREEZE" ] && { echo "$FREEZE exists; the build is frozen once" >&2; exit 1; }
    git -C "$REPO" diff --quiet HEAD -- svo_ros svo_registration analysis/run_euroc.sh analysis/run_phase10_d4.sh analysis/run_phase10_rep.sh runs/phase10_d4/calibration.json || { echo "uncommitted changes in the build" >&2; exit 1; }
    { echo "# Phase 10 V2_02 replication frozen build (= D4 build 0ed4c47 + this driver): git $(git -C "$REPO" rev-parse HEAD), $(date -Iseconds)"; checksums; } > "$FREEZE"
    cat "$FREEZE" ;;
  batch)
    first="${2:-1}"; last="${3:-20}"
    for r in $(seq "$first" "$last"); do
      for seq in "${SEQS[@]}"; do
        for m in M2F M3; do
          one_run "${D}_${seq}_${m}-heur_p10rep_r$(printf %02d "$r")" "$seq" "$m" heuristic "${m}-heur"
        done
      done
    done
    echo "$(date -Iseconds) batch rounds $first..$last complete" >> "$P9/progress.log" ;;
  *) sed -n 2,10p "$0"; exit 1 ;;
esac
