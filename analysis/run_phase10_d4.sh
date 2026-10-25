#!/usr/bin/env bash
# FYP Phase 10 D4 weighting-ablation batch driver (pre-registered in docs/PLAN.md, "Phase 10 D4 ablation
# pre-registration"). Copy of analysis/run_phase10_tc.sh with: sequences V1_01, V1_02, V2_02; arms
# {M2F, M3} x {heuristic, hessian, hessian_calibrated} (launch argument information:=...), D4 configs
# euroc_<room>_d4.yaml (M2F) / euroc_<room>_m3_d4.yaml (M3); run ids ..._<M2F|M3>-<heur|hess|cal>_p10d4_rNN;
# freeze file runs/phase10_d4/.
# Usage (WITHOUT the analysis venv active; no other ROS session may run):
#   analysis/run_phase10_d4.sh freeze                 write runs/phase10_d4/build_freeze.txt (once)
#   analysis/run_phase10_d4.sh smoke                  M2F-cal and M3-cal on V1_02_medium (technical check, not in the statistics)
#   analysis/run_phase10_d4.sh batch [first] [last]   rounds first..last (default 1..10); per round, per sequence:
#                                                     M2F-heur, M2F-hess, M2F-cal, M3-heur, M3-hess, M3-cal
# Before every run the frozen build is verified (checksums of binaries, module, nodes, configs, launch
# files, harness, twins and grids); any difference aborts the batch. Each run is followed by its
# evaluation (eval_vio, initial_alignment, phase6_events, phase9_run_metrics).
# Progress: runs/phase10_d4/progress.log. Runs are never repeated (pre-registered reset rule).
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
WS="$REPO/../catkin_ws"
P9="$REPO/runs/phase10_d4"
FREEZE="$P9/build_freeze.txt"
SEQS=(V1_01_easy V1_02_medium V2_02_medium)
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
    git -C "$REPO" diff --quiet HEAD -- svo_ros svo_registration analysis/run_euroc.sh analysis/run_phase10_d4.sh runs/phase10_d4/calibration.json || { echo "uncommitted changes in the build" >&2; exit 1; }
    { echo "# Phase 10 D4 ablation frozen build: git $(git -C "$REPO" rev-parse HEAD), $(date -Iseconds)"; checksums; } > "$FREEZE"
    cat "$FREEZE" ;;
  smoke)
    one_run "${D}_V1_02_medium_M2F-cal_p10d4_smoke" V1_02_medium M2F hessian_calibrated M2F-cal
    one_run "${D}_V1_02_medium_M3-cal_p10d4_smoke" V1_02_medium M3 hessian_calibrated M3-cal
    cat "$REPO"/runs/${D}_V1_02_medium_M*-cal_p10d4_smoke/phase9_metrics.log ;;
  batch)
    first="${2:-1}"; last="${3:-10}"
    for r in $(seq "$first" "$last"); do
      for seq in "${SEQS[@]}"; do
        for m in M2F M3; do
          for info in heuristic hessian hessian_calibrated; do
            case "$info" in heuristic) s=heur ;; hessian) s=hess ;; hessian_calibrated) s=cal ;; esac
            one_run "${D}_${seq}_${m}-${s}_p10d4_r$(printf %02d "$r")" "$seq" "$m" "$info" "${m}-${s}"
          done
        done
      done
    done
    echo "$(date -Iseconds) batch rounds $first..$last complete" >> "$P9/progress.log" ;;
  *) sed -n 2,10p "$0"; exit 1 ;;
esac
