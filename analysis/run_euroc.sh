#!/usr/bin/env bash
# One headless EuRoC run.
# Usage: analysis/run_euroc.sh <run_id> <sequence> <launch_file_in_svo_ros> [extra roslaunch args...]
#   e.g. analysis/run_euroc.sh 2026-10-23_MH_01_easy_M1_r01 MH_01_easy euroc_m1.launch
# Output: runs/<run_id>/{meta.txt, launch/, config/, svo.log, run.bag, cpu.csv[, frame_events.csv]}
# The frame-event log path is always set to runs/<run_id>/frame_events.csv; it is only written if
# log_frame_events:=true is passed.
# Must be run WITHOUT the analysis venv active.
REPO="$(cd "$(dirname "$0")/.." && pwd)"
WS="$REPO/../catkin_ws"
RUN_ID="$1"; SEQ="$2"; LAUNCH="$3"; shift 3
RUN="$REPO/runs/$RUN_ID"
INPUT_BAG="$(ls "$REPO"/dataset/*/"$SEQ"/"$SEQ".bag)"
mkdir -p "$RUN/launch" "$RUN/config"

source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
set -u   # after sourcing: ROS setup scripts use unset variables

if rosnode list >/dev/null 2>&1; then
  echo "A ROS master is already running; stop it first." >&2; exit 1
fi

LAUNCH_PATH="$(find "$REPO/svo_ros/launch/fyp" -name "$LAUNCH")"
LAUNCH_NAME="$LAUNCH"
# Per-run log paths and the sequence's GT file, passed only if the launch file declares the arg.
EXTRA=(frame_event_log_file:="$RUN/frame_events.csv")
grep -q 'name="d1_gt_file"' "$LAUNCH_PATH" && EXTRA+=(d1_gt_file:="$REPO/dataset/derived/asl_meta/$SEQ/mav0/state_groundtruth_estimate0/data.csv" d1_log_file:="$RUN/d1_alignment.csv")
grep -q 'name="registration_log_file"' "$LAUNCH_PATH" && EXTRA+=(registration_log_file:="$RUN/registration.csv")
# Phase 6 logging (always on when the launch supports it).
grep -q 'name="registration_event_dir"' "$LAUNCH_PATH" && EXTRA+=(registration_event_dir:="$RUN/reg_events" log_registration_events:=true registration_event_log_file:="$RUN/registration_events.csv")
# Pre-flight: the launch file must parse (catches XML errors before anything starts).
if ! roslaunch --dump-params svo_ros "$LAUNCH_NAME" "${EXTRA[@]}" "$@" > /dev/null 2> "$RUN/preflight.err"; then
  echo "PREFLIGHT FAILED: launch file does not parse, see $RUN/preflight.err" | tee -a "$RUN/meta.txt" >&2; exit 1
fi
{
  echo "run_id: $RUN_ID"
  echo "sequence: $SEQ"
  echo "date: $(date -Iseconds)"
  echo "git_commit: $(git -C "$REPO" rev-parse HEAD)"
  echo "git_dirty: $(git -C "$REPO" status --porcelain | grep -v '^??' | wc -l) tracked files modified"
  echo "launch: svo_ros/launch/fyp/$LAUNCH $*"
  echo "input_bag: $INPUT_BAG (rosbag play, rate 1, no --clock)"
  echo "svo_node binary: $(readlink -f "$WS/devel/lib/svo_ros/svo_node") $(stat -c %y "$(readlink -f "$WS/devel/lib/svo_ros/svo_node")")"
  LIB="$WS/devel/.private/svo_ros/lib/libsvo_ros.so"; echo "libsvo_ros: $(stat -c %y "$LIB") md5 $(md5sum "$LIB" | cut -c1-12)"
  echo "loadavg_before: $(cat /proc/loadavg)"
  echo "nproc: $(nproc)"
} > "$RUN/meta.txt"
cp "$LAUNCH_PATH" "$RUN/launch/"
cp "$REPO"/svo_ros/param/fyp/euroc/*.yaml "$RUN/config/"
cp "$REPO"/svo_registration/config/euroc_v1.yaml "$REPO"/dataset/derived/euroc_twin/*.json "$RUN/config/" 2>/dev/null

roslaunch svo_ros "$LAUNCH" "${EXTRA[@]}" "$@" > "$RUN/svo.log" 2>&1 &
LAUNCH_PID=$!
for i in $(seq 1 60); do
  rosnode list 2>/dev/null | grep -q '^/svo$' && break
  sleep 1
done
sleep 3
SVO_PID="$(pgrep -x svo_node | head -1)"
echo "nodes ready after ~${i}s, svo_node pid ${SVO_PID}" >> "$RUN/meta.txt"
if [ -z "$SVO_PID" ]; then
  echo "ABORT: svo_node did not start within 60 s (see svo.log)" | tee -a "$RUN/meta.txt" >&2
  kill -INT $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null; exit 1
fi

# CPU sampler: 1 Hz, system-wide busy % and svo_node % of one core.
(
  echo "wall_time,sys_busy_pct,svo_node_pct_one_core"
  read -r _ a b c d e f g _ < /proc/stat; prev_tot=$((a+b+c+d+e+f+g)); prev_idle=$((d+e))
  prev_p=$(awk '{print $14+$15}' /proc/$SVO_PID/stat 2>/dev/null || echo 0)
  hz=$(getconf CLK_TCK)
  while kill -0 "$SVO_PID" 2>/dev/null; do
    sleep 1
    read -r _ a b c d e f g _ < /proc/stat; tot=$((a+b+c+d+e+f+g)); idle=$((d+e))
    p=$(awk '{print $14+$15}' /proc/$SVO_PID/stat 2>/dev/null || echo "$prev_p")
    echo "$(date +%s.%N),$(( 100*((tot-prev_tot)-(idle-prev_idle))/(tot-prev_tot) )),$(( 100*(p-prev_p)/hz ))"
    prev_tot=$tot; prev_idle=$idle; prev_p=$p
  done
) > "$RUN/cpu.csv" &

rosbag record -O "$RUN/run.bag" /svo/vio_backend_pose_imu /svo/pose_imu /svo/info /imu0 \
  /svo/w_backend_pose_imu /svo/w_registrated_pose /svo/w_backend_pose_points /svo/w_backend_points_registered /tf_static \
  > "$RUN/record.log" 2>&1 &
REC_PID=$!
sleep 3

START=$(date +%s.%N)
# Hard limit: bag duration + 120 s, so a dead master can never hang the harness.
BAG_S=$(rosbag info -y -k duration "$INPUT_BAG" | cut -d. -f1)
timeout $((BAG_S + 120)) rosbag play -q "$INPUT_BAG" > "$RUN/play.log" 2>&1 &
PLAY_PID=$!
# Watchdog: if svo_node dies during playback, stop the bag and mark the run FAILED.
while kill -0 $PLAY_PID 2>/dev/null; do
  if ! kill -0 "$SVO_PID" 2>/dev/null; then
    echo "FAILED: svo_node (pid $SVO_PID) died during playback; see svo.log" | tee -a "$RUN/meta.txt" >&2
    kill -INT $PLAY_PID 2>/dev/null; break
  fi
  sleep 1
done
wait $PLAY_PID 2>/dev/null
END=$(date +%s.%N)
echo "play_wall_seconds: $(echo "$END - $START" | bc)" >> "$RUN/meta.txt"

sleep 5
kill -INT $REC_PID; wait $REC_PID
kill -INT $LAUNCH_PID; wait $LAUNCH_PID
echo "loadavg_after: $(cat /proc/loadavg)" >> "$RUN/meta.txt"
RUN_ID_ROS="$(grep -o 'setting /run_id to [0-9a-f-]*' "$RUN/svo.log" | awk '{print $4}')"
[ -n "$RUN_ID_ROS" ] && cp "$HOME/.ros/log/$RUN_ID_ROS/rosout.log" "$RUN/rosout.log" 2>/dev/null
echo "ros_run_id: $RUN_ID_ROS" >> "$RUN/meta.txt"
if grep -q "registered with same name" "$RUN/svo.log"; then
  echo "FAILED: a node was replaced by another node with the same name (external ROS session on this master)" | tee -a "$RUN/meta.txt" >&2
fi
if grep -q "process has died" "$RUN/svo.log" && ! grep -q "^FAILED" "$RUN/meta.txt"; then
  echo "FAILED: a node died (see svo.log)" | tee -a "$RUN/meta.txt" >&2
fi
# Phase 6: manifest of the per-KF event dumps (the dumps themselves are git-ignored).
if [ -d "$RUN/reg_events" ]; then
  python3 - "$RUN" <<'PYEOF'
import csv, glob, hashlib, os, sys
run = sys.argv[1]
with open(os.path.join(run, "reg_events_manifest.csv"), "w", newline="") as fh:
    w = csv.writer(fh); w.writerow(["file", "bytes", "sha256"])
    for f in sorted(glob.glob(os.path.join(run, "reg_events", "*.npz"))):
        b = open(f, "rb").read(); w.writerow([os.path.basename(f), len(b), hashlib.sha256(b).hexdigest()])
PYEOF
fi
grep -q "^FAILED" "$RUN/meta.txt" && exit 1
echo "done" >> "$RUN/meta.txt"
