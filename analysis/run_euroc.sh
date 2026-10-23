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
# Pre-flight: the launch file must parse (catches XML errors before anything starts).
if ! roslaunch --dump-params svo_ros "$LAUNCH_NAME" frame_event_log_file:="$RUN/frame_events.csv" "$@" > /dev/null 2> "$RUN/preflight.err"; then
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

roslaunch svo_ros "$LAUNCH" frame_event_log_file:="$RUN/frame_events.csv" "$@" > "$RUN/svo.log" 2>&1 &
LAUNCH_PID=$!
for i in $(seq 1 60); do
  rosnode list 2>/dev/null | grep -q '^/svo$' && break
  sleep 1
done
sleep 3
SVO_PID="$(pgrep -f 'svo_ros/svo_node' | head -1)"
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
  > "$RUN/record.log" 2>&1 &
REC_PID=$!
sleep 3

START=$(date +%s.%N)
# Hard limit: bag duration + 120 s, so a dead master can never hang the harness.
BAG_S=$(rosbag info -y -k duration "$INPUT_BAG" | cut -d. -f1)
timeout $((BAG_S + 120)) rosbag play -q "$INPUT_BAG" > "$RUN/play.log" 2>&1 || echo "WARNING: rosbag play exit code $?" >> "$RUN/meta.txt"
END=$(date +%s.%N)
echo "play_wall_seconds: $(echo "$END - $START" | bc)" >> "$RUN/meta.txt"

sleep 5
kill -INT $REC_PID; wait $REC_PID
kill -INT $LAUNCH_PID; wait $LAUNCH_PID
echo "loadavg_after: $(cat /proc/loadavg)" >> "$RUN/meta.txt"
echo "done" >> "$RUN/meta.txt"
