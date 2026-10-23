#!/usr/bin/env bash
# Phase 3: one headless Oerlikon M2 run (original pipeline, launch copy svo_ros/launch/fyp/oerlikon_m2.launch).
# Usage: analysis/run_oerlikon_m2.sh <run_id>
# Output: runs/<run_id>/{meta.txt, launch/, config/, svo.log, run.bag}
# Must be run WITHOUT the analysis venv active (nodes use system python3 + ~/.local).
REPO="$(cd "$(dirname "$0")/.." && pwd)"
WS="$REPO/../catkin_ws"
RUN="$REPO/runs/$1"
INPUT_BAG="$REPO/dataset/oerlikon.bag"
MAP="$REPO/dataset/Oerlikon.ply"
mkdir -p "$RUN/launch" "$RUN/config"

source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
set -u   # after sourcing: ROS setup scripts use unset variables

if rosnode list >/dev/null 2>&1; then
  echo "A ROS master is already running; stop it first." >&2; exit 1
fi

# Snapshot exactly what is run.
{
  echo "run_id: $1"
  echo "date: $(date -Iseconds)"
  echo "git_commit: $(git -C "$REPO" rev-parse HEAD)"
  echo "git_dirty: $(git -C "$REPO" status --porcelain | grep -v '^??' | wc -l) tracked files modified"
  echo "launch: svo_ros/launch/fyp/oerlikon_m2.launch rviz:=false reference_map:=$MAP"
  echo "input_bag: $INPUT_BAG (rosbag play, rate 1, no --clock; /use_sim_time=false)"
  LIB="$WS/devel/.private/svo_ros/lib/libsvo_ros.so"; echo "libsvo_ros: $(stat -c %y "$LIB") md5 $(md5sum "$LIB" | cut -c1-12)"
  echo "svo_node binary: $(readlink -f "$WS/devel/lib/svo_ros/svo_node") $(stat -c %y "$(readlink -f "$WS/devel/lib/svo_ros/svo_node")")"
} > "$RUN/meta.txt"
cp "$REPO/svo_ros/launch/fyp/oerlikon_m2.launch" "$RUN/launch/"
LAUNCH_NAME=oerlikon_m2.launch
# Pre-flight: the launch file must parse (catches XML errors before anything starts).
if ! roslaunch --dump-params svo_ros "$LAUNCH_NAME" rviz:=false reference_map:="$MAP" > /dev/null 2> "$RUN/preflight.err"; then
  echo "PREFLIGHT FAILED: launch file does not parse, see $RUN/preflight.err" | tee -a "$RUN/meta.txt" >&2; exit 1
fi
cp "$REPO"/svo_ros/param/Oerlikon/*.yaml "$REPO/svo_registration/config/oerlikon.yaml" "$RUN/config/"

roslaunch svo_ros oerlikon_m2.launch rviz:=false reference_map:="$MAP" > "$RUN/svo.log" 2>&1 &
LAUNCH_PID=$!

# Wait until the registration node has loaded the twin (it then advertises /city_twin).
for i in $(seq 1 120); do
  rostopic list 2>/dev/null | grep -q '^/city_twin$' && rosnode list 2>/dev/null | grep -q '^/svo$' && break
  sleep 1
done
sleep 3
echo "nodes ready after ~${i}s" >> "$RUN/meta.txt"
SVO_PID="$(pgrep -x svo_node | head -1)"
if [ -z "$SVO_PID" ]; then
  echo "ABORT: svo_node did not start within 120 s (see svo.log)" | tee -a "$RUN/meta.txt" >&2
  kill -INT $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null; exit 1
fi

rosbag record -O "$RUN/run.bag" \
  /svo/vio_backend_pose_imu /svo/w_backend_pose_imu \
  /svo/w_backend_pose_points /svo/w_registrated_pose /svo/w_backend_points_registered \
  /groundtruth /gps > "$RUN/record.log" 2>&1 &
REC_PID=$!
sleep 3

START=$(date +%s)
timeout 520 rosbag play -q "$INPUT_BAG" > "$RUN/play.log" 2>&1 &
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
END=$(date +%s)
echo "play_wall_seconds: $((END-START))" >> "$RUN/meta.txt"

sleep 10   # let the last keyframes / registrations finish
kill -INT $REC_PID; wait $REC_PID
kill -INT $LAUNCH_PID; wait $LAUNCH_PID
RUN_ID_ROS="$(grep -o 'setting /run_id to [0-9a-f-]*' "$RUN/svo.log" | awk '{print $4}')"
[ -n "$RUN_ID_ROS" ] && cp "$HOME/.ros/log/$RUN_ID_ROS/rosout.log" "$RUN/rosout.log" 2>/dev/null
echo "ros_run_id: $RUN_ID_ROS" >> "$RUN/meta.txt"
if grep -q "registered with same name" "$RUN/svo.log"; then
  echo "FAILED: a node was replaced by another node with the same name (external ROS session on this master)" | tee -a "$RUN/meta.txt" >&2
fi
if grep -q "process has died" "$RUN/svo.log" && ! grep -q "^FAILED" "$RUN/meta.txt"; then
  echo "FAILED: a node died (see svo.log)" | tee -a "$RUN/meta.txt" >&2
fi
grep -q "^FAILED" "$RUN/meta.txt" && exit 1
echo "done" >> "$RUN/meta.txt"
