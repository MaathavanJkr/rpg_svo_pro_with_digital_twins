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

rosbag record -O "$RUN/run.bag" \
  /svo/vio_backend_pose_imu /svo/w_backend_pose_imu \
  /svo/w_backend_pose_points /svo/w_registrated_pose /svo/w_backend_points_registered \
  /groundtruth /gps > "$RUN/record.log" 2>&1 &
REC_PID=$!
sleep 3

START=$(date +%s)
rosbag play -q "$INPUT_BAG" > "$RUN/play.log" 2>&1
END=$(date +%s)
echo "play_wall_seconds: $((END-START))" >> "$RUN/meta.txt"

sleep 10   # let the last keyframes / registrations finish
kill -INT $REC_PID; wait $REC_PID
kill -INT $LAUNCH_PID; wait $LAUNCH_PID
echo "done" >> "$RUN/meta.txt"
