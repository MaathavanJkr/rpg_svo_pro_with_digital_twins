# EuRoC
# run: ./record_vio.sh $dir
ros2 bag record -o $1/vio_estimate /svo/vio_backend_pose_imu
