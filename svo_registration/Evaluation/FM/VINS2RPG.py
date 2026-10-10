from vikit_py.rosbag_utils import BagReader, BagWriter, stamp_to_sec, sec_to_stamp  # ROS 2 bags (ROS 1: rosbag/rospy)
import csv
from geometry_msgs.msg import PoseStamped
import os
import subprocess
from datetime import datetime

"""
Input: ROS bag file with ground truth and estimated poses
Output: CSV files with ground truth and estimated poses, RPG format. They will be stored in the adapted folder for the RPG evaluation toolbox.
"""

# Writes pose with covariance msgs to a CSV file
def write_to_csv_cov_pose(filename, messages):
    with open(filename, 'w') as csvfile:
        writer = csv.writer(csvfile, delimiter=' ')
        csvfile.write("# time x y z qx qy qz qw\n")
        for msg in messages:
            timestamp = stamp_to_sec(msg.header.stamp)
            pose = msg.pose.pose
            position = pose.position
            orientation = pose.orientation
            writer.writerow([timestamp, position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w])

# Writes odometry msg to a CSV file
def write_to_csv_odo(filename, messages):
    with open(filename, 'w') as csvfile:
        writer = csv.writer(csvfile, delimiter=' ')
        csvfile.write("# time x y z qx qy qz qw\n")
        for msg in messages:
            timestamp = stamp_to_sec(msg.header.stamp)
            pose = msg.pose.pose
            position = pose.position
            orientation = pose.orientation
            writer.writerow([timestamp, position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w])

# Writes pose to a CSV file
def write_to_csv_pose(filename, messages, topic):
    k=0
    with open(filename, 'w') as csvfile:
        writer = csv.writer(csvfile, delimiter=' ')
        csvfile.write("# time x y z qx qy qz qw\n")
        for msg in messages:
            k=k+1
            if ((k%2==0) and topic == ground_truth_topic):
                continue
            timestamp = stamp_to_sec(msg.header.stamp)
            pose = msg.pose
            position = pose.position
            orientation = pose.orientation
            writer.writerow([timestamp, position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w])

# Generate a unique folder name with date and time
folder_name = 'VINS_OERLIKON' + datetime.now().strftime("svo_%Y-%m-%d_%H-%M-%S")
result_dir = os.path.join('/home/roxane/ros_ws/src/rpg_trajectory_evaluation/results', folder_name)

bag_path = '/home/roxane/SVO_MR_DATA/FM/2024-06-09-18-43-22.bag' #VINS/res.bag' #/home/roxane/SVO_MR_DATA/FM/SVO_MR_GPS/SVO_VIO_GPS_MR.bag' #home/roxane/SVO_MR_DATA/Oerlikon/SVO_GPS_MR/res.bag' #/home/roxane/Results/V2/VINS/oerlikon_vins_res.bag' #'/home/roxane/Results/V2/VINS/FM_vins.bag' #/home/roxane/Results/Recordings/FM2/withdiffg.bag' #Recordings/TODO/SVO_VIO_GPS_MR.bag'#' /home/roxane/Results/Recordings/VIO_GPS_MR_oerlikon_out_v3.bag' #/home/roxane/Results/Recordings/VIO_GPS_oerlikon_L_gt.bag' #2024-06-03-15-57-48.bag' #2024-06-03-15-43-41.bag' #2024-06-03-15-17-02.bag' #SVO_GPS.bag' #Datasets/1605/HigherWeights.bag' #/home/roxane/elios3/Fly_Data/Results/MR_endbif.bag' #/home/roxane/Thesis_Results/2604/v2/2024-04-28-23-13-07.bag' #Flightmare/2404/Results/Results_No_GPS.bag' #/Results/Results_NO_MR.bag' #   /home/roxane/Oerlikon_Data/result_vio_no_mr.bag' #/home/roxane/Oerlikon_Data/result_vio.bag' #resultVO.bag' #/home/roxane//Datasets/Results2803/withmapreg.bag' #svo_ws/result_v2.bag' #/home/roxane/Datasets/FM1503/withreg.bag' #/resultwithcorrectstamp_dowysampled.bag' #/home/roxane/Datasets/1203/resultFM0703.bag' #AGZDataset/results/result.bag' #/home/roxane/Datasets/FM07036/results/2024-03-08-18-01-13.bag'
# bag_path = '/home/roxane/SVO_MR_DATA/Oerlikon/SVO_VIO_GPS_v2_res.bag' #'/home/roxane/Results/V2/VINS/oerlikon_vins_res.bag' #'/home/roxane/Results/V2/VINS/FM_vins.bag' #/home/roxane/Results/Recordings/FM2/withdiffg.bag' #Recordings/TODO/SVO_VIO_GPS_MR.bag'#' /home/roxane/Results/Recordings/VIO_GPS_MR_oerlikon_out_v3.bag' #/home/roxane/Results/Recordings/VIO_GPS_oerlikon_L_gt.bag' #2024-06-03-15-57-48.bag' #2024-06-03-15-43-41.bag' #2024-06-03-15-17-02.bag' #SVO_GPS.bag' #Datasets/1605/HigherWeights.bag' #/home/roxane/elios3/Fly_Data/Results/MR_endbif.bag' #/home/roxane/Thesis_Results/2604/v2/2024-04-28-23-13-07.bag' #Flightmare/2404/Results/Results_No_GPS.bag' #/Results/Results_NO_MR.bag' #   /home/roxane/Oerlikon_Data/result_vio_no_mr.bag' #/home/roxane/Oerlikon_Data/result_vio.bag' #resultVO.bag' #/home/roxane//Datasets/Results2803/withmapreg.bag' #svo_ws/result_v2.bag' #/home/roxane/Datasets/FM1503/withreg.bag' #/resultwithcorrectstamp_dowysampled.bag' #/home/roxane/Datasets/1203/resultFM0703.bag' #AGZDataset/results/result.bag' #/home/roxane/Datasets/FM07036/results/2024-03-08-18-01-13.bag'
ground_truth_topic = '/kingfisher/agiros_pilot/groundtruth/pose' #/col_T_enu_b' #kingfisher/agiros_pilot/groundtruth/pose' #/col_T_enu_b' #/kingfisher/agiros_pilot/groundtruth/pose' #/vio/odometry' #/kingfisher/agiros_pilot/groundtruth/pose' #/vio/odometry' #/groundtruth/pose' #/kingfisher/agiros_pilot/groundtruth/pose' #ground_truth/xyz' #/kingfisher/agiros_pilot/groundtruth/pose' #'ground_truth/xyz'
estimated_pose_topic = '/vins_estimator/odometry' #/globalEstimator/global_odometry' #/svo/vio_backend_pose_imu_viz' #/ENU_GPS_t0' #/svo/vio_backend_pose_imu_viz' #/svo/vio_backend_pose_imu_viz' #/map_alignement_T_w_b' #/svo/vio_backend_pose_imu_viz' #/svo/pose_cam/0' #/svo/vio_backend_pose_imu' #/svo/pose_imu' #/svo/vio_backend_pose_imu_viz' #/svo/pose_imu' #/svo/vio_backend_pose_imu_viz'
# estimated_pose_topic = '/svo/vio_backend_pose_imu_viz' #/svo/vio_backend_pose_imu_viz' #/map_alignement_T_w_b' #/svo/vio_backend_pose_imu_viz' #/svo/pose_cam/0' #/svo/vio_backend_pose_imu' #/svo/pose_imu' #/svo/vio_backend_pose_imu_viz' #/svo/pose_imu' #/svo/vio_backend_pose_imu_viz'

ground_truth_msgs = []
estimated_pose_msgs = []

with BagReader(bag_path) as bag:
    for topic, msg, t in bag.read_messages(topics=[ground_truth_topic, estimated_pose_topic]):
        if(stamp_to_sec(msg.header.stamp) > 300):
            break

        if topic == ground_truth_topic:
            ground_truth_msgs.append(msg)
        elif topic == estimated_pose_topic:
            estimated_pose_msgs.append(msg)

if not os.path.exists(result_dir):
    os.makedirs(result_dir)

write_to_csv_pose(os.path.join(result_dir, 'stamped_groundtruth.txt'), ground_truth_msgs, ground_truth_topic)
# write_to_csv_odo(os.path.join(result_dir, 'stamped_groundtruth.txt'), ground_truth_msgs)
# write_to_csv_pose(os.path.join(result_dir, 'stamped_traj_estimate.txt'), estimated_pose_msgs, estimated_pose_msgs)
write_to_csv_cov_pose(os.path.join(result_dir, 'stamped_traj_estimate.txt'), estimated_pose_msgs)


# Create eval_cfg.yaml and start_end_time.yaml
with open(os.path.join(result_dir, 'eval_cfg.yaml'), 'w') as file:
    file.write('align_type: se3\nalign_num_frames: -1\n') #put posyaw/none for w frame.

subprocess.run([
    'python2',
    '/home/roxane/ros_ws/src/rpg_trajectory_evaluation/scripts/analyze_trajectory_single.py',
    '--result_dir',
    f'{result_dir}'
])

# subprocess.run(['python2', '/home/roxane/ros_ws/src/rpg_trajectory_evaluation/scripts/analyze_trajectory_single.py', "--result_dir /home/roxane/ros_ws/src/rpg_trajectory_evaluation/results/" + result_dir])


# then to run the evaluation script : python2 analyze_trajectory_single.py --result_dir ./../results/<folder_name>
