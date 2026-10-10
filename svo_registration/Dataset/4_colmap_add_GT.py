
import numpy as np
from vikit_py.rosbag_utils import BagReader, BagWriter, stamp_to_sec, sec_to_stamp  # ROS 2 bags (ROS 1: rosbag/rospy)
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R


def load_trajectory(file_path):
    trajectory = []
    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith('#'):
                continue  # Skip the header line
            parts = line.split()
            timestamp = float(parts[0])
            tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
            qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            # Adjust the timestamp as needed (e.g., subtract a specific offset)
            # adjusted_timestamp = timestamp - 129.4563
            # Store the trajectory as (timestamp, translation vector, quaternion)
            trajectory.append((timestamp, np.array([tx, ty, tz, 1.0]), (qx, qy, qz, qw)))
    return trajectory


def transform_trajectory(trajectory, scale, T_WL, T_BC):
    transformed_trajectory = []
    T_CB = np.eye(4)
    T_CB = np.linalg.inv(T_BC.reshape(4, 4))

    for timestamp, position, orientation in trajectory:
        # Apply scale
        scaled_position = position * scale
        scaled_position[3] = 1.0

        # Create transformation matrix from quaternion orientation and position
        rotation = R.from_quat(orientation) #format x y z w
        T = np.eye(4)
        T[:3, :3] = rotation.as_matrix()
        T[:3, 3] = scaled_position[:3]

        # Apply transformation
        T_new = T_WL @ T @ T_CB
        transformed_position = T_new[:3, 3]
        orientation_new = R.from_matrix(T_new[:3, :3]).as_quat() # x y z w
        orientation_new = orientation_new / np.linalg.norm(orientation_new)
        # orientation_new = orientation 
        # transformed_position = scaled_position
        
        # Debug prints
        # print(f"Original Position: {position[:3]}, Scaled Position: {scaled_position[:3]}, Transformed Position: {transformed_position}")
        # print(f"Original Orientation: {orientation}, Transformed Orientation: {orientation_new}")
       
        transformed_trajectory.append((timestamp, transformed_position, orientation_new))
    return transformed_trajectory

def save_to_rosbag(new_bag_path, old_bag_path, trajectory):
    with BagWriter(new_bag_path) as new_bag:
        with BagReader(old_bag_path) as old_bag:
            for topic, msg, t in old_bag.read_messages():
                if topic in ['/camera_2/image_raw', '/vio_module/imu_filtered', '/gps_lidar_enu', '/gps_lidar']:
                        msg.header.stamp = sec_to_stamp(stamp_to_sec(t)) #1520.036681984)
                        new_bag.write(topic, msg, msg.header.stamp)
        ini = True
        for timestamp, position, orientation in trajectory:
            if(ini):
                t0 = sec_to_stamp(timestamp)
                ini = False
            # if(timestamp - stamp_to_sec(t0) > 275):
            #     break

            pose = PoseStamped()
            pose.header.stamp = sec_to_stamp(timestamp)
            pose.header.frame_id = 'world'
            pose.pose.position.x = position[0]
            pose.pose.position.y = position[1]
            pose.pose.position.z = position[2]
            pose.pose.orientation.x = orientation[0]
            pose.pose.orientation.y = orientation[1]
            pose.pose.orientation.z = orientation[2]
            pose.pose.orientation.w = orientation[3]
            new_bag.write('/col_T_enu_b', pose, pose.header.stamp)

def save_to_ply(file_path, trajectory):
    with open(file_path, 'w') as file:
        file.write('ply\n')
        file.write('format ascii 1.0\n')
        file.write(f'element vertex {len(trajectory)}\n')
        file.write('property float x\n')
        file.write('property float y\n')
        file.write('property float z\n')
        file.write('end_header\n')
        for _, position, _ in trajectory:
            file.write(f'{position[0]} {position[1]} {position[2]}\n')




#  [[-7.00777620e-01 -1.15698932e-01  7.03935000e-01 -2.79914408e+01]
#  [-7.10027186e-01  1.75673842e-02 -7.03955100e-01 -5.15874149e-01]
#  [ 6.90805566e-02 -9.93128967e-01 -9.44602168e-02  1.10370128e+01]
#  [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]

# T_WL = np.array([[-7.00843090e-01 -1.16009973e-01  7.03818620e-01 -2.50330413e+01],
#  [-7.09954609e-01  1.77765147e-02 -7.04023046e-01  5.21430996e+00],
#  [ 6.91622523e-02 -9.93088960e-01 -9.48203623e-02  6.18837094e+00],
#  [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]])

# Best Scale: 5.953
# Best Transformation matrix:
#  [[ -0.72850086  -0.13685219   0.67123615 -28.071696  ]
#  [ -0.68070636   0.03451004  -0.73174306   0.03351191]
#  [  0.07697626  -0.98999017  -0.11829675  11.1723853 ]
#  [  0.           0.           0.           1.        ]]

# scale = 5.948
scale = 5.953

T_WL = np.array([[-0.72850086, -0.13685219, 0.67123615, -28.071696],
                    [-0.68070636, 0.03451004, -0.73174306, 0.03351191],
                    [0.07697626, -0.98999017, -0.11829675, 11.1723853],
                    [0, 0, 0, 1]])
# T_WL = np.array([-7.00843090e-01, -1.16009973e-01, 7.03818620e-01, -2.50330413e+01,
#                     -7.09954609e-01, 1.77765147e-02, -7.04023046e-01, 5.21430996e+00,
#                     6.91622523e-02, -9.93088960e-01, -9.48203623e-02, 6.18837094e+00,
#                     0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]).reshape(4, 4)




T_BC = np.array([0.003282939472533192, -0.19294942909971347, 0.9812032104098082, 0.030087415920571525,
                -0.9999469827980318, 0.008943046094240282, 0.005104264850980736, 0.014700960773767908,
                -0.009759810527481877, -0.9811679467535876, -0.19290983998183384, -0.010556222345895877,
                0.0, 0.0, 0.0, 1.0]).reshape(4, 4)

#T_WB = T_WL*T_LC*T_CB

# trajectory = load_trajectory("/home/roxane/elios3_2706/EL300825315585_01645_214_1flight/output_no_LC_v1/colmap_cam_estimates.txt") # /home/roxane/ros_ws/src/svo_mr_utility/Spline_Alignement_V3/colmap_cam_estimates.txt') #/home/roxane/DatasetForPres/colmap_v6/output/0/colmap_body_estimates.txt')
trajectory = load_trajectory("/home/roxane/PaperData/NewOerlikonColmap/calibration_from_kalibr/colmap_5Hz/output/0/cam_traj.txt") # /home/roxane/ros_ws/src/svo_mr_utility/Spline_Alignement_V3/colmap_cam_estimates.txt') #/home/roxane/DatasetForPres/colmap_v6/output/0/colmap_body_estimates.txt')
transformed_trajectory = transform_trajectory(trajectory, scale, T_WL, T_BC)

input_bag = "/home/roxane/PaperData/RW/Dataset/Flight.bag" #/home/roxane/elios3_2706/EL300825315585_01645_214_1flight/Flight.bag"
output_bag = "/home/roxane/PaperData/RW/Dataset/Flight2.bag"

save_to_rosbag(output_bag, input_bag, transformed_trajectory)

# save_to_rosbag('/home/roxane/SVO_MR_DATA/Oerlikon/elios2.bag', 
#                '/home/roxane/SVO_MR_DATA/Oerlikon/elios1.bag', 
#                transformed_trajectory)


save_to_ply('/home/roxane/PaperData/RW/Dataset/colmap_scaled.ply', transformed_trajectory)
