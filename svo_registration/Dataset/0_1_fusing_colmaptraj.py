# import pandas as pd
# import numpy as np
# from scipy.spatial.transform import Rotation as R
# import rosbag
# from geometry_msgs.msg import PoseStamped
# import rospy

# def load_trajectory(file_path):
#     df = pd.read_csv(file_path, sep=" ", comment='#', header=None, names=["timestamp", "tx", "ty", "tz", "qx", "qy", "qz", "qw"])
#     return df

# def eliminate_interval(traj1, start_time, end_time):
#     return traj1[(traj1['timestamp'] < start_time) | (traj1['timestamp'] > end_time)].reset_index(drop=True)

# def find_scale_translation_rotation(p1_first, p1_last, p2_first, p2_last):
#     scale = np.linalg.norm(p1_last[:3] - p1_first[:3]) / np.linalg.norm(p2_last[:3] - p2_first[:3])
#     translation = p1_first[:3] - p2_first[:3] * scale

#     # Quaternion to rotation matrix
#     r1 = R.from_quat(p1_first[3:])
#     r2 = R.from_quat(p2_first[3:])
    
#     rotation = r1 * r2.inv()

#     return scale, translation, rotation

# def apply_transformation(traj, scale, translation, rotation):
#     traj[['tx', 'ty', 'tz']] = traj[['tx', 'ty', 'tz']] * scale + translation
#     rotated_quats = traj.apply(lambda row: rotation * R.from_quat([row['qx'], row['qy'], row['qz'], row['qw']]), axis=1)
#     traj[['qx', 'qy', 'qz', 'qw']] = pd.DataFrame(rotated_quats.apply(lambda r: r.as_quat()).tolist(), index=traj.index)
#     return traj

# # def apply_transformation(traj, scale, translation, rotation):
# #     traj[['tx', 'ty', 'tz']] = traj[['tx', 'ty', 'tz']] * scale + translation
# #     quat_rot = rotation.as_quat()
# #     traj[['qx', 'qy', 'qz', 'qw']] = traj.apply(lambda row: R.from_quat([row['qx'], row['qy'], row['qz'], row['qw']]).as_quat(), axis=1)
# #     return traj

# def save_to_rosbag(file_path, traj1, traj2, combined_traj):
#     bag = BagWriter(file_path)
#     try:
#         for _, row in traj1.iterrows():
#             pose = PoseStamped()
#             pose.header.stamp = sec_to_stamp(row['timestamp'])
#             pose.pose.position.x = row['tx']
#             pose.pose.position.y = row['ty']
#             pose.pose.position.z = row['tz']
#             pose.pose.orientation.x = row['qx']
#             pose.pose.orientation.y = row['qy']
#             pose.pose.orientation.z = row['qz']
#             pose.pose.orientation.w = row['qw']
#             bag.write('/trajectory1', pose, pose.header.stamp)
        
#         for _, row in traj2.iterrows():
#             pose = PoseStamped()
#             pose.header.stamp = sec_to_stamp(row['timestamp'])
#             pose.pose.position.x = row['tx']
#             pose.pose.position.y = row['ty']
#             pose.pose.position.z = row['tz']
#             pose.pose.orientation.x = row['qx']
#             pose.pose.orientation.y = row['qy']
#             pose.pose.orientation.z = row['qz']
#             pose.pose.orientation.w = row['qw']
#             bag.write('/trajectory2', pose, pose.header.stamp)
        
#         for _, row in combined_traj.iterrows():
#             pose = PoseStamped()
#             pose.header.stamp = sec_to_stamp(row['timestamp'])
#             pose.pose.position.x = row['tx']
#             pose.pose.position.y = row['ty']
#             pose.pose.position.z = row['tz']
#             pose.pose.orientation.x = row['qx']
#             pose.pose.orientation.y = row['qy']
#             pose.pose.orientation.z = row['qz']
#             pose.pose.orientation.w = row['qw']
#             bag.write('/combined_trajectory', pose, pose.header.stamp)
#     finally:
#         bag.close()
# # Load trajectories
# trajectory1 = load_trajectory("/home/roxane/PaperData/RW/colmap/colmap_cam_estimates_1.txt")
# trajectory2 = load_trajectory("/home/roxane/PaperData/RW/colmap/colmap_cam_estimates_2.txt")

# # Eliminate overlapping interval from trajectory1
# start_time = trajectory2.iloc[0]['timestamp']
# end_time = trajectory2.iloc[-1]['timestamp']
# trajectory1_cleaned = eliminate_interval(trajectory1, start_time, end_time)

# # Align trajectory2 to trajectory1
# p1_first = trajectory1[abs(trajectory1['timestamp'] - start_time)<0.05].iloc[0][['tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']].to_numpy()
# p1_last = trajectory1[abs(trajectory1['timestamp'] - end_time)<0.05].iloc[0][['tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']].to_numpy()
# p2_first = trajectory2.iloc[0][['tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']].to_numpy()
# p2_last = trajectory2.iloc[-1][['tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']].to_numpy()

# scale, translation, rotation = find_scale_translation_rotation(p1_first, p1_last, p2_first, p2_last)
# aligned_trajectory2 = apply_transformation(trajectory2, scale, translation, rotation)

# # Combine the trajectories
# combined_trajectory = pd.concat([trajectory1_cleaned, aligned_trajectory2]).sort_values(by='timestamp').reset_index(drop=True)

# # Save everything to a rosbag
# save_to_rosbag('/home/roxane/PaperData/RW/colmap/trajectories.bag', trajectory1, trajectory2, combined_trajectory)
import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R
from vikit_py.rosbag_utils import BagReader, BagWriter, stamp_to_sec, sec_to_stamp  # ROS 2 bags (ROS 1: rosbag/rospy)
from geometry_msgs.msg import PoseStamped

def load_trajectory(file_path):
    df = pd.read_csv(file_path, sep=" ", comment='#', header=None, names=["timestamp", "tx", "ty", "tz", "qx", "qy", "qz", "qw"])
    return df

def preprocess_trajectory(traj):
    traj['norm'] = np.linalg.norm(traj[['tx', 'ty', 'tz']], axis=1)
    return traj[traj['norm'] <= 10000].drop(columns=['norm']).reset_index(drop=True)

def eliminate_interval(traj1, start_time, end_time):
    return traj1[(traj1['timestamp'] < start_time) | (traj1['timestamp'] > end_time)].reset_index(drop=True)

def find_scale_translation_rotation(p1_first, p1_last, p2_first, p2_last):
    scale = (p1_last[:3] - p1_first[:3]) / (p2_last[:3] - p2_first[:3])
    translation = p1_first[:3] - p2_first[:3] * scale

    # Quaternion to rotation matrix
    r1 = R.from_quat(p1_first[3:])
    r2 = R.from_quat(p2_first[3:])
    
    rotation = r1 * r2.inv()

    return scale, translation, rotation

def apply_transformation(traj, scale, translation, rotation):
    traj[['tx', 'ty', 'tz']] = traj[['tx', 'ty', 'tz']] * scale + translation
    rotated_quats = traj.apply(lambda row: rotation * R.from_quat([row['qx'], row['qy'], row['qz'], row['qw']]), axis=1)
    traj[['qx', 'qy', 'qz', 'qw']] = pd.DataFrame(rotated_quats.apply(lambda r: r.as_quat()).tolist(), index=traj.index)
    return traj

def save_to_rosbag(file_path, traj1, traj2, combined_traj):
    bag = BagWriter(file_path)
    try:
        for _, row in traj1.iterrows():
            pose = PoseStamped()
            pose.header.stamp = sec_to_stamp(row['timestamp'])
            pose.pose.position.x = row['tx']
            pose.pose.position.y = row['ty']
            pose.pose.position.z = row['tz']
            pose.pose.orientation.x = row['qx']
            pose.pose.orientation.y = row['qy']
            pose.pose.orientation.z = row['qz']
            pose.pose.orientation.w = row['qw']
            bag.write('/trajectory1', pose, pose.header.stamp)
        
        for _, row in traj2.iterrows():
            pose = PoseStamped()
            pose.header.stamp = sec_to_stamp(row['timestamp'])
            pose.pose.position.x = row['tx']
            pose.pose.position.y = row['ty']
            pose.pose.position.z = row['tz']
            pose.pose.orientation.x = row['qx']
            pose.pose.orientation.y = row['qy']
            pose.pose.orientation.z = row['qz']
            pose.pose.orientation.w = row['qw']
            bag.write('/trajectory2', pose, pose.header.stamp)
        
        for _, row in combined_traj.iterrows():
            pose = PoseStamped()
            pose.header.stamp = sec_to_stamp(row['timestamp'])
            pose.pose.position.x = row['tx']
            pose.pose.position.y = row['ty']
            pose.pose.position.z = row['tz']
            pose.pose.orientation.x = row['qx']
            pose.pose.orientation.y = row['qy']
            pose.pose.orientation.z = row['qz']
            pose.pose.orientation.w = row['qw']
            bag.write('/combined_trajectory', pose, pose.header.stamp)
    finally:
        bag.close()

# Load trajectories
trajectory1 = load_trajectory("/home/roxane/PaperData/RW/colmap/colmap_cam_estimates_1.txt")
trajectory2 = load_trajectory("/home/roxane/PaperData/Colmap4Giocanni/Trajectory2_Local/output/0/colmap_cam_estimates.txt")

# Preprocess trajectory2 to remove rows where the norm of the position is greater than 10000
# trajectory2 = preprocess_trajectory(trajectory2)

# Eliminate overlapping interval from trajectory1
start_time = trajectory2.iloc[0]['timestamp']
end_time = trajectory2.iloc[-1]['timestamp']
trajectory1_cleaned = eliminate_interval(trajectory1, start_time, end_time)

# Align trajectory2 to trajectory1
p1_first = trajectory1[abs(trajectory1['timestamp'] - start_time)<0.05].iloc[0][['tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']].to_numpy()
p1_last = trajectory1[abs(trajectory1['timestamp'] - end_time)<0.05].iloc[0][['tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']].to_numpy()
p2_first = trajectory2.iloc[0][['tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']].to_numpy()
p2_last = trajectory2.iloc[-1][['tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']].to_numpy()

scale, translation, rotation = find_scale_translation_rotation(p1_first, p1_last, p2_first, p2_last)
aligned_trajectory2 = apply_transformation(trajectory2, scale, translation, rotation)

# Combine the trajectories
combined_trajectory = pd.concat([trajectory1_cleaned, aligned_trajectory2]).sort_values(by='timestamp').reset_index(drop=True)

# Save everything to a rosbag
save_to_rosbag('/home/roxane/PaperData/RW/colmap/trajectories.bag', trajectory1, trajectory2, combined_trajectory)
