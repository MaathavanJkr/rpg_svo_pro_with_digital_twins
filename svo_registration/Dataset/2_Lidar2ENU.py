import open3d as o3d
import numpy as np
import laspy
from vikit_py.rosbag_utils import BagReader, BagWriter, stamp_to_sec, sec_to_stamp  # ROS 2 bags (ROS 1: rosbag/rospy)
from geometry_msgs.msg import PoseStamped

def filter_point_cloud(pcd, x_min=-1000, y_min= -10000, z_min=-1500, xmax=1000, ymax=1050, zmax=1500):
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])
    points = np.asarray(pcd.points)
    mask = (points[:, 0] >= x_min) & (points[:, 1] >= y_min) & (points[:, 0] <= xmax) & (points[:, 1] <= ymax) & (points[:, 2] >= z_min) & (points[:, 2] <= zmax)
    pcd.points = o3d.utility.Vector3dVector(points[mask])

    # o3d.visualization.draw_geometries([pcd, axis], window_name="Lidar Map used for regittration", point_show_normal=False)
    return pcd


def create_transformation_matrix(translation, roll, pitch, yaw):
    rotation_x_radians = np.radians(roll)
    rotation_y_radians = np.radians(pitch)
    rotation_z_radians = np.radians(yaw)

    rotation_matrix = o3d.geometry.get_rotation_matrix_from_xyz((rotation_x_radians, rotation_y_radians, rotation_z_radians))
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    transformation_matrix[:3, 3] = translation
    return transformation_matrix


def execute_icp(source, target, initial_transformation):
    icp_result = o3d.pipelines.registration.registration_icp(
        source, target, 3, initial_transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    
    return icp_result.transformation


def read_rosbag_trajectory(bag_file_path, topic_name):
    bag = BagReader(bag_file_path)
    points = []

    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        position = msg.pose.position
        points.append([position.x, position.y, position.z])

    bag.close()
    return np.array(points)


def transform_pose_stamped(pose, transformation_matrix):
    point = np.array([pose.position.x, pose.position.y, pose.position.z, 1.0])
    transformed_point = transformation_matrix @ point
    pose.position.x, pose.position.y, pose.position.z = transformed_point[:3]
    return pose

def create_outbag(input_bag_path, output_bag_path, topic_name, transformation_matrix):
    in_bag = BagReader(input_bag_path)
    out_bag = BagWriter(output_bag_path)
    
    for topic, msg, t in in_bag.read_messages():
        if topic != topic_name:
            out_bag.write(topic, msg, t)
            continue

        transformed_pose = transform_pose_stamped(msg.pose, transformation_matrix)
        transformed_msg = PoseStamped()
        transformed_msg.header = msg.header
        transformed_msg.pose = transformed_pose
        out_bag.write(topic_name + "_enu", transformed_msg, t)
        # print("t : ", stamp_to_sec(t))
    
    in_bag.close()
    out_bag.close()

# Transformation parameters from Blender
translation = np.array([-20,10,-470]) #-10,60,0])
roll = 0 #- 20
pitch = 0
yaw = 10 +90 # -90 
transformation_matrix = create_transformation_matrix(translation, roll, pitch, yaw)
map_lidar_path = "/home/roxane/ros_ws/src/svo_mr_utility/Elios3Utiityv2/Data/3d_raw/LiDAR.ply" #/home/roxane/ros_ws/src/svo_mr_utility/maps/Oerlikon/LiDAR.ply"
map_google_path = "/home/roxane/ros_ws/src/svo_mr_utility/Elios3Utiityv2/Data/3d_enu/Oerlikon_ENU.ply"
rosbag_path = "/home/roxane/ros_ws/src/svo_mr_utility/Elios3Utiityv2/Data/flight_data.bag"
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0, 0, 0])

map_lidar = o3d.io.read_point_cloud(map_lidar_path) #read_las_file(map_lidar_path)
map_google = o3d.io.read_point_cloud(map_google_path)
# map_lidar, map_google = visualize(map_lidar, map_google, transformation_matrix)

# map_lidar.transform(transformation_matrix)
# o3d.visualization.draw_geometries([map_lidar, map_google, axis], window_name="INITIAL")


trajectory_points = read_rosbag_trajectory(rosbag_path, "/gps_lidar")
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])
trajectory_pcd = o3d.geometry.PointCloud()
trajectory_pcd.points = o3d.utility.Vector3dVector(trajectory_points)
trajectory_pcd.paint_uniform_color([0, 0, 1])  # Blue
o3d.visualization.draw_geometries([map_lidar, trajectory_pcd, axis], window_name="LiDAR Map and Trajectory")


map_lidar = filter_point_cloud(map_lidar)

T_enu_r = execute_icp(map_lidar, map_google, transformation_matrix)

map_lidar.transform(T_enu_r)
trajectory_pcd.transform(T_enu_r)
map_lidar.paint_uniform_color([1, 0, 0])  # Red 
map_google.paint_uniform_color([0, 1, 0])  # Green 
trajectory_pcd.paint_uniform_color([0, 0, 1])  # Blue
o3d.visualization.draw_geometries([map_lidar, map_google, trajectory_pcd, axis], window_name="Google Map adjusted to LiDAR")

print("Final transform from raw lidar to ENU: ", T_enu_r)


with open("/home/roxane/ros_ws/src/svo_mr_utility/Elios3Utiityv2/Data/LinearizationPt.txt", "a") as f:
    f.write("\n T_enu_R \n")
    f.write("\n " + str(T_enu_r) + "\n")


create_outbag(rosbag_path, "/home/roxane/SVO_MR_DATA/Oerlikon/elios1.bag", "/gps_lidar", T_enu_r)


# NOW read trajectory fromr rosbag and transform it to ENU frame. 

