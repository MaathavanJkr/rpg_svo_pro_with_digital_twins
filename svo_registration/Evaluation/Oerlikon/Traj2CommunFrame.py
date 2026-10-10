#Put everything in ENU frame for the plotting. For this : 
#calculate the true local to world transform. 
#apply the transform to the trajectory.


from vikit_py.rosbag_utils import BagReader, BagWriter, stamp_to_sec, sec_to_stamp  # ROS 2 bags (ROS 1: rosbag/rospy)
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_matrix, quaternion_matrix
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

def pose_to_matrix(pose):
    """
    Convert a PoseStamped message to a 4x4 transformation matrix.
    """
    quat = (pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w)
    
    trans = (pose.pose.position.x,
             pose.pose.position.y,
             pose.pose.position.z)
    
    T = quaternion_matrix(quat)
    T[0:3, 3] = trans
    
    return T

def extract_rpy_translation(T):
    """
    Extract roll, pitch, yaw and translation from a 4x4 transformation matrix.
    """
    rpy = euler_from_matrix(T[:3, :3], axes='sxyz')
    translation = T[0:3, 3]
    
    return rpy, translation

def find_closest_pose(bag, target_time, topic):
    """
    Find the pose with the closest timestamp to the target time in the given topic.
    """
    closest_pose = None
    min_time_diff = float('inf')
    
    for topic, msg, t in bag.read_messages(topics=[topic]):
        time_diff = abs((stamp_to_sec(msg.header.stamp) - target_time))
        if time_diff < min_time_diff:
            closest_pose = msg
            min_time_diff = time_diff
            t2 = stamp_to_sec(msg.header.stamp)
    
    return closest_pose, t2

def create_transformation_matrix(yaw_deg, translation):
    yaw_rad = np.deg2rad(yaw_deg)
    
    R = np.array([
        [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
        [np.sin(yaw_rad), np.cos(yaw_rad), 0],
        [0, 0, 1]
    ])
    
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = translation
    
    return T

def transform_pose(T, msg):
    p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 1])
    p_transformed = T @ p
    
    transformed_pose = PoseStamped()
    transformed_pose.header = msg.header
    transformed_pose.pose.position.x = p_transformed[0]
    transformed_pose.pose.position.y = p_transformed[1]
    transformed_pose.pose.position.z = p_transformed[2]
    
    q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    R_pose = tf.transformations.quaternion_matrix(q)[:3, :3]
    R_transformed = T[:3, :3] @ R_pose
    q_transformed = tf.transformations.quaternion_from_matrix(np.vstack([np.hstack([R_transformed, [[0], [0], [0]]]), [0, 0, 0, 1]]))
    
    transformed_pose.pose.orientation.x = q_transformed[0]
    transformed_pose.pose.orientation.y = q_transformed[1]
    transformed_pose.pose.orientation.z = q_transformed[2]
    transformed_pose.pose.orientation.w = q_transformed[3]
    
    return transformed_pose

def transform_cov_pose(T, msg):
    p = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, 1])
    p_transformed = T @ p
    
    transformed_pose = PoseStamped()
    transformed_pose.header = msg.header
    transformed_pose.pose.position.x = p_transformed[0]
    transformed_pose.pose.position.y = p_transformed[1]
    transformed_pose.pose.position.z = p_transformed[2]
    
    q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    R_pose = tf.transformations.quaternion_matrix(q)[:3, :3]
    R_transformed = T[:3, :3] @ R_pose
    q_transformed = tf.transformations.quaternion_from_matrix(np.vstack([np.hstack([R_transformed, [[0], [0], [0]]]), [0, 0, 0, 1]]))
    
    transformed_pose.pose.orientation.x = q_transformed[0]
    transformed_pose.pose.orientation.y = q_transformed[1]
    transformed_pose.pose.orientation.z = q_transformed[2]
    transformed_pose.pose.orientation.w = q_transformed[3]
    
    return transformed_pose

def main():
    input_bag = '/home/roxane/SVO_MR_DATA/Oerlikon/SVO_GPS/res4.bag' #/res.bag' #SVO/res.bag' #/res2.bag' #Results/V2/VINS/oerlikon_vins_res.bag' #'/home/roxane/SVO_MR_DATA/Oerlikon/2024-06-09-11-08-18.bag' #SVO_VIO_GPS_v1.bag' #/home/roxane/Results/Recordings/TODO/SVO_VIO_FM.bag' #2024-06-04-20-38-51.bag' #Results/Recordings/2024-06-04-20-09-02.bag' #2024-06-04-10-24-04.bag' #2024-06-03-19-01-55.bag' #2024-06-03-18-25-09.bag' #VIO_GPS_oerlikon.bag'
    output_bag = '/home/roxane/SVO_MR_DATA/Oerlikon/SVO_GPS/res5.bag' #/res2.bag'
    bag = BagReader(input_bag)

    idx = 0
    N = 160 #we cant just start at the begining as the yaw isnt initialized yet
    M = 200

    yaw_arr = []
    translation_arr = []

    local_x = []
    local_y = []
    local_z = []
    enu_x = []
    enu_y = []
    enu_z = []
    transformed_x = []
    transformed_y = []
    transformed_z = []
    ini=True

    for topic, msg, t in bag.read_messages(topics=['/col_T_enu_b']):
       
        if idx == M:
            break
        idx += 1

        if idx > N: 
            if idx % 10 == 0:
                print(idx)
            
            T_WB = pose_to_matrix(msg)
            t1 = stamp_to_sec(msg.header.stamp)

            closest_pose, t2 = find_closest_pose(bag, t1, '/svo/vio_backend_pose_imu') #/svo/vio_backend_pose_imu_viz')
            T_LB = pose_to_matrix(closest_pose.pose)

            T_BL = np.linalg.inv(T_LB)
            T_WL = np.dot(T_WB, T_BL)
            T_LW = np.linalg.inv(T_WL)

            rpy, translation = extract_rpy_translation(T_LW)
            
        
            # if idx > 200:
            yaw_arr.append(rpy[2])
            translation_arr.append(translation)

    mean_yaw = np.mean(yaw_arr)
    mean_translation = np.mean(translation_arr, axis=0)

    print("\n-----------T_LW----------- :")
    print(f"Yaw: {np.degrees(mean_yaw)} degrees")
    print(f"Translation: {mean_translation}")

    bag.close()

    T_LW = create_transformation_matrix(np.degrees(mean_yaw), mean_translation)
    T_WL = np.linalg.inv(T_LW)
    
    
    with BagWriter(output_bag) as outbag:
        with BagReader(input_bag) as inbag:
            for topic, msg, t in inbag.read_messages():
                if(topic == '/colmap_L_T'):
                    continue
                outbag.write(topic, msg, t)

                if topic == '/svo/vio_backend_pose_imu' : #'/globalEstimator/global_odometry' : #'/svo/vio_backend_pose_imu_viz':
                    translation = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
                    local_x.append(translation[0])
                    local_y.append(translation[1])
                    local_z.append(translation[2])
                    transformed_msg = transform_cov_pose(T_WL, msg)
                    enu_x = transformed_msg.pose.position.x
                    enu_y = transformed_msg.pose.position.y
                    enu_z = transformed_msg.pose.position.z
                    outbag.write('/svo/enu_vio', transformed_msg, t)
                
                if topic == '/col_T_enu_b': 
                    transformed_msg = transform_pose(T_LW, msg)
                    transformed_msg.header.stamp = msg.header.stamp
                    outbag.write('/colmap_L_T', transformed_msg, t)
                    transformed_x.append(transformed_msg.pose.position.x)
                    transformed_y.append(transformed_msg.pose.position.y)
                    transformed_z.append(transformed_msg.pose.position.z)

    print("New bag saved to", output_bag)

    plt.figure()
    plt.plot(local_y, local_x, 'g', label='Estimated Trajectory in Local Frame')
    
    # Plot GT Trajectory with red color for indexes between 5 and M
    print("size of transformed_x: ", len(transformed_x))
    plt.plot(transformed_y[:N], transformed_x[:N], 'b--',  label='GT Trajectory in Local Frame')
    plt.plot(transformed_y[N:M], transformed_x[N:M], 'r--')
    plt.plot(transformed_y[M:], transformed_x[M:], 'b--')
    
    plt.xlabel('Z')
    plt.ylabel('X')
    plt.legend()
    plt.axis('equal')
    plt.title('Trajectory Comparison in Local Frame')
    plt.show()


    # plt.figure()
    # plt.plot(local_y, local_x, label='Estimated Trajectory in Local Frame')
    # plt.plot(transformed_y, transformed_x, label='GT Trajectory in Local Frame', linestyle='--') #need to color in red the messae for idx>5 and <N
    # plt.xlabel('Z')
    # plt.ylabel('X')
    # plt.legend()
    # plt.title('Trajectory Comparison in Local Frame')
    # plt.show()


if __name__ == "__main__":
    main()


