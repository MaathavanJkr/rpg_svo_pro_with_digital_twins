import numpy as np
from vikit_py.rosbag_utils import BagReader, BagWriter, stamp_to_sec, sec_to_stamp  # ROS 2 bags (ROS 1: rosbag/rospy)
from geometry_msgs.msg import PointStamped, PoseStamped
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import open3d as o3d

def plot_trajectories_3d(points1, points2):
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(points1[:, 0], points1[:, 1], points1[:, 2], label='GPS Trajectory', color='b')
    ax.plot(points2[:, 0], points2[:, 1], points2[:, 2], label='Colmap Trajectory (aligned)', color='r')

    ax.set_xlabel('E [m]')
    ax.set_ylabel('N [m]')
    ax.set_zlabel('U [m]')
    ax.legend()
    ax.set_title('3D Trajectories')

    # plt.show()

def read_trajectory(bag, topic_2):
    timestamps = []
    points = []
    orientations = []
    for topic, msg, t in bag.read_messages(topics=[topic_2]):
        if topic == '/ENU_gps':
            timestamps.append(stamp_to_sec(msg.header.stamp))
            points.append(np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]))
        elif topic == '/col_T_enu_b':
            if(stamp_to_sec(msg.header.stamp) >25100):
                break
            timestamps.append(stamp_to_sec(msg.header.stamp))
            orientations.append(np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]))
            points.append(np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]))
    return np.array(timestamps), np.array(points), np.array(orientations)

def compute_derivatives(timestamps, points):
    dt = np.diff(timestamps)
    derivatives = np.diff(points, axis=0) / dt[:, None]
    return derivatives

def correlate_trajectories(timestamps1, deriv1, timestamps2, deriv2, title):
    time_offsets = np.arange(-150, 150, 0.01)  # Adjust the range and step size as needed
    correlations = []

    for offset in time_offsets:
        interpolated_deriv2_x = np.interp(timestamps1 + offset, timestamps2, deriv2[:, 0])  # Interpolate x
        correlation_x = np.corrcoef(deriv1[:, 0], interpolated_deriv2_x)[0, 1]  # Correlation of x velocities

        interpolated_deriv2_y = np.interp(timestamps1 + offset, timestamps2, deriv2[:, 1])  # Interpolate y
        correlation_y = np.corrcoef(deriv1[:, 1], interpolated_deriv2_y)[0, 1]  # Correlation of y velocities

        # interpolated_deriv2_z = np.interp(timestamps1[:-1] + offset, timestamps2[:-1], deriv2[:, 2])  # Interpolate z
        # correlation_z = np.corrcoef(deriv1[:, 2], interpolated_deriv2_z)[0, 1]  # Correlation of z velocities

        correlation = (correlation_x + correlation_y ) / 2  # Average correlation
        correlations.append(correlation)

    # plot correlation
    plt.figure(figsize=(10, 5))
    plt.plot(time_offsets, correlations)
    plt.xlabel('Time Offset (s)')
    plt.ylabel('Correlation')
    plt.title(title)

    best_offset_index = np.argmax(correlations)
    best_offset = time_offsets[best_offset_index]
    # best_offset = best_offset - 10
    return best_offset, correlations[best_offset_index]

def plot_trajectories(timestamps1, points1, timestamps2, points2, best_offset):
    plt.figure(figsize=(10, 5))
    
    plt.subplot(2,3, 1)
    plt.plot(timestamps1, points1[:, 0], label='GPS X')
    plt.plot(timestamps2 - best_offset, points2[:, 0], label='Lidar VIO X (aligned)')
    plt.xlabel('Time (s)')
    plt.ylabel('X Coordinate')
    plt.legend()
    plt.title('X Coordinates - Post correlation')

    plt.subplot(2,3, 2)
    plt.plot(timestamps1, points1[:, 1], label='GPS Y')
    plt.plot(timestamps2 - best_offset, points2[:, 1], label='Lidar VIO Y (aligned)')
    plt.xlabel('Time (s)')
    plt.ylabel('Y Coordinate')
    plt.legend()
    plt.title('Y Coordinates- Post correlation')

    plt.subplot(2,3, 3)
    plt.plot(timestamps1, points1[:, 2], label='GPS Y')
    plt.plot(timestamps2 - best_offset, points2[:, 2], label='Lidar VIO Y (aligned)')
    plt.xlabel('Time (s)')
    plt.ylabel('Z Coordinate')
    plt.legend()
    plt.title('Z Coordinates- Post correlation')

    plt.subplot(2,3, 4)
    plt.plot(timestamps1, points1[:, 0], label='GPS X')
    plt.plot(timestamps2, points2[:, 0], label='Lidar VIO X (aligned)')
    plt.xlabel('Time (s)')
    plt.ylabel('X Coordinate')
    plt.legend()
    plt.title('X Coordinates Pre Correlation')

    plt.subplot(2,3, 5)
    plt.plot(timestamps1, points1[:, 1], label='GPS Y')
    plt.plot(timestamps2, points2[:, 1], label='Lidar VIO Y (aligned)')
    plt.xlabel('Time (s)')
    plt.ylabel('Y Coordinate')
    plt.legend()
    plt.title('Y Coordinates Pre Correlation')

    plt.subplot(2,3, 6)
    plt.plot(timestamps1, points1[:, 2], label='GPS Z')
    plt.plot(timestamps2, points2[:, 2], label='LIDAR VIO Z (aligned)')
    plt.xlabel('Time (s)')
    plt.ylabel('Z Coordinate')
    plt.legend()
    plt.title('Z Coordinates Pre Correlation')

    plt.figure(figsize=(10, 5))
    plt.plot(timestamps1, points1[:, 0], label='GPS X')
    plt.plot(timestamps2 - best_offset, points2[:, 0], label='Lidar VIO X (aligned)')
    plt.xlabel('Time (s)')
    plt.ylabel('X Coordinate')
    plt.legend()


    # plt.tight_layout()
    # plt.show()

if __name__ == "__main__":
    bag_file = "/home/roxane/PaperData/RW/Dataset/Flight3.bag" #/home/roxane/elios3_2706/EL300825315585_01645_214_1flight/Bags/Flight3.bag" #'/home/roxane/SVO_MR_DATA/Oerlikon/elios3.bag' #/home/roxane/SVO_MR_DATA/Oerlikon/VINS_Fusion/elios1.bag' #/home/roxane/SVO_MR_DATA/Oerlikon/VINS_Fusion/elios1.bag' #/home/roxane/ros_ws/src/svo_mr_utility/Elios3Utiityv2/Data/flight_data_vins.bag' #/home/roxane/ros_ws/src/svo_mr_utility/Elios3Utiityv2/Data/ENU_flight_data.bag'
    topic1 = '/ENU_gps'
    topic2 = '/col_T_enu_b' #/ENU_lidar_vio'
    topic3 = '/LLA_gps'
    topic_raw_gps = '/gps'
    output_bag_file = "/home/roxane/PaperData/RW/Dataset/Flight4.bag" #/home/roxane/elios3_2706/EL300825315585_01645_214_1flight/Bags/Flight4_v2.bag" #'/home/roxane/SVO_MR_DATA/Oerlikon/elios4.bag' # ¼/home/roxane/SVO_MR_DATA/Oerlikon/VINS_Fusion/elios2.bag' #/home/roxane/SVO_MR_DATA/Oerlikon/VINS_Fusion/elios2.bag' #/home/roxane/ros_ws/src/svo_mr_utility/Elios3Utiityv2/Data/flight_data_vins_TA__.bag'

    bag = BagReader(bag_file)
    
    timestamps1, points1, _ = read_trajectory(bag, topic1)
    timestamps2, points2, orientations2 = read_trajectory(bag, topic2)
    plot_trajectories_3d(points1, points2)

    #save traj as ply file : 
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points1)
    o3d.io.write_point_cloud("/home/roxane/elios3_2706/EL300825315585_01645_214_1flight/Bags/Flight3.ply", pcd) #"/home/roxane/ros_ws/src/svo_mr_utility/Elios3Utiityv2/Data/3d_enu/traj_Oerlikon_ENU.ply", pcd)

    best_offset, best_correlation = correlate_trajectories(timestamps1, points1, timestamps2, points2, 'Position Correlation')
    print(f"Best time offset: {best_offset} seconds")
    print(f"Correlation at best offset: {best_correlation}")
    plot_trajectories(timestamps1, points1, timestamps2, points2, best_offset)
    plot_trajectories(timestamps1, points1, timestamps2, points2, best_offset)
    # plt.show()

    #write back to bag file
    with BagWriter(output_bag_file) as outbag:

        for p, time in zip(points1, timestamps1):
            e,n,u = p
            pose_msg = PoseStamped()

            if(time + best_offset < 120): #begining to skip
                continue

            if (time > timestamps2[-1] - best_offset):
                continue

            # if (time+best_offset < 140):
            #     continue

            pose_msg.header.stamp = sec_to_stamp(time + best_offset)
            pose_msg.header.frame_id = 'world'
            pose_msg.pose.position.x = e
            pose_msg.pose.position.y = n
            pose_msg.pose.position.z = u
            pose_msg.pose.orientation.w = 1

            outbag.write('/ENU_GPS_t0', pose_msg, sec_to_stamp(time + best_offset))

        for point, orientation, time in zip(points2, orientations2, timestamps2):
            e,n,u = point
            pose_msg = PoseStamped()
            pose_msg.header.stamp = sec_to_stamp(time)
            pose_msg.header.frame_id = 'world'
            pose_msg.pose.position.x = e
            pose_msg.pose.position.y = n
            pose_msg.pose.position.z = u
            pose_msg.pose.orientation.w = orientation[3]
            pose_msg.pose.orientation.x = orientation[0]
            pose_msg.pose.orientation.y = orientation[1]
            pose_msg.pose.orientation.z = orientation[2]


            outbag.write('/ENU_Lidar_VIO_t0', pose_msg, sec_to_stamp(time))

        with BagReader(bag_file) as bag:
            for topic, msg, t in bag.read_messages():
                if(topic == '/ENU_gps' or topic == '/ENU_lidar_vio'):
                    continue

                if(topic == '/gps_t'):
                    if (stamp_to_sec(msg.header.stamp) + best_offset <= 120):
                        continue
            
                    t = sec_to_stamp(stamp_to_sec(msg.header.stamp) + best_offset)
                    if (stamp_to_sec(t) > timestamps2[-1]):
                        continue

                    msg.header.stamp = t
                    outbag.write('/gps', msg, (t)) #t + rospy.Duration(best_offset))
                    continue

                outbag.write(topic, msg, t)
                print(stamp_to_sec(t))


    bag.close()


