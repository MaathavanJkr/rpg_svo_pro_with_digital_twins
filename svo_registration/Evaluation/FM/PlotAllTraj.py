from vikit_py.rosbag_utils import BagReader, BagWriter, stamp_to_sec, sec_to_stamp  # ROS 2 bags (ROS 1: rosbag/rospy)
import matplotlib.pyplot as plt
import numpy as np



def extract_pose_data(bag_file, topic):

    bag = BagReader(bag_file)
    x_data = []
    y_data = []
    z_data = []

    if(topic == '/globalEstimator/global_odometry'):
        for topic, msg, t in bag.read_messages(topics=[topic]):
            print(stamp_to_sec(msg.header.stamp)    )
            if(stamp_to_sec(msg.header.stamp) > 284):
                break
         
            x,y,z = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z

            p = np.array([x,y,z]) #, 1])
  


            R = np.array([[0.91472302, -0.40408143, 0],
                            [0.40408143, 0.91472302, 0],
                            [0, 0, 1]])

            t = np.array([18.82606956, 10.38710951, 10.00184886])


            p = np.dot(R,p) + t

            x_data.append(p[0])
            y_data.append(p[1])
            z_data.append(p[2])

    else :
        for topic, msg, t in bag.read_messages(topics=[topic]):
            x_data.append(msg.pose.position.x)
            y_data.append(msg.pose.position.y)
            z_data.append(msg.pose.position.z)
            print(stamp_to_sec(msg.header.stamp))
    bag.close()
    return np.array(x_data), np.array(y_data), np.array(z_data)

path = '/home/roxane/SVO_MR_DATA/FM' #/home/roxane/SVO_MR_DATA/Oerlikon/' #/home/roxane/Results/Recordings/' #/home/roxane/Results/Recordings/TODO/'
rosbag_files = [path + 'SVO/res2.bag', path + 'SVO_GPS/res3.bag', path +'SVO_GPS_MR/res.bag', path + 'VINS_Fusion/res2.bag'] #/home/roxane/SVO_MR_DATA/Oerlikon/VINS_Fusion/ResFusion.bag' ] #'/home/roxane/Results/V2/VINS/oerlikon_vins_res.bag']
groundtruth_topic = '/kingfisher/agiros_pilot/groundtruth/pose'


# path = '/home/roxane/Results/Recordings/TODO/'
# rosbag_files = [path + 'SVO_VIO_FM.bag', path + 'SVO_VIO_GPS_FM.bag', path +'SVO_VIO_GPS_MR.bag',  '/home/roxane/Results/V2/VINS/FM_vins.bag']
# groundtruth_topic = '/kingfisher/agiros_pilot/groundtruth/pose'

vio_topics = ['/svo/enu_vio']*4
              #/svo/vio_backend_pose_imu_viz', '/svo/vio_backend_pose_imu_viz', '/svo/vio_backend_pose_imu_viz', '/globalEstimator/global_odometry'] 

# Extract data
vio_data = [extract_pose_data(bag_file, vio_topic) for bag_file, vio_topic in zip(rosbag_files, vio_topics)]
# vio_data.append(extract_pose_data(rosbag_files[3], vio_topics[3], is_odo=True))

groundtruth_data = extract_pose_data(rosbag_files[0], groundtruth_topic)

# Plotting
labels = ['VIO', 'VIO + GPS', 'VIO + GPS + MR', 'VINS-Fusion']

plt.figure()
for i, (x, y, z) in enumerate(vio_data):
    plt.plot(x,y, label=labels[i])
plt.plot(groundtruth_data[0], groundtruth_data[1], label='Ground Truth', linestyle='--')
plt.legend()
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.axis('equal')
plt.gca().set_aspect('equal', adjustable='box')  # Ensure the same scale for both axes
plt.legend(loc='lower center', bbox_to_anchor=(0.5, 1.02), ncol=4)  # Move legend to top

plt.figure()
for i, (x, y, z) in enumerate(vio_data):
    plt.plot(y,z, label=labels[i])
plt.plot(groundtruth_data[1], groundtruth_data[2], label='Ground Truth', linestyle='--')
# plt.legend()
plt.xlabel('Y')
plt.ylabel('Z')
plt.axis('equal')
plt.gca().set_aspect('equal', adjustable='box')  # Ensure the same scale for both axes
plt.ylim(-0.5, 12)
# plt.legend(loc='lower center', bbox_to_anchor=(0.5, 1.02), ncol=4)  # Move legend to top

plt.show()