from vikit_py.rosbag_utils import BagReader, BagWriter, stamp_to_sec, sec_to_stamp  # ROS 2 bags (ROS 1: rosbag/rospy)
import cv2
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image

# Load camera parameters from YAML
camera_info = {
    'image_height': 400,
    'image_width': 640,
    'intrinsics': [387.7113338951933, 387.8098232626537, 313.42251525400184, 211.99430613976855],
    # 'distortion': [-0.07870476770826547, 0.010047494513115098, -0.03470804911024709, 0.022112022196975237]
    'distortion': [-0.07321701837897686, -0.02964399348764655, 0.028207592603742136, -0.013234383835932766]
}

# Distortion coefficients
D = np.array(camera_info['distortion'])

# Image size
image_size = (camera_info['image_width'], camera_info['image_height'])

# Undistortion maps
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, image_size, cv2.CV_16SC2)

# Initialize CV Bridge
bridge = CvBridge()

def undistort_image(image_msg):
    # Convert ROS Image message to OpenCV image
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
    # Undistort image
    undistorted_image = cv2.remap(cv_image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    # Convert back to ROS Image message
    undistorted_image_msg = bridge.cv2_to_imgmsg(undistorted_image, encoding='bgr8')
    undistorted_image_msg.header = image_msg.header  # Preserve the header information
    return undistorted_image_msg

def process_bag(input_bag_path, output_bag_path):
    with BagWriter(output_bag_path) as outbag:
        with BagReader(input_bag_path) as inbag:
            ini = True
            for topic, msg, t in inbag.read_messages():
                if(ini):
                    t0 = t
                    ini = False
                # if(stamp_to_sec(t) - stamp_to_sec(t0) >275):
                #     break
                print(stamp_to_sec(t))
                if msg._type == 'sensor_msgs/Image':
                    undistorted_msg = undistort_image(msg)
                    outbag.write(topic, undistorted_msg, t)
                else:
                    outbag.write(topic, msg, t)
                # if(stamp_to_sec(t) > 100):
                #     break

if __name__ == '__main__':
    input_bag_path = '/home/roxane/PaperData/RW/Dataset/Flight4.bag' #/home/roxane/Results/Recordings/elios3_colmap_.bag' #/home/roxane/Results/V2/elios3.bag' #/home/roxane/ros_ws/src/svo_mr_utility/Elios3Utiityv2/Data/flight_data_vins_TA.bag'
    output_bag_path = '/home/roxane/PaperData/RW/Dataset/Flight_VINS.bag' #/home/roxane/vins_ws/src/VINS-Fusion/bags/Oerlikon_GPS_VINS.bag'
    process_bag(input_bag_path, output_bag_path)
