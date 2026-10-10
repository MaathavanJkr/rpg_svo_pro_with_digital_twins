

import open3d as o3d
import numpy as np
import gpxpy
import pyproj
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from vikit_py.rosbag_utils import BagReader, BagWriter, stamp_to_sec, sec_to_stamp  # ROS 2 bags (ROS 1: rosbag/rospy)
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import NavSatFix
from datetime import datetime
from tf.transformations import quaternion_from_matrix, quaternion_matrix

class GeoTransform:
    def __init__(self, ref_lat, ref_lon, ref_alt=0):
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self.ref_alt = ref_alt
        self.ref_x, self.ref_y, self.ref_z = self.geodetic_to_ecef(ref_lat, ref_lon, ref_alt)
    
    def geodetic_to_ecef(self, lat, lon, alt):
        wgs84 = pyproj.CRS("EPSG:4326")
        ecef = pyproj.CRS("EPSG:4978")
        transformer = pyproj.Transformer.from_crs(wgs84, ecef, always_xy=True)
        x, y, z = transformer.transform(lon, lat, alt)
        return x, y, z

    def ecef_to_enu(self, x, y, z):
        dx = x - self.ref_x
        dy = y - self.ref_y
        dz = z - self.ref_z

        lat = np.radians(self.ref_lat)
        lon = np.radians(self.ref_lon)

        t = np.array([
            [-np.sin(lon), np.cos(lon), 0],
            [-np.sin(lat)*np.cos(lon), -np.sin(lat)*np.sin(lon), np.cos(lat)],
            [np.cos(lat)*np.cos(lon), np.cos(lat)*np.sin(lon), np.sin(lat)]
        ])

        enu = np.dot(t, np.array([dx, dy, dz]))
        return enu[0], enu[1], enu[2]
    
    def enu_to_ecef(self, e, n, u):
        lat = np.radians(self.ref_lat)
        lon = np.radians(self.ref_lon)

        t_inv = np.array([
            [-np.sin(lon), -np.sin(lat) * np.cos(lon), np.cos(lat) * np.cos(lon)],
            [np.cos(lon), -np.sin(lat) * np.sin(lon), np.cos(lat) * np.sin(lon)],
            [0, np.cos(lat), np.sin(lat)]
        ])
        
        dxyz = np.dot(t_inv, np.array([e, n, u]))
        x = dxyz[0] + self.ref_x
        y = dxyz[1] + self.ref_y
        z = dxyz[2] + self.ref_z
        return x, y, z
    
    def ecef_to_geodetic(self, x, y, z):
        ecef = pyproj.CRS("EPSG:4978")
        wgs84 = pyproj.CRS("EPSG:4326")
        transformer = pyproj.Transformer.from_crs(ecef, wgs84, always_xy=True)
        lon, lat, alt = transformer.transform(x, y, z)
        return lat, lon, alt

    def enu2geo(self, e, n, u):
        x, y, z = self.enu_to_ecef(e, n, u)
        return self.ecef_to_geodetic(x, y, z)

    def geo2enu(self, lat, lon, alt):
        x, y, z = self.geodetic_to_ecef(lat, lon, alt)
        return self.ecef_to_enu(x, y, z)
    
def transform_pose(pose, T, header):
    position = np.array([pose.position.x, pose.position.y, pose.position.z, 1.0])
    orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    
    transformed_position = np.dot(T, position)
    
    orientation_matrix = quaternion_matrix(orientation)
    transformed_orientation_matrix = np.dot(T, orientation_matrix)
    transformed_orientation = quaternion_from_matrix(transformed_orientation_matrix)
    
    transformed_pose = PoseStamped()
    transformed_pose.header = header
    transformed_pose.pose.position.x = transformed_position[0]
    transformed_pose.pose.position.y = transformed_position[1]
    transformed_pose.pose.position.z = transformed_position[2]
    transformed_pose.pose.orientation.x = transformed_orientation[0]
    transformed_pose.pose.orientation.y = transformed_orientation[1]
    transformed_pose.pose.orientation.z = transformed_orientation[2]
    transformed_pose.pose.orientation.w = transformed_orientation[3]
    
    return transformed_pose

# T_enu_raw =  np.asarray([[-1.96824114e-01,-9.80430630e-01, 4.00603074e-03 ,-2.42455401e+01],
#  [ 9.80382941e-01, -1.96854875e-01 ,-9.87154031e-03, 8.12997421e+00],
#  [ 1.04669672e-02, 1.98448702e-03, 9.99943251e-01 ,-2.08014924e-01],
#  [ 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

# lat = 47.413570225 lon = 8.551473380000001


lin_pt = np.array([47.413570225, 8.551473380000001, 390+30-28, 1.0]) #425-80+470, 1.0]) #-80 discussed with giovanni.


# Linearization point: 
# Latitude: 47.413570225
# Longitude: 8.551473380000001


in_bag_file = "/home/roxane/PaperData/RW/Dataset/Flight2.bag" #/home/roxane/elios3_2706/EL300825315585_01645_214_1flight/Bags/Flight2.bag" #"/home/roxane/SVO_MR_DATA/Oerlikon/elios2.bag" #/home/roxane/SVO_MR_DATA/Oerlikon/elios3.bag" #/home/roxane/ros_ws/src/svo_mr_utility/Elios3Utiityv2/Data/flight_data.bag"
out_bag_file = "/home/roxane/PaperData/RW/Dataset/Flight3.bag" #/home/roxane/elios3_2706/EL300825315585_01645_214_1flight/Bags/Flight3.bag" #"/home/roxane/SVO_MR_DATA/Oerlikon/elios3.bag"
# gpx_file = "/home/roxane/svo_gps_ws/output.gpx" #/home/roxane/elios3_2706/EL300825315585_01645_214_1flight/gps.gpx" #home/roxane/ros_ws/src/svo_mr_utility/Elios3Utiityv2/Data/gps.gpx"
gpx_file = "/home/roxane/elios3_2706/EL300825315585_01645_214_1flight/gps.gpx" #home/roxane/ros_ws/src/svo_mr_utility/Elios3Utiityv2/Data/gps.gpx"
lidar_vio_point = []
lidar_vio_orientation = []
lidar_stamp = []
ini = True
transformed_msg_list = []
stamp_list = []

geo_transform = GeoTransform(lin_pt[0], lin_pt[1], lin_pt[2])

gpx_data = []
with open(gpx_file, 'r') as gpx_file:
    gpx = gpxpy.parse(gpx_file)
    for track in gpx.tracks:
        for segment in track.segments:
            for point in segment.points:
                gpx_data.append({
                    'time': point.time,
                    'latitude': point.latitude,
                    'longitude': point.longitude,
                    'elevation': point.elevation
                })

# rosbag2 cannot append to a bag: keep the writer open for both passes.
outbag = BagWriter(out_bag_file)
if True:
    with BagReader(in_bag_file) as inbag:
        for topic, msg, t in inbag.read_messages():
            # if(ini):
            #     t0_lidar = stamp_to_sec(t)
            #     ini = False
            #     print('t0 removed : ', t0_lidar)
            # if topic == '/gps_lidar':
            #     header = Header(stamp=sec_to_stamp(stamp_to_sec(t)-t0_lidar), frame_id='world')
            #     T_raw = np.eye(4)
            #     T_raw[:3, 3] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            #     T_raw[:3, :3] = quaternion_matrix([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[:3, :3]
            #     T_enu = T_enu_raw @ T_raw
            #     m = PoseStamped()
            #     m.header = header
            #     m.pose.position.x = T_enu[0, 3]
            #     m.pose.position.y = T_enu[1, 3]
            #     m.pose.position.z = T_enu[2, 3]
            #     q = quaternion_from_matrix(T_enu)
            #     m.pose.orientation.x = q[0]
            #     m.pose.orientation.y = q[1]
            #     m.pose.orientation.z = q[2]
            #     m.pose.orientation.w = q[3]

            #     outbag.write('/ENU_lidar_vio', m, sec_to_stamp(stamp_to_sec(t)-t0_lidar))
            #     transformed_msg_list.append(m)
            #     stamp_list.append(stamp_to_sec(t) - t0_lidar)
            # else:
            if topic in ['/camera_2/image_raw', '/vio_module/imu_filtered', '/col_T_enu_b'] : #, '/gps_lidar_enu', '/gps_lidar']:
                msg.header = Header(stamp=sec_to_stamp(stamp_to_sec(t)), frame_id='world')
                outbag.write(topic, msg, sec_to_stamp(stamp_to_sec(t)))

if True:  # (ROS 1: re-opened the bag in append mode)
    t0 = gpx_data[0]['time'].timestamp()
    for data in gpx_data:
        enu_coords = geo_transform.geo2enu(data['latitude'], data['longitude'], data['elevation'])
        
        gps_pose_msg = PoseStamped()
        gps_pose_msg.header = Header(stamp=sec_to_stamp(data['time'].timestamp()-t0), frame_id='world')
        gps_pose_msg.pose.position.x = enu_coords[0] - 26
        gps_pose_msg.pose.position.y = enu_coords[1]
        gps_pose_msg.pose.position.z = enu_coords[2]
        gps_pose_msg.pose.orientation.w = 1.0


        geo_coords = geo_transform.enu2geo(gps_pose_msg.pose.position.x, gps_pose_msg.pose.position.y, gps_pose_msg.pose.position.z)
        navsat_msg = NavSatFix()
        navsat_msg.header = gps_pose_msg.header
        navsat_msg.latitude = geo_coords[0] #data['latitude']
        navsat_msg.longitude = geo_coords[1] #data['longitude']
        navsat_msg.altitude = geo_coords[2] #data['elevation']

        navsat_msg.position_covariance = [
            400.0, 0.0, 0.0,
            0.0, 400.0, 0.0,
            0.0, 0.0, 400.0
        ]

        navsat_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                
        outbag.write('/ENU_gps', gps_pose_msg, sec_to_stamp(data['time'].timestamp()-t0+129)) #approximate guess of the time offset, will be corrected next =script.
        outbag.write('/gps_t', navsat_msg, sec_to_stamp(data['time'].timestamp()-t0+129))

outbag.close()
print("saved to ", out_bag_file)
