# SVO (stereo) on Flightmare data with global position measurements (simulated time).
#
#   ros2 launch svo_ros flightmare_vio_stereo.launch.py
#
# The ROS 1 version referenced files outside this repository (a stereo fisheye
# calibration); pass your own calib_file:=... params_file:=... for stereo data.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    param_dir = PathJoinSubstitution([FindPackageShare('svo_ros'), 'param'])

    args = [
        DeclareLaunchArgument(
            'calib_file', default_value=PathJoinSubstitution([param_dir, 'Flightmare', 'calibration.yaml'])),
        DeclareLaunchArgument(
            'params_file', default_value=PathJoinSubstitution([param_dir, 'Default', 'vio_stereo.yaml'])),
        DeclareLaunchArgument(
            'gp_settings_file', default_value=PathJoinSubstitution([param_dir, 'Flightmare', 'gp_settings.yaml'])),
        DeclareLaunchArgument(
            'map_reg_settings_file', default_value=PathJoinSubstitution([param_dir, 'Flightmare', 'gpose_settings.yaml'])),
        DeclareLaunchArgument('rviz', default_value='true', description='Start rviz2.'),
    ]

    svo_node = Node(
        package='svo_ros',
        executable='svo_node',
        name='svo',
        output='screen',
        arguments=['--v=0'],
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_sim_time': True,
                'cam0_topic': '/hil/image_l',
                'cam1_topic': '/hil/image_r',
                'imu_topic': '/kingfisher/agiros_pilot/sensors/imu',
                'gp_topic': '/gps/ENU',
                'use_global_measurements': True,
                'map_alignement_T_w_b': '/map_alignement',
                'use_map_registration': False,
                'imu_temporal_stationary_check': False,
                'imu_temporal_window_length_sec': 1.0,
                'stationary_acc_sigma_thresh': 0.001,
                'stationary_gyr_sigma_thresh': 0.001,
                'calib_file': LaunchConfiguration('calib_file'),
                'gp_settings_file': LaunchConfiguration('gp_settings_file'),
                'map_reg_settings_file': LaunchConfiguration('map_reg_settings_file'),
            },
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='vis',
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('svo_ros'), 'RVIZ', 'rviz_config_digital_twins.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(args + [svo_node, rviz_node])
