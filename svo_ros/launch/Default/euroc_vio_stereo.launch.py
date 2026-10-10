# SVO (stereo) with ceres backend, EuRoC-style topics.
#
#   ros2 launch svo_ros euroc_vio_stereo.launch.py
#
# Note: the ROS 1 version set runlc:=true, but loop closing is not supported in this
# version of SVO (svo_interface.cpp aborts when runlc is set), so it is disabled here.
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
            'calib_file', default_value=PathJoinSubstitution([param_dir, 'Default', 'euroc_stereo.yaml'])),
        DeclareLaunchArgument(
            'params_file', default_value=PathJoinSubstitution([param_dir, 'Default', 'vio_stereo.yaml'])),
        DeclareLaunchArgument(
            'gp_settings_file', default_value=PathJoinSubstitution([param_dir, 'Default', 'gp_settings.yaml'])),
        DeclareLaunchArgument(
            'map_reg_settings_file', default_value=PathJoinSubstitution([param_dir, 'Default', 'gpose_settings.yaml'])),
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
                'cam0_topic': '/cam0/image_raw',
                'cam1_topic': '/cam1/image_raw',
                'imu_topic': '/imu0',
                'runlc': False,
                'imu_temporal_stationary_check': False,
                'imu_temporal_window_length_sec': 0.5,
                'stationary_acc_sigma_thresh': 0.001,
                'stationary_gyr_sigma_thresh': 5e-05,
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
