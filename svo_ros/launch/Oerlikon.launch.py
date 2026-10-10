# Drift-free Visual SLAM using Digital Twins: SVO Pro with global position measurements
# and global poses from the registration to a city twin, on the Oerlikon dataset.
#
#   ros2 launch svo_ros Oerlikon.launch.py reference_map:=/path/to/Oerlikon.ply
#
# and play the (ROS 2) bag in another terminal, or pass bag:=/path/to/oerlikon_ros2.
#
# This will launch:
#  - svo_ros with the global pose accepted as input to the optimization problem
#  - svo_registration, which registers the local point cloud to the global city twin
#  - rviz2
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    svo_param_dir = PathJoinSubstitution([FindPackageShare('svo_ros'), 'param', 'Oerlikon'])

    args = [
        DeclareLaunchArgument(
            'reference_map',
            default_value=PathJoinSubstitution(
                [FindPackageShare('svo_registration'), 'maps', 'Oerlikon.ply']),
            description='City twin point cloud (.ply) to register to.'),
        DeclareLaunchArgument(
            'calib_file', default_value=PathJoinSubstitution([svo_param_dir, 'calibration.yaml'])),
        DeclareLaunchArgument(
            'params_file', default_value=PathJoinSubstitution([svo_param_dir, 'mono_vio.yaml'])),
        DeclareLaunchArgument(
            'gp_settings_file', default_value=PathJoinSubstitution([svo_param_dir, 'gp_settings.yaml'])),
        DeclareLaunchArgument(
            'map_reg_settings_file',
            default_value=PathJoinSubstitution([svo_param_dir, 'gpose_settings.yaml'])),
        DeclareLaunchArgument(
            'registration_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('svo_registration'), 'config', 'oerlikon.yaml'])),
        DeclareLaunchArgument('rviz', default_value='true', description='Start rviz2.'),
        DeclareLaunchArgument('registration', default_value='true',
                              description='Start the city twin registration node.'),
        DeclareLaunchArgument('bag', default_value='',
                              description='Optional ROS 2 bag to play once the nodes are up.'),
        DeclareLaunchArgument('rate', default_value='1.0', description='Bag playback rate.'),
        DeclareLaunchArgument('bag_delay', default_value='15.0',
                              description='Seconds to wait before playing the bag.'),
        DeclareLaunchArgument('glog_v', default_value='0', description='glog verbosity (--v).'),
    ]

    svo_node = Node(
        package='svo_ros',
        executable='svo_node',
        name='svo',
        output='screen',
        arguments=[['--v=', LaunchConfiguration('glog_v')]],
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'cam0_topic': '/camera_2/image_raw',
                'imu_topic': '/vio_module/imu_filtered',
                'calib_file': LaunchConfiguration('calib_file'),

                'gp_topic': '/gps',
                'use_global_measurements': True,
                'gp_settings_file': LaunchConfiguration('gp_settings_file'),

                'map_alignement_T_w_b': '/svo/w_registrated_pose',
                'use_map_registration': True,
                'map_reg_settings_file': LaunchConfiguration('map_reg_settings_file'),

                'imu_temporal_stationary_check': False,
                'imu_temporal_window_length_sec': 1.0,
                'stationary_acc_sigma_thresh': 1e-3,
                'stationary_gyr_sigma_thresh': 1e-3,
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

    registration_node = Node(
        package='svo_registration',
        executable='online_registration.py',
        name='online_registration',
        output='screen',
        arguments=['--config-file', LaunchConfiguration('registration_config'),
                   '--reference-map', LaunchConfiguration('reference_map')],
        condition=IfCondition(LaunchConfiguration('registration')),
    )

    play_bag = TimerAction(
        period=LaunchConfiguration('bag_delay'),
        actions=[ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'),
                 '--rate', LaunchConfiguration('rate')],
            output='screen',
        )],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('bag'), "' != ''"])),
    )

    return LaunchDescription(args + [svo_node, rviz_node, registration_node, play_bag])
