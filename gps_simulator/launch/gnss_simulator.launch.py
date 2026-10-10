# GPS simulator: trilaterates simulated GNSS pseudoranges (with multipath errors) from a
# ground-truth pose topic.
#
#   ros2 launch satellite_simulator gnss_simulator.launch.py gps_noise_model_folder:=/path/to/models
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('satellite_simulator')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'config_path', default_value=PathJoinSubstitution([share, 'config', 'reciever_config.yaml'])),
        DeclareLaunchArgument(
            'constellation_file', default_value=PathJoinSubstitution([share, 'config', 'constellation.yaml'])),
        DeclareLaunchArgument(
            'gps_noise_model_folder', default_value=PathJoinSubstitution([share, 'models'])),

        Node(
            package='satellite_simulator',
            executable='gnss_simulator',
            name='gnss_simulator',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'constellation_file': LaunchConfiguration('constellation_file'),
                'config_path': LaunchConfiguration('config_path'),
            }],
        ),
        Node(
            package='satellite_simulator',
            executable='gps_error_publisher.py',
            name='publish_mp_nsat',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'config_path': LaunchConfiguration('config_path'),
                'gps_noise_model_folder': LaunchConfiguration('gps_noise_model_folder'),
            }],
        ),
    ])
