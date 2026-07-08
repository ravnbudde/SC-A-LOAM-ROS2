from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    save_directory = LaunchConfiguration('save_directory')
    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_topic = LaunchConfiguration('odom_topic')
    cloud_topic = LaunchConfiguration('cloud_topic')
    gps_topic = LaunchConfiguration('gps_topic')

    params = {
        'use_sim_time': use_sim_time,
        'save_directory': save_directory,
        'keyframe_meter_gap': LaunchConfiguration('keyframe_meter_gap'),
        'keyframe_deg_gap': LaunchConfiguration('keyframe_deg_gap'),
        'sc_dist_thres': LaunchConfiguration('sc_dist_thres'),
        'sc_max_radius': LaunchConfiguration('sc_max_radius'),
        'mapviz_filter_size': LaunchConfiguration('mapviz_filter_size'),
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('save_directory', default_value='/tmp/sc_pgo/'),
        DeclareLaunchArgument('odom_topic', default_value='/lonewolf/odometry/local'),
        DeclareLaunchArgument('cloud_topic', default_value='/lonewolf/fast_lio/cloud_registered_body'),
        DeclareLaunchArgument('gps_topic', default_value='/lonewolf/vectornav/gnss'),
        DeclareLaunchArgument('keyframe_meter_gap', default_value='2.0'),
        DeclareLaunchArgument('keyframe_deg_gap', default_value='10.0'),
        DeclareLaunchArgument('sc_dist_thres', default_value='0.4'),
        DeclareLaunchArgument('sc_max_radius', default_value='80.0'),
        DeclareLaunchArgument('mapviz_filter_size', default_value='0.4'),
        Node(
            package='aloam_velodyne',
            executable='alaserPGO',
            name='alaserPGO',
            output='screen',
            parameters=[params],
            remappings=[
                ('/aft_mapped_to_init', odom_topic),
                ('/velodyne_cloud_registered_local', cloud_topic),
                ('/gps/fix', gps_topic),
            ],
        ),
    ])
