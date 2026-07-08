from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz = LaunchConfiguration('rviz')
    save_directory = LaunchConfiguration('save_directory')
    params = {'scan_line': 16, 'mapping_skip_frame': 1, 'minimum_range': 0.1, 'mapping_line_resolution': 0.2, 'mapping_plane_resolution': 0.4, 'mapviz_filter_size': 0.1, 'keyframe_meter_gap': 1.0, 'keyframe_deg_gap': 10.0, 'sc_dist_thres': 0.2, 'sc_max_radius': 40.0, 'lidar_type': 'VLP16', 'save_directory': save_directory}

    nodes = [
        Node(package='aloam_velodyne', executable='ascanRegistration', name='ascanRegistration', output='screen', parameters=[params]),
        Node(package='aloam_velodyne', executable='alaserOdometry', name='alaserOdometry', output='screen', parameters=[params]),
        Node(package='aloam_velodyne', executable='alaserMapping', name='alaserMapping', output='screen', parameters=[params]),
        Node(package='aloam_velodyne', executable='alaserPGO', name='alaserPGO', output='screen', parameters=[params]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            prefix='nice',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('aloam_velodyne'), 'rviz_cfg', 'aloam_velodyne.rviz'])],
            condition=IfCondition(rviz),
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('save_directory', default_value='/tmp/aloam_velodyne/'),
        *nodes,
    ])
