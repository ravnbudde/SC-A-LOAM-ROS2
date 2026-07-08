from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aloam_velodyne',
            executable='kittiHelper',
            name='kittiHelper',
            output='screen',
            parameters=[{
                'dataset_folder': '/data/KITTI/odometry/',
                'sequence_number': '00',
                'to_bag': False,
                'output_bag_file': '/tmp/kitti.bag',
                'publish_delay': 1,
            }],
        ),
    ])
