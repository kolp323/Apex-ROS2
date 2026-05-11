from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/bev/obstacles'),
                ('scan', '/scan')
            ],
            parameters=[{
                'target_frame': 'base_link',       # 要与点云的 frame_id 对应
                'transform_tolerance': 0.01,
                'min_height': -0.2,
                'max_height': 0.5,
                'angle_min': -3.1415,
                'angle_max': 3.1415,
                'angle_increment': 0.0087,
                'scan_time': 0.1,
                'range_min': 0.25,
                'range_max': 1.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }]
        )
    ])
