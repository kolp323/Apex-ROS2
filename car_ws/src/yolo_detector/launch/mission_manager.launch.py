import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument       # <--- 1. 导入 DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration   # <--- 2. 导入 LaunchConfiguration

def generate_launch_description():
    pkg_dir = get_package_share_directory('yolo_detector') # 假设在同一个包
    
    params_file = os.path.join(pkg_dir, 'config', 'mission_manager_params.yaml')

    enable_debug_arg = DeclareLaunchArgument(
        'enable_debug',  #<-- 参数的名称
        default_value='false', #<-- 默认值
        description='Enable debug' #<-- 描述信息
    )

    mission_manager_node = Node(
        package='yolo_detector', # 假设在同一个包
        executable='mission_manager',    # 新的 Python 脚本
        name='mission_manager',
        output='screen',
        parameters=[
            params_file,
            {'enable_debug': LaunchConfiguration('enable_debug')}  
        ]
    )

    return LaunchDescription([
        enable_debug_arg,
        mission_manager_node
    ])