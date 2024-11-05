import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 獲取包的路徑
    multi_robot_pkg = get_package_share_directory('turtlebot3_gazebo')  # 第一個包名
    merge_map_pkg = get_package_share_directory('merge_map')  # 第二個包名
    
    # 定義multi_robot_launch.py的路徑
    multi_robot_launch = os.path.join(multi_robot_pkg, 'launch', 'multi_robot_launch.py')
    
    # 定義merge_map_launch.py的路徑
    merge_map_launch = os.path.join(merge_map_pkg, 'launch', 'merge_map_launch.py')
    
    # 創建啟動描述
    ld = LaunchDescription()
    
    # 添加multi_robot_launch.py
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(multi_robot_launch)
        )
    )
    
    # 添加merge_map_launch.py
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(merge_map_launch)
        )
    )
    
    return ld