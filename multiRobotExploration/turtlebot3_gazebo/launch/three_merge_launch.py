import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 獲取包的路徑
    multi_robot_pkg = get_package_share_directory('turtlebot3_gazebo')  # 第一個包名
    merge_map_pkg = get_package_share_directory('merge_map')  # 第二個包名
    
    # 定義multi_robot_launch.py的路徑
    multi_robot_launch = os.path.join(multi_robot_pkg, 'launch', 'multi_robot_launch.py')
    
    # 定義merge_map_launch.py的路徑
    merge_map_launch = os.path.join(merge_map_pkg, 'launch', 'merge_map_launch.py')
    
    # 定義膨脹參數
    dilation_size_arg = DeclareLaunchArgument(
        'dilation_size',
        default_value='3',
        description='Size of dilation kernel for obstacle inflation'
    )

    # 定義地圖大小參數
    map_size_arg = DeclareLaunchArgument(
        'map_size',
        default_value='40.0',
        description='Size of the merged map in meters'
    )

    # 定義原點偏移參數
    origin_offset_arg = DeclareLaunchArgument(
        'origin_offset',
        default_value='-20.0',
        description='Offset of the origin from the map edge'
    )
    
    # 創建啟動描述
    ld = LaunchDescription([
        # 添加參數聲明
        dilation_size_arg,
        map_size_arg,
        origin_offset_arg,
        
        # 添加multi_robot_launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(multi_robot_launch)
        ),
        
        # 添加merge_map_launch.py (帶參數)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(merge_map_launch),
            launch_arguments={
                'dilation_size': LaunchConfiguration('dilation_size'),
                'map_size': LaunchConfiguration('map_size'),
                'origin_offset': LaunchConfiguration('origin_offset')
            }.items()
        )
    ])
    
    return ld