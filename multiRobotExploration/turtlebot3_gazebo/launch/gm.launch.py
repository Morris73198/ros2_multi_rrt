import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    model_folder = 'turtlebot3_burger'
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    robot_desc_path = os.path.join(get_package_share_directory("turtlebot3_gazebo"), "urdf", "turtlebot3_burger.urdf")
    world = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'worlds','turtlebot3_house.world')
    
    # 為3個機器人創建SDF文件路徑
    urdf_path1 = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'models',model_folder+'_0','model.sdf')
    urdf_path2 = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'models',model_folder+'_1','model.sdf')
    urdf_path3 = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'models',model_folder+'_2','model.sdf')

    with open(robot_desc_path, 'r') as infp:
        robot_desc = infp.read()

    name1 = "tb3_0"
    name2 = "tb3_1" 
    name3 = "tb3_2"

    # 機器人1的節點
    spawn_robot1 = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py', 
        arguments=[
            '-entity', name1, 
            '-file', urdf_path1, 
            '-x', '5.0', 
            '-y', '2.5', 
            '-z', '0.01',
            '-robot_namespace', name1,
        ],
        output='screen'
    )

    robot_state_publisher1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=name1,
        output='screen',
        parameters=[{'frame_prefix': name1 + '/',
                    'use_sim_time': True,
                    'robot_description': robot_desc}]
    )

    slam_gmapping1 = Node(
        package='slam_gmapping',
        executable='slam_gmapping',
        name='slam_gmapping',
        namespace=name1,
        parameters=[{
            'use_sim_time': True,
            'base_frame': name1 + '/base_footprint',
            'odom_frame': name1 + '/odom',
            'map_frame': name1 + '/map',
            'xmin': -10.0,
            'ymin': -10.0,
            'xmax': 10.0,
            'ymax': 10.0,
            'delta': 0.05,  # 解析度
            'map_update_interval': 2.0,
            'maxUrange': 8.0,
            'sigma': 0.05,
            'kernelSize': 1,
            'lstep': 0.05,
            'astep': 0.05,
            'iterations': 5,
            'lsigma': 0.075,
            'ogain': 3.0,
            'minimumScore': 50
        }],
        remappings=[
            ('/scan', 'scan'),
            ('/map', 'map'),
            ('/map_metadata', 'map_metadata'),
        ]
    )

    rviz1 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'config',name1+'.rviz')]
    )

    # 機器人2的節點
    spawn_robot2 = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py', 
        arguments=[
            '-entity', name2, 
            '-file', urdf_path2, 
            '-x', '-4.6', 
            '-y', '3.0', 
            '-z', '0.01',
            '-robot_namespace', name2,
        ],
        output='screen'
    )

    robot_state_publisher2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=name2,
        output='screen',
        parameters=[{'frame_prefix': name2 + '/',
                    'use_sim_time': True,
                    'robot_description': robot_desc}]
    )

    slam_gmapping2 = Node(
        package='slam_gmapping',
        executable='slam_gmapping',
        name='slam_gmapping',
        namespace=name2,
        parameters=[{
            'use_sim_time': True,
            'base_frame': name2 + '/base_footprint',
            'odom_frame': name2 + '/odom',
            'map_frame': name2 + '/map',
            'xmin': -10.0,
            'ymin': -10.0,
            'xmax': 10.0,
            'ymax': 10.0,
            'delta': 0.05,
            'map_update_interval': 2.0,
            'maxUrange': 8.0,
            'sigma': 0.05,
            'kernelSize': 1,
            'lstep': 0.05,
            'astep': 0.05,
            'iterations': 5,
            'lsigma': 0.075,
            'ogain': 3.0,
            'minimumScore': 50
        }],
        remappings=[
            ('/scan', 'scan'),
            ('/map', 'map'),
            ('/map_metadata', 'map_metadata'),
        ]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'config', name2+'.rviz')]
    )

    # 機器人3的節點
    spawn_robot3 = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py', 
        arguments=[
            '-entity', name3, 
            '-file', urdf_path3, 
            '-x', '0.0', 
            '-y', '3.0', 
            '-z', '0.01',
            '-robot_namespace', name3,
        ],
        output='screen'
    )

    robot_state_publisher3 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=name3,
        output='screen',
        parameters=[{'frame_prefix': name3 + '/',
                    'use_sim_time': True,
                    'robot_description': robot_desc}]
    )

    slam_gmapping3 = Node(
        package='slam_gmapping',
        executable='slam_gmapping',
        name='slam_gmapping',
        namespace=name3,
        parameters=[{
            'use_sim_time': True,
            'base_frame': name3 + '/base_footprint',
            'odom_frame': name3 + '/odom',
            'map_frame': name3 + '/map',
            'xmin': -10.0,
            'ymin': -10.0,
            'xmax': 10.0,
            'ymax': 10.0,
            'delta': 0.05,
            'map_update_interval': 2.0,
            'maxUrange': 8.0,
            'sigma': 0.05,
            'kernelSize': 1,
            'lstep': 0.05,
            'astep': 0.05,
            'iterations': 5,
            'lsigma': 0.075,
            'ogain': 3.0,
            'minimumScore': 50
        }],
        remappings=[
            ('/scan', 'scan'),
            ('/map', 'map'),
            ('/map_metadata', 'map_metadata'),
        ]
    )

    rviz3 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'config', name3+'.rviz')]
    )
    
    # Gazebo服務器和客戶端
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world,'verbose':"true",'extra_gazebo_args': 'verbose'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={'verbose':"true"}.items()
    )    

    # 路徑跟隨節點
    pathFollow1 = Node(
        package='path_follow',
        executable='path_follow',
        name='pathFollow',
        output='screen',
        namespace=name1,
        parameters=[{
            'use_sim_time': True,
        }],
        remappings=[
            ("/path", "/"+name1+"/path"),
            ("/cmd_vel", "/"+name1+"/cmd_vel"),
            ("/odom", "/"+name1+"/odom"),
            ("/visual_path", "/"+name2+"/visual_path"),
        ]
    )

    pathFollow2 = Node(
        package='path_follow',
        executable='path_follow',
        name='pathFollow',
        output='screen',
        namespace=name2,
        parameters=[{
            'use_sim_time': True,
        }],
        remappings=[
            ("/path", "/"+name2+"/path"),
            ("/cmd_vel", "/"+name2+"/cmd_vel"),
            ("/odom", "/"+name2+"/odom"),
            ("/visual_path", "/"+name1+"/visual_path"),
        ]
    )

    pathFollow3 = Node(
        package='path_follow',
        executable='path_follow',
        name='pathFollow',
        output='screen',
        namespace=name3,
        parameters=[{
            'use_sim_time': True,
        }],
        remappings=[
            ("/path", "/"+name3+"/path"),
            ("/cmd_vel", "/"+name3+"/cmd_vel"),
            ("/odom", "/"+name3+"/odom"),
            ("/visual_path", "/"+name1+"/visual_path"),
        ]
    )

    # 地圖合併節點
    map_merge = Node(
        package='multirobot_map_merge',
        executable='map_merge',
        name='map_merge',
        parameters=[{
            'robot_map_topic': 'map',
            'robot_namespace': ['tb3_0', 'tb3_1', 'tb3_2'],
            'merged_map_topic': 'merged_map',
            'known_init_poses': True,
            'merging_rate': 4.0,
            'discovery_rate': 0.05,
        }]
    )

    # 創建啟動描述
    ld = LaunchDescription()
    
    # 添加Gazebo節點
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    
    # 添加路徑跟隨節點
    ld.add_action(pathFollow1)
    ld.add_action(pathFollow2)
    ld.add_action(pathFollow3)
    
    # 添加地圖合併節點
    ld.add_action(map_merge)
    
    # 添加機器人1的節點
    ld.add_action(spawn_robot1)
    ld.add_action(robot_state_publisher1)
    ld.add_action(slam_gmapping1)
    ld.add_action(rviz1)
    
    # 添加機器人2的節點 
    ld.add_action(spawn_robot2)
    ld.add_action(robot_state_publisher2)
    ld.add_action(slam_gmapping2)
    ld.add_action(rviz2)
    
    # 添加機器人3的節點
    ld.add_action(spawn_robot3)
    ld.add_action(robot_state_publisher3)
    ld.add_action(slam_gmapping3)
    ld.add_action(rviz3)
    
    return ld