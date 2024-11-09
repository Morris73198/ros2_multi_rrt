#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from tf2_ros import Buffer, TransformListener
import tf2_ros
from rclpy.time import Time
from scipy import ndimage
from std_msgs.msg import ColorRGBA

class MergeMapNode(Node):
    def __init__(self):
        super().__init__('merge_map_node')
        
        # 聲明參數
        self.declare_parameter('map_size', 40.0)
        self.declare_parameter('origin_offset', -20.0)
        self.declare_parameter('dilation_size', 3)
        
        # 獲取參數
        self.map_size = self.get_parameter('map_size').value
        self.origin_offset = self.get_parameter('origin_offset').value
        self.dilation_size = self.get_parameter('dilation_size').value
        
        # 創建合併地圖發佈器
        self.map_publisher = self.create_publisher(
            OccupancyGrid, 
            '/merge_map', 
            10
        )
        
        # 創建機器人marker發佈器
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/robot_markers',
            10
        )
        
        # 創建各個機器人地圖訂閱器
        self.subscription1 = self.create_subscription(
            OccupancyGrid,
            '/tb3_0/map',
            self.map1_callback,
            10
        )
        self.subscription2 = self.create_subscription(
            OccupancyGrid,
            '/tb3_1/map',
            self.map2_callback,
            10
        )
        self.subscription3 = self.create_subscription(
            OccupancyGrid,
            '/tb3_2/map',
            self.map3_callback,
            10
        )
        
        # 初始化TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 初始化地圖數據
        self.map1 = None
        self.map2 = None
        self.map3 = None
        
        # 設置機器人顏色
        self.robot_colors = {
            0: ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # 紅色
            1: ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),  # 綠色
            2: ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)   # 藍色
        }
        
        # 創建定時器進行定期地圖合併和marker更新
        self.timer = self.create_timer(1.0, self.merge_and_publish)
        
        self.get_logger().info('Map merge node initialized with marker visualization')

    def create_robot_marker(self, robot_id, position):
        """
        創建機器人的marker
        """
        marker = Marker()
        marker.header.frame_id = "merge_map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"robot_{robot_id}"
        marker.id = robot_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # 設置位置
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = 0.0
        
        # 設置方向 (正向朝上)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # 設置大小
        marker.scale.x = 0.3  # 直徑
        marker.scale.y = 0.3  # 直徑
        marker.scale.z = 0.1  # 高度
        
        # 設置顏色
        marker.color = self.robot_colors[robot_id]
        
        # 設置存活時間 (2秒)
        marker.lifetime = rclpy.duration.Duration(seconds=2.0).to_msg()
        
        return marker

    def map1_callback(self, msg):
        self.map1 = msg
        
    def map2_callback(self, msg):
        self.map2 = msg
        
    def map3_callback(self, msg):
        self.map3 = msg
    
    def get_robot_position(self, robot_name):
        """
        從TF獲取機器人位置
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                f'{robot_name}/map',
                f'{robot_name}/base_footprint',
                Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            
            pos = Point()
            pos.x = transform.transform.translation.x
            pos.y = transform.transform.translation.y
            pos.z = transform.transform.translation.z
            
            self.get_logger().debug(f'{robot_name} position: x={pos.x:.2f}, y={pos.y:.2f}')
            
            return pos
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warning(f'Failed to get transform for {robot_name}: {str(e)}')
            return None

    def merge_maps(self, *maps):
        """
        合併多個地圖並進行膨脹處理
        """
        if not maps:
            return None
    
        merged_map = OccupancyGrid()
        merged_map.header = maps[0].header
        merged_map.header.frame_id = 'merge_map'
    
        # 使用最小的分辨率
        merged_map.info.resolution = min(map.info.resolution for map in maps)
        merged_map.info.origin.position.x = self.origin_offset
        merged_map.info.origin.position.y = self.origin_offset
        merged_map.info.width = int(self.map_size / merged_map.info.resolution)
        merged_map.info.height = int(self.map_size / merged_map.info.resolution)
    
        # 初始化為未知區域
        merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)
    
        # 合併地圖
        for map_data in maps:
            for y in range(map_data.info.height):
                for x in range(map_data.info.width):
                    i = x + y * map_data.info.width
                    merged_x = int(np.floor((map_data.info.origin.position.x + 
                                           x * map_data.info.resolution - self.origin_offset) / 
                                          merged_map.info.resolution))
                    merged_y = int(np.floor((map_data.info.origin.position.y + 
                                           y * map_data.info.resolution - self.origin_offset) / 
                                          merged_map.info.resolution))
                
                    if (0 <= merged_x < merged_map.info.width and 
                        0 <= merged_y < merged_map.info.height):
                        merged_i = merged_x + merged_y * merged_map.info.width
                    
                        # 合併規則
                        if merged_map.data[merged_i] == -1:
                            merged_map.data[merged_i] = map_data.data[i]
                        elif merged_map.data[merged_i] == 100 and map_data.data[i] == 100:
                            merged_map.data[merged_i] = 100
                        elif (merged_map.data[merged_i] == 0 and map_data.data[i] == 100) or \
                             (merged_map.data[merged_i] == 100 and map_data.data[i] == 0):
                            merged_map.data[merged_i] = 100

        # 轉換為numpy數組進行膨脹處理
        map_array = np.array(merged_map.data).reshape(merged_map.info.height, merged_map.info.width)
    
        # 創建障礙物二值圖
        obstacle_map = (map_array == 100).astype(np.uint8)
    
        # 創建膨脹核
        kernel = np.ones((self.dilation_size, self.dilation_size), np.uint8)
    
        # 應用膨脹
        dilated_obstacles = ndimage.binary_dilation(obstacle_map, kernel).astype(np.uint8)
    
        # 更新膨脹後的地圖，保留未知區域
        dilated_map = map_array.copy()
        # 只在已知區域進行膨脹
        known_area = (map_array != -1)
        dilated_map[known_area] = 0  # 先將已知區域設為空閒
        dilated_map[dilated_obstacles == 1] = 100  # 設置膨脹後的障礙物
    
        # 轉換回一維列表
        merged_map.data = dilated_map.flatten().tolist()
    
        return merged_map
        
    def merge_and_publish(self):
        """
        定期合併地圖並發布機器人marker
        """
        if self.map1 is not None and self.map2 is not None and self.map3 is not None:
            # 合併地圖
            merged_map = self.merge_maps(self.map1, self.map2, self.map3)
            
            if merged_map is not None:
                # 發布合併的地圖
                merged_map.header.stamp = self.get_clock().now().to_msg()
                self.map_publisher.publish(merged_map)
                
                # 創建並發布機器人markers
                marker_array = MarkerArray()
                robot_positions = [
                    self.get_robot_position('tb3_0'),
                    self.get_robot_position('tb3_1'),
                    self.get_robot_position('tb3_2')
                ]
                
                for robot_id, pos in enumerate(robot_positions):
                    if pos is not None:
                        marker = self.create_robot_marker(robot_id, pos)
                        marker_array.markers.append(marker)
                
                self.marker_publisher.publish(marker_array)
                self.get_logger().debug('Published merged map and robot markers')

def main(args=None):
    rclpy.init(args=args)
    merge_map_node = MergeMapNode()
    
    try:
        rclpy.spin(merge_map_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        merge_map_node.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        merge_map_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()