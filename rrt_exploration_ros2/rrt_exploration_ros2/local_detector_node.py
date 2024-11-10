#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
import tf2_ros
from tf2_ros import TransformException
from rclpy.time import Time
import math
import traceback

class LocalRRTDetector(Node):
    def __init__(self):
        super().__init__('local_rrt_detector')
        
        # 原有的參數聲明
        self.declare_parameter('eta', 1.0)
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('robot_name', 'tb3_0')
        self.declare_parameter('update_frequency', 10.0)
        
        # 獲取參數
        self.eta = self.get_parameter('eta').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.robot_name = self.get_parameter('robot_name').value
        self.update_frequency = self.get_parameter('update_frequency').value
        
        # 常數設置
        self.MAX_VERTICES = 500
        self.TF_TIMEOUT = 1.0
        self.MAX_FRONTIERS = 100  # 最大儲存的 frontier 數量
        
        # 初始化變量
        self.mapData = None
        self.V = []
        self.parents = {}
        self.init_map_x = 0.0
        self.init_map_y = 0.0
        self.init_x = 0.0
        self.init_y = 0.0
        self.tf_ready = False
        self.frontiers = []  # 儲存所有找到的 frontiers
        
        # 設置 TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 發布器
        self.frontier_pub = self.create_publisher(
            PointStamped,
            '/detected_points',
            10
        )
        
        self.marker_pub = self.create_publisher(
            Marker,
            f'/{self.robot_name}/local_rrt_markers',
            10
        )
        
        # 新增：Frontier MarkerArray 發布器
        self.frontier_markers_pub = self.create_publisher(
            MarkerArray,
            f'/{self.robot_name}/frontier_markers',
            10
        )
        
        # 調試發布器
        self.debug_publisher = self.create_publisher(
            String,
            f'/{self.robot_name}/debug',
            10
        )
        
        # 訂閱合併地圖
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/merge_map',
            self.map_callback,
            10
        )
        
        # 初始化可視化標記
        self.points_marker = self.create_points_marker()
        self.line_marker = self.create_line_marker()
        self.frontier_marker_array = MarkerArray()
        
        # 創建定時器
        self.create_timer(1.0 / self.update_frequency, self.rrt_iteration)
        self.create_timer(1.0, self.check_tf_available)
        self.create_timer(0.1, self.publish_markers)
        self.create_timer(0.1, self.publish_frontier_markers)  # 新增：發布 frontier markers
        
        self.get_logger().info('Local RRT detector initialized with frontier visualization')

    def create_points_marker(self):
        """創建點的可視化標記"""
        marker = Marker()
        marker.header.frame_id = "merge_map"  # 使用合併地圖的frame
        marker.ns = f'{self.robot_name}_points'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # 增大點的大小
        marker.scale.y = 0.1
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()  # 永久顯示
        
        # 設置不同機器人的顏色
        if self.robot_name == 'tb3_0':
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif self.robot_name == 'tb3_1':
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:  # tb3_2
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            
        marker.color.a = 1.0
        return marker

    def create_line_marker(self):
        """創建線的可視化標記"""
        marker = Marker()
        marker.header.frame_id = "merge_map"  # 使用合併地圖的frame
        marker.ns = f'{self.robot_name}_lines'
        marker.id = 1
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05  # 增加線條寬度
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()  # 永久顯示
        
        # 設置不同機器人的顏色
        if self.robot_name == 'tb3_0':
            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.2
        elif self.robot_name == 'tb3_1':
            marker.color.r = 0.2
            marker.color.g = 1.0
            marker.color.b = 0.2
        else:  # tb3_2
            marker.color.r = 0.2
            marker.color.g = 0.2
            marker.color.b = 1.0
            
        marker.color.a = 0.6
        return marker

    def publish_markers(self):
        """定期發布標記，使用實際的RRT結構"""
        if self.V:
            # 發布點標記
            self.points_marker.points = []
            for vertex in self.V:
                p = Point()
                p.x = vertex[0]
                p.y = vertex[1]
                p.z = 0.0
                self.points_marker.points.append(p)
            self.points_marker.header.stamp = self.get_clock().now().to_msg()
            self.marker_pub.publish(self.points_marker)
            
            # 發布線標記（根據實際的樹結構）
            if len(self.V) > 1:
                self.line_marker.points = []
                # 從每個點回溯到其父節點
                for child_str, parent in self.parents.items():
                    child = eval(child_str)  # 將字符串轉回列表
                    
                    # 添加父節點
                    p1 = Point()
                    p1.x = parent[0]
                    p1.y = parent[1]
                    p1.z = 0.0
                    
                    # 添加子節點
                    p2 = Point()
                    p2.x = child[0]
                    p2.y = child[1]
                    p2.z = 0.0
                    
                    self.line_marker.points.extend([p1, p2])
                
                self.line_marker.header.stamp = self.get_clock().now().to_msg()
                self.marker_pub.publish(self.line_marker)

    def check_tf_available(self):
        """檢查 TF 是否可用"""
        if not self.tf_ready:
            try:
                transform = self.tf_buffer.lookup_transform(
                    f'{self.robot_name}/map',
                    f'{self.robot_name}/base_footprint',
                    rclpy.time.Time(seconds=0),
                    timeout=rclpy.duration.Duration(seconds=0.1))
                    
                self.tf_ready = True
                self.get_logger().info('TF transform is now available')
                
            except TransformException:
                self.get_logger().debug('Waiting for TF transform to become available...')

    def create_frontier_marker(self, point, marker_id):
        """創建單個 frontier 的標記"""
        marker = Marker()
        marker.header.frame_id = "merge_map"
        marker.ns = f'{self.robot_name}_frontier'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        
        # 根據機器人設置不同顏色
        if self.robot_name == 'tb3_0':
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif self.robot_name == 'tb3_1':
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:  # tb3_2
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        
        marker.color.a = 0.8
        marker.lifetime = rclpy.duration.Duration(seconds=5.0).to_msg()  # 5秒後消失
        
        return marker

    def publish_frontier_markers(self):
        """發布所有 frontier 標記"""
        if not self.frontiers:
            return
        
        marker_array = MarkerArray()
        
        # 為每個 frontier 點創建標記
        for i, frontier in enumerate(self.frontiers):
            marker = self.create_frontier_marker(frontier, i)
            marker.header.stamp = self.get_clock().now().to_msg()
            marker_array.markers.append(marker)
        
        # 發布 marker array
        self.frontier_markers_pub.publish(marker_array)

    def add_frontier(self, point):
        """添加新的 frontier 點"""
        # 檢查是否與現有的 frontier 點太近
        MIN_DISTANCE = 0.5  # 最小距離閾值（米）
        
        for existing_point in self.frontiers:
            distance = np.linalg.norm(np.array(point) - np.array(existing_point))
            if distance < MIN_DISTANCE:
                return False
        
        # 添加新的 frontier 點
        self.frontiers.append(point)
        
        # 如果超過最大數量，移除最舊的點
        if len(self.frontiers) > self.MAX_FRONTIERS:
            self.frontiers.pop(0)
        
        return True

    def publish_frontier(self, point):
        """發布 frontier 點"""
        # 檢查並添加新的 frontier
        if self.add_frontier(point):
            msg = PointStamped()
            msg.header.frame_id = "merge_map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.point.x = point[0]
            msg.point.y = point[1]
            msg.point.z = 0.0
            self.frontier_pub.publish(msg)
            
            # 發布調試信息
            debug_msg = String()
            debug_msg.data = f'Found new frontier at: ({point[0]:.2f}, {point[1]:.2f})'
            self.debug_publisher.publish(debug_msg)

    def map_callback(self, msg):
        """處理地圖數據"""
        if self.mapData is None:
            self.get_logger().info('First map data received')
            self.get_logger().info(f'Map size: {msg.info.width}x{msg.info.height}, resolution: {msg.info.resolution}')
            
            # 初始化地圖尺寸
            self.init_map_x = msg.info.width * msg.info.resolution
            self.init_map_y = msg.info.height * msg.info.resolution
            self.init_x = msg.info.origin.position.x + self.init_map_x/2
            self.init_y = msg.info.origin.position.y + self.init_map_y/2
            
        self.mapData = msg

    def get_robot_position(self):
        """獲取機器人位置"""
        try:
            transform = self.tf_buffer.lookup_transform(
                f'{self.robot_name}/map',
                f'{self.robot_name}/base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
                
            return [transform.transform.translation.x, transform.transform.translation.y]
                
        except TransformException:
            try:
                transform = self.tf_buffer.lookup_transform(
                    f'{self.robot_name}/map',
                    f'{self.robot_name}/base_footprint',
                    rclpy.time.Time(seconds=0),
                    timeout=rclpy.duration.Duration(seconds=0.1))
                    
                return [transform.transform.translation.x, transform.transform.translation.y]
                    
            except TransformException as ex:
                self.get_logger().warning(
                    f'無法獲取機器人 {self.robot_name} 位置: {str(ex)}')
                return None

    def is_valid_point(self, point):
        """檢查點是否有效"""
        if not self.mapData:
            return False
            
        resolution = self.mapData.info.resolution
        origin_x = self.mapData.info.origin.position.x
        origin_y = self.mapData.info.origin.position.y
        width = self.mapData.info.width
        
        # 轉換為地圖座標
        x = int((point[0] - origin_x) / resolution)
        y = int((point[1] - origin_y) / resolution)
        
        # 檢查是否在地圖範圍內
        if not (0 <= x < width and 0 <= y < self.mapData.info.height):
            return False
            
        cell_value = self.mapData.data[y * width + x]
        
        
        # 考慮機器人標記單元格 (50-80) 為有效
        return cell_value == 0 or (50 <= cell_value <= 80)

    def check_path(self, start, end):
        """檢查路徑狀態"""
        if not self.mapData:
            return 0
            
        resolution = self.mapData.info.resolution
        steps = int(np.ceil(np.linalg.norm(np.array(end) - np.array(start)) / (resolution)))
        
        has_unknown = False
        unknown_count = 0
        obstacle_count = 0
        
        for i in range(steps + 1):
            t = i / steps
            point = [
                start[0] + t * (end[0] - start[0]),
                start[1] + t * (end[1] - start[1])
            ]
            
            # 獲取單元格值
            x = int((point[0] - self.mapData.info.origin.position.x) / resolution)
            y = int((point[1] - self.mapData.info.origin.position.y) / resolution)
            
            if not (0 <= x < self.mapData.info.width and 0 <= y < self.mapData.info.height):
                return 0
                
            cell = self.mapData.data[y * self.mapData.info.width + x]
            
            if cell > 0 and not (50 <= cell <= 80):  # 障礙物
                obstacle_count += 1
            elif cell == -1:  # 未知區域
                unknown_count += 1
                has_unknown = True
                
        # 如果障礙物太多或未知區域太少，返回0
        if obstacle_count > steps * 0.1:  # 允許10%的障礙
            return 0
        if unknown_count > steps * 0.3:  # 至少30%是未知區域才算frontier
            return -1
            
        return 1

    def rrt_iteration(self):
        """執行一次RRT迭代"""
        if not self.mapData or not self.tf_ready:
            return
            
        try:
            robot_pos = self.get_robot_position()
            if not robot_pos:
                return
                
            if not self.V:
                self.V = [robot_pos]
                self.parents = {}
                self.get_logger().info(f'Tree initialized at {robot_pos}')
            
            # 增加多次嘗試的機會
            for _ in range(10):
                angle = np.random.uniform(0, 2 * np.pi)
                r = np.random.uniform(0, 5.0)
                x_rand = [
                    robot_pos[0] + r * np.cos(angle),
                    robot_pos[1] + r * np.sin(angle)
                ]
                
                if not self.is_valid_point(x_rand):
                    continue
                
                V_array = np.array(self.V)
                dist = np.linalg.norm(V_array - np.array(x_rand), axis=1)
                nearest_idx = np.argmin(dist)
                x_nearest = self.V[nearest_idx]
                
                dist = np.linalg.norm(np.array(x_rand) - np.array(x_nearest))
                if dist <= self.eta:
                    x_new = x_rand
                else:
                    dir_vector = np.array(x_rand) - np.array(x_nearest)
                    x_new = (x_nearest + (dir_vector / dist) * self.eta).tolist()
                
                if not self.is_valid_point(x_new):
                    continue
                    
                path_status = self.check_path(x_nearest, x_new)
                
                if path_status == -1:  # 找到frontier
                    self.get_logger().info(f'Found frontier point at {x_new}')
                    self.publish_frontier(x_new)
                    self.V = [robot_pos]  # 重置樹
                    self.parents = {}  # 重置父節點記錄
                    break
                    
                elif path_status == 1:  # 有效路徑
                    self.V.append(x_new)
                    self.parents[str(x_new)] = x_nearest
                    self.get_logger().info(f'Added new point to tree: {x_new}')
                    break
                    
            # 控制樹的大小
            if len(self.V) > self.MAX_VERTICES:
                self.V = [robot_pos] + self.V[-(self.MAX_VERTICES-1):]
                new_parents = {}
                for child_str, parent in self.parents.items():
                    child = eval(child_str)
                    if child in self.V and parent in self.V:
                        new_parents[child_str] = parent
                self.parents = new_parents

        except Exception as e:
            self.get_logger().error(f'RRT iteration error: {str(e)}')
            traceback.print_exc()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = LocalRRTDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Caught exception: {str(e)}')
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()