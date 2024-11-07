#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PointStamped, PolygonStamped
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.duration import Duration
import math
import traceback

class GlobalRRTDetector(Node):
    def __init__(self):
        super().__init__('global_rrt_detector')
        
        # 声明参数
        self.declare_parameter('eta', 0.5)
        self.declare_parameter('map_topic', '/merge_map')
        
        # 获取参数
        self.eta = self.get_parameter('eta').value
        self.map_topic = self.get_parameter('map_topic').value
        
        # 初始化变量
        self.mapData = None
        self.boundary_received = False
        self.start_point_received = False
        self.boundary = None
        self.V = []  # RRT顶点
        self.init_x = 0.0
        self.init_y = 0.0
        self.init_map_x = 0.0
        self.init_map_y = 0.0
        self.frontier_count = 0  # 用于记录发现的frontier点数量

        # 订阅地图数据
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )

        # 订阅探索边界
        self.boundary_sub = self.create_subscription(
            PolygonStamped,
            '/exploration_boundary',
            self.boundary_callback,
            10
        )

        # 订阅RViz点击点
        self.click_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )

        # 发布前沿点
        self.frontier_pub = self.create_publisher(
            PointStamped,
            '/detected_points',
            10
        )

        # 发布可视化标记
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/visualization_marker_array',
            10
        )

        # 初始化所有可视化标记
        self.init_markers()

        # RRT迭代定时器
        self.create_timer(0.01, self.rrt_iteration)
        
        self.get_logger().info('Global RRT detector initialized')
        self.get_logger().info('Waiting for map and boundary...')

    def init_markers(self):
        """初始化所有可视化标记"""
        self.marker_array = MarkerArray()

        # RRT树标记
        self.tree_marker = Marker()
        self.tree_marker.header.frame_id = 'map'
        self.tree_marker.ns = "rrt_tree"
        self.tree_marker.id = 0
        self.tree_marker.type = Marker.LINE_LIST
        self.tree_marker.action = Marker.ADD
        self.tree_marker.pose.orientation.w = 1.0
        self.tree_marker.scale.x = 0.05  # 线宽
        self.tree_marker.color.r = 0.0
        self.tree_marker.color.g = 0.8
        self.tree_marker.color.b = 0.8
        self.tree_marker.color.a = 0.8
        self.tree_marker.points = []
        self.marker_array.markers.append(self.tree_marker)

        # Frontier点标记
        self.frontier_marker = Marker()
        self.frontier_marker.header.frame_id = 'map'
        self.frontier_marker.ns = "frontiers"
        self.frontier_marker.id = 1
        self.frontier_marker.type = Marker.SPHERE_LIST
        self.frontier_marker.action = Marker.ADD
        self.frontier_marker.pose.orientation.w = 1.0
        self.frontier_marker.scale.x = 0.2
        self.frontier_marker.scale.y = 0.2
        self.frontier_marker.scale.z = 0.2
        self.frontier_marker.color.r = 1.0
        self.frontier_marker.color.g = 0.0
        self.frontier_marker.color.b = 0.0
        self.frontier_marker.color.a = 0.8
        self.frontier_marker.points = []
        self.marker_array.markers.append(self.frontier_marker)

        # 起始点标记
        self.start_marker = Marker()
        self.start_marker.header.frame_id = 'map'
        self.start_marker.ns = "start_point"
        self.start_marker.id = 2
        self.start_marker.type = Marker.CUBE
        self.start_marker.action = Marker.ADD
        self.start_marker.pose.orientation.w = 1.0
        self.start_marker.scale.x = 0.3
        self.start_marker.scale.y = 0.3
        self.start_marker.scale.z = 0.3
        self.start_marker.color.r = 0.0
        self.start_marker.color.g = 1.0
        self.start_marker.color.b = 0.0
        self.start_marker.color.a = 1.0
        self.marker_array.markers.append(self.start_marker)

    def map_callback(self, msg):
        """地图数据回调"""
        if self.mapData is None:
            self.get_logger().info('First map data received')
            self.get_logger().info(f'Map size: {msg.info.width}x{msg.info.height}, resolution: {msg.info.resolution}')
        self.mapData = msg
        
        # 更新所有标记的frame_id
        for marker in self.marker_array.markers:
            marker.header.frame_id = msg.header.frame_id

    def boundary_callback(self, msg):
        """边界回调"""
        if not self.boundary_received:
            self.boundary = msg.polygon.points
            points = msg.polygon.points
            
            # 计算边界框
            x_coords = [p.x for p in points]
            y_coords = [p.y for p in points]
            
            self.init_map_x = max(x_coords) - min(x_coords)
            self.init_map_y = max(y_coords) - min(y_coords)
            self.init_x = (max(x_coords) + min(x_coords)) / 2
            self.init_y = (max(y_coords) + min(y_coords)) / 2
            
            self.boundary_received = True
            self.get_logger().info('Received exploration boundary')
            self.get_logger().info(f'Area size: {self.init_map_x:.2f} x {self.init_map_y:.2f}')
            self.get_logger().info(f'Center: ({self.init_x:.2f}, {self.init_y:.2f})')
            self.get_logger().info('Please click a point in RViz to set RRT start position')

    # def clicked_point_callback(self, msg):
    #     """处理RViz点击点"""
    #     if not self.boundary_received:
    #         self.get_logger().warn('Waiting for boundary to be set first')
    #         return

    #     if self.start_point_received:
    #         self.get_logger().warn('Start point already set. Ignoring new click.')
    #         return

    #     point = [msg.point.x, msg.point.y]
    #     if not self.is_point_in_boundary(point):
    #         self.get_logger().warn('Clicked point is outside boundary. Please click inside the boundary.')
    #         return

    #     if self.check_point(point) != 1:
    #         self.get_logger().warn('Clicked point is not in free space. Please choose another point.')
    #         return

    #     # 设置起始点
    #     self.V = [point]
    #     self.start_point_received = True
        
    #     # 更新起始点标记
    #     self.start_marker.pose.position.x = point[0]
    #     self.start_marker.pose.position.y = point[1]
    #     self.start_marker.pose.position.z = 0.0
    #     self.start_marker.header.stamp = self.get_clock().now().to_msg()
        
    #     # 发布标记数组
    #     self.marker_array.markers[2] = self.start_marker  # 更新起始点标记
    #     self.marker_pub.publish(self.marker_array)

    #     self.get_logger().info(f'Set RRT start point: ({point[0]:.2f}, {point[1]:.2f})')
    #     self.get_logger().info('Starting RRT exploration...')

    # def is_point_in_boundary(self, point):
    #     """检查点是否在边界内"""
    #     if not self.boundary:
    #         return False
        
    #     inside = False
    #     j = len(self.boundary) - 1
        
    #     for i in range(len(self.boundary)):
    #         if ((self.boundary[i].y > point[1]) != (self.boundary[j].y > point[1]) and
    #             point[0] < (self.boundary[j].x - self.boundary[i].x) * 
    #             (point[1] - self.boundary[i].y) / 
    #             (self.boundary[j].y - self.boundary[i].y) + 
    #             self.boundary[i].x):
    #             inside = not inside
    #         j = i
        
    #     return inside

    # def check_point(self, point):
    #     """检查点状态: -1:前沿点, 0:障碍物, 1:自由空间"""
    #     if not self.mapData:
    #         return 0

    #     resolution = self.mapData.info.resolution
    #     origin_x = self.mapData.info.origin.position.x
    #     origin_y = self.mapData.info.origin.position.y
    #     width = self.mapData.info.width
        
    #     x = int((point[0] - origin_x) / resolution)
    #     y = int((point[1] - origin_y) / resolution)
        
    #     if (0 <= x < width and 0 <= y < self.mapData.info.height):
    #         idx = y * width + x
            
    #         has_unknown = False
    #         has_free = False
            
    #         for dx in [-1, 0, 1]:
    #             for dy in [-1, 0, 1]:
    #                 nx = x + dx
    #                 ny = y + dy
    #                 if (0 <= nx < width and 0 <= ny < self.mapData.info.height):
    #                     nidx = ny * width + nx
    #                     value = self.mapData.data[nidx]
    #                     if value == -1:
    #                         has_unknown = True
    #                     elif value == 0:
    #                         has_free = True
            
    #         if has_unknown and has_free:
    #             return -1  # 前沿点
    #         elif self.mapData.data[idx] == 100:
    #             return 0   # 障碍物
    #         else:
    #             return 1   # 自由空间
        
    #     return 0

    # def check_path(self, p1, p2):
    #     """检查路径是否无障碍"""
    #     if not self.mapData:
    #         return 0

    #     resolution = self.mapData.info.resolution
    #     steps = int(np.ceil(np.linalg.norm(np.array(p2) - np.array(p1)) / (resolution * 0.5)))
        
    #     for i in range(steps + 1):
    #         t = i / steps
    #         point = [p1[0] + t * (p2[0] - p1[0]), 
    #                 p1[1] + t * (p2[1] - p1[1])]
    #         status = self.check_point(point)
    #         if status <= 0:
    #             return status
        
    #     return 1



    def clicked_point_callback(self, msg):
        """处理RViz点击点"""
        if not self.boundary_received:
            self.get_logger().warn('Waiting for boundary to be set first')
            return

        if self.start_point_received:
            self.get_logger().warn('Start point already set. Ignoring new click.')
            return

        point = [msg.point.x, msg.point.y]
        if not self.is_point_in_boundary(point):
            self.get_logger().warn('Clicked point is outside boundary. Please click inside the boundary.')
            return

        if self.check_point(point) != 1:
            self.get_logger().warn('Clicked point is not in free space. Please choose another point.')
            return

        # 设置起始点
        self.V = [point]
        self.start_point_received = True
    
        # 更新起始点标记
        self.start_marker.pose.position.x = point[0]
        self.start_marker.pose.position.y = point[1]
        self.start_marker.pose.position.z = 0.0
        self.start_marker.header.stamp = self.get_clock().now().to_msg()
    
        # 发布标记数组
        self.marker_array.markers[2] = self.start_marker  # 更新起始点标记
        self.marker_pub.publish(self.marker_array)

        self.get_logger().info(f'Set RRT start point: ({point[0]:.2f}, {point[1]:.2f})')
        self.get_logger().info('Starting RRT exploration...')

    def is_point_in_boundary(self, point):
        """检查点是否在边界内"""
        if not self.boundary:
            return False
    
        inside = False
        j = len(self.boundary) - 1
    
        for i in range(len(self.boundary)):
            if ((self.boundary[i].y > point[1]) != (self.boundary[j].y > point[1]) and
                point[0] < (self.boundary[j].x - self.boundary[i].x) * 
                (point[1] - self.boundary[i].y) / 
                (self.boundary[j].y - self.boundary[i].y) + 
                self.boundary[i].x):
                inside = not inside
            j = i
    
        return inside

    def check_point(self, point):
        """检查点状态: 
        -1: frontier点 (已知空间和未知空间的边界)
        0: 无效点 (障碍物或超出边界或未知区域)
        1: 有效点 (已知的自由空间)
        """
        if not self.mapData or not self.is_point_in_boundary(point):
            return 0  # 超出边界或无地图

        resolution = self.mapData.info.resolution
        origin_x = self.mapData.info.origin.position.x
        origin_y = self.mapData.info.origin.position.y
        width = self.mapData.info.width
    
        # 转换为栅格坐标
        x = int((point[0] - origin_x) / resolution)
        y = int((point[1] - origin_y) / resolution)
    
        if not (0 <= x < width and 0 <= y < self.mapData.info.height):
            return 0  # 超出地图范围

        cell_value = self.mapData.data[y * width + x]
    
        # 首先检查当前点是否在已知的自由空间
        if cell_value != 0:  # 如果不是已知的自由空间
            return 0
        
        # 检查周围8个相邻点
        has_unknown = False
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
            
                nx = x + dx
                ny = y + dy
                if (0 <= nx < width and 0 <= ny < self.mapData.info.height):
                    neighbor_value = self.mapData.data[ny * width + nx]
                    if neighbor_value == -1:  # 未知区域
                        has_unknown = True
                    
        return -1 if has_unknown else 1  # 如果邻接未知区域则是frontier点，否则是普通自由空间

    def check_path(self, p1, p2):
        """检查路径是否有效
        返回:
        -1: 路径通向frontier
        0: 路径无效（碰到障碍物、未知区域或边界）
        1: 路径有效（完全在已知自由空间内）
        """
        if not self.mapData:
            return 0

        # 检查起点和终点是否在边界内
        if not (self.is_point_in_boundary(p1) and self.is_point_in_boundary(p2)):
            return 0

        resolution = self.mapData.info.resolution
        steps = int(np.ceil(np.linalg.norm(np.array(p2) - np.array(p1)) / (resolution * 0.5)))
    
        found_frontier = False
    
        # 检查路径上的所有点
        for i in range(steps + 1):
            t = i / steps
            point = [
                p1[0] + t * (p2[0] - p1[0]),
                p1[1] + t * (p2[1] - p1[1])
            ]
        
            status = self.check_point(point)
        
            if status == 0:  # 如果是障碍物或未知区域
                return 0
            elif status == -1:  # 如果是frontier点
                found_frontier = True
            
        return -1 if found_frontier else 1







    def publish_tree(self, p1, p2):
        """发布RRT树的新边"""
        point1 = Point()
        point1.x = float(p1[0])
        point1.y = float(p1[1])
        point1.z = 0.0
        
        point2 = Point()
        point2.x = float(p2[0])
        point2.y = float(p2[1])
        point2.z = 0.0
        
        if len(self.tree_marker.points) > 200:  # 限制100条边
            self.tree_marker.points = self.tree_marker.points[2:]
        
        self.tree_marker.points.extend([point1, point2])
        self.marker_array.markers[0] = self.tree_marker
        self.marker_pub.publish(self.marker_array)

    def publish_frontier(self, point):
        """发布前沿点"""
        # 发布消息
        msg = PointStamped()
        msg.header.frame_id = self.mapData.header.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(point[0])
        msg.point.y = float(point[1])
        msg.point.z = 0.0
        self.frontier_pub.publish(msg)

        # 添加到可视化
        p = Point()
        p.x = float(point[0])
        p.y = float(point[1])
        p.z = 0.0
        
        if len(self.frontier_marker.points) > 50:
            self.frontier_marker.points = self.frontier_marker.points[1:]
        self.frontier_marker.points.append(p)
        
        self.marker_array.markers[1] = self.frontier_marker
        self.marker_pub.publish(self.marker_array)

        self.frontier_count += 1
        self.get_logger().info(f'Found frontier {self.frontier_count}: ({point[0]:.2f}, {point[1]:.2f})')

    # def rrt_iteration(self):
    #     """RRT迭代"""
    #     if not all([self.boundary_received, self.start_point_received, self.mapData]):
    #         return

    #     try:
    #         # 生成随机点
    #         attempts = 0
    #         x_rand = None
    #         while attempts < 100:
    #             x_rand = [
    #                 np.random.uniform(self.init_x - self.init_map_x/2, 
    #                                 self.init_x + self.init_map_x/2),
    #                 np.random.uniform(self.init_y - self.init_map_y/2, 
    #                                 self.init_y + self.init_map_y/2)
    #             ]
    #             if self.is_point_in_boundary(x_rand):
    #                 break
    #             attempts += 1
            
    #         if x_rand is None or attempts >= 100:
    #             return

    #         # 找最近点
    #         V_array = np.array(self.V)
    #         dist = np.linalg.norm(V_array - np.array(x_rand).reshape(1, 2), axis=1)
    #         nearest_idx = np.argmin(dist)
    #         x_nearest = self.V[nearest_idx]
            
    #         # Steer
    #         dist = np.linalg.norm(np.array(x_rand) - np.array(x_nearest))
    #         if dist <= self.eta:
    #             x_new = x_rand
    #         else:
    #             dir_vector = np.array(x_rand) - np.array(x_nearest)
    #             x_new = (x_nearest + (dir_vector / dist) * self.eta).tolist()

    #         # 检查路径
    #         status = self.check_path(x_nearest, x_new)

    #         if status == -1:  # 前沿点
    #             self.publish_frontier(x_new)
    #             self.V = [x_new]  # 重置RRT树
    #             # 清除树可视化
    #             self.tree_marker.points = []
    #             self.marker_array.markers[0] = self.tree_marker
    #             self.marker_pub.publish(self.marker_array)
        
    #         elif status == 1:  # 自由空间
    #             self.V.append(x_new)
    #             self.publish_tree(x_nearest, x_new)

    #     except Exception as e:
    #         self.get_logger().error(f'Error in RRT iteration: {str(e)}')
    #         traceback.print_exc()


    def rrt_iteration(self):
        """RRT迭代"""
        if not all([self.boundary_received, self.start_point_received, self.mapData]):
            return

        try:
            # 生成随机点
            attempts = 0
            x_rand = None
            while attempts < 100:
                x_rand = [
                    np.random.uniform(self.init_x - self.init_map_x/2, 
                                    self.init_x + self.init_map_x/2),
                    np.random.uniform(self.init_y - self.init_map_y/2, 
                                    self.init_y + self.init_map_y/2)
                ]
                if self.is_point_in_boundary(x_rand):
                    break
                attempts += 1
        
            if x_rand is None or attempts >= 100:
                return

            # 找最近点
            V_array = np.array(self.V)
            dist = np.linalg.norm(V_array - np.array(x_rand).reshape(1, 2), axis=1)
            nearest_idx = np.argmin(dist)
            x_nearest = self.V[nearest_idx]
        
            # Steer
            dist = np.linalg.norm(np.array(x_rand) - np.array(x_nearest))
            if dist <= self.eta:
                x_new = x_rand
            else:
                dir_vector = np.array(x_rand) - np.array(x_nearest)
                x_new = (x_nearest + (dir_vector / dist) * self.eta).tolist()

            # 检查路径
            status = self.check_path(x_nearest, x_new)

            if status == -1:  # 前沿点
                self.publish_frontier(x_new)
                # 不再重置RRT树，而是继续从当前树生长
                self.V.append(x_new)  # 将frontier点添加到树中
                self.publish_tree(x_nearest, x_new)
        
            elif status == 1:  # 自由空间
                self.V.append(x_new)
                self.publish_tree(x_nearest, x_new)

        except Exception as e:
            self.get_logger().error(f'Error in RRT iteration: {str(e)}')
            traceback.print_exc()







def main(args=None):
    rclpy.init(args=args)
    try:
        node = GlobalRRTDetector()
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