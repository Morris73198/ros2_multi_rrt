#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PolygonStamped, Polygon, PointStamped, Point32
from visualization_msgs.msg import Marker, MarkerArray
import math

class ExplorationBoundaryNode(Node):
    def __init__(self):
        super().__init__('exploration_boundary')
        
        # 初始化变量
        self.recorded_points = []
        self.frame_id = 'merge_map'
        
        # 创建订阅者
        self.point_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.point_callback,
            10
        )
        
        # 创建发布者
        self.boundary_pub = self.create_publisher(
            PolygonStamped,
            '/exploration_boundary',
            10
        )
        
        # 创建圆圈标记发布者
        self.circles_pub = self.create_publisher(
            MarkerArray,
            '/boundary_circles',
            10
        )
        
        # 创建线标记发布者
        self.line_marker_pub = self.create_publisher(
            Marker,
            '/boundary_line',
            10
        )
        
        # 创建定时器
        self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Boundary node started')

    def create_circle_marker(self, point, index):
        """创建圆圈标记"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'boundary_circles'
        marker.id = index
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # 设置线宽
        marker.scale.x = 0.05  # 线宽
        
        # 设置颜色（红色）
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # 创建圆圈点
        num_points = 32  # 圆圈的分段数
        radius = 0.3     # 圆圈半径
        for i in range(num_points + 1):
            angle = 2 * math.pi * i / num_points
            p = Point()
            p.x = point.x + radius * math.cos(angle)
            p.y = point.y + radius * math.sin(angle)
            p.z = 0.0
            marker.points.append(p)
        
        return marker

    def point_callback(self, msg):
        """处理点击点"""
        if len(self.recorded_points) < 4:
            point = Point32()
            point.x = float(msg.point.x)
            point.y = float(msg.point.y)
            point.z = 0.0
            self.recorded_points.append(point)
            self.get_logger().info(f'Point {len(self.recorded_points)} recorded: ({point.x}, {point.y})')
            
            # 发布更新的标记
            self.publish_markers()
            
            if len(self.recorded_points) == 4:
                self.publish_boundary()
                self.publish_line()

    def publish_markers(self):
        """发布所有标记"""
        # 发布圆圈标记
        marker_array = MarkerArray()
        for i, point in enumerate(self.recorded_points):
            circle_marker = self.create_circle_marker(point, i)
            marker_array.markers.append(circle_marker)
        self.circles_pub.publish(marker_array)

    def publish_line(self):
        """发布连接线标记"""
        if len(self.recorded_points) == 4:
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'boundary_line'
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # 设置线宽
            marker.scale.x = 0.05
            
            # 设置颜色（绿色）
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            # 添加所有点并闭合
            for point in self.recorded_points:
                p = Point()
                p.x = point.x
                p.y = point.y
                p.z = 0.0
                marker.points.append(p)
            
            # 闭合边界
            if len(marker.points) > 0:
                p = Point()
                p.x = self.recorded_points[0].x
                p.y = self.recorded_points[0].y
                p.z = 0.0
                marker.points.append(p)
            
            self.line_marker_pub.publish(marker)

    def publish_boundary(self):
        """发布边界多边形"""
        if len(self.recorded_points) == 4:
            msg = PolygonStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            
            points = self.recorded_points.copy()
            points.append(points[0])  # 闭合多边形
            msg.polygon.points = points
            
            self.boundary_pub.publish(msg)

    def timer_callback(self):
        """定时发布更新"""
        if len(self.recorded_points) > 0:
            self.publish_markers()
            
        if len(self.recorded_points) == 4:
            self.publish_boundary()
            self.publish_line()

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationBoundaryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()