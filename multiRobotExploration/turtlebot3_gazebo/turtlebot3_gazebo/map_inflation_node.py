#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from copy import deepcopy
import numpy as np

class MapInflationNode(Node):
    def __init__(self):
        super().__init__('map_inflation_node')
        
        # 訂閱三個機器人的原始地圖
        self.subscription1 = self.create_subscription(
            OccupancyGrid,
            'tb3_0/map_temp',
            lambda msg: self.map_callback(msg, 'tb3_0'),
            10)
            
        self.subscription2 = self.create_subscription(
            OccupancyGrid,
            'tb3_1/map_temp',
            lambda msg: self.map_callback(msg, 'tb3_1'),
            10)
            
        self.subscription3 = self.create_subscription(
            OccupancyGrid,
            'tb3_2/map_temp',
            lambda msg: self.map_callback(msg, 'tb3_2'),
            10)

        # 創建三個發布者
        self.publisher1 = self.create_publisher(OccupancyGrid, 'tb3_0/map', 10)
        self.publisher2 = self.create_publisher(OccupancyGrid, 'tb3_1/map', 10)
        self.publisher3 = self.create_publisher(OccupancyGrid, 'tb3_2/map', 10)

        # 設置膨脹參數
        self.declare_parameter('inflation_radius', 3)
        self.inflation_radius = self.get_parameter('inflation_radius').value

    def map_callback(self, msg, robot_name):
        # 創建新的地圖消息
        inflated_map = deepcopy(msg)
        
        # 將地圖數據轉換為numpy數組
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape(height, width)
        
        # 創建輸出數組
        inflated_data = np.full_like(data, -1)  # 初始化為未知區域
        
        # 計算kernel半徑（像素）
        kernel_radius = int(self.inflation_radius / msg.info.resolution)
        
        # 創建圓形kernel
        y, x = np.ogrid[-kernel_radius:kernel_radius+1, -kernel_radius:kernel_radius+1]
        kernel_mask = x*x + y*y <= kernel_radius*kernel_radius
        
        # 執行膨脹
        for y in range(height):
            for x in range(width):
                if data[y, x] == 100:  # 如果是障礙物
                    # 計算kernel範圍
                    y_min = max(0, y - kernel_radius)
                    y_max = min(height, y + kernel_radius + 1)
                    x_min = max(0, x - kernel_radius)
                    x_max = min(width, x + kernel_radius + 1)
                    
                    # 應用kernel
                    kernel_y_start = kernel_radius - (y - y_min)
                    kernel_y_end = kernel_radius + (y_max - y)
                    kernel_x_start = kernel_radius - (x - x_min)
                    kernel_x_end = kernel_radius + (x_max - x)
                    
                    current_kernel = kernel_mask[kernel_y_start:kernel_y_end, 
                                               kernel_x_start:kernel_x_end]
                    
                    # 設置膨脹區域
                    inflated_data[y_min:y_max, x_min:x_max][current_kernel] = 100
                elif data[y, x] == 0 and inflated_data[y, x] != 100:
                    inflated_data[y, x] = 0  # 如果是自由空間且未被膨脹影響
        
        # 更新地圖數據
        inflated_map.data = inflated_data.flatten().tolist()
        
        # 發布膨脹後的地圖
        if robot_name == 'tb3_0':
            self.publisher1.publish(inflated_map)
        elif robot_name == 'tb3_1':
            self.publisher2.publish(inflated_map)
        else:
            self.publisher3.publish(inflated_map)

def main(args=None):
    rclpy.init(args=args)
    node = MapInflationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()