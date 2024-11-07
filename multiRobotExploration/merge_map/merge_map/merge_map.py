import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, TransformStamped
import numpy as np
from tf2_ros import Buffer, TransformListener
import tf2_ros
from rclpy.time import Time
import math
from scipy import ndimage

def merge_maps(*maps, map_size=40.0, origin_offset=-20.0, robot_positions=None, dilation_size=3):
    """
    Merge multiple maps, mark robot positions and apply dilation
    Args:
        *maps: Variable number of OccupancyGrid maps
        map_size: Size of the map in meters
        origin_offset: Offset of the origin from the map edge
        robot_positions: List of robot positions (x, y)
        dilation_size: Size of the dilation kernel (odd number)
    Returns:
        merged_map: Combined OccupancyGrid map with robot positions marked and obstacles dilated
    """
    if not maps:
        return None
    
    merged_map = OccupancyGrid()
    merged_map.header = maps[0].header
    merged_map.header.frame_id = 'merge_map'
    
    merged_map.info.resolution = min(map.info.resolution for map in maps)
    merged_map.info.origin.position.x = origin_offset
    merged_map.info.origin.position.y = origin_offset
    merged_map.info.width = int(map_size / merged_map.info.resolution)
    merged_map.info.height = int(map_size / merged_map.info.resolution)
    
    # Initialize merged map data
    merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)
    
    # Merge maps
    for map_data in maps:
        for y in range(map_data.info.height):
            for x in range(map_data.info.width):
                i = x + y * map_data.info.width
                merged_x = int(np.floor((map_data.info.origin.position.x + 
                                       x * map_data.info.resolution - origin_offset) / 
                                      merged_map.info.resolution))
                merged_y = int(np.floor((map_data.info.origin.position.y + 
                                       y * map_data.info.resolution - origin_offset) / 
                                      merged_map.info.resolution))
                
                if (0 <= merged_x < merged_map.info.width and 
                    0 <= merged_y < merged_map.info.height):
                    merged_i = merged_x + merged_y * merged_map.info.width
                    
                    if merged_map.data[merged_i] == -1:
                        merged_map.data[merged_i] = map_data.data[i]
                    elif merged_map.data[merged_i] == 100 and map_data.data[i] == 100:
                        merged_map.data[merged_i] = 100
                    elif (merged_map.data[merged_i] == 0 and map_data.data[i] == 100) or \
                         (merged_map.data[merged_i] == 100 and map_data.data[i] == 0):
                        merged_map.data[merged_i] = 100

    # Convert to numpy array for dilation
    map_array = np.array(merged_map.data).reshape(merged_map.info.height, merged_map.info.width)
    
    # Create binary obstacle map (1 for obstacles, 0 for free space)
    obstacle_map = (map_array == 100).astype(np.uint8)
    
    # Create dilation kernel
    kernel = np.ones((dilation_size, dilation_size), np.uint8)
    
    # Apply dilation
    dilated_obstacles = ndimage.binary_dilation(obstacle_map, kernel).astype(np.uint8)
    
    # Convert back to occupancy grid format
    dilated_map = map_array.copy()
    dilated_map[dilated_obstacles == 1] = 100
    
    # Convert back to list
    merged_map.data = dilated_map.flatten().tolist()
    
    # Mark robot positions on the map
    if robot_positions:
        for robot_id, pos in enumerate(robot_positions):
            if pos is not None:
                # Convert robot position to grid coordinates
                grid_x = int((pos.x - origin_offset) / merged_map.info.resolution)
                grid_y = int((pos.y - origin_offset) / merged_map.info.resolution)
                
                # Mark robot position with different values for each robot (50, 60, 70)
                if (0 <= grid_x < merged_map.info.width and 
                    0 <= grid_y < merged_map.info.height):
                    robot_marker = 50 + (robot_id * 10)  # Different value for each robot
                    merged_i = grid_x + grid_y * merged_map.info.width
                    merged_map.data[merged_i] = robot_marker
                    
                    # Mark surrounding cells to make robot more visible (5x5 grid)
                    for dx in [-2, -1, 0, 1, 2]:
                        for dy in [-2, -1, 0, 1, 2]:
                            mark_x = grid_x + dx
                            mark_y = grid_y + dy
                            if (0 <= mark_x < merged_map.info.width and 
                                0 <= mark_y < merged_map.info.height):
                                mark_i = mark_x + mark_y * merged_map.info.width
                                # Center pixel gets the main value, surrounding pixels slightly different
                                if dx == 0 and dy == 0:
                                    merged_map.data[mark_i] = robot_marker
                                else:
                                    merged_map.data[mark_i] = robot_marker - 5
    
    return merged_map

class MergeMapNode(Node):
    def __init__(self):
        super().__init__('merge_map_node')
        
        # Declare parameters
        self.declare_parameter('map_size', 40.0)
        self.declare_parameter('origin_offset', -20.0)
        self.declare_parameter('dilation_size', 3)  # New parameter for dilation
        
        # Get parameters
        self.map_size = self.get_parameter('map_size').value
        self.origin_offset = self.get_parameter('origin_offset').value
        self.dilation_size = self.get_parameter('dilation_size').value
        
        # Create publisher for merged map
        self.publisher = self.create_publisher(OccupancyGrid, '/merge_map', 10)
        
        # Create subscriptions for maps
        self.subscription1 = self.create_subscription(
            OccupancyGrid, '/tb3_0/map', self.map1_callback, 10)
        self.subscription2 = self.create_subscription(
            OccupancyGrid, '/tb3_1/map', self.map2_callback, 10)
        self.subscription3 = self.create_subscription(
            OccupancyGrid, '/tb3_2/map', self.map3_callback, 10)
        
        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize storage for maps
        self.map1 = None
        self.map2 = None
        self.map3 = None
        
        # Create timer for periodic map merging
        self.timer = self.create_timer(1.0, self.merge_and_publish)
        
        self.get_logger().info('Map merge node initialized with TF listener and dilation')

    def map1_callback(self, msg):
        self.map1 = msg
        
    def map2_callback(self, msg):
        self.map2 = msg
        
    def map3_callback(self, msg):
        self.map3 = msg
    
    def get_robot_position(self, robot_name):
        """
        Get robot position from TF
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
            
            self.get_logger().info(f'{robot_name} position: x={pos.x:.2f}, y={pos.y:.2f}')
            
            return pos
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warning(f'Failed to get transform for {robot_name}: {str(e)}')
            return None
        
    def merge_and_publish(self):
        """
        Periodically merge maps and add robot positions from TF
        """
        if self.map1 is not None and self.map2 is not None and self.map3 is not None:
            robot_positions = [
                self.get_robot_position('tb3_0'),
                self.get_robot_position('tb3_1'),
                self.get_robot_position('tb3_2')
            ]
            
            merged_map = merge_maps(
                self.map1, 
                self.map2, 
                self.map3, 
                map_size=self.map_size,
                origin_offset=self.origin_offset,
                robot_positions=robot_positions,
                dilation_size=self.dilation_size  # Add dilation size parameter
            )
            
            if merged_map is not None:
                merged_map.header.stamp = self.get_clock().now().to_msg()
                self.publisher.publish(merged_map)
                self.get_logger().debug('Published merged map with robot positions and dilation')

def main(args=None):
    rclpy.init(args=args)
    merge_map_node = MergeMapNode()
    rclpy.spin(merge_map_node)
    merge_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()