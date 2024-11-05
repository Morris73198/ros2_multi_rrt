import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

def merge_maps(*maps):
    """
    Merge multiple maps into one
    Args:
        *maps: Variable number of OccupancyGrid maps
    Returns:
        merged_map: Combined OccupancyGrid map
    """
    if not maps:
        return None
    
    merged_map = OccupancyGrid()
    merged_map.header = maps[0].header
    merged_map.header.frame_id = 'merge_map'
    
    # Find the bounds of all maps
    min_x = min(map.info.origin.position.x for map in maps)
    min_y = min(map.info.origin.position.y for map in maps)
    max_x = max(map.info.origin.position.x + (map.info.width * map.info.resolution) 
                for map in maps)
    max_y = max(map.info.origin.position.y + (map.info.height * map.info.resolution) 
                for map in maps)
    
    # Set merged map properties
    merged_map.info.origin.position.x = min_x
    merged_map.info.origin.position.y = min_y
    merged_map.info.resolution = min(map.info.resolution for map in maps)
    
    # Calculate dimensions
    merged_map.info.width = int(np.ceil((max_x - min_x) / merged_map.info.resolution))
    merged_map.info.height = int(np.ceil((max_y - min_y) / merged_map.info.resolution))
    
    # Initialize merged map data
    merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)
    
    # Merge each map
    for map_data in maps:
        for y in range(map_data.info.height):
            for x in range(map_data.info.width):
                i = x + y * map_data.info.width
                merged_x = int(np.floor((map_data.info.origin.position.x + 
                                       x * map_data.info.resolution - min_x) / 
                                      merged_map.info.resolution))
                merged_y = int(np.floor((map_data.info.origin.position.y + 
                                       y * map_data.info.resolution - min_y) / 
                                      merged_map.info.resolution))
                merged_i = merged_x + merged_y * merged_map.info.width
                
                # Only update if current cell is unknown (-1)
                if merged_map.data[merged_i] == -1:
                    merged_map.data[merged_i] = map_data.data[i]
                # If both cells are occupied (100), keep occupied
                elif merged_map.data[merged_i] == 100 and map_data.data[i] == 100:
                    merged_map.data[merged_i] = 100
                # If one cell is free (0) and other is occupied (100), prefer occupied
                elif (merged_map.data[merged_i] == 0 and map_data.data[i] == 100) or \
                     (merged_map.data[merged_i] == 100 and map_data.data[i] == 0):
                    merged_map.data[merged_i] = 100
    
    return merged_map

class MergeMapNode(Node):
    def __init__(self):
        super().__init__('merge_map_node')
        
        # Create publisher for merged map
        self.publisher = self.create_publisher(OccupancyGrid, '/merge_map', 10)
        
        # Create subscriptions for all three robots
        self.subscription1 = self.create_subscription(
            OccupancyGrid, '/tb3_0/map', self.map1_callback, 10)
        self.subscription2 = self.create_subscription(
            OccupancyGrid, '/tb3_1/map', self.map2_callback, 10)
        self.subscription3 = self.create_subscription(
            OccupancyGrid, '/tb3_2/map', self.map3_callback, 10)
        
        # Initialize map storage
        self.map1 = None
        self.map2 = None
        self.map3 = None
        
        # Create timer for periodic map merging
        self.timer = self.create_timer(1.0, self.merge_and_publish)
        
        self.get_logger().info('Map merge node initialized for three robots')

    def map1_callback(self, msg):
        self.map1 = msg
        
    def map2_callback(self, msg):
        self.map2 = msg
        
    def map3_callback(self, msg):
        self.map3 = msg
        
    def merge_and_publish(self):
        """
        Periodically check if we have all maps and merge them if we do
        """
        if self.map1 is not None and self.map2 is not None and self.map3 is not None:
            merged_map = merge_maps(self.map1, self.map2, self.map3)
            if merged_map is not None:
                self.publisher.publish(merged_map)
                self.get_logger().debug('Published merged map')

def main(args=None):
    rclpy.init(args=args)
    merge_map_node = MergeMapNode()
    rclpy.spin(merge_map_node)
    merge_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()