#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, MultiArrayDimension
from quadcopter_interfaces.srv import GetGrid

class GridService(Node):
    def __init__(self):
        super().__init__('grid_service_node')
        
        # Create a service to provide grid configuration
        self.grid_service = self.create_service(
            GetGrid, 
            '/quadcopter/get_grid', 
            self.handle_grid_request
        )
        
        # Grid configuration
        self.grid = [
            [1, 1, 1, 1, 1],
            [1, 0, 1, 1, 1],
            [1, 1, 1, 0, 1],
            [1, 0, 1, 1, 1],
            [1, 1, 1, 1, 1]
        ]
        
        self.get_logger().info("Grid Service Node Initialized")
    
    def handle_grid_request(self, request, response):
        """
        Service handler to provide grid configuration
        """
        # Convert grid to Int8MultiArray
        grid_msg = Int8MultiArray()
        
        # Set up dimensions
        dim1 = MultiArrayDimension()
        dim1.label = "rows"
        dim1.size = len(self.grid)
        dim1.stride = len(self.grid[0])
        
        dim2 = MultiArrayDimension()
        dim2.label = "columns"
        dim2.size = len(self.grid[0])
        dim2.stride = 1
        
        grid_msg.layout.dim = [dim1, dim2]
        
        # Flatten grid
        grid_msg.data = [cell for row in self.grid for cell in row]
        
        response.grid = grid_msg
        return response

def main(args=None):
    rclpy.init(args=args)
    grid_service = GridService()
    try:
        rclpy.spin(grid_service)
    except KeyboardInterrupt:
        pass
    finally:
        grid_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




