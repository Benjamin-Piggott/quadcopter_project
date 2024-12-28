#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import heapq
import numpy as np
from std_msgs.msg import Int8MultiArray, Float32MultiArray
from quadcopter_interfaces.srv import FindPath, GetGrid
from geometry_msgs.msg import Point

class PathfindingService(Node):
    def __init__(self):
        super().__init__('pathfinding_service_node')
        
        # Create pathfinding service
        self.pathfinding_service = self.create_service(
            FindPath,
            '/quadcopter/find_path', 
            self.handle_pathfinding_request
        )
        
        # Publisher for path
        self.path_pub = self.create_publisher(
            Float32MultiArray, 
            '/quadcopter/optimal_path', 
            10
        )
        
        self.get_logger().info("Pathfinding Service Node Initialized")
    
    def handle_pathfinding_request(self, request, response):
        """
        Service handler for pathfinding
        """
        try:
            # Request grid from Grid Service
            client = self.create_client(GetGrid, '/quadcopter/get_grid')
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Grid service not available, waiting...')
            
            grid_request = GetGrid.Request()
            grid_future = client.call_async(grid_request)
            rclpy.spin_until_future_complete(self, grid_future)
            grid_response = grid_future.result()
            
            # Convert grid message to 2D numpy array
            grid_data = grid_response.grid.data
            grid_rows = grid_response.grid.layout.dim[0].size
            grid_cols = grid_response.grid.layout.dim[1].size
            grid = np.array(grid_data).reshape((grid_rows, grid_cols))
            
            # Define start and end points
            start = (request.start_x, request.start_y)
            end = (request.end_x, request.end_y)
            
            # Perform A* pathfinding
            path = self.a_star_search(grid, start, end)
            
            if path:
                # Prepare response
                response.path_x = [float(p[0]) for p in path]
                response.path_y = [float(p[1]) for p in path]
                response.path_found = True
                
                # Publish path as a Float32MultiArray
                path_msg = Float32MultiArray()
                path_msg.data = [coord for point in path for coord in point]
                self.path_pub.publish(path_msg)
            else:
                response.path_found = False
            
            return response
        
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            response.path_found = False
            return response
    
    def a_star_search(self, grid, src, dest):
        """
        A* pathfinding algorithm implementation
        
        Args:
            grid (numpy.ndarray): 2D grid where 1 represents traversable cells and 0 represents obstacles
            src (tuple): Starting coordinates (x, y)
            dest (tuple): Destination coordinates (x, y)
            
        Returns:
            list: List of coordinates representing the path, or None if no path found
        """
        rows, cols = grid.shape
        
        # Helper functions for path validation
        def is_valid(row, col):
            return 0 <= row < rows and 0 <= col < cols
        
        def is_unblocked(row, col):
            return grid[row, col] == 1
        
        def calculate_h_value(row, col):
            return math.sqrt((row - dest[0]) ** 2 + (col - dest[1]) ** 2)
        
        # Initialize closed list
        closed_list = np.zeros_like(grid, dtype=bool)
        
        # Cell class to store path information
        class Cell:
            def __init__(self):
                self.parent_i = 0
                self.parent_j = 0
                self.f = float("inf")
                self.g = float("inf")
                self.h = 0.0
        
        # Initialize cell details
        cell_details = np.full((rows, cols), None)
        for i in range(rows):
            for j in range(cols):
                cell_details[i, j] = Cell()
        
        # Initialize start cell
        i, j = src
        cell_details[i, j].f = 0.0
        cell_details[i, j].g = 0.0
        cell_details[i, j].h = 0.0
        cell_details[i, j].parent_i = i
        cell_details[i, j].parent_j = j
        
        # Initialize open list (priority queue)
        # Format: (f_value, row, col)
        open_list = []
        heapq.heappush(open_list, (0.0, i, j))
        
        # Define possible movement directions (8-direction movement)
        directions = [
            (0, 1),   # right
            (0, -1),  # left
            (1, 0),   # down
            (-1, 0),  # up
            (1, 1),   # diagonal down-right
            (1, -1),  # diagonal down-left
            (-1, 1),  # diagonal up-right
            (-1, -1)  # diagonal up-left
        ]
        
        # Main search loop
        while open_list:
            # Get cell with minimum f_value
            f_val, i, j = heapq.heappop(open_list)
            closed_list[i, j] = True
            
            # Check all adjacent cells
            for di, dj in directions:
                new_i, new_j = i + di, j + dj
                
                # Skip invalid or blocked cells
                if not (is_valid(new_i, new_j) and 
                       is_unblocked(new_i, new_j) and 
                       not closed_list[new_i, new_j]):
                    continue
                
                # Check if destination is reached
                if (new_i, new_j) == dest:
                    cell_details[new_i, new_j].parent_i = i
                    cell_details[new_i, new_j].parent_j = j
                    return self.trace_path(cell_details, dest)
                
                # Calculate new path costs
                g_new = cell_details[i, j].g + (1.0 if di * dj == 0 else 1.414)
                h_new = calculate_h_value(new_i, new_j)
                f_new = g_new + h_new
                
                # Update cell if better path found
                if (cell_details[new_i, new_j].f == float("inf") or 
                    cell_details[new_i, new_j].f > f_new):
                    heapq.heappush(open_list, (f_new, new_i, new_j))
                    
                    cell_details[new_i, new_j].f = f_new
                    cell_details[new_i, new_j].g = g_new
                    cell_details[new_i, new_j].h = h_new
                    cell_details[new_i, new_j].parent_i = i
                    cell_details[new_i, new_j].parent_j = j
        
        self.get_logger().warn("No path found")
        return None
    
    def trace_path(self, cell_details, dest):
        """
        Reconstruct path from destination back to start
        
        Args:
            cell_details (numpy.ndarray): Array of Cell objects containing path information
            dest (tuple): Destination coordinates (x, y)
            
        Returns:
            list: List of coordinates representing the path from start to destination
        """
        path = []
        row, col = dest
        
        while not (cell_details[row, col].parent_i == row and 
                  cell_details[row, col].parent_j == col):
            path.append((row, col))
            temp_row = cell_details[row, col].parent_i
            temp_col = cell_details[row, col].parent_j
            row = temp_row
            col = temp_col
        
        path.append((row, col))
        path.reverse()
        
        return path

def main(args=None):
    rclpy.init(args=args)
    pathfinding_service = PathfindingService()
    
    try:
        rclpy.spin(pathfinding_service)
    except KeyboardInterrupt:
        pass
    finally:
        pathfinding_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()