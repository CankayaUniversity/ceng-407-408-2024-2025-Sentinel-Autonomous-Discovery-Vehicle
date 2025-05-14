from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from path_finder_interfaces.srv import OdomToImage
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import base64
import io
import numpy as np
import rclpy
import math
from queue import PriorityQueue
from scipy import ndimage
import time

class PathFinder(Node):

    def __init__(self):
        super().__init__('path_finder_service')
        self.srv = self.create_service(OdomToImage, 'odom_to_image', self.handle_odom_to_image)
        self.map_data = None
        self.map_subscription = None
        self.get_logger().info("Service 'odom_to_image' is ready.")
        
        # Create map subscription at initialization
        self.create_map_subscription()

    def create_map_subscription(self):
        if self.map_subscription is None:
            self.get_logger().info("Creating map subscription to '/map' topic")
            self.map_subscription = self.create_subscription(
                OccupancyGrid,
                '/map',
                self.map_callback,
                10  # QoS profile depth
            )

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info(f'Received map: width={msg.info.width}, height={msg.info.height}')
        
        # Don't destroy the subscription immediately - keep listening for updates
        # self.destroy_subscription(self.map_subscription)
        # self.map_subscription = None
        # self.get_logger().info("Map subscription removed")

    def handle_odom_to_image(self, request, response):
        self.get_logger().info("Received request for odom_to_image")
        
        if self.map_data is None:
            self.get_logger().info("Map data not available, waiting for map...")
            
            # If not already subscribed, create subscription
            if self.map_subscription is None:
                self.create_map_subscription()
            
            # Increase timeout and add more frequent status updates
            wait_count = 0
            max_wait_count = 100  # Increased from 30 to 100 (10 seconds with 10Hz polling)
            rate = self.create_rate(10)  # 10Hz
            
            start_time = time.time()
            while self.map_data is None and wait_count < max_wait_count:
                if wait_count % 10 == 0:  # Log status every 1 second
                    self.get_logger().info(f"Waiting for map data... ({wait_count}/{max_wait_count})")
                
                rclpy.spin_once(self, timeout_sec=0.1)
                wait_count += 1
                
            elapsed = time.time() - start_time
            
            if self.map_data is None:
                self.get_logger().error(f"Failed to receive map data within {elapsed:.2f} seconds")
                response.image_data = ""
                return response
            else:
                self.get_logger().info(f"Successfully received map after {elapsed:.2f} seconds")
        
        odom_data = request.odom
        self.get_logger().info(f"Processing odometry at position: ({odom_data.pose.pose.position.x}, {odom_data.pose.pose.position.y})")
        
        # Generate path and create image
        image_base64 = self.process_path_planning(self.map_data, odom_data)
        response.image_data = image_base64
        
        return response

    def process_path_planning(self, map_data, odom_data):
        robot_width = 0.24  # Robot width in meters
        robot_length = 0.26  # Robot length in meters
        min_obstacle_size = 5  # Minimum obstacle size threshold
        
        self.get_logger().info(f"Planning with robot dimensions: {robot_width}m x {robot_length}m")
        
        # Create occupancy grid
        grid, resolution, origin, filtered_grid, initial_grid = self.create_occupancy_grid(
            map_data, robot_width, robot_length, min_obstacle_size
        )
        
        # Convert world coordinates to grid coordinates
        start_world = (0, 0, 0)  # Assuming start is at the origin
        goal_world = (
            odom_data.pose.pose.position.x,
            odom_data.pose.pose.position.y,
            0
        )
        
        start_grid = self.world_to_grid(
            start_world[0], start_world[1], origin.x, origin.y, resolution
        )
        goal_grid = self.world_to_grid(
            goal_world[0], goal_world[1], origin.x, origin.y, resolution
        )
        
        # Find nearest free space if points are in obstacles
        original_start = start_grid
        original_goal = goal_grid
        start_grid = self.find_nearest_free(grid, start_grid)
        goal_grid = self.find_nearest_free(grid, goal_grid)
        
        if original_start != start_grid:
            self.get_logger().info(f"Start point adjusted from {original_start} to {start_grid}")
        if original_goal != goal_grid:
            self.get_logger().info(f"Goal point adjusted from {original_goal} to {goal_grid}")
        
        # Calculate path using A* algorithm
        path = self.astar(grid, start_grid, goal_grid)
        
        # Generate visualization and return as base64 string
        return self.generate_visualization(map_data, path, start_grid, goal_grid)

    def create_occupancy_grid(self, map_data, robot_width=0.25, robot_length=0.24, min_obstacle_size=3):
        width = map_data.info.width
        height = map_data.info.height
        data = map_data.data
        resolution = map_data.info.resolution
        origin = map_data.info.origin.position
        
        map_array = np.array(data, dtype=np.int8).reshape(height, width)
        
        initial_grid = np.zeros((height, width), dtype=np.uint8)
        initial_grid[map_array == 100] = 1  # Occupied cells
        initial_grid[map_array == -1] = 1   # Unknown cells (treated as obstacles)
        
        labeled_array, num_features = ndimage.label(initial_grid)
        if num_features > 0:
            component_sizes = np.bincount(labeled_array.ravel())
            
            small_obstacles = np.zeros_like(labeled_array, dtype=bool)
            for label in range(1, num_features + 1):
                if component_sizes[label] < min_obstacle_size:
                    small_obstacles[labeled_array == label] = True
            
            filtered_grid = initial_grid.copy()
            filtered_grid[small_obstacles] = 0
        else:
            filtered_grid = initial_grid.copy()
        
        robot_radius = np.sqrt((robot_width/2)**2 + (robot_length/2)**2)
        inflation_cells = int(np.ceil(robot_radius / resolution))
        
        kernel = ndimage.generate_binary_structure(2, 2)
        inflated_grid = filtered_grid.copy()
        for _ in range(inflation_cells):
            inflated_grid = ndimage.binary_dilation(inflated_grid, kernel)
        
        inflated_grid = inflated_grid.astype(np.uint8)
        
        return inflated_grid, resolution, origin, filtered_grid, initial_grid

    def world_to_grid(self, x, y, origin_x, origin_y, resolution):
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y, origin_x, origin_y, resolution):
        world_x = grid_x * resolution + origin_x
        world_y = grid_y * resolution + origin_y
        return world_x, world_y

    def heuristic(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))  # Euclidean Distance

    def astar(self, grid, start, goal):
        neighbors = [(0,1),(1,0),(-1,0),(0,-1), (1,1), (-1,-1), (-1,1), (1,-1)]
        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: self.heuristic(start, goal)}
        oheap = PriorityQueue()
        oheap.put((fscore[start], 0, start))
        
        height, width = grid.shape
        counter = 1
        
        while not oheap.empty():
            _, _, current = oheap.get()
            
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]
            
            close_set.add(current)
            
            for dx, dy in neighbors:
                neighbor = (current[0]+dx, current[1]+dy)
                
                if not (0 <= neighbor[0] < width and 0 <= neighbor[1] < height and grid[neighbor[1], neighbor[0]] == 0):
                    continue
                
                move_cost = math.sqrt(2) if abs(dx) + abs(dy) == 2 else 1
                tentative_g_score = gscore[current] + move_cost
                
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, float('inf')):
                    continue
                
                if tentative_g_score < gscore.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    f = tentative_g_score + self.heuristic(neighbor, goal)
                    fscore[neighbor] = f
                    oheap.put((f, counter, neighbor))
                    counter += 1
        
        return None

    def find_nearest_free(self, grid, point, max_radius=20):
        x, y = point
        height, width = grid.shape
        
        if 0 <= y < height and 0 <= x < width and grid[y, x] == 0:
            return point
        
        dx, dy = 0, 0
        step = 1
        direction = 0  # 0: right, 1: down, 2: left, 3: up
        
        for _ in range(max_radius * max_radius * 4):
            if direction == 0:  # Move right
                dx += 1
                if dx == step:
                    direction = 1
            elif direction == 1:  # Move down
                dy += 1
                if dy == step:
                    direction = 2
            elif direction == 2:  # Move left
                dx -= 1
                if -dx == step:
                    direction = 3
            elif direction == 3:  # Move up
                dy -= 1
                if -dy == step:
                    direction = 0
                    step += 1  # Increase step for next spiral
            
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:
                if grid[ny, nx] == 0:
                    return (nx, ny)
            
            if abs(dx) > max_radius or abs(dy) > max_radius:
                break
        
        return point  # Return original point if no free cell found

    def generate_visualization(self, map_data, path, start, goal):
        width = map_data.info.width
        height = map_data.info.height
        data = map_data.data
        
        map_array = np.array(data).reshape(height, width)
        
        colors = [(200/255, 200/255, 210/255),  # unknown (-1)
                  (240/255, 248/255, 255/255),  # free (0)
                  (128/255, 148/255, 168/255),  # gradient mid
                  (0/255, 20/255, 40/255),      # gradient high
                  (0/255, 0/255, 0/255)]        # occupied (100)
        
        cmap = LinearSegmentedColormap.from_list('custom_map', colors)
        
        plt.ioff()  # Turn off interactive mode
        fig, ax = plt.subplots(figsize=(10, 8))
        
        vis_data = map_array.copy().astype(float)
        vis_data[map_array == -1] = 0.0
        vis_data[map_array == 0] = 0.25
        vis_data[map_array == 100] = 1.0
        
        # Handle gradient values between 0 and 100
        mask = (map_array > 0) & (map_array < 100)
        if np.any(mask):
            vis_data[mask] = 0.3 + (map_array[mask] / 100) * 0.45
        
        ax.imshow(vis_data, cmap=cmap, interpolation='none')
        
        if path:  # Draw path on the map
            path_x = [p[0] for p in path]
            path_y = [p[1] for p in path]
            ax.plot(path_x, path_y, 'g-', linewidth=2, alpha=0.7)
        
        ax.plot(start[0], start[1], 'bo', markersize=8)
        ax.plot(goal[0], goal[1], 'ro', markersize=8)
        
        from matplotlib.lines import Line2D
        
        legend_elements = [
            Line2D([0], [0], color='green', lw=2, label='Path'),
            Line2D([0], [0], marker='o', color='blue', label='Start', 
                   markerfacecolor='blue', markersize=8),
            Line2D([0], [0], marker='o', color='red', label='Goal', 
                   markerfacecolor='red', markersize=8)
        ]
        
        ax.legend(handles=legend_elements, loc='upper right')
        
        ax.set_title('Map with Path')
        ax.axis('off')
        
        plt.tight_layout()
        
        buf = io.BytesIO()
        fig.savefig(buf, format='png', dpi=150, bbox_inches='tight')
        plt.close(fig)
        
        buf.seek(0)
        img_base64 = base64.b64encode(buf.read()).decode('utf-8')
        
        return img_base64