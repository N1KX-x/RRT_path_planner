import math

from .config import MAX_LIDAR_OBSTACLE_RANGE
from .utils import lidar_range_to_meters


class OccupancyGrid:
    """Local grid map where 0 means free space and 1 means obstacle."""

    def __init__(self, width_m=6.0, height_m=6.0, resolution=0.1, inflation_radius=0.2):
        """Create an empty map centered on the robot's starting position."""
        self.width_m = width_m
        self.height_m = height_m
        self.resolution = resolution
        self.inflation_radius = inflation_radius

        self.cols = int(width_m / resolution)
        self.rows = int(height_m / resolution)

        # The robot starts in the middle of the local map.
        self.origin_row = self.rows // 2
        self.origin_col = self.cols // 2

        self.grid = []
        for _ in range(self.rows):
            self.grid.append([0 for _ in range(self.cols)])

    def world_to_grid(self, x, y):
        """Convert local world coordinates in meters into a grid row and column."""
        col = int(round(x / self.resolution)) + self.origin_col
        row = self.origin_row - int(round(y / self.resolution))

        if row < 0 or row >= self.rows:
            return None

        if col < 0 or col >= self.cols:
            return None

        return row, col

    def grid_to_world(self, cell):
        """Convert a grid cell back into local world coordinates in meters."""
        row, col = cell

        x = (col - self.origin_col) * self.resolution
        y = (self.origin_row - row) * self.resolution

        return x, y

    def is_free_cell(self, cell):
        """Return True when a grid cell is inside the map and not occupied."""
        row, col = cell

        if row < 0 or row >= self.rows:
            return False

        if col < 0 or col >= self.cols:
            return False

        return self.grid[row][col] == 0

    def mark_free_cell(self, cell):
        """Set one grid cell to free if it is inside the map."""
        if cell is None:
            return

        row, col = cell

        if row < 0 or row >= self.rows:
            return

        if col < 0 or col >= self.cols:
            return

        self.grid[row][col] = 0

    def mark_obstacle_world(self, x, y):
        """Mark an obstacle using local world coordinates."""
        cell = self.world_to_grid(x, y)

        if cell is not None:
            self.inflate_obstacle(cell)

    def inflate_obstacle(self, center_cell):
        """Mark an obstacle plus a safety radius around it."""
        center_row, center_col = center_cell
        inflation_cells = int(math.ceil(self.inflation_radius / self.resolution))

        # Mark nearby cells too, so the planner leaves room for the robot body.
        for dr in range(-inflation_cells, inflation_cells + 1):
            for dc in range(-inflation_cells, inflation_cells + 1):
                row = center_row + dr
                col = center_col + dc

                if row < 0 or row >= self.rows:
                    continue

                if col < 0 or col >= self.cols:
                    continue

                distance = math.sqrt(dr ** 2 + dc ** 2) * self.resolution

                if distance <= self.inflation_radius:
                    self.grid[row][col] = 1

    def clear_ray(self, robot_x, robot_y, angle, distance, leave_end_cells=2):
        """Clear cells along a LiDAR ray before the measured obstacle point."""
        if distance <= 0.0:
            return

        step = self.resolution * 0.5
        num_steps = int(distance / step)

        if num_steps <= 0:
            return

        end_step = max(0, num_steps - leave_end_cells)

        for i in range(end_step):
            d = i * step
            x = robot_x + d * math.cos(angle)
            y = robot_y + d * math.sin(angle)
            self.mark_free_cell(self.world_to_grid(x, y))

    def update_from_lidar(self, scan_msg, robot_x, robot_y, robot_theta):
        """Use one LaserScan message to clear visible free space and mark obstacles."""
        # Clear free space along each ray first, then mark the obstacle endpoints.
        obstacle_points = []
        angle = scan_msg.angle_min

        for raw_r in scan_msg.ranges:
            world_angle = robot_theta + angle

            if math.isinf(raw_r) or math.isnan(raw_r):
                self.clear_ray(
                    robot_x=robot_x,
                    robot_y=robot_y,
                    angle=world_angle,
                    distance=min(MAX_LIDAR_OBSTACLE_RANGE, scan_msg.range_max),
                    leave_end_cells=0
                )
                angle += scan_msg.angle_increment
                continue

            r = lidar_range_to_meters(raw_r)

            if r <= 0.0:
                angle += scan_msg.angle_increment
                continue

            if r > MAX_LIDAR_OBSTACLE_RANGE:
                self.clear_ray(
                    robot_x=robot_x,
                    robot_y=robot_y,
                    angle=world_angle,
                    distance=MAX_LIDAR_OBSTACLE_RANGE,
                    leave_end_cells=0
                )
                angle += scan_msg.angle_increment
                continue

            self.clear_ray(
                robot_x=robot_x,
                robot_y=robot_y,
                angle=world_angle,
                distance=r,
                leave_end_cells=2
            )

            obstacle_x = robot_x + r * math.cos(world_angle)
            obstacle_y = robot_y + r * math.sin(world_angle)
            obstacle_points.append((obstacle_x, obstacle_y))

            angle += scan_msg.angle_increment

        for obstacle_x, obstacle_y in obstacle_points:
            self.mark_obstacle_world(obstacle_x, obstacle_y)
