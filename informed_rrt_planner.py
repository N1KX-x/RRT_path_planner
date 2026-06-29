import math
import random

from .config import (
    GRID_RESOLUTION,
    RRT_COLLISION_CHECK_STEP,
    RRT_GOAL_CONNECT_DISTANCE,
    RRT_GOAL_SAMPLE_RATE,
    RRT_MAX_ITERATIONS,
    RRT_REWIRE_RADIUS,
    RRT_SHORTCUT_ATTEMPTS,
    RRT_STEP_SIZE,
)


class _Node:
    """One node in the RRT* tree."""

    def __init__(self, row, col, parent=None, cost=0.0):
        self.row = float(row)
        self.col = float(col)
        self.parent = parent
        self.cost = float(cost)

    def cell(self):
        return int(round(self.row)), int(round(self.col))


class InformedRRTPlanner:
    """
    Informed RRT* planner for a 2D occupancy grid.

    Input:
        grid: 2D list where 0 means free and 1 means obstacle
        start: (row, col)
        goal: (row, col)

    Output:
        List of grid cells [(row, col), ...], or None if no path is found.

    This planner does not read LaserScan directly. LiDAR should first update the
    OccupancyGrid, then this planner searches through the resulting grid map.
    """

    def __init__(self):
        self.max_iterations = int(RRT_MAX_ITERATIONS)
        self.step_cells = max(1.0, float(RRT_STEP_SIZE) / float(GRID_RESOLUTION))
        self.goal_sample_rate = float(RRT_GOAL_SAMPLE_RATE)
        self.goal_connect_cells = max(1.0, float(RRT_GOAL_CONNECT_DISTANCE) / float(GRID_RESOLUTION))
        self.collision_step_cells = max(0.25, float(RRT_COLLISION_CHECK_STEP) / float(GRID_RESOLUTION))
        self.rewire_radius_cells = max(1.0, float(RRT_REWIRE_RADIUS) / float(GRID_RESOLUTION))
        self.shortcut_attempts = int(RRT_SHORTCUT_ATTEMPTS)
        self.rng = random.Random()

    def plan(self, grid, start, goal):
        """Run Informed RRT* and return a collision-free path in grid cells."""
        if not grid or not grid[0]:
            return None

        rows = len(grid)
        cols = len(grid[0])

        if not self.in_bounds(start, rows, cols):
            return None

        if not self.in_bounds(goal, rows, cols):
            return None

        if not self.is_free(grid, start):
            return None

        if not self.is_free(grid, goal):
            return None

        # Fast path: if the direct line is already clear, use it.
        if self.collision_free(grid, start, goal):
            return self.discretize_path([start, goal])

        start_node = _Node(start[0], start[1], parent=None, cost=0.0)
        nodes = [start_node]
        goal_node = None
        best_cost = math.inf

        c_min = self.distance(start, goal)

        for _ in range(self.max_iterations):
            sample = self.sample(grid, start, goal, best_cost, c_min)
            nearest = self.nearest_node(nodes, sample)
            new_point = self.steer((nearest.row, nearest.col), sample)
            new_cell = self.point_to_cell(new_point)

            if not self.in_bounds(new_cell, rows, cols):
                continue

            if not self.is_free(grid, new_cell):
                continue

            if new_cell == nearest.cell():
                continue

            if not self.collision_free(grid, nearest.cell(), new_cell):
                continue

            near_nodes = self.near_nodes(nodes, new_point)

            parent = nearest
            parent_cost = nearest.cost + self.distance(nearest.cell(), new_cell)

            # Choose the cheapest valid parent among nearby nodes.
            for near in near_nodes:
                near_cell = near.cell()

                if not self.collision_free(grid, near_cell, new_cell):
                    continue

                candidate_cost = near.cost + self.distance(near_cell, new_cell)

                if candidate_cost < parent_cost:
                    parent = near
                    parent_cost = candidate_cost

            new_node = _Node(new_point[0], new_point[1], parent=parent, cost=parent_cost)
            nodes.append(new_node)

            # Rewire nearby nodes through the new node when it improves cost.
            for near in near_nodes:
                if near is parent:
                    continue

                near_cell = near.cell()
                new_node_cell = new_node.cell()
                candidate_cost = new_node.cost + self.distance(new_node_cell, near_cell)

                if candidate_cost + 1e-9 >= near.cost:
                    continue

                if not self.collision_free(grid, new_node_cell, near_cell):
                    continue

                near.parent = new_node
                near.cost = candidate_cost

            # Try to connect the new node to the goal.
            if self.distance(new_node.cell(), goal) <= self.goal_connect_cells:
                if self.collision_free(grid, new_node.cell(), goal):
                    candidate_goal_cost = new_node.cost + self.distance(new_node.cell(), goal)

                    if candidate_goal_cost < best_cost:
                        goal_node = _Node(goal[0], goal[1], parent=new_node, cost=candidate_goal_cost)
                        best_cost = candidate_goal_cost

        if goal_node is None:
            return None

        path = self.reconstruct_path(goal_node)
        path = self.shortcut_path(grid, path)
        path = self.discretize_path(path)

        if len(path) == 0:
            return None

        return path

    def sample(self, grid, start, goal, best_cost, c_min):
        """Sample globally before a solution exists, then inside the informed ellipse."""
        if math.isfinite(best_cost):
            informed = self.sample_informed(grid, start, goal, best_cost, c_min)

            if informed is not None:
                return informed

        if self.rng.random() < self.goal_sample_rate:
            return float(goal[0]), float(goal[1])

        return self.sample_free(grid)

    def sample_free(self, grid):
        """Sample a random free grid point."""
        rows = len(grid)
        cols = len(grid[0])

        for _ in range(1000):
            row = self.rng.randrange(rows)
            col = self.rng.randrange(cols)

            if grid[row][col] == 0:
                return float(row), float(col)

        # Extremely full maps: fall back to any random point.
        return float(self.rng.randrange(rows)), float(self.rng.randrange(cols))

    def sample_informed(self, grid, start, goal, best_cost, c_min):
        """
        Sample inside the prolate ellipse whose foci are start and goal.

        Informed RRT* improves an existing solution by sampling only states that
        can possibly produce a shorter path than the current best path.
        """
        if not math.isfinite(best_cost):
            return None

        if best_cost <= 0.0:
            return None

        rows = len(grid)
        cols = len(grid[0])

        start_p = (float(start[0]), float(start[1]))
        goal_p = (float(goal[0]), float(goal[1]))

        center = (
            0.5 * (start_p[0] + goal_p[0]),
            0.5 * (start_p[1] + goal_p[1])
        )

        a = best_cost / 2.0
        b_sq = max(best_cost * best_cost - c_min * c_min, 0.0)
        b = math.sqrt(b_sq) / 2.0

        if c_min < 1e-9:
            cos_t = 1.0
            sin_t = 0.0
        else:
            # Direction from start to goal in row/col coordinates.
            cos_t = (goal_p[0] - start_p[0]) / c_min
            sin_t = (goal_p[1] - start_p[1]) / c_min

        for _ in range(100):
            # Uniform sample inside unit disk.
            radius = math.sqrt(self.rng.random())
            angle = self.rng.uniform(0.0, 2.0 * math.pi)

            x = radius * math.cos(angle)
            y = radius * math.sin(angle)

            # Scale by ellipse axes.
            local_x = a * x
            local_y = b * y

            # Rotate from ellipse frame into grid frame.
            row = center[0] + cos_t * local_x - sin_t * local_y
            col = center[1] + sin_t * local_x + cos_t * local_y
            cell = self.point_to_cell((row, col))

            if not self.in_bounds(cell, rows, cols):
                continue

            if grid[cell[0]][cell[1]] != 0:
                continue

            return row, col

        return None

    def nearest_node(self, nodes, point):
        """Return the tree node closest to point."""
        best_node = nodes[0]
        best_distance = math.inf

        for node in nodes:
            d = self.distance((node.row, node.col), point)

            if d < best_distance:
                best_distance = d
                best_node = node

        return best_node

    def near_nodes(self, nodes, point):
        """Return nearby nodes for RRT* parent selection and rewiring."""
        result = []

        for node in nodes:
            if self.distance((node.row, node.col), point) <= self.rewire_radius_cells:
                result.append(node)

        return result

    def steer(self, from_point, to_point):
        """Move from from_point toward to_point by at most step_cells."""
        d = self.distance(from_point, to_point)

        if d <= self.step_cells:
            return float(to_point[0]), float(to_point[1])

        ratio = self.step_cells / d
        row = from_point[0] + ratio * (to_point[0] - from_point[0])
        col = from_point[1] + ratio * (to_point[1] - from_point[1])

        return row, col

    def collision_free(self, grid, cell_a, cell_b):
        """Check whether the straight segment between two cells avoids obstacles."""
        rows = len(grid)
        cols = len(grid[0])

        if not self.in_bounds(cell_a, rows, cols):
            return False

        if not self.in_bounds(cell_b, rows, cols):
            return False

        if not self.is_free(grid, cell_a):
            return False

        if not self.is_free(grid, cell_b):
            return False

        row_a, col_a = cell_a
        row_b, col_b = cell_b

        d_row = row_b - row_a
        d_col = col_b - col_a
        distance = math.sqrt(d_row ** 2 + d_col ** 2)
        steps = max(1, int(math.ceil(distance / self.collision_step_cells)))

        previous_cell = cell_a

        for i in range(1, steps + 1):
            t = i / steps
            row = row_a + t * d_row
            col = col_a + t * d_col
            current_cell = self.point_to_cell((row, col))

            if not self.in_bounds(current_cell, rows, cols):
                return False

            if not self.is_free(grid, current_cell):
                return False

            # Extra corner check for diagonal cell transitions.
            if current_cell != previous_cell:
                pr, pc = previous_cell
                cr, cc = current_cell

                if pr != cr and pc != cc:
                    if not self.is_free(grid, (pr, cc)):
                        return False

                    if not self.is_free(grid, (cr, pc)):
                        return False

            previous_cell = current_cell

        return True

    def shortcut_path(self, grid, path):
        """Try random shortcuts to simplify the final path."""
        if len(path) <= 2:
            return path

        path = list(path)

        for _ in range(self.shortcut_attempts):
            if len(path) <= 2:
                break

            i = self.rng.randrange(0, len(path) - 1)
            j = self.rng.randrange(i + 1, len(path))

            if j <= i + 1:
                continue

            if self.collision_free(grid, path[i], path[j]):
                path = path[:i + 1] + path[j:]

        return path

    def discretize_path(self, path):
        """
        Fill long edges with intermediate cells.

        This makes downstream path-blocking checks more reliable and gives the
        waypoint follower enough points along each straight segment.
        """
        if len(path) == 0:
            return []

        result = [self.point_to_cell(path[0])]

        for point in path[1:]:
            start = result[-1]
            end = self.point_to_cell(point)

            row_a, col_a = start
            row_b, col_b = end

            d_row = row_b - row_a
            d_col = col_b - col_a
            steps = max(1, int(math.ceil(math.sqrt(d_row ** 2 + d_col ** 2))))

            for i in range(1, steps + 1):
                t = i / steps
                row = int(round(row_a + t * d_row))
                col = int(round(col_a + t * d_col))
                cell = (row, col)

                if cell != result[-1]:
                    result.append(cell)

        return result

    def reconstruct_path(self, goal_node):
        """Follow parent pointers from goal to start."""
        path = []
        current = goal_node

        while current is not None:
            path.append(current.cell())
            current = current.parent

        path.reverse()
        return path

    def point_to_cell(self, point):
        """Convert a continuous row/col point to an integer grid cell."""
        return int(round(point[0])), int(round(point[1]))

    def is_free(self, grid, cell):
        """Return True if a cell is inside the map and not occupied."""
        rows = len(grid)
        cols = len(grid[0])
        row, col = cell

        if row < 0 or row >= rows:
            return False

        if col < 0 or col >= cols:
            return False

        return grid[row][col] == 0

    def in_bounds(self, cell, rows, cols):
        """Check whether a cell is inside the grid."""
        row, col = cell
        return 0 <= row < rows and 0 <= col < cols

    def distance(self, a, b):
        """Euclidean distance in grid-cell units."""
        return math.sqrt((float(a[0]) - float(b[0])) ** 2 + (float(a[1]) - float(b[1])) ** 2)
