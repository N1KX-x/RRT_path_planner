import json
import math
import threading
import time
from dataclasses import dataclass, field

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from .config import (
    MULTIBOT_CONTROL_TOPIC,
    MULTIBOT_COORDINATOR_PERIOD,
    MULTIBOT_RESUME_HYSTERESIS,
    MULTIBOT_SAFETY_MARGIN,
    MULTIBOT_SLOW_LINEAR_SPEED,
    MULTIBOT_START_TOPIC,
    MULTIBOT_STATE_TIMEOUT,
    MULTIBOT_STATE_TOPIC,
    ROBOT_RADIUS,
    TRAFFIC_GRID_RESOLUTION,
    TRAFFIC_LOOKAHEAD_CELLS,
    TRAFFIC_YELLOW_RADIUS_CELLS,
)


COMMAND_SEVERITY = {
    "RUN": 0,
    "SLOW": 1,
    "HOLD": 2,
    "EMERGENCY_STOP": 3,
}


@dataclass
class RobotTrafficState:
    """Latest central-coordinator view of one robot."""

    robot_id: str
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    linear_x: float = 0.0
    angular_z: float = 0.0
    robot_radius: float = ROBOT_RADIUS
    goal_x: float = 0.0
    goal_y: float = 0.0
    active_goal_x: float = 0.0
    active_goal_y: float = 0.0
    pose_ready: bool = False
    started: bool = False
    status: str = "UNKNOWN"
    path: list = field(default_factory=list)
    goal_setup_ready: bool = False
    goal_setup_id: str = ""
    last_seen: float = 0.0


class MultibotCoordinator(Node):
    """
    Central ATP-style traffic supervisor for multiple TurtleBots.

    Before START, this node distributes all robot goals so each local RRT map
    can reserve the other robots' endpoints. During motion, it publishes RUN,
    HOLD, SLOW, or EMERGENCY_STOP commands for live traffic conflicts.
    """

    def __init__(self):
        super().__init__("multibot_coordinator")

        self.declare_parameter("robot_ids", "")
        self.declare_parameter("state_topic", MULTIBOT_STATE_TOPIC)
        self.declare_parameter("start_topic", MULTIBOT_START_TOPIC)
        self.declare_parameter("control_topic", MULTIBOT_CONTROL_TOPIC)
        self.declare_parameter("traffic_grid_resolution", TRAFFIC_GRID_RESOLUTION)
        self.declare_parameter("yellow_radius_cells", TRAFFIC_YELLOW_RADIUS_CELLS)
        self.declare_parameter("lookahead_cells", TRAFFIC_LOOKAHEAD_CELLS)
        self.declare_parameter("safety_margin", MULTIBOT_SAFETY_MARGIN)
        self.declare_parameter("resume_hysteresis", MULTIBOT_RESUME_HYSTERESIS)
        self.declare_parameter("state_timeout", MULTIBOT_STATE_TIMEOUT)
        self.declare_parameter("slow_linear_speed", MULTIBOT_SLOW_LINEAR_SPEED)
        self.declare_parameter("fairness_wait_seconds", 3.0)

        self.robot_filter = self.parse_robot_ids(
            self.get_parameter("robot_ids").value
        )
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.start_topic = str(self.get_parameter("start_topic").value)
        self.control_topic = str(self.get_parameter("control_topic").value)
        self.traffic_grid_resolution = float(
            self.get_parameter("traffic_grid_resolution").value
        )
        self.yellow_radius_cells = int(
            self.get_parameter("yellow_radius_cells").value
        )
        self.lookahead_cells = int(self.get_parameter("lookahead_cells").value)
        self.safety_margin = float(self.get_parameter("safety_margin").value)
        self.resume_hysteresis = float(
            self.get_parameter("resume_hysteresis").value
        )
        self.state_timeout = float(self.get_parameter("state_timeout").value)
        self.slow_linear_speed = float(
            self.get_parameter("slow_linear_speed").value
        )
        self.fairness_wait_seconds = float(
            self.get_parameter("fairness_wait_seconds").value
        )

        self.robot_states = {}
        self.last_commands = {}
        self.hold_since = {}
        self.start_requested = False
        self.setup_ready = False
        self.setup_id = ""
        self.setup_ready_event = threading.Event()
        self.last_status_log = 0.0

        self.state_sub = self.create_subscription(
            String,
            self.state_topic,
            self.robot_state_callback,
            10
        )
        self.start_pub = self.create_publisher(String, self.start_topic, 10)
        self.control_pub = self.create_publisher(String, self.control_topic, 10)
        self.timer = self.create_timer(
            MULTIBOT_COORDINATOR_PERIOD,
            self.control_loop
        )

        self.input_thread = threading.Thread(
            target=self.prompt_for_start,
            daemon=True
        )
        self.input_thread.start()

        self.get_logger().info(
            "Multibot coordinator ready. Waiting for robot states and user START."
        )

    def parse_robot_ids(self, value):
        """Return an optional set of robot IDs to control."""
        if value is None:
            return set()

        if isinstance(value, (list, tuple)):
            return set(str(item).strip() for item in value if str(item).strip())

        text = str(value).strip()

        if not text:
            return set()

        text = text.replace("[", "").replace("]", "").replace("'", "")
        text = text.replace('"', "")

        return set(item.strip() for item in text.split(",") if item.strip())

    def prompt_for_start(self):
        """Wait for the operator's command-line confirmation."""
        self.setup_ready_event.wait()

        if not rclpy.ok():
            return

        print("")
        print("Initial Setup is Ready, Goal Point Marked, Press Y to Run")

        while rclpy.ok():
            try:
                answer = input("> ").strip().lower()
            except EOFError:
                return

            if answer in ("y", "yes", "start"):
                self.start_requested = True
                print("START sent. Coordinator is supervising traffic.")
                return

            if answer in ("q", "quit", "exit"):
                print("No START sent. Shut down with Ctrl+C when ready.")
                return

            print("Press Y to run, or Ctrl+C to exit.")

    def robot_state_callback(self, msg):
        """Read one robot state JSON message."""
        try:
            data = json.loads(msg.data)
        except ValueError:
            self.get_logger().warn("Ignoring invalid robot_state JSON.")
            return

        if not isinstance(data, dict):
            return

        robot_id = str(data.get("robot_id", "")).strip()

        if not robot_id:
            return

        if self.robot_filter and robot_id not in self.robot_filter:
            return

        state = self.robot_states.get(robot_id, RobotTrafficState(robot_id))
        state.x = self.get_float(data, "x", state.x)
        state.y = self.get_float(data, "y", state.y)
        state.theta = self.get_float(data, "theta", state.theta)
        state.linear_x = self.get_float(data, "linear_x", 0.0)
        state.angular_z = self.get_float(data, "angular_z", 0.0)
        state.robot_radius = self.get_float(
            data,
            "robot_radius",
            state.robot_radius
        )
        state.goal_x = self.get_float(data, "goal_x", state.goal_x)
        state.goal_y = self.get_float(data, "goal_y", state.goal_y)
        state.active_goal_x = self.get_float(
            data,
            "active_goal_x",
            state.goal_x
        )
        state.active_goal_y = self.get_float(
            data,
            "active_goal_y",
            state.goal_y
        )
        state.pose_ready = bool(data.get("pose_ready", False))
        state.started = bool(data.get("started", False))
        state.status = str(data.get("status", "UNKNOWN"))
        state.path = self.parse_path(data.get("path", []))
        state.goal_setup_ready = bool(data.get("goal_setup_ready", False))
        state.goal_setup_id = str(data.get("goal_setup_id", ""))
        state.last_seen = time.monotonic()

        self.robot_states[robot_id] = state

    def get_float(self, data, key, default):
        """Read a float from JSON data with a fallback."""
        try:
            return float(data.get(key, default))
        except (TypeError, ValueError):
            return default

    def parse_path(self, raw_path):
        """Convert a JSON path list to [(x, y), ...]."""
        path = []

        if not isinstance(raw_path, list):
            return path

        for point in raw_path:
            if not isinstance(point, dict):
                continue

            try:
                path.append((float(point["x"]), float(point["y"])))
            except (KeyError, TypeError, ValueError):
                continue

        return path

    def control_loop(self):
        """Publish central safety commands for all live robots."""
        now = time.monotonic()
        active_states = self.get_active_states(now)

        if not self.start_requested:
            self.update_initial_goal_setup(active_states)

        if self.start_requested:
            self.publish_start()

        commands = {}

        for robot_id in active_states:
            if self.start_requested:
                commands[robot_id] = self.make_command("RUN", "clear")
            else:
                commands[robot_id] = self.make_command("HOLD", "waiting_start")

        if self.start_requested:
            self.apply_traffic_rules(active_states, commands, now)

        self.publish_commands(commands, now)
        self.log_status(active_states, commands, now)

    def update_initial_goal_setup(self, states):
        """Send all final goals to each robot and wait for matching acknowledgements."""
        expected_ids = self.robot_filter or set(states)

        if len(expected_ids) < 2 or not expected_ids.issubset(states):
            return

        goals = [
            {
                "robot_id": robot_id,
                "x": states[robot_id].goal_x,
                "y": states[robot_id].goal_y,
                "robot_radius": states[robot_id].robot_radius,
            }
            for robot_id in sorted(expected_ids)
        ]
        setup = {
            "goals": goals,
            "safety_margin": self.safety_margin,
        }
        setup_id = json.dumps(setup, sort_keys=True, separators=(",", ":"))

        if setup_id != self.setup_id:
            self.setup_id = setup_id
            self.setup_ready = False
            self.setup_ready_event.clear()

        for robot_id in sorted(expected_ids):
            payload = {
                "command": "SETUP_GOALS",
                "robot_id": robot_id,
                "setup_id": self.setup_id,
                "goals": goals,
                "safety_margin": self.safety_margin,
            }
            msg = String()
            msg.data = json.dumps(payload, separators=(",", ":"))
            self.control_pub.publish(msg)

        all_ready = all(
            states[robot_id].goal_setup_ready
            and states[robot_id].goal_setup_id == self.setup_id
            for robot_id in expected_ids
        )

        if all_ready and not self.setup_ready:
            self.setup_ready = True
            self.setup_ready_event.set()

    def get_active_states(self, now):
        """Return pose-ready states that have not timed out."""
        active = {}

        for robot_id, state in self.robot_states.items():
            if not state.pose_ready:
                continue

            if now - state.last_seen > self.state_timeout:
                continue

            active[robot_id] = state

        return active

    def publish_start(self):
        """Publish START repeatedly so bridged domains and late nodes receive it."""
        msg = String()
        msg.data = json.dumps({"command": "START"}, separators=(",", ":"))
        self.start_pub.publish(msg)

    def apply_traffic_rules(self, states, commands, now):
        """Apply red-zone block control and yellow-zone distance checks."""
        red_cells = {}
        yellow_cells = {}
        future_cells = {}

        for robot_id, state in states.items():
            red_cells[robot_id] = self.footprint_cells(state)
            yellow_cells[robot_id] = self.expand_cells(
                red_cells[robot_id],
                self.yellow_radius_cells
            )
            future_cells[robot_id] = self.future_path_cells(state)

        robot_ids = sorted(states.keys())

        for index, robot_a_id in enumerate(robot_ids):
            for robot_b_id in robot_ids[index + 1:]:
                state_a = states[robot_a_id]
                state_b = states[robot_b_id]

                self.apply_pair_rules(
                    state_a,
                    state_b,
                    red_cells,
                    yellow_cells,
                    future_cells,
                    commands,
                    now
                )

    def apply_pair_rules(
        self,
        state_a,
        state_b,
        red_cells,
        yellow_cells,
        future_cells,
        commands,
        now
    ):
        """Apply safety rules for one robot pair."""
        robot_a_id = state_a.robot_id
        robot_b_id = state_b.robot_id

        red_overlap = red_cells[robot_a_id] & red_cells[robot_b_id]

        if red_overlap:
            winner_id = self.choose_priority(state_a, state_b, now)
            loser_id = robot_b_id if winner_id == robot_a_id else robot_a_id
            self.raise_command(
                commands,
                loser_id,
                "EMERGENCY_STOP",
                f"red_overlap_with_{winner_id}",
                winner_id
            )
            return

        a_enters_b_red = future_cells[robot_a_id] & red_cells[robot_b_id]
        b_enters_a_red = future_cells[robot_b_id] & red_cells[robot_a_id]

        # A one-way red-cell conflict has an inherent right of way: the robot
        # that already occupies the cell must be allowed to clear it.  Do not
        # let fairness reverse this ordering after the approaching robot has
        # waited for a few seconds, otherwise the approaching robot remains
        # held by this rule while a later rule also holds the leading robot.
        pair_winner_id = None
        pair_loser_id = None

        if a_enters_b_red and not b_enters_a_red:
            pair_winner_id = robot_b_id
            pair_loser_id = robot_a_id
        elif b_enters_a_red and not a_enters_b_red:
            pair_winner_id = robot_a_id
            pair_loser_id = robot_b_id

        if a_enters_b_red and b_enters_a_red:
            winner_id = self.choose_priority(state_a, state_b, now)
            loser_id = robot_b_id if winner_id == robot_a_id else robot_a_id
            self.raise_command(
                commands,
                loser_id,
                "HOLD",
                f"red_cell_priority_to_{winner_id}",
                winner_id
            )
        elif a_enters_b_red:
            self.raise_command(
                commands,
                robot_a_id,
                "HOLD",
                f"red_cell_owned_by_{robot_b_id}",
                robot_b_id
            )
        elif b_enters_a_red:
            self.raise_command(
                commands,
                robot_b_id,
                "HOLD",
                f"red_cell_owned_by_{robot_a_id}",
                robot_a_id
            )

        future_overlap = future_cells[robot_a_id] & future_cells[robot_b_id]

        if future_overlap:
            winner_id = pair_winner_id or self.choose_priority(
                state_a,
                state_b,
                now
            )
            loser_id = pair_loser_id or (
                robot_b_id if winner_id == robot_a_id else robot_a_id
            )
            self.raise_command(
                commands,
                loser_id,
                "HOLD",
                f"future_cell_reserved_by_{winner_id}",
                winner_id
            )

        pair_in_yellow = (
            (red_cells[robot_a_id] | future_cells[robot_a_id])
            & yellow_cells[robot_b_id]
        ) or (
            (red_cells[robot_b_id] | future_cells[robot_b_id])
            & yellow_cells[robot_a_id]
        )

        if not pair_in_yellow:
            return

        distance = self.distance_between(state_a, state_b)
        stop_distance = (
            state_a.robot_radius +
            state_b.robot_radius +
            self.safety_margin
        )
        resume_distance = stop_distance + self.resume_hysteresis
        already_holding = self.pair_was_holding(robot_a_id, robot_b_id)

        if already_holding:
            conflict = distance < resume_distance
        else:
            conflict = distance <= stop_distance

        if not conflict:
            return

        winner_id = pair_winner_id or self.choose_priority(
            state_a,
            state_b,
            now
        )
        loser_id = pair_loser_id or (
            robot_b_id if winner_id == robot_a_id else robot_a_id
        )
        self.raise_command(
            commands,
            loser_id,
            "HOLD",
            f"yellow_distance_{distance:.2f}_with_{winner_id}",
            winner_id
        )

    def make_command(self, command, reason, conflict_with=None):
        """Create one command payload."""
        payload = {
            "command": command,
            "reason": reason,
        }

        if conflict_with is not None:
            payload["conflict_with"] = conflict_with

        if command == "SLOW":
            payload["speed_limit"] = self.slow_linear_speed

        return payload

    def raise_command(self, commands, robot_id, command, reason, conflict_with):
        """Replace a command only when the new one is more restrictive."""
        current = commands.get(robot_id, self.make_command("RUN", "clear"))
        current_command = current["command"]

        if COMMAND_SEVERITY[command] < COMMAND_SEVERITY[current_command]:
            return

        commands[robot_id] = self.make_command(command, reason, conflict_with)

    def publish_commands(self, commands, now):
        """Publish one filtered control command for each robot."""
        for robot_id, payload in commands.items():
            payload["robot_id"] = robot_id
            msg = String()
            msg.data = json.dumps(payload, separators=(",", ":"))
            self.control_pub.publish(msg)
            self.update_hold_state(robot_id, payload, now)
            self.last_commands[robot_id] = payload

    def update_hold_state(self, robot_id, payload, now):
        """Track how long each robot has been held for priority fairness."""
        if payload["command"] in ("HOLD", "EMERGENCY_STOP"):
            if robot_id not in self.hold_since:
                self.hold_since[robot_id] = now
        else:
            self.hold_since.pop(robot_id, None)

    def choose_priority(self, state_a, state_b, now):
        """Choose which robot may move through a conflict first."""
        wait_a = self.wait_time(state_a.robot_id, now)
        wait_b = self.wait_time(state_b.robot_id, now)

        if abs(wait_a - wait_b) >= self.fairness_wait_seconds:
            if wait_a > wait_b:
                return state_a.robot_id

            return state_b.robot_id

        active_a = self.motion_score(state_a)
        active_b = self.motion_score(state_b)

        if abs(active_a - active_b) > 0.03:
            if active_a > active_b:
                return state_a.robot_id

            return state_b.robot_id

        goal_a = self.distance_to_goal(state_a)
        goal_b = self.distance_to_goal(state_b)

        if abs(goal_a - goal_b) > 0.20:
            if goal_a < goal_b:
                return state_a.robot_id

            return state_b.robot_id

        return min(state_a.robot_id, state_b.robot_id)

    def wait_time(self, robot_id, now):
        """Return how long a robot has been held."""
        if robot_id not in self.hold_since:
            return 0.0

        return now - self.hold_since[robot_id]

    def motion_score(self, state):
        """Small activity score used to let a moving robot clear a conflict."""
        return abs(state.linear_x) + 0.10 * abs(state.angular_z)

    def distance_to_goal(self, state):
        """Return distance from robot to its active goal."""
        return math.sqrt(
            (state.active_goal_x - state.x) ** 2 +
            (state.active_goal_y - state.y) ** 2
        )

    def pair_was_holding(self, robot_a_id, robot_b_id):
        """Return True when the previous command held either robot for this pair."""
        for robot_id, other_id in (
            (robot_a_id, robot_b_id),
            (robot_b_id, robot_a_id)
        ):
            command = self.last_commands.get(robot_id)

            if command is None:
                continue

            if command.get("command") not in ("HOLD", "EMERGENCY_STOP"):
                continue

            if command.get("conflict_with") == other_id:
                return True

        return False

    def coarse_cell(self, x, y):
        """Convert world coordinates to one central traffic-grid cell."""
        return (
            int(math.floor(x / self.traffic_grid_resolution)),
            int(math.floor(y / self.traffic_grid_resolution))
        )

    def footprint_cells(self, state):
        """Return all coarse cells touched by the robot's circular footprint."""
        radius = max(0.0, state.robot_radius)
        min_col = int(math.floor((state.x - radius) / self.traffic_grid_resolution))
        max_col = int(math.floor((state.x + radius) / self.traffic_grid_resolution))
        min_row = int(math.floor((state.y - radius) / self.traffic_grid_resolution))
        max_row = int(math.floor((state.y + radius) / self.traffic_grid_resolution))

        cells = set()

        for col in range(min_col, max_col + 1):
            for row in range(min_row, max_row + 1):
                cells.add((col, row))

        return cells

    def future_path_cells(self, state):
        """Return upcoming coarse cells from the robot's current local path."""
        cells = []
        seen = set(self.footprint_cells(state))

        for x, y in state.path:
            cell = self.coarse_cell(x, y)

            if cell in seen:
                continue

            seen.add(cell)
            cells.append(cell)

            if len(cells) >= self.lookahead_cells:
                break

        return set(cells)

    def expand_cells(self, cells, radius_cells):
        """Expand a set of coarse cells by a square cell radius."""
        expanded = set()

        for col, row in cells:
            for d_col in range(-radius_cells, radius_cells + 1):
                for d_row in range(-radius_cells, radius_cells + 1):
                    expanded.add((col + d_col, row + d_row))

        return expanded

    def distance_between(self, state_a, state_b):
        """Return center-to-center distance between two robots."""
        return math.sqrt(
            (state_a.x - state_b.x) ** 2 +
            (state_a.y - state_b.y) ** 2
        )

    def log_status(self, states, commands, now):
        """Log a compact traffic status periodically."""
        if now - self.last_status_log < 2.0:
            return

        self.last_status_log = now

        if not states:
            self.get_logger().info("Traffic: waiting for robot_state messages.")
            return

        parts = []

        for robot_id in sorted(states):
            command = commands.get(robot_id, {}).get("command", "NO_COMMAND")
            status = states[robot_id].status
            parts.append(f"{robot_id}:{command}/{status}")

        self.get_logger().info("Traffic: " + ", ".join(parts))


def main(args=None):
    """Start the central multibot coordinator."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MultibotCoordinator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Ctrl+C detected. Multibot coordinator shutting down.")
    except ExternalShutdownException:
        pass
    finally:
        try:
            node.timer.cancel()
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
