import math
import time
from functools import wraps
import random
import rclpy
from simple_arm_demo.collision_checker import CollisionChecker
from simple_arm_demo.node import Node
from nav_msgs.msg import Path
import heapq
from rclpy.time import Time

def timeit(func):
    @wraps(func)
    def timeit_wrapper(*args, **kwargs):
        start_time = time.perf_counter()
        result = func(*args, **kwargs)
        end_time = time.perf_counter()
        total_time = end_time - start_time
        rclpy.logging.get_logger("PRM motion planner").info(
            f"Function {func.__name__}{args} {kwargs} Took {total_time:.4f} seconds"
        )
        return result

    return timeit_wrapper


class PRM:
    """PRM algorithm implementation as a planner to solve start-to-goal queries"""

    def __init__(self):
        self.start: Node = None
        self.goal: Node = None

        self.step_size = 0.001
        self.neighbor_distance = 0.5

        self.collision_checker: CollisionChecker = None
        self.tolerance = 0.001

        self.graph = {}
        self.connections = set()

        self.solution_node = None

        self.solution_path = None

        # astar attributes
        self.open_list = []
        self.closed_list = []
        self.g = {}
        self.nodes = {}

    def set_start(self, start):
        """Set start position of the robot"""
        self.start = start

    def set_goal(self, goal):
        """Set goal of the start to goal query"""
        self.goal = goal

    def set_collision_checker(self, collision_checker):
        """Set collision checker to detect collision free nodes"""
        self.collision_checker = collision_checker

    def set_step_size(self, step_size):
        """Set the step size to determine if connection between two
        nodes is possible with collision checking"""
        self.step_size = step_size

    def set_neighbor_distance(self, neighbor_distance):
        """Set the maximum expansion for a new node in the tree"""
        self.neighbor_distance = neighbor_distance

    def get_solution_path(self):
        """Obtain stored solution path"""
        return self.solution_path

    def get_path_length(self):
        """Obtain the length of the solution path"""
        distance = 0

        for i in range(0, len(self.solution_path) - 2):
            distance += math.sqrt(
                math.pow(
                    self.solution_path[i].theta1 - self.solution_path[i + 1].theta1, 2
                )
                + math.pow(
                    self.solution_path[i].theta2 - self.solution_path[i + 1].theta2, 2
                )
            )

        return distance

    def get_graph(self):
        return self.connections

    @timeit
    def solve(self) -> list:
        """Function to find path from start to goal using Dijkstra algorithm"""

        if not self.collision_checker:
            rclpy.logging.get_logger("PRM motion planner").warn(
                "Collision checker has not been set on the planner"
            )
            return False
        if not self.start:
            rclpy.logging.get_logger("PRM motion planner").warn(
                "Start position has not been set on the planner"
            )
            return False
        if not self.goal:
            rclpy.logging.get_logger("PRM motion planner").warn(
                "Goal position has not been set on the planner"
            )
            return False

        self.open_list = []
        self.closed_list = []
        self.g = {}
        self.nodes = {}

        # finding nearest node from start
        start_coord = None
        init_dis = float("inf")

        for node in self.graph:
            cur_dis = self.graph[node].calculate_distance(self.start)
            if cur_dis < init_dis:
                if self.collision_checker.is_connection_free(
                    self.graph[node], self.start, self.step_size
                ):
                    start_coord = node
                    init_dis = cur_dis
        if start_coord is None:
            rclpy.logging.get_logger("PRM motion planner").warn(
                "Could not connect the start to a roadmap."
            )
            return False

        # finding nearest node from goal
        goal_coord = None
        init_dis = float("inf")

        for node in self.graph:
            cur_dis = self.graph[node].calculate_distance(self.goal)
            if cur_dis < init_dis:
                if self.collision_checker.is_connection_free(
                    self.graph[node], self.goal, self.step_size
                ):
                    goal_coord = node
                    init_dis = cur_dis
        if goal_coord is None:
            rclpy.logging.get_logger("PRM motion planner").warn(
                "Could not connect the goal to a roadmap."
            )
            return False

        # #########################

        start_node = self.graph[start_coord]

        goal_node = self.graph[goal_coord]

        self.g[(start_node.theta1, start_node.theta2)] = 0
        self.g[(goal_node.theta1, goal_node.theta2)] = math.inf

        heapq.heappush(self.open_list, (0.0, (start_node.theta1, start_node.theta2)))
        self.nodes[(start_node.theta1, start_node.theta2)] = start_node

        while len(self.open_list) > 0:
            temp_node = heapq.heappop(self.open_list)[1]
            current_node = self.nodes[temp_node]

            for connection in self.graph[
                (current_node.theta1, current_node.theta2)
            ].connections.items():
                if (
                    connection[1].calculate_distance(goal_node) < self.tolerance
                ):  # Found path
                    connection[1].parent = current_node
                    self.solution_path = connection[1].backtrack_path()

                    if len(self.solution_path) > 0:
                        self.solution_path = (
                            [self.start] + self.solution_path + [self.goal]
                        )
                        return True
                    else:
                        return False

                new_cost = current_node.g + connection[1].calculate_distance(
                    current_node
                )

                if (connection[1].theta1, connection[1].theta2) not in self.g:
                    self.g[(connection[1].theta1, connection[1].theta2)] = math.inf

                if new_cost < self.g[(connection[1].theta1, connection[1].theta2)]:
                    self.g[(connection[1].theta1, connection[1].theta2)] = new_cost
                    connection[1].g = new_cost
                    connection[1].h = connection[1].calculate_distance(self.goal)
                    connection[1].f = connection[1].g + connection[1].h
                    connection[1].parent = current_node

                    heapq.heappush(
                        self.open_list,
                        (
                            connection[1].f,
                            (connection[1].theta1, connection[1].theta2),
                        ),
                    )
                    self.nodes[
                        (connection[1].theta1, connection[1].theta2)
                    ] = connection[1]

            self.closed_list.append(current_node)

        return False

        # ################################

    @timeit
    def generate_samples(self, samples_number):
        for i in range(samples_number):
            nrand = self.sample()
            if self.collision_checker.is_node_free(nrand):
                if (nrand.theta1, nrand.theta2) not in self.graph:
                    self.graph[(nrand.theta1, nrand.theta2)] = nrand

    @timeit
    def generate_connections(self):
        for n1_coord in self.graph:
            _n1 = self.graph[n1_coord]
            for n2_coord in self.graph:
                if (
                    n1_coord != n2_coord
                    and n2_coord not in self.graph[n1_coord].connections
                ):
                    if _n1.calculate_distance(
                        self.graph[n2_coord]
                    ) <= self.neighbor_distance and self.collision_checker.is_connection_free(
                        _n1, self.graph[n2_coord], self.step_size
                    ):
                        self.graph[n1_coord].connections[n2_coord] = self.graph[
                            n2_coord
                        ]
                        self.graph[n2_coord].connections[n1_coord] = self.graph[
                            n1_coord
                        ]
                        if (n1_coord, n2_coord) not in self.connections and (
                            n2_coord,
                            n1_coord,
                        ) not in self.connections:
                            self.connections.add((n1_coord, n2_coord))

    def sample(self):
        """Sample node from the workspace"""
        _i = int(random.uniform(0, self.collision_checker.map_width - 1))
        _j = int(random.uniform(0, self.collision_checker.map_height - 1))
        _x, _y = self.collision_checker.indices_to_coordinates(_i, _j)
        node = Node(_x, _y)
        return node

    def nodes_to_path_msg(self, path_nodes: list) -> Path:
        """Transforms the list of path nodes into a Path type of object"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = Time().to_msg()

        for node in path_nodes:
            path.poses.append(node.to_pose_stamped())

        return path

    def interpolate_solution(self, step):
        interpolated_path = []

        interpolated_path.append(self.solution_path[0])

        for i in range(len(self.solution_path) - 1):
            interpolated_path.append(self.solution_path[i])
            distance = self.solution_path[i].calculate_distance(
                self.solution_path[i + 1]
            )
            _u = int(distance / step)

            theta = math.atan2(
                self.solution_path[i + 1].theta2 - self.solution_path[i].theta2,
                self.solution_path[i + 1].theta1 - self.solution_path[i].theta1,
            )

            for j in range(1, _u):
                interpolated_path.append(
                    Node(
                        self.solution_path[i].theta1 + step * j * math.cos(theta),
                        self.solution_path[i].theta2 + step * j * math.sin(theta),
                    )
                )

        self.solution_path = interpolated_path
