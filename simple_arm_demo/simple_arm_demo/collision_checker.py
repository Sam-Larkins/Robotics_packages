from nav_msgs.msg import OccupancyGrid
from simple_arm_demo.node import Node
import math
from geometry_msgs.msg import Point
import numpy as np
from simple_arm_demo.tools.robot_tools import PlanningProblem
import rclpy


class CollisionChecker:
    """Collision checker to if nodes are free of collision or not. Uses OccupancyGrid in order to check the collision based on robot radius. Contains additional function to get indexes from the OccupancyGrid and coordinates"""

    def __init__(
        self,
        arm_problem: PlanningProblem,
        robot_radius: float = 0.1,
    ):
        # TRANSFORM THE C-SPACE TO GRIDMAP

        self.c_space = None
        self.theta = None

        self.arm_problem = arm_problem

        self.theta, self.c_space = self.construct_config_space(300)

        c_space_gridmap = self.c_space_to_gridmap(self.c_space)

        self.map_data = c_space_gridmap.data
        self.map_width = c_space_gridmap.info.width
        self.map_height = c_space_gridmap.info.height
        self.map_resolution = c_space_gridmap.info.resolution
        self.map_origin = c_space_gridmap.info.origin.position

        self.map_origin_x = self.map_origin.x
        self.map_origin_y = self.map_origin.y

        self.robot_radius = robot_radius

        self.tolerance = math.ceil(self.robot_radius / self.map_resolution) + 1

    def construct_config_space(self, nx=101):
        theta1, theta2 = np.linspace(0, 2.0 * np.pi, nx), np.linspace(
            0, 2.0 * np.pi, nx
        )
        _v = np.zeros((len(theta1), len(theta2)), dtype=int)

        for i, t1 in enumerate(theta1):
            for j, t2 in enumerate(theta2):
                self.arm_problem.robot.set_link_angles([t1, t2])
                in_obs = 0
                fp = self.arm_problem.robot.get_current_polygon()
                for o_num, o in enumerate(self.arm_problem.workspace.obstacles):
                    if fp.intersect(o):
                        in_obs = o_num + 1
                        break
                _v[i, j] = in_obs

        return [theta1, theta2], _v

    def c_space_to_gridmap(self, c_space):
        grid_map = OccupancyGrid()

        map_data = [0] * len(c_space) * len(c_space)

        width = len(c_space)
        height = len(c_space)

        k = 0

        for i in range(0, len(c_space)):
            for j in range(0, len(c_space)):
                if c_space[j][i] > 0:
                    map_data[k] = 100
                k += 1

        resolution = float((2.0 * np.pi) / len(c_space))
        origin = Point()
        origin.x = 0.0
        origin.y = 0.0

        grid_map.info.width = width
        grid_map.info.height = height
        grid_map.data = map_data
        grid_map.info.origin.position = origin
        grid_map.info.resolution = resolution
        grid_map.header.frame_id = "map"

        return grid_map

    def get_by_indices(self, i: int, j: int) -> int:
        """Obtain index of OccupancyGrid data according to i and j from grid.

        Args:
            i (int): index in x
            j (int): index in j

        Returns:
            int: index from data list in OccupancyGrid
        """
        index = None
        try:
            index = self.map_data[j * self.map_width + i]
        except:
            rclpy.logging.get_logger("Collision Checker").warning(
                "Position not available."
            )
        return index

    def get_by_coordinates(self, x: float, y: float) -> int:
        """Obtain index of OccupancyGrid data according to i and j from grid.

        Args:
            x (float): x coordinate in the world
            y (float): y coordinate in the world

        Returns:
            int: index from data list in OccupancyGrid
        """
        indices = self.coordinates_to_indices(x, y)
        return self.get_by_indices(indices[0], indices[1])

    def coordinates_to_indices(self, x: float, y: float) -> tuple:
        """Transform coordinates to indices of the grid map

        Args:
            x (float): x coordinate of the grid map
            y (float): y coordinate of the grid map

        Returns:
            i, j (tuple): tuple of the indexes
        """
        i = int((x - self.map_origin.x) / self.map_resolution)
        j = int((y - self.map_origin.y) / self.map_resolution)
        return i, j

    def indices_to_coordinates(self, i: int, j: int) -> tuple:
        """Transforms indices to coordinates of the grid map

        Args:
            i (int): index in x axis grid map
            j (int): index in y axis grid map

        Returns:
            x, y (tuple): tuple of the coordinates
        """
        x = i * self.map_resolution + self.map_origin.x
        y = j * self.map_resolution + self.map_origin.y
        return x, y

    def is_node_free(self, node: Node) -> bool:
        """Check if a node is collision free according to its position in the grid_map

        Args:
            node (Node): the node to check if collision free

        Returns:
            bool: Wether the node is collision free or not
        """
        i, j = self.coordinates_to_indices(node.theta1, node.theta2)

        if not 0 <= i < self.map_width or not 0 <= j < self.map_height:
            return False

        for offset_x in range(-self.tolerance, self.tolerance):
            for offset_y in range(-self.tolerance, self.tolerance):
                val = self.get_by_indices(i - offset_x, j - offset_y)
                if val is None:
                    return False
                if not -1 < val < 100:
                    return False
        return True

    def is_2d_state_free(self, state):
        """Checks if a state is free of collision or not. Currently this method is only useful for a 2D arm robot. Also, as this method is usually passed as the validity checker for the SimpleSetup class from the OMPL library, its only parameter should be state.

        Args:
            state (ompl.base.State): a state expressing a configuration for the robot. For example: in the case of a 2D robot arm, the angle values of its two joints. https://ompl.kavrakilab.org/api_overview.html

        Returns:
            Bool: if the state is collision free or not according to the workspace and robot configurations
        """
        self.arm_problem.robot.set_link_angles([state[0], state[1]])

        fp = self.arm_problem.robot.get_current_polygon()
        return not self.arm_problem.workspace.in_collision_poly(fp)

    def is_connection_free(self, _n1: Node, _n2: Node, step):
        """Checks whether a connection between two nodes is possible.

        Args:
            _n1 (Node): initial node.
            _n2 (Node): final node.

        Returns:
            Bool: whether the connection is possible or not.
        """

        theta = math.atan2(_n2.theta2 - _n1.theta2, _n2.theta1 - _n1.theta1)
        _u = int(_n1.calculate_distance(_n2) / step) + 1

        for i in range(-1, _u):
            node = Node(
                _n1.theta1 + i * step * math.cos(theta),
                _n1.theta2 + i * step * math.sin(theta),
            )
            if not self.is_node_free(node):
                return False
        return True

    def is_path_free(self, path):
        """Checks whether a path is free of collision or not by
        returning the list of nodes that are in collision.

        Args:
            path (list(Node)): list of Node objects

        Returns:
            occ_nodes (list((x, y))): list of coordinates as tuples
        """
        occ_nodes = []

        for node in path:
            if not self.is_node_free(node):
                occ_nodes.append((node.theta1, node.theta2))
        if len(occ_nodes) > 0:
            rclpy.logging.get_logger("Collision Checker").warning(
                "Current path is not collision free, calculating a new path"
            )

        return occ_nodes
