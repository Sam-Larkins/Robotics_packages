import numpy as np
from simple_arm_demo.tools import polygon_tools as poly
import yaml
import sys
import matplotlib as plt
from matplotlib.patches import Polygon as PlotPolygon
from matplotlib.collections import PatchCollection
import matplotlib.cm as cm
from numpy import arccos, arctan2, pi
import rclpy


def robot_builder(robot):
    # Note that the robot type must be implemented in this module, so the example robot:
    #  {type: RobotArm2D, parameters: {base_position: [5.0, 5.0], link_lengths: [2.1, 2.1]}
    # would call as constructor: robot_tools.RobotArm2D(base_position=[5.0, 5.0], link_lengths=[2.1, 2.1])
    return getattr(sys.modules[__name__], robot["type"])(**robot["parameters"])


def workspace_builder(workspace):
    # Note that the workspace type must be implemented in this module, so the example workspace:
    #  {type: Workspace2D, parameters: {limits=[[0,1.0],[0,1.0]], obstacles=[]}
    # would call as constructor: robot_tools.Workspace2D(limits=[[0,1.0],[0,1.0]], obstacles=[])
    return getattr(sys.modules[__name__], workspace["type"])(**workspace["parameters"])


class Workspace2D(object):
    """This object is used for the plotting as well for the whole demo.
    It contains some important method that can help to interact with obstacles for collision checking.
    """

    def __init__(self, limits=[[0, 1.0], [0, 1.0]], obstacles=[]):
        self.limits = np.array(limits)
        assert self.limits.shape == (
            2,
            2,
        ), "Currently only implemented for 2D workspaces"
        self.obstacles = []
        for ob in obstacles:
            # Add each obstacle (must be a Polygon or derived class like Rectangle from poly_tools)
            self.obstacles.append(getattr(poly, ob["type"])(**ob["parameters"]))

    def in_collision_point(self, point):
        """Checks if a point is in collision with the obstacles in the workspace

        Args:
            point (poly.Point): Point type of object with XY coordinates

        Returns:
            Bool: return if point is in collision with any obstacle or not.
            If it is in collision, returns True, otherwise False.
        """
        p = poly.Point(*point)
        collision = False
        for o in self.obstacles:
            if o.point_inside(p):
                collision = True
                break
        return collision

    def in_collision_poly(self, polygon):
        """Checks if a point is in collision with the obstacles in the workspace

        Args:
            point (poly.Point): Point type of object with XY coordinates

        Returns:
            Bool: return if point is in collision with any obstacle or not.
            If it is in collision, returns True, otherwise False.
        """
        collision = False
        for o in self.obstacles:
            if polygon.intersect(o):
                collision = True
                break
        return collision

    def plot(self, hax=None, cmap=cm.viridis):
        if hax is None:
            f, hax = plt.subplots(1)
        h_obs = []
        for o in self.obstacles:
            h_obs.append(PlotPolygon(o, zorder=1))
        c_obs = PatchCollection(h_obs, cmap=cmap)
        # This sets colors for some reason (command in Polygon does not)
        c_obs.set_array(np.linspace(0, 1.0, len(self.obstacles) + 1)[1:])
        hax.add_collection(c_obs)

        hax.set_aspect("equal")

        hax.set_xlabel(r"$x$")
        hax.set_ylabel(r"$y$")
        hax.set_xlim(self.limits[0])
        hax.set_ylim(self.limits[1])


# ! CLASS IMPLEMENTATION FOR A ROBOT ARM OF TWO DoFs
class RobotArm2D(object):
    _spine_pts = None

    def __init__(
        self, base_position=[0.0, 0.0], link_lengths=[1.0, 1.0], link_angles=[0.0, 0.0]
    ):
        # Assume arm angles are relative (can be summed)

        self._base_position = poly.Point(base_position[0], base_position[1])

        assert len(link_lengths) == len(link_angles)
        self._link_lengths = np.array(link_lengths)
        self._link_angles = np.array(link_angles)

        self._R = [np.eye(2)] * 2
        self._set_rotation_transforms()

    def set_link_angles(self, link_angles):
        self._link_angles = np.array(link_angles)
        self._set_rotation_transforms()

    def _set_rotation_transforms(self):

        self._R[0] = np.array(
            [
                [np.cos(self._link_angles[0]), -np.sin(self._link_angles[0])],
                [np.sin(self._link_angles[0]), np.cos(self._link_angles[0])],
            ]
        )

        self._R[1] = np.array(
            [
                [
                    np.cos(self._link_angles[0] + self._link_angles[1]),
                    -np.sin(self._link_angles[0] + self._link_angles[1]),
                ],
                [
                    np.sin(self._link_angles[0] + self._link_angles[1]),
                    np.cos(self._link_angles[0] + self._link_angles[1]),
                ],
            ]
        )

        self._set_spine_points()

    def get_current_polygon(self):
        # Run backwards through the points to make a polygon
        return poly.Polygon(self._spine_pts + self._spine_pts[-2:0:-1])

    def _set_spine_points(self):
        self._spine_pts = [self._base_position]

        self._spine_pts.append(
            poly.Point(
                *(
                    np.matmul(self._R[0], [self._link_lengths[0], 0])
                    + self._spine_pts[-1]
                )
            )
        )
        self._spine_pts.append(
            poly.Point(
                *(
                    np.matmul(self._R[1], [self._link_lengths[1], 0])
                    + self._spine_pts[-1]
                )
            )
        )

    def get_spine_points(self):

        return [self._spine_pts[0].x, self._spine_pts[1].x, self._spine_pts[2].x], [
            self._spine_pts[0].y,
            self._spine_pts[1].y,
            self._spine_pts[2].y,
        ]

    def get_end_effector_position(self):
        return self._spine_pts[-1]

    def end_effector_path(self, config_path):
        c_pose = self._link_angles.copy()
        ee_path = []
        for pose in config_path:
            self.set_link_angles(pose)
            ee_path.append(self.get_end_effector_position())
        self.set_link_angles(c_pose)
        return np.array(ee_path)

    def get_inv_kin(self, _x, _y):
        _x -= self._base_position.x
        _y -= self._base_position.y

        L1 = self._link_lengths[0]
        L2 = self._link_lengths[1]

        d2 = _x**2 + _y**2
        d = d2**0.5

        base_angle = arctan2(_y, _x)

        C = (L1**2 + d2 - L2**2) / (2 * L1 * d)
        c = arccos(C)

        if not -1 <= C <= 1:
            rclpy.logging.get_logger("Inverse Kinematics solver").warn(
                "Collision checker has not been set on the planner"
            )
            return

        B = (L1**2 + L2**2 - d2) / (2 * L1 * L2)
        b = arccos(B)

        inv_kin_sol = []

        theta1 = base_angle + c
        theta2 = b - pi
        if theta1 < 0:
            theta1 = 2 * pi + theta1
        if theta2 < 0:
            theta2 = 2 * pi + theta2

        inv_kin_sol.append([theta1, theta2])

        theta1 = base_angle - c
        theta2 = pi - b
        if theta1 < 0:
            theta1 = 2 * pi + theta1
        if theta2 < 0:
            theta2 = 2 * pi + theta2

        inv_kin_sol.append([theta1, theta2])

        return inv_kin_sol


class PlanningProblem(object):
    def __init__(self, world_file):
        # Load world
        with open(world_file, "r") as fh:
            world = yaml.safe_load(fh)

        self.workspace = workspace_builder(world["workspace"])

        self.robot = robot_builder(world["robot"])

        if "queries" in world:
            self.queries = world["queries"]

        self.start_x = 0
        self.start_y = 0

        self.goal_x = 0
        self.goal_y = 0
