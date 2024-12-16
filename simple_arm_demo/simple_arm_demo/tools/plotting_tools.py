from matplotlib.ticker import MultipleLocator
from matplotlib.collections import PatchCollection
from matplotlib.patches import Polygon as PlotPolygon
import numpy as np
from simple_arm_demo.tools import polygon_tools as poly
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import shutil
import math
import matplotlib.animation as animation

if shutil.which("latex"):
    plt.rc("font", **{"family": "serif", "sans-serif": ["Computer Modern Roman"]})
    plt.rc("text", usetex=False)


def linear_path(points, n=50):
    c_point = points[0]
    path = np.array([c_point])

    for end_point in points[1:]:
        new_path = np.linspace(path[-1], end_point, n)  # Requires numpy > 1.16.0
        path = np.concatenate((path, new_path[1:]))

    return path


# ============================
# PLOTTING TOOLS FOR 2 DoF ARM
# ============================


def plot_config_space(
    ax,
    obstacles,
    arm,
    cspace_array,
    col_map,
    xlim,
    ylim,
    theta1_lim,
    theta2_lim,
    tree,
    path,
    arm_problem,
    graph=False,
):
    h_obs = []
    for o in obstacles:
        h_obs.append(PlotPolygon(o, zorder=1))
    c_obs = PatchCollection(h_obs, cmap=col_map)
    # This sets colors for some reason (command in Polygon does not)
    c_obs.set_array(np.linspace(0, 1.0, len(obstacles) + 1)[1:])
    ax[0].add_collection(c_obs)

    (h_arm,) = ax[0].plot(*arm.get_spine_points(), c="black", lw=3.0)

    for a in ax:
        a.set_aspect("equal")

    ax[0].set_xlabel(r"$x$", fontsize=30)
    ax[0].set_ylabel(r"$y$", fontsize=30)
    ax[0].set_xlim(xlim)
    ax[0].set_ylim(ylim)

    ax[0].tick_params(axis="x", labelsize=20)
    ax[0].tick_params(axis="y", labelsize=20)

    ax[1].set_xlabel(r"$\theta_1$", fontsize=30)
    ax[1].set_ylabel(r"$\theta_2$", fontsize=30)
    ax[1].set_xlim(theta1_lim[0], theta1_lim[-1])
    ax[1].set_ylim(theta2_lim[0], theta2_lim[-1])

    # This is a bit dumb, should probably just assume [0, 2pi) everywhere, but meh
    ax[0].xaxis.set_major_locator(MultipleLocator(2.0))
    ax[0].xaxis.set_minor_locator(MultipleLocator(1.0))
    ax[0].yaxis.set_major_locator(MultipleLocator(2.0))
    ax[0].yaxis.set_minor_locator(MultipleLocator(1.0))
    ax[0].set_title("Manipulator Robot Workspace", fontsize=20)
    ax[1].set_xticks([0, np.pi / 2, np.pi, 3 * np.pi / 2, 2 * np.pi])
    ax[1].set_yticks([0, np.pi / 2, np.pi, 3 * np.pi / 2, 2 * np.pi])
    ax[1].xaxis.set_minor_locator(MultipleLocator(np.pi / 4))
    ax[1].yaxis.set_minor_locator(MultipleLocator(np.pi / 4))

    ax[1].set_xticklabels(
        [r"$0$", r"$\pi/2$", r"$\pi$", r"3$\pi/2$", r"$2\pi$"], fontsize=20
    )
    ax[1].set_yticklabels(
        [r"$0$", r"$\pi/2$", r"$\pi$", r"3$\pi/2$", r"$2\pi$"], fontsize=20
    )
    ax[1].set_title("Configuration Space (C-Space)", fontsize=20)
    for a in ax:
        a.grid(which="both", axis="both")
        a.set_axisbelow(True)

    cspace_array = np.ma.masked_where(cspace_array == 0.0, cspace_array)
    col_map.set_bad(color="white")
    ax[1].imshow(
        cspace_array.transpose(),
        origin="lower",
        cmap=col_map,
        extent=[theta1_lim[0], theta1_lim[1], theta2_lim[0], theta2_lim[1]],
    )

    # PLOTTING TREE

    if graph:
        list_nodes = list(tree)

        for node in list_nodes:
            ax[1].plot(node[0][0], node[0][1], "bo")
            _n1 = node[0]
            _n2 = node[1]
            ax[1].plot(_n2[0], _n2[1], "bo")

            distance = ((_n1[0] - _n2[0]) ** 2 + (_n1[1] - _n2[1]) ** 2) ** (1 / 2)

            theta = math.atan2(
                _n2[1] - _n1[1],
                _n2[0] - _n1[0],
            )
            step = 0.05
            _u = int(distance / step)
            for i in range(1, _u):
                ax[1].plot(
                    _n1[0] + i * step * math.cos(theta),
                    _n1[1] + i * step * math.sin(theta),
                    "bo",
                    markersize=1,
                )
    else:
        tree_colours = ["bo", "go", "ko", "yo"]
        for t in tree:
            tree_index = tree.index(t)
            list_nodes = list(t.keys())

            for node in list_nodes:
                ax[1].plot(t[node].theta1, [t[node].theta2], tree_colours[tree_index])
                if t[node].parent is not None:
                    _n1 = t[node]
                    _n2 = t[node].parent
                    ax[1].plot(_n2.theta1, _n2.theta2, tree_colours[tree_index])

                    distance = _n1.calculate_distance(_n2)

                    theta = math.atan2(
                        _n2.theta2 - _n1.theta2,
                        _n2.theta1 - _n1.theta1,
                    )
                    step = 0.05
                    _u = int(distance / step)
                    for i in range(1, _u):
                        ax[1].plot(
                            _n1.theta1 + i * step * math.cos(theta),
                            _n1.theta2 + i * step * math.sin(theta),
                            tree_colours[tree_index],
                            markersize=1,
                        )

    # PLOT INITIAL POINT AND FINAL
    # IN C_SPACE
    ax[1].plot(path[0][0], path[0][1], "go", markersize=8)
    ax[0].plot(arm_problem.start_x, arm_problem.start_y, "go", markersize=8)

    ax[1].plot(path[-1][0], path[-1][1], "ro", markersize=8)
    ax[0].plot(arm_problem.goal_x, arm_problem.goal_y, "ro", markersize=8)

    # ===================
    return h_arm


def plot_animation_2d(
    arm_problem, collision_checker, solution_path, tree, arm_shadows, graph=False
):
    """Function to plot the animation.

    Args:
        arm_problem (robot_tools.PlanningProblem): arm problem class with the information of the problem.
        collision_checker (CollisionChecker): collision checker used for the planner.
        solution_path (list[Node]): list of nodes that have the solution path.
        tree (list[tree]): receives a list of trees, e.g., [tree1, tree2].
        arm_shadows (int): number of skips for each shadow.
        graph (bool, optional): wether the input tree should be treated as graph. Defaults to False.
    """
    path_full = []
    for node in solution_path:
        path_full.append([node.theta1, node.theta2])
    path_full = np.array(path_full)

    arm_problem.robot.set_link_angles(path_full[0])
    try:
        map_lims = arm_problem.workspace.limits
    except KeyError:
        map_lims = [[0, 10], [0, 10]]

    # Animation
    animation_length = 10.0
    arm_anim = ArmAnimator(
        arm_problem.robot,
        arm_problem.workspace.obstacles,
        arm_problem,
        collision_checker.c_space,
        path_full,
        tree,
        map_lims[0],
        map_lims[1],
        collision_checker.theta[0][[0, -1]],
        collision_checker.theta[1][[0, -1]],
        shadow_skip=arm_shadows,
        graph=graph,
    )
    delta_t = animation_length * 1000.0 / arm_anim.max_frames
    arm_animation = animation.FuncAnimation(
        arm_anim.fig,
        arm_anim.animate,
        init_func=arm_anim.init_fig,
        frames=arm_anim.max_frames,
        interval=delta_t,
        blit=True,
    )

    plt.show()


class ArmAnimator(object):
    h_arm = None
    plot_artists = []

    def __init__(
        self,
        arm,
        obstacles,
        arm_problem,
        cspace_array,
        path,
        tree,
        x_lim,
        y_lim,
        t1_lim,
        t2_lim,
        col_map=cm.get_cmap(),
        shadow_skip=0,
        graph=False,
    ):
        self.fig, self.ax = plt.subplots(1, 2)
        self.fig.set_size_inches([9.6, 5.4])  # 1920*1080 at 200 dpi
        self.arm = arm
        self.obstacles = obstacles
        self.arm_problem = arm_problem
        self.cspace_array = cspace_array
        self.path = path
        self.tree = tree
        self.cmap = col_map
        self.x_lim = x_lim
        self.y_lim = y_lim
        self.t1lim = t1_lim
        self.t2lim = t2_lim
        self.max_frames = self.path.shape[0]
        self.end_effector_path = poly.PointList([])
        self._shadow_skip = shadow_skip
        self.graph = graph

    def init_fig(self):
        for a in self.ax:
            a.cla()

        self.arm.set_link_angles(self.path[0])
        self.h_arm = plot_config_space(
            self.ax,
            self.obstacles,
            self.arm,
            self.cspace_array,
            self.cmap,
            self.x_lim,
            self.y_lim,
            self.t1lim,
            self.t2lim,
            self.tree,
            self.path,
            self.arm_problem,
            self.graph,
        )

        self.last_break = 0
        (self.h_path,) = self.ax[1].plot(self.path[:1, 0], self.path[:1, 1], "r--")
        (self.h_pathend,) = self.ax[1].plot(self.path[0, 0], self.path[0, 1], "ro")

        self.end_effector_path = poly.PointList([self.arm.get_end_effector_position()])
        (self.h_ee_path,) = self.ax[0].plot(
            [self.end_effector_path[0].x], [self.end_effector_path[0].y], "r--"
        )
        (self.h_ee_pathend,) = self.ax[0].plot(
            [self.end_effector_path[0].x], [self.end_effector_path[0].y], "ro"
        )
        self.plot_artists = [
            self.h_arm,
            self.h_path,
            self.h_pathend,
            self.h_ee_path,
            self.h_ee_pathend,
        ]

        return self.plot_artists

    def animate(self, i):
        """Animation function for plotting"""
        # ! DO NOT MODIFY
        # If plotting extra arm shadows, add them to the plot_artists
        if self._shadow_skip != 0 and i % self._shadow_skip == 0:
            gv = 0.9 - float(i) / self.max_frames * 0.9
            h_arm_shadow = self.ax[0].plot(
                *self.arm.get_spine_points(), c=[gv, gv, gv], lw=1.0
            )
            h_arm_shadow.extend(self.plot_artists)
            self.plot_artists = h_arm_shadow

        self.arm.set_link_angles(self.path[i])
        self.h_arm.set_data(*self.arm.get_spine_points())

        # If the path crosses one of the boundaries, break it and add a new path
        if any(abs(self.path[i] - self.path[i - 1]) > np.pi):
            (old_path,) = self.ax[1].plot(
                self.path[self.last_break : i, 0],
                self.path[self.last_break : i, 1],
                "r--",
            )
            self.plot_artists.append(old_path)
            self.last_break = i

        self.h_path.set_data(
            self.path[self.last_break : (i + 1), 0],
            self.path[self.last_break : (i + 1), 1],
        )
        self.h_pathend.set_data(self.path[i, 0], self.path[i, 1])

        self.end_effector_path.append(self.arm.get_end_effector_position())
        self.h_ee_path.set_data(*self.end_effector_path.get_xy())
        self.h_ee_pathend.set_data(
            [self.end_effector_path[-1].x], [self.end_effector_path[-1].y]
        )

        return self.plot_artists


# ============================
# PLOTTING TOOLS FOR N DoF ARM
# ============================


def plot_work_space(
    ax,
    obstacles,
    arm,
    col_map,
    xlim,
    ylim,
    arm_problem,
):
    h_obs = []
    for o in obstacles:
        h_obs.append(PlotPolygon(o, zorder=1))
    c_obs = PatchCollection(h_obs, cmap=col_map)
    # This sets colors for some reason (command in Polygon does not)
    c_obs.set_array(np.linspace(0, 1.0, len(obstacles) + 1)[1:])
    ax.add_collection(c_obs)

    (h_arm,) = ax.plot(*arm.get_spine_points(), c="black", lw=3.0)

    ax.set_aspect("equal")

    ax.set_xlabel(r"$x$", fontsize=30)
    ax.set_ylabel(r"$y$", fontsize=30)
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)

    ax.tick_params(axis="x", labelsize=20)
    ax.tick_params(axis="y", labelsize=20)

    # This is a bit dumb, should probably just assume [0, 2pi) everywhere, but meh
    ax.xaxis.set_major_locator(MultipleLocator(2.0))
    ax.xaxis.set_minor_locator(MultipleLocator(1.0))
    ax.yaxis.set_major_locator(MultipleLocator(2.0))
    ax.yaxis.set_minor_locator(MultipleLocator(1.0))
    ax.set_title("Manipulator Robot Workspace", fontsize=20)

    col_map.set_bad(color="white")

    # ===================
    return h_arm


def plot_animation_nd(arm_problem, solution_path, arm_shadows):
    """Function to plot the animation.

    Args:
        arm_problem (robot_tools.PlanningProblem): arm problem class with the information of the problem.
        solution_path (list[list[angle_values]]): list of the angles for the joints of the robot towards the solution. For example, for a 3DoF arm: [[0.1, 0.1, 0.1], [0.2, 0.2, 0.2], ...]
        arm_shadows (int): number of skips for each shadow.
    """

    solution_path = np.array(solution_path)

    arm_problem.robot.set_link_angles(solution_path[0])
    try:
        map_lims = arm_problem.workspace.limits
    except KeyError:
        map_lims = [[0, 10], [0, 10]]

    # Animation
    animation_length = 10.0
    arm_anim = ArmAnimatorNDoF(
        arm_problem.robot,
        arm_problem.workspace.obstacles,
        arm_problem,
        solution_path,
        map_lims[0],
        map_lims[1],
        shadow_skip=arm_shadows,
    )

    delta_t = animation_length * 1000.0 / arm_anim.max_frames
    arm_animation = animation.FuncAnimation(
        arm_anim.fig,
        arm_anim.animate,
        init_func=arm_anim.init_fig,
        frames=arm_anim.max_frames,
        interval=delta_t,
        blit=True,
    )

    plt.show()


class ArmAnimatorNDoF(object):
    h_arm = None
    plot_artists = []

    def __init__(
        self,
        arm,
        obstacles,
        arm_problem,
        path,
        x_lim,
        y_lim,
        col_map=cm.get_cmap(),
        shadow_skip=0,
    ):
        self.fig, self.ax = plt.subplots(1, 1)
        self.fig.set_size_inches([9.6, 5.4])  # 1920*1080 at 200 dpi
        self.arm = arm
        self.obstacles = obstacles
        self.arm_problem = arm_problem
        self.path = path
        self.cmap = col_map
        self.x_lim = x_lim
        self.y_lim = y_lim
        self.max_frames = self.path.shape[0]
        self.end_effector_path = poly.PointList([])
        self._shadow_skip = shadow_skip

    def init_fig(self):
        self.ax.cla()

        self.arm.set_link_angles(self.path[0])

        self.h_arm = plot_work_space(
            self.ax,
            self.obstacles,
            self.arm,
            self.cmap,
            self.x_lim,
            self.y_lim,
            self.arm_problem,
        )

        self.last_break = 0
        # (self.h_path,) = self.ax.plot(self.path[:1, 0], self.path[:1, 1], "r--")
        (self.h_pathend,) = self.ax.plot(self.path[0, 0], self.path[0, 1], "ro")

        self.end_effector_path = poly.PointList([self.arm.get_end_effector_position()])
        (self.h_ee_path,) = self.ax.plot(
            [self.end_effector_path[0].x], [self.end_effector_path[0].y], "r--"
        )
        (self.h_ee_pathend,) = self.ax.plot(
            [self.end_effector_path[0].x], [self.end_effector_path[0].y], "ro"
        )
        self.plot_artists = [
            self.h_arm,
            # self.h_path,
            # self.h_pathend,
            self.h_ee_path,
            self.h_ee_pathend,
        ]

        return self.plot_artists

    def animate(self, i):
        # If plotting extra arm shadows, add them to the plot_artists
        if self._shadow_skip != 0 and i % self._shadow_skip == 0:
            gv = 0.9 - float(i) / self.max_frames * 0.9
            h_arm_shadow = self.ax.plot(
                *self.arm.get_spine_points(), c=[gv, gv, gv], lw=1.0
            )
            h_arm_shadow.extend(self.plot_artists)
            self.plot_artists = h_arm_shadow

        self.arm.set_link_angles(self.path[i])
        self.h_arm.set_data(*self.arm.get_spine_points())

        # If the path crosses one of the boundaries, break it and add a new path
        if any(abs(self.path[i] - self.path[i - 1]) > np.pi):
            (old_path,) = self.ax.plot(
                self.path[self.last_break : i, 0],
                self.path[self.last_break : i, 1],
                "r--",
            )
            self.plot_artists.append(old_path)
            self.last_break = i

        # self.h_path.set_data(
        #     self.path[self.last_break : (i + 1), 0],
        #     self.path[self.last_break : (i + 1), 1],
        # )
        # self.h_path.set_data(
        #     self.path[0, 0],
        #     self.path[0, 1],
        # )
        self.h_pathend.set_data(self.path[i, 0], self.path[i, 1])

        self.end_effector_path.append(self.arm.get_end_effector_position())
        self.h_ee_path.set_data(*self.end_effector_path.get_xy())
        self.h_ee_pathend.set_data(
            [self.end_effector_path[-1].x], [self.end_effector_path[-1].y]
        )

        return self.plot_artists
