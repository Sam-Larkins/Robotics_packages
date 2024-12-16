import sys
import rclpy
import rclpy.node
from simple_arm_demo.tools import robot_tools
from simple_arm_demo.planners.prm import PRM
from simple_arm_demo.planners.lazy_prm import LazyPRM

from simple_arm_demo.collision_checker import CollisionChecker
from simple_arm_demo.node import Node
from simple_arm_demo.tools import plotting_tools


class ArmDemo(rclpy.node.Node):
    """Manages the incoming start-to-goal query of a 2D simple arm."""

    def __init__(self):
        super().__init__("arm_demo_node")

        # DEMO ARGUMENTS
        # ==================================
        self.world_file = (
            self.declare_parameter("world_file", "world_file")
            .get_parameter_value()
            .string_value
        )

        self.arm_shadows = (
            self.declare_parameter("arm_shadows", 0).get_parameter_value().integer_value
        )

        # PLANNER PARAMETERS
        self.step_size = (
            self.declare_parameter("step_size", 0.01).get_parameter_value().double_value
        )
        self.collision_radius = (
            self.declare_parameter("collision_radius", 0.1)
            .get_parameter_value()
            .double_value
        )
        self.samples_number = (
            self.declare_parameter("samples_number", 600)
            .get_parameter_value()
            .integer_value
        )
        self.neighbor_distance = (
            self.declare_parameter("neighbor_distance", 0.1)
            .get_parameter_value()
            .double_value
        )
        # ======================================

    def run(self):
        """Main demo execution"""

        # ! DEFINE PLANNING PROBLEM
        arm_problem = robot_tools.PlanningProblem(self.world_file)

        # ! GENERATE COLLISION CHECKER
        collision_checker = CollisionChecker(arm_problem, self.collision_radius)

        # ======================================================

        # ! DEFINE THE PLANNER
        planner = LazyPRM()

        # =======================================================

        # ! SET PARAMETERS TO THE PLANNER
        planner.set_step_size(self.step_size)
        planner.set_collision_checker(collision_checker)
        planner.set_neighbor_distance(self.neighbor_distance)

        # ! PREPROCESSING
        self.get_logger().info("Learning phase of the PRM algorithm.")
        planner.generate_samples(self.samples_number)
        planner.generate_connections()

        # =======================================================
        # TRY TO SOLVE DIFFERENT QUERIES
        queries = arm_problem.queries

        for query in queries:
            self.get_logger().info("Attempting to solve a query.")
            # ! DEFINE START AND GOAL WITH INVERSE KINEMATICS

            arm_problem.start_x = query[0][0]
            arm_problem.start_y = query[0][1]

            arm_problem.goal_x = query[1][0]
            arm_problem.goal_y = query[1][1]

            start_conf = arm_problem.robot.get_inv_kin(query[0][0], query[0][1])
            start = Node(start_conf[0][0], start_conf[0][1])
            if collision_checker.is_node_free(start):
                pass
            else:
                start = Node(start_conf[1][0], start_conf[1][1])
                if not collision_checker.is_node_free(start):
                    self.get_logger().warning(
                        "None of inverse kinematics positions for the start are collision free."
                    )
                    continue

            goal_conf = arm_problem.robot.get_inv_kin(query[1][0], query[1][1])
            goal = Node(goal_conf[0][0], goal_conf[0][1])
            if collision_checker.is_node_free(goal):
                pass
            else:
                goal = Node(goal_conf[1][0], goal_conf[1][1])
                if not collision_checker.is_node_free(goal):
                    self.get_logger().warning(
                        "None of inverse kinematics positions for the goal are collision free."
                    )
                    continue

            # ================================================

            # ! SET START AND GOAL
            planner.set_start(start)
            planner.set_goal(goal)

            self.get_logger().info("Query phase of the PRM algorithm.")
            # ! ATTEMPT TO SOLVE
            solved = planner.solve()

            # ===================================
            # ! PLOTTING
            if solved:
                self.get_logger().info("Query was solved sucessfully.")
                planner.interpolate_solution(0.01)
                solution_path = planner.get_solution_path()
                graph = planner.get_graph()

                plotting_tools.plot_animation_2d(
                    arm_problem,
                    collision_checker,
                    solution_path,
                    graph,
                    self.arm_shadows,
                    True,
                )
            else:  # ! IF NOT SOLVED
                self.get_logger().info("Query could not be solved.")
                graph = planner.get_graph()
                plotting_tools.plot_animation_2d(
                    arm_problem,
                    collision_checker,
                    [start, goal],
                    graph,
                    self.arm_shadows,
                    True,
                )


################################################################
# MAIN DEPLOY
def main(args=None):
    rclpy.init(args=args)
    arm_demo_node = ArmDemo()
    arm_demo_node.run()

    arm_demo_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
