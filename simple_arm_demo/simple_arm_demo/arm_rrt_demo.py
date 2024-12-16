import sys
import rclpy
import rclpy.node
from simple_arm_demo.tools import robot_tools, plotting_tools
from simple_arm_demo.planners.rrt import RRT
from simple_arm_demo.planners.rrt_star import RRTstar
from simple_arm_demo.collision_checker import CollisionChecker
from simple_arm_demo.node import Node


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

        # ======================================

        # PLANNER PARAMETERS
        self.planner_name = (
            self.declare_parameter("planner", "RRT").get_parameter_value().string_value
        )
        self.step_size = (
            self.declare_parameter("step_size", 0.01).get_parameter_value().double_value
        )
        self.expansion_size = (
            self.declare_parameter("expansion_size", 0.5)
            .get_parameter_value()
            .double_value
        )
        self.bias = (
            self.declare_parameter("bias", 0.05).get_parameter_value().double_value
        )
        self.solve_time = (
            self.declare_parameter("solve_time", 0.2).get_parameter_value().double_value
        )
        self.collision_radius = (
            self.declare_parameter("collision_radius", 0.1)
            .get_parameter_value()
            .double_value
        )
        self.goal_tolerance = (
            self.declare_parameter("goal_tolerance", 0.01)
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

        # ! DEFINE START AND GOAL WITH INVERSE KINEMATICS

        query = arm_problem.queries[0]

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
                sys.exit(0)

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
                sys.exit(0)

        arm_problem.start_x = query[0][0]
        arm_problem.start_y = query[0][1]

        arm_problem.goal_x = query[1][0]
        arm_problem.goal_y = query[1][1]

        # ======================================================

        # ! DEFINE THE PLANNER
        planner = None
        if self.planner_name == "RRT":
            planner = RRT()
        elif self.planner_name == "RRTstar":
            planner = RRTstar()

        # =======================================================

        # ! SET PARAMETERS TO THE PLANNER
        planner.set_start(start)
        planner.set_goal(goal)
        planner.set_bias(self.bias)
        planner.set_expansion_size(self.expansion_size)
        planner.set_step_size(self.step_size)
        planner.set_collision_checker(collision_checker)
        planner.set_goal_tolerance(self.goal_tolerance)

        # =======================================================
        self.get_logger().info("About to solve the query.")
        # ! ATTEMPT TO SOLVE
        solved = planner.solve(self.solve_time)

        # ===================================
        # ! PLOTTING
        if solved:
            self.get_logger().info("Query was solved sucessfully.")
            planner.interpolate_solution(0.01)
            solution_path = planner.get_solution_path()
            tree = planner.get_tree()

            plotting_tools.plot_animation_2d(
                arm_problem, collision_checker, solution_path, [tree], self.arm_shadows
            )
        else:  # ! IF NOT SOLVED
            self.get_logger().info("Query could not be solved.")
            tree = planner.get_tree()
            plotting_tools.plot_animation_2d(
                arm_problem, collision_checker, [start, goal], [tree], self.arm_shadows
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
