import rclpy
import rclpy.node
from simple_arm_demo.tools import robot_tools, plotting_tools
from simple_arm_demo.collision_checker import CollisionChecker
from simple_arm_demo.node import Node
from simple_arm_demo.tree import Tree
from ompl import base as ob
from ompl import geometric as og
from math import pi


class ArmDemoOMPL(rclpy.node.Node):
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
        self.solve_time = (
            self.declare_parameter("solve_time", 0.2).get_parameter_value().double_value
        )
        # ======================================

    def generate_random_valid_sample(self, collision_checker, space):
        """Generate a random valid sample from a specific state space. This method tries up to 500 times to find a collision free state and then returns it.

        Args:
            collision_checker (CollisionChecker): class that helps with everything related to check the collision of nodes or states.
            space (ompl.base.StateSpace): a StateSpace object, check the ones available in https://ompl.kavrakilab.org/classompl_1_1base_1_1StateSpace.html

        Returns:
            state (ompl.base.State): a state expressing a configuration for the robot. For example: in the case of a 2D robot arm, the angle values of its two joints. https://ompl.kavrakilab.org/api_overview.html
        """
        state = ob.State(space)

        attempts = 0

        while attempts < 500:
            state.random()
            if collision_checker.is_2d_state_free(state):
                return state
            attempts += 1

    def run(self):
        """Main demo execution"""

        # ! DEFINE PLANNING PROBLEM
        arm_problem = robot_tools.PlanningProblem(self.world_file)

        # ! GENERATE COLLISION CHECKER
        collision_checker = CollisionChecker(arm_problem)

        # OMPL USAGE

        # ! STATE SPACE DEFINITION

        space = ob.RealVectorStateSpace()
        space.addDimension(0.0, 2 * pi)
        space.addDimension(0.0, 2 * pi)

        # ! BOUNDS DEFINITION

        bounds = ob.RealVectorBounds(2)

        bounds.setLow(0, 0)
        bounds.setHigh(0, 2 * pi)
        bounds.setLow(1, 0)
        bounds.setHigh(1, 2 * pi)

        space.setBounds(bounds)

        # ! CREATE SIMPLE SETUP OBJECT
        ss = og.SimpleSetup(space)

        # ! DEFINE START AND GOAL

        start = self.generate_random_valid_sample(collision_checker, space)
        goal = self.generate_random_valid_sample(collision_checker, space)
        ss.setStartAndGoalStates(start, goal)

        # ! SET STATE VALIDITY CHECKER

        ss.setStateValidityChecker(
            ob.StateValidityCheckerFn(collision_checker.is_2d_state_free)
        )

        # ! DEFINE THE PLANNER

        si = ss.getSpaceInformation()
        planner = None
        if self.planner_name == "RRT":
            planner = og.RRT(si)
        elif self.planner_name == "RRTstar":
            planner = og.RRTstar(si)
        elif self.planner_name == "RRTConnect":
            planner = og.RRTConnect(si)
        elif self.planner_name == "PRM":
            planner = og.PRM(si)

        ss.setPlanner(planner)

        # ! ATTEMPT TO SOLVE THE QUERY
        self.get_logger().info("About to solve the query.")
        solved = ss.solve(self.solve_time)

        # ===================================
        tree = Tree()  # TREE JUST FOR PLOTTING
        tree.set_start(Node(start[0], start[1]))

        if solved.asString() == "Exact solution":
            # PLOT SOLUTION IF SOLVED SUCCESFULLY

            self.get_logger().info("Query was solved sucessfully.")
            path = ss.getSolutionPath()
            path.interpolate(
                int(path.length() / 0.2)
            )  # interpolate path to show intermediate points

            path = path.getStates()
            solution_path = []

            for p in path:
                solution_path.append(Node(p[0], p[1]))
                tree.add_node(Node(p[0], p[1]))  # ADDING NODES TO TREE FOR PLOT
            plotting_tools.plot_animation_2d(
                arm_problem,
                collision_checker,
                solution_path,
                [tree.get_tree()],
                self.arm_shadows,
            )
        else:
            # PLOT START AND GOAL IF NOT SOLVED SUCCESSFULLY
            self.get_logger().warn("Query could not be solved.")

            plotting_tools.plot_animation_2d(
                arm_problem,
                collision_checker,
                [Node(start[0], start[1]), Node(goal[0], goal[1])] * 10,
                [Tree().get_tree()],
                self.arm_shadows,
            )


################################################################
# MAIN DEPLOY
def main(args=None):
    rclpy.init(args=args)
    arm_demo_node = ArmDemoOMPL()
    arm_demo_node.run()

    arm_demo_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
