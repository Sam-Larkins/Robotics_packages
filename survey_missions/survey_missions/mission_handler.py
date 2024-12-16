import time
import rclpy
import rclpy.node
# from collision_checker import CollisionChecker
# from node import Node
# from planners.rrt_star import RRTstar
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from std_msgs.msg import Bool
import subprocess
# from visualization_msgs.msg import Marker
# from tf2_msgs.msg import TFMessage
# from tf2_ros import TransformListener, Buffer


class MissionHandler(rclpy.node.Node):
    # self.get_logger().info("hey prick for debug purposes")
    def __init__(self):
        super().__init__("mission_handler_node")

        # Counter for waypoint
        self.waypoint_counter = 0
        self.number_of_waypoints = 0


        # Get waypoints from yaml        
        self.declare_parameter("waypointsX", [1.0])
        self.waypointsX = self.get_parameter("waypointsX").value

        self.declare_parameter("waypointsY", [1.0])
        self.waypointsY = self.get_parameter("waypointsY").value

        self.declare_parameter("waypointsZ", [1.0])
        self.waypointsZ = self.get_parameter("waypointsZ").value

        self.number_of_waypoints = len(self.get_parameter("waypointsX").value)


        # # SUBSCRIBERS
        
        self.goal_reached_subscriber_ = self.create_subscription(
            Bool, "/goal_reached", self.waypoint_controller, 10
            )


        # # PUBLISHERS
        
        self.goal_publisher = self.create_publisher(
            PoseStamped, "/goal_pose", 10
        )
        
        self.timer = self.create_timer(1, self.first_waypoint)


    def create_waypoint_pose(self):
        pose = Pose()
        position = Point()
        orientation = Quaternion()

        position.x = float(self.get_parameter("waypointsX").value[self.waypoint_counter])
        position.y = float(self.get_parameter("waypointsY").value[self.waypoint_counter])
        position.z = float(self.get_parameter("waypointsZ").value[self.waypoint_counter])

        orientation.x = 0.0
        orientation.y = 0.0
        orientation.z = 0.0
        orientation.w = 1.0

        pose.position = position
        pose.orientation = orientation

        self.get_logger().info("pose %s" %pose)

        return pose


    def first_waypoint(self):
        self.send_next_waypoint()       
        self.timer.cancel()


    def send_next_waypoint(self):
        waypoint = self.create_waypoint_pose()
        time.sleep(1)
        self.send_goal(waypoint)
        self.get_logger().info("Sending waypoint: %s" %(self.waypoint_counter +1))


    def waypoint_controller(self, goal_reached: Bool):

        if goal_reached:
            # Either need to send the next goal, or all waypoints have been reached.
            self.waypoint_counter += 1
            if self.waypoint_counter < self.number_of_waypoints:
                self.send_next_waypoint()

            else:
                self.get_logger().info("All waypoints reached")
                self.map_saver()


    def send_goal(self, pose: Pose) -> bool:
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose = pose

        self.goal_publisher.publish(goal)


    def map_saver(self):
        try:
            subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', 'my_map_cw2'],
                check=True
            )
            self.get_logger().info('Map saved successfully as my_map_cw2.yaml and my_map_cw2.pgm.')
        
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Failed to save map: {e}')


################################################################
# MAIN DEPLOY

def main(args=None):
    rclpy.init(args=args)
    mission_handler_node = MissionHandler()
    try:
        rclpy.spin(mission_handler_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        mission_handler_node.cancel_goal()
    finally:
        rclpy.try_shutdown()

    mission_handler_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

    
