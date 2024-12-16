from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():

    # ! PACKAGES DIR
    survey_missions_dir = FindPackageShare(
        package="survey_missions"
    ).find("survey_missions")


     # ! NODES
    mission_handler_node = Node(
        package="survey_missions",
        executable="mission_handler",
        name="mission_handler_node",
        output="screen",
        parameters=[survey_missions_dir + "/config/survey_waypoints_2.yaml"],
    )

     # ! LAUNCH DESCRIPTION DECLARATION
    ld = LaunchDescription()

    # ! NODES
    ld.add_action(mission_handler_node)

    return ld