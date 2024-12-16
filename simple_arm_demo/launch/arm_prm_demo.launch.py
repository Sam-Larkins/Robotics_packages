from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    # ! PACKAGES DIR
    simple_arm_demo_dir = FindPackageShare(package="simple_arm_demo").find(
        "simple_arm_demo"
    )

    # ! NODES
    simple_arm_node = Node(
        package="simple_arm_demo",
        executable="arm_prm_demo",
        name="arm_demo_node",
        output="screen",
        parameters=[
            {
                "world_file": simple_arm_demo_dir + "/config/coursework_sim2.yaml",
                "arm_shadows": 10,
            },
            simple_arm_demo_dir + "/config/prm_params.yaml",
        ],
    )

    # ! LAUNCH DESCRIPTION DECLARATION
    ld = LaunchDescription()

    # ! NODES
    ld.add_action(simple_arm_node)

    return ld
