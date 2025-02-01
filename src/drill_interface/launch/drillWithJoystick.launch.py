import launch
import launch_ros.actions
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""

    container = ComposableNodeContainer(
        name="PhoenixContainer",
        namespace="",
        package="ros_phoenix",
        executable="phoenix_container",
        parameters=[{"interface": "can0"}],
        composable_node_descriptions=[
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="lead",
                parameters=[
                    {"id": 8},
                    {"P": 5.0},
                    {"I": 0.0},
                    {"D": 0.0},
                    ],
            ),
            #ComposableNode(
            #    package="ros_phoenix",
            #    plugin="ros_phoenix::TalonSRX",
            #    name="drill",
            #    parameters=[
            #        {"id": 2}, 
            #        {"P": 5.0}, 
            #        {"I": 0.0}, 
            #        {"D": 0.0}
            #        ],
            #),
        ],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            container,
            launch_ros.actions.Node(
                package="drill_interface",
                executable="joystickDrill_Controller",
                name="joystick_Drill_Controller_node",
            ),
            launch_ros.actions.Node(
                package="joy", executable="joy_node", name="joystick"
            )
        ]
    )
