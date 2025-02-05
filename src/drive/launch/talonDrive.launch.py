import launch
import launch_ros.actions
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

P = 2.0
I = 0.012
D = 0.0


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
                name="frontLeft",
                parameters=[
                    {"id": 1},
                    {"P": P},
                    {"I": I},
                    {"D": D},
                    {"max_voltage": 24.0},
                    {"brake_mode": True},
                ],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="backLeft",
                parameters=[
                    {"id": 2},
                    {"P": P},
                    {"I": I},
                    {"D": D},
                    {"max_voltage": 24.0},
                    {"brake_mode": True},
                ],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="frontRight",
                parameters=[
                    {"id": 3},
                    {"P": P},
                    {"I": I},
                    {"D": D},
                    {"max_voltage": 24.0},
                    {"brake_mode": True},
                ],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="backRight",
                parameters=[
                    {"id": 4},
                    {"P": P},
                    {"I": I},
                    {"D": D},
                    {"max_voltage": 24.0},
                    {"brake_mode": True},
                ],
            ),
        ],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            container,
            launch_ros.actions.Node(
                package="drive_cpp",
                executable="drive_cpp",
                name="talon_control_node",
                parameters=[
                    {"wheels": ["frontRight", "frontLeft", "backRight", "backLeft"]},
                    {"max_speed": 1.0},
                    {"base_width": 0.9},
                    {"pub_odom": True},
                    {"pub_elec": True},
                ],
            ),
            launch_ros.actions.Node(
                package="joy", executable="joy_node", name="joystick"
            ),
            launch_ros.actions.Node(
                package="drive",
                executable="joystick_controller",
                name="joystick_controller",
                parameters=[
                    {"linear_axis_index": 5},
                    {"turn_axis_index": 4},
                    {"max_linear_speed": 1.0},
                ],
            ),
        ]
    )
