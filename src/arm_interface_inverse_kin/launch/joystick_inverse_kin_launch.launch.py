import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="arm_interface_inverse_kin",  
                executable="joystick_kin_arm_controller",  
                name="joystick_inveser_kin",  
            ),
            launch_ros.actions.Node(
                launch_ros.actions.Node(
                package="joy", executable="joy_node", name="joystick"
            ),
            )
        ]
    )