import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="arm_interface_inverse_kin",
                executable="joystickKinArmController",
                name="joystick_inveser_kin",
            ),
            launch_ros.actions.Node(
                package="servo_pkg",
                executable="i2c_Servo",
                name="i2c_servo_node",
            ),
            launch_ros.actions.Node(
                package="gpio_controller",
                executable="gpioManager",
                name="gpio_manager",
            ),
        ]
    )
