import launch
import launch_ros.actions
def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="gps",
                executable="heading_pub_node",
                name="gps_heading_node",
                parameters=[
                    {"TimingMode": 1},  # Survey In mode
                    {"MinTime": 600},  # Survey in time (s)
                    {"MinAcc": 2.0},  # Survey In minimum accuracy (m)
                    {"Freq": 5.0},  # Publish rate (hz)
                    {"Baudrate": 115200},
                    {"Device": "/dev/ttyUSB0"},
                ],
            ),
        ]
    )