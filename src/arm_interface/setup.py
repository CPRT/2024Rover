from setuptools import find_packages, setup
import os
from glob import glob

package_name = "arm_interface"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="will",
    maintainer_email="51888361+MaybeWilli@users.noreply.github.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "trajectory_interpreter = arm_interface.trajectoryInterpreter:main",
            "typing_controller = arm_interface.typingController:main",
            "keyboard_arm_publisher = arm_interface.keyboardArmPublisher:main",
            "keyboard_arm_controller = arm_interface.keyboardArmController:main",
        ],
    },
)
