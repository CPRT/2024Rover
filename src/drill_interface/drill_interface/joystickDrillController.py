from math import radians
import rclpy
import rclpy.logging
from rclpy.node import Node

import rclpy.time
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8, Float32, Bool
from ros_phoenix.msg import MotorControl, MotorStatus
from math import pi


class joystickDrillController(Node):
    def __init__(self):
        super().__init__("joystickDrill_Controller")

        self.lead = MotorControl()
        self.drill = MotorControl()

        self.drill_direction = 0.1

        self.estop = Bool()
        self.estopTimestamp = 0.0
        self.lastTimestamp = 0

        self.leadCommand = self.create_publisher(MotorControl, "/lead/set", 1)
        self.drillCommand = self.create_publisher(MotorControl, "/drill/set", 1)

        self.joystick = self.create_subscription(Joy, "/joy", self.joy_callback, 5)

        freq = 10
        self.rate = self.create_rate(freq)
        period = 1 / freq
        self.timer = self.create_timer(period, self.controlPublisher)

    def controlPublisher(self):
        if self.estop.data == True:
            return
        self.leadCommand.publish(self.lead)

    def joy_callback(self, msg: Joy):
        self.lastTimestamp = msg.header.stamp.sec
        self.lead.mode = 0
        self.drill.mode = 0

        if msg.buttons[0]:
            self.drill.value = self.drill_direction  # drill go brrrrrr
        else:
            self.drill.value = 0.0

        if msg.buttons[3]:
            self.drill = -(self.drill_direction)  # switch drill direction

        if msg.buttons[14]:
            self.lead.value = 0.1  # leadscrew up?
        elif msg.buttons[15]:
            self.lead.value = -0.1  # leadscrew down?
        else:
            self.lead.value = 0.0

        if msg.buttons[9]:
            self.estop.data = True
            self.estopTimestamp = msg.header.stamp.sec
        if msg.buttons[8]:
            self.estop.data = False


def main(args=None):
    rclpy.init(args=args)
    node = joystickDrillController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
