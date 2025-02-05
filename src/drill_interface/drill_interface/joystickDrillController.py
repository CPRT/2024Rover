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
        super().__init__("joystickDrillControl")
        
        
        self.lead = MotorControl()
        self.drill = MotorControl()
        
        self.leadCommand = self.create_publisher(MotorControl, "/lead/set", 1)
        self.drillCommand = self.create_publisher(MotorControl, "/drill/set", 1)
        
        self.joystick = self.create_subscription(
            Joy, "/joy", self.joy_callback, 5
        )

        freq = 10
        self.rate = self.create_rate(freq)
        period = 1 / freq
        self.timer = self.create_timer(period, self.controlPublisher)
        
    def controlPublisher(self):
        self.leadCommand.publish(self.lead)
        self.drillCommand.publish(self.drill)
        
    def joy_callback(self, msg: Joy):
        self.lastTimestamp = msg.header.stamp.sec
        self.lead.mode = 0
        self.drill.mode = 0
        
        if msg.buttons[0]:  # Drill go brrrr
            self.drill.value = 1.0
        elif msg.buttons[1]:  # Drill go brrrr
            self.drill.value = -1.0
        else:
            self.drill.value = 0.0
            
        if msg.buttons[12]:  # Lead go up
            self.lead.value = -0.50
        elif msg.buttons[13]:  # Lead go down
            self.lead.value = 0.50
        else:
            self.lead.value = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = joystickDrillController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
