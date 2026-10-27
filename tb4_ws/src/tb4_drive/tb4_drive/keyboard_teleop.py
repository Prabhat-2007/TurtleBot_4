"""Keyboard teleop -> cmd_vel_raw (goes through velocity_gate).

   w / s : forward / backward        a / d : turn left / right
   x or space : stop                 q / z : speed up / down
   Ctrl-C : quit
Speeds are sticky: the last command is repeated until you change or stop it.
"""
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

HELP = __doc__


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('angular_speed', 1.0)
        self.lin_speed = self.get_parameter('linear_speed').value
        self.ang_speed = self.get_parameter('angular_speed').value
        self.lin = 0.0
        self.ang = 0.0
        self.pub = self.create_publisher(Twist, 'cmd_vel_raw', 10)
        self.create_timer(0.1, self.publish)

    def publish(self):
        msg = Twist()
        msg.linear.x, msg.angular.z = self.lin, self.ang
        self.pub.publish(msg)

    def handle(self, key):
        if key == 'w':
            self.lin = self.lin_speed
        elif key == 's':
            self.lin = -self.lin_speed
        elif key == 'a':
            self.ang = self.ang_speed
        elif key == 'd':
            self.ang = -self.ang_speed
        elif key in ('x', ' '):
            self.lin = self.ang = 0.0
        elif key == 'q':
            self.lin_speed = min(self.lin_speed * 1.1, 0.46)
            self.ang_speed = min(self.ang_speed * 1.1, 1.9)
        elif key == 'z':
            self.lin_speed *= 0.9
            self.ang_speed *= 0.9
        else:
            return
        self.get_logger().info('cmd: linear=%.2f angular=%.2f (speeds %.2f / %.2f)'
                               % (self.lin, self.ang, self.lin_speed, self.ang_speed))


def main():
    if not sys.stdin.isatty():
        print('keyboard_teleop needs an interactive terminal')
        return
    rclpy.init()
    node = KeyboardTeleop()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if select.select([sys.stdin], [], [], 0)[0]:
                node.handle(sys.stdin.read(1))
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.lin = node.ang = 0.0
        node.publish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
