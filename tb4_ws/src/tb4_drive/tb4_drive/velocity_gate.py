"""Velocity gate: cmd_vel_raw -> cmd_vel.

Sits between whatever produces velocity commands (keyboard, Nav2, your own script) and the robot:
  * clamps to the TurtleBot 4 speed limits
  * ramps with acceleration limits (no wheel-slip jerks)
  * stops on command timeout
  * lidar safety: slows and stops linear motion toward a close obstacle (rotation stays free)
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from tb4_drive.safety import clamp, ramp, safe_linear, sector_min


class VelocityGate(Node):
    def __init__(self):
        super().__init__('velocity_gate')
        p = self.declare_parameter
        p('max_linear', 0.46)
        p('max_angular', 1.9)
        p('max_linear_accel', 0.8)       # m/s^2
        p('max_angular_accel', 4.0)      # rad/s^2
        p('rate', 30.0)                  # Hz
        p('cmd_timeout', 0.5)            # s without a command -> stop
        p('safety_enabled', True)
        p('stop_distance', 0.35)         # m from the lidar centre (robot edge is ~0.21 m away)
        p('slow_distance', 0.80)         # m, linear speed fades out between slow and stop
        p('front_half_angle', 0.6)       # rad, half-width of the forward / rear check sector
        p('lidar_yaw', math.pi / 2)      # yaw of rplidar_link in the robot frame (URDF mount)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.max_lin, self.max_ang = g('max_linear'), g('max_angular')
        self.acc_lin, self.acc_ang = g('max_linear_accel'), g('max_angular_accel')
        self.timeout = g('cmd_timeout')
        self.safety = g('safety_enabled')
        self.stop_d, self.slow_d = g('stop_distance'), g('slow_distance')
        self.half = g('front_half_angle')
        self.lidar_yaw = g('lidar_yaw')

        self.target = (0.0, 0.0)
        self.out = (0.0, 0.0)
        self.last_cmd = None
        self.front = math.inf
        self.rear = math.inf
        self.warned_block = False

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'cmd_vel_raw', self.on_cmd, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.dt = 1.0 / g('rate')
        self.create_timer(self.dt, self.tick)
        self.get_logger().info('velocity_gate: cmd_vel_raw -> cmd_vel (safety %s)'
                               % ('on' if self.safety else 'off'))

    def on_cmd(self, msg):
        self.target = (clamp(msg.linear.x, -self.max_lin, self.max_lin),
                       clamp(msg.angular.z, -self.max_ang, self.max_ang))
        self.last_cmd = self.get_clock().now()

    def on_scan(self, scan):
        args = (scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max)
        self.front = sector_min(*args, center=0.0, half_width=self.half, frame_yaw=self.lidar_yaw)
        self.rear = sector_min(*args, center=math.pi, half_width=self.half, frame_yaw=self.lidar_yaw)

    def tick(self):
        lin, ang = self.target
        if self.last_cmd is None or \
                (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9 > self.timeout:
            lin, ang = 0.0, 0.0

        if self.safety:
            safe = safe_linear(lin, self.front, self.rear, self.stop_d, self.slow_d)
            blocked = safe != lin
            if blocked and not self.warned_block:
                self.get_logger().warn('Obstacle close: limiting linear speed')
            self.warned_block = blocked
            lin = safe

        self.out = (ramp(self.out[0], lin, self.acc_lin * self.dt),
                    ramp(self.out[1], ang, self.acc_ang * self.dt))
        msg = Twist()
        msg.linear.x, msg.angular.z = self.out
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = VelocityGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
