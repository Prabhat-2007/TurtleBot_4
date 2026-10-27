"""Minimal example of reading the lidar and the stereo pair from your own code.

Run:  ros2 run tb4_drive sensor_demo
Prints nearest-obstacle distance, and size / rate of both camera images once per second.
Images arrive as sensor_msgs/Image (rgb8); convert with numpy, no cv_bridge needed:
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan


class SensorDemo(Node):
    def __init__(self):
        super().__init__('sensor_demo')
        self.counts = {'left': 0, 'right': 0}
        self.scan = None
        self.last_img = {}
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        for eye in ('left', 'right'):
            self.create_subscription(Image, f'stereo/{eye}/image_raw',
                                     lambda m, e=eye: self.on_image(e, m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, 'stereo/right/camera_info', self.on_info,
                                 qos_profile_sensor_data)
        self.baseline = None
        self.create_timer(1.0, self.report)

    def on_scan(self, msg):
        self.scan = msg

    def on_image(self, eye, msg):
        self.counts[eye] += 1
        self.last_img[eye] = msg

    def on_info(self, msg):
        if msg.p[3] != 0.0 and msg.p[0] != 0.0:
            self.baseline = -msg.p[3] / msg.p[0]

    def report(self):
        parts = []
        if self.scan is not None:
            r = np.array(self.scan.ranges, dtype=float)
            ok = np.isfinite(r) & (r > self.scan.range_min)
            if ok.any():
                i = int(np.argmin(np.where(ok, r, np.inf)))
                ang = math.degrees(self.scan.angle_min + i * self.scan.angle_increment)
                parts.append(f'lidar: {ok.sum()} pts, nearest {r[i]:.2f} m @ {ang:.0f} deg (lidar frame)')
        for eye in ('left', 'right'):
            m = self.last_img.get(eye)
            if m is not None:
                parts.append(f'{eye}: {m.width}x{m.height} {m.encoding} {self.counts[eye]} Hz')
            self.counts[eye] = 0
        if self.baseline is not None:
            parts.append(f'baseline {self.baseline * 100:.1f} cm')
        self.get_logger().info(' | '.join(parts) if parts else 'waiting for data...')


def main():
    rclpy.init()
    node = SensorDemo()
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
