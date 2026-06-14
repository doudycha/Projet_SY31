#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2

from tp4.utils import make_pointcloud2

class Transformer(Node):
    def __init__(self):
        super().__init__("transformer")
        self.pub = self.create_publisher(PointCloud2, "points", 10)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, "scan", self.callback, qos)

    def callback(self, msg: LaserScan):
        x = []
        y = []
        intensities = []

        for i, theta in enumerate(np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)):
            # TODO: Remove points too close
            if msg.ranges[i] > 0.1:
                # TODO: Use msg.ranges and theta to determine the Cartesian coordinates x and y
                x.append(msg.ranges[i] * np.cos(theta))
                y.append(msg.ranges[i] * np.sin(theta))

                # TODO: Extract the intensity
                intensities.append(msg.intensities[i])

        self.pub.publish(make_pointcloud2(msg.header, x, y, intensities))


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(Transformer())
    except KeyboardInterrupt:
        pass
