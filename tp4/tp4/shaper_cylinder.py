#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from visualization_msgs.msg import MarkerArray

from .utils import make_markers


class ShaperCylinder(Node):
    def __init__(self):
        super().__init__("shaper_cylinder")
        self.pub = self.create_publisher(MarkerArray, "cylinders", 10)
        self.sub = self.create_subscription(PointCloud2, "clusters", self.callback, 10)

    def callback(self, msg: PointCloud2):
        # Decodes the points in a numpy array of shape [[x0, y0, c0], [x1, y1, c1], ...]
        points = read_points_numpy(msg, ["x", "y", "clusterId"])

        clusters = []
        # TODO: Group xy by cluster ids
        # ...

        cylinders = []
        # TODO: Fit cylinders (x, y, radius) around each cluster
        # ...

        self.pub.publish(make_markers(msg.header, cylinders))


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(ShaperCylinder())
    except KeyboardInterrupt:
        pass
