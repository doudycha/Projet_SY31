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
        for c in np.unique(points[:, 2]):
            if c == 0:
                continue  # 0 = points sans cluster, on les ignore
            clusters.append(points[points[:, 2] == c][:, :2])

        cylinders = []
        # TODO: Fit cylinders (x, y, radius) around each cluster
        for cluster in clusters:
            center = cluster.mean(axis=0)                         # (cx, cy)
            radius = np.linalg.norm(cluster - center, axis=1).max()  # point le plus loin
            cylinders.append((center[0], center[1], radius))

        self.pub.publish(make_markers(msg.header, cylinders))


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(ShaperCylinder())
    except KeyboardInterrupt:
        pass
