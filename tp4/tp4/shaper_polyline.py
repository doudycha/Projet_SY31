#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from visualization_msgs.msg import MarkerArray

from .utils import make_markers, declare_param


def dist_seg(point, segbeg, segend):
    """Computes point to segment distance

    :param point: Point to compute the distance with
    :param segbeg: Beginning point of the segment
    :param segend: Ending point of the segment
    :return: Distance between point and the [segbeg, segend] segment
    """
    vec = segend - segbeg
    vec /= np.linalg.norm(vec)
    vecT = np.array([-vec[1], vec[0]])
    return np.abs(np.sum((point - segbeg) * vecT))


class ShaperPolyline(Node):
    def __init__(self):
        super().__init__("shaper_polyline")
        declare_param(self, "eps", 0.05)

        self.pub = self.create_publisher(MarkerArray, "polylines", 10)
        self.sub = self.create_subscription(PointCloud2, "clusters", self.callback, 10)

    def callback(self, msg: PointCloud2):
        points = read_points_numpy(msg, ["x", "y", "clusterId"])

        clusters = []
        # TODO: Group xy by cluster ids
        # ...

        # Sort points in clusters clockwise
        clusters = [c[np.argsort(np.arctan2(c[:, 1], c[:, 0]))] for c in clusters]

        polylines = []
        for cluster in clusters:
            polylines.append(self.rdp(cluster))

        self.pub.publish(make_markers(msg.header, polylines, self.eps))

    def rdp(self, xy):
        """Simplifies a polyline using the Ramer-Douglas-Peucker algorithm

        :note: This function is recursive.
        :param points: Polyline to simplify of the shape [[x, y], [x, y], ...]
        :return: Simplified polyline of the shape [[x, y], [x, y], ...]
        """
        # TODO: Implement the RDP algorithm using self.eps
        # ...


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(ShaperPolyline())
    except KeyboardInterrupt:
        pass
