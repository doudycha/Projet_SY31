#!/usr/bin/env python3

#Pour chaque noeud, remplir la fonction et le message à envoyer
#Utiliser plusieurs noeuds pour le projet

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud, read_points_numpy

from .utils import make_pointcloud2, declare_param


class Clusterer(Node):
    def __init__(self):
        super().__init__("clusterer")

        # TODO: Determine parameter values
        declare_param(self, "k", 0)
        declare_param(self, "D", 0.0)

        self.pub = self.create_publisher(PointCloud2, "clusters", 10)
        self.sub = self.create_subscription(PointCloud2, "points_filtered", self.callback, 10)

    def callback(self, msg: PointCloud2):
        # Decodes the points in a numpy array of shape [[x0, y0, i0, c0], [x1, y1, i1, c1], ...]
        points = read_points_numpy(msg, ["x", "y", "intensity", "clusterId"])

        # TODO: Cluster the points by filling their clusterId field (points[:, 3])
        # Note: Use self.k, self.D to replace the algorithm parameters
        # ...

        self.pub.publish(make_pointcloud2(msg.header, *points.T))


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(Clusterer())
    except KeyboardInterrupt:
        pass
