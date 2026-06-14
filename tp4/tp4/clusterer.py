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
        declare_param(self, "k", 5)
        declare_param(self, "D", 0.1)
        declare_param(self, "min_size", 3)
        declare_param(self, "max_size", 100)
    

        self.pub = self.create_publisher(PointCloud2, "clusters", 10)
        self.sub = self.create_subscription(PointCloud2, "points_filtered", self.callback, 10)

    def callback(self, msg: PointCloud2):
        # Decodes the points in a numpy array of shape [[x0, y0, i0, c0], [x1, y1, i1, c1], ...]
        points = read_points_numpy(msg, ["x", "y", "intensity", "clusterId"])

        # TODO: Cluster the points by filling their clusterId field (points[:, 3])
        # Note: Use self.k, self.D to replace the algorithm parameters
        n = len(points)
        for i in range(n):
            d = []
            for j in range(1, self.k + 1):
                if i - j < 0:
                    break
                d.append(np.linalg.norm(points[i, :2] - points[i - j, :2]))

            if len(d) == 0:
                continue

            d = np.array(d)
            d_min = d.min()
            j_min = d.argmin() + 1

            r = np.linalg.norm(points[i, :2])          # distance du point au robot
            if d_min < self.D * r:                      # seuil proportionnel à la distance
                if points[i - j_min, 3] == 0:
                    points[i - j_min, 3] = points[:, 3].max() + 1
                points[i, 3] = points[i - j_min, 3]
        # Exercice 9 : fusionner le premier et le dernier cluster (le scan boucle)
        if n > self.k:
            for i in range(self.k):          # premiers points du scan
                for j in range(1, self.k + 1):
                    if np.linalg.norm(points[i, :2] - points[-j, :2]) < self.D:
                        c_start = points[i, 3]
                        c_end = points[-j, 3]
                        if c_start != 0 and c_end != 0 and c_start != c_end:
                            # tous les points du cluster de fin prennent l'ID du cluster de début
                            points[points[:, 3] == c_end, 3] = c_start
        # Exercice 8 : filtrage par taille de cluster                    
        for c in np.unique(points[:, 3]):
                if c == 0:
                    continue
                taille = np.sum(points[:, 3] == c)
                if taille < self.min_size or taille > self.max_size:
                    points[points[:, 3] == c, 3] = 0

        self.pub.publish(make_pointcloud2(msg.header, *points.T))


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(Clusterer())
    except KeyboardInterrupt:
        pass
