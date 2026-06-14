#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from transforms3d.euler import quat2euler

from .utils import make_pointcloud2


class Mapper(Node):
    def __init__(self):
        super().__init__("mapper")

        # Pose courante du robot (mise à jour par /odom)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Carte accumulée : listes de coordonnées monde
        self.map_x = []
        self.map_y = []
        self.map_c = []

        self.pub = self.create_publisher(PointCloud2, "map", 10)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.callback_odom, 10)
        self.sub_pts = self.create_subscription(PointCloud2, "clusters", self.callback_points, 10)

    def callback_odom(self, msg: Odometry):
        # Position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Orientation : quaternion -> yaw (theta)
        q = msg.pose.pose.orientation
        _, _, self.theta = quat2euler([q.w, q.x, q.y, q.z])

    def callback_points(self, msg: PointCloud2):
        points = read_points_numpy(msg, ["x", "y", "clusterId"])
        if len(points) == 0:
            return


        # Ne garder que les points appartenant à un cluster (élimine le bruit isolé)
        points = points[points[:, 2] != 0]
        if len(points) == 0:
            return


        #  Filtre de distance, on ne garde que les points proches du robot
        seuil_dist = 0.6   # mètres
        r = np.linalg.norm(points[:, :2], axis=1)   # distance de chaque point au robot
        points = points[r < seuil_dist]
        if len(points) == 0:
            return
 

        # Matrice de rotation 2D selon la pose courante
        c, s = np.cos(self.theta), np.sin(self.theta)
        R = np.array([[c, -s], [s, c]])
        

        # Transformation repère robot -> repère monde : R * point + translation
        xy_local = points[:, :2]                      # (N, 2)
        xy_world = xy_local @ R.T + np.array([self.x, self.y])

        # Accumulation
        self.map_x.extend(xy_world[:, 0].tolist())
        self.map_y.extend(xy_world[:, 1].tolist())
        self.map_c.extend(points[:, 2].tolist())

        # Republication de toute la carte dans le repère "odom"
        header = msg.header
        header.frame_id = "odom"
        intensities = np.zeros(len(self.map_x))

        #  Déduplication par voxel (rend la carte plus lisible et plus légère)
        voxel = 0.02
        xs = np.array(self.map_x)
        ys = np.array(self.map_y)
        cs = np.array(self.map_c)

        # Sécurité : aligner les longueurs
        n_pts = min(len(xs), len(ys), len(cs))
        xs, ys, cs = xs[:n_pts], ys[:n_pts], cs[:n_pts]

        # Un représentant par case voxel
        keys = np.round(np.column_stack((xs, ys)) / voxel).astype(int)
        _, unique_idx = np.unique(keys, axis=0, return_index=True)

        xs_v = xs[unique_idx]
        ys_v = ys[unique_idx]
        cs_v = cs[unique_idx]
        intensities = np.zeros(len(xs_v))

        header = msg.header
        header.frame_id = "odom"
        self.pub.publish(make_pointcloud2(header, xs_v, ys_v, intensities, cs_v))



def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(Mapper())
    except KeyboardInterrupt:
        pass