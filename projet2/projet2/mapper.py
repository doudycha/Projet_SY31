#!/usr/bin/env python3
"""Cartographie du labyrinthe par accumulation de points LiDAR (sujet 1.2).

Chaque scan LiDAR (repère robot L) est replacé dans le repère fixe (repère 0)
à l'aide de la matrice de transformation homogène donnée dans l'énoncé :

    ⎡x₁ x₂ …⎤   ⎡cos θ  −sin θ  x⎤   ⎡x₁ x₂ …⎤
    ⎢y₁ y₂ …⎥ = ⎢sin θ   cos θ  y⎥ · ⎢y₁ y₂ …⎥
    ⎣ 1  1 …⎦   ⎣  0       0    1⎦   ⎣ 1  1 …⎦
       (0)                              (L)

où (x, y, θ) est la pose du robot fournie par le nœud `odometry` (/odom_est).

Traitements (repris du TP4) :
  - filtrage des distances (trop proches = robot lui-même, trop loin = bruit)
  - clustering séquentiel pour ne garder que les structures (murs) cohérentes
  - accumulation dans une grille de voxels (déduplication spatiale)

Bonus (sujet) : les flèches détectées (/direction_cmd) sont propagées sur la
carte sous forme de markers colorés, posés à la position courante du robot.

Publications :
  /map_points    PointCloud2   carte accumulée du labyrinthe (repère "odom")
  /filtered_scan PointCloud2   scan courant filtré et transformé (debug)
  /arrow_map     MarkerArray   flèches propagées sur la carte (bonus)
"""
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header, String
from visualization_msgs.msg import MarkerArray

from .utils import declare_param, make_markers, make_pointcloud2

# Couleur RViz selon la direction de la flèche
ARROW_COLORS = {
    "left": (1.0, 0.0, 0.0),    # rouge = tourner à gauche
    "right": (0.0, 0.4, 1.0),   # bleu  = tourner à droite
}


class Mapper(Node):
    def __init__(self):
        super().__init__("mapper")

        # --- Paramètres (réglables à chaud) ---
        declare_param(self, "min_range", 0.08)        # m, ignore le robot lui-même
        declare_param(self, "max_range", 3.5)         # m, ignore le lointain/bruit
        declare_param(self, "voxel_size", 0.02)       # m, résolution de la carte
        declare_param(self, "cluster_dist", 0.08)     # m, distance max intra-cluster
        declare_param(self, "min_cluster_size", 4)    # points min pour valider
        declare_param(self, "process_every_n", 1)     # traite 1 scan sur n

        # --- État de la pose (depuis /odom_est) ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pose_ready = False

        # --- Carte : ensemble de voxels occupés (dédup) ---
        self.voxels = set()
        self.scan_buffer = deque(maxlen=3)
        self.scan_count = 0

        # --- Bonus : flèches propagées sur la carte ---
        self.last_direction = "none"
        self.arrow_points = []   # liste de (x, y)
        self.arrow_cols = []     # couleur associée

        # --- Publications ---
        self.pub_map = self.create_publisher(PointCloud2, "/map_points", 10)
        self.pub_filtered = self.create_publisher(PointCloud2, "/filtered_scan", 10)
        self.pub_arrows = self.create_publisher(MarkerArray, "/arrow_map", 10)

        # --- Souscriptions ---
        self.create_subscription(Odometry, "/odom_est", self.callback_odom, 10)
        self.create_subscription(String, "/direction_cmd", self.callback_direction, 10)
        # Le bag publie /scan en BEST_EFFORT : le QoS du subscriber doit correspondre
        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, "/scan", self.callback_scan, scan_qos)

        # Publication périodique de la carte complète
        self.create_timer(0.5, self.publish_map)

        self.get_logger().info("Nœud mapper démarré (accumulation LiDAR + clustering)")

    # ------------------------------------------------------------------ #
    # Pose du robot                                                       #
    # ------------------------------------------------------------------ #
    def callback_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # Quaternion (z, w) → yaw
        self.theta = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.x = p.x
        self.y = p.y
        self.pose_ready = True

    def callback_direction(self, msg: String):
        """Bonus : à chaque NOUVELLE flèche, on pose un marker à la position robot."""
        d = msg.data
        if d in ARROW_COLORS and d != self.last_direction and self.pose_ready:
            self.arrow_points.append((self.x, self.y))
            self.arrow_cols.append(ARROW_COLORS[d])
            self.publish_arrow_markers()
        self.last_direction = d

    # ------------------------------------------------------------------ #
    # Scan LiDAR → carte                                                  #
    # ------------------------------------------------------------------ #
    def callback_scan(self, msg: LaserScan):
        if not self.pose_ready:
            return

        self.scan_count += 1

        # 1) Polaire → cartésien dans le repère robot, avec filtrage de distance
        ranges = np.asarray(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        valid = np.isfinite(ranges) & (ranges >= self.min_range) & (ranges <= self.max_range)
        if not np.any(valid):
            return
        r = ranges[valid]
        a = angles[valid]
        x_local = r * np.cos(a)
        y_local = r * np.sin(a)

        # 2) Transformation repère robot (L) → repère fixe (0) : matrice homogène 3×3
        ct, st = np.cos(self.theta), np.sin(self.theta)
        T = np.array([
            [ct, -st, self.x],
            [st,  ct, self.y],
            [0.0, 0.0, 1.0],
        ])
        # Points en coordonnées homogènes : [[x, y, 1], ...].T → (3, n)
        P_local = np.vstack((x_local, y_local, np.ones_like(x_local)))
        P_global = T @ P_local
        points_global = P_global[:2].T   # (n, 2)

        # Publication du scan filtré transformé (debug)
        self.publish_cloud(self.pub_filtered, points_global, msg.header.stamp)

        # 3) Bufferisation + clustering périodique
        self.scan_buffer.append(points_global)
        if self.scan_count % max(1, self.process_every_n) == 0 and len(self.scan_buffer) >= 1:
            self.accumulate(np.vstack(self.scan_buffer))

    # ------------------------------------------------------------------ #
    # Clustering (TP4) + accumulation voxels                              #
    # ------------------------------------------------------------------ #
    def accumulate(self, points: np.ndarray):
        if len(points) > 1500:   # borne le coût pour rester temps réel
            idx = np.random.choice(len(points), 1500, replace=False)
            points = points[idx]

        labels = self.cluster(points)
        for lab in np.unique(labels[labels > 0]):
            cluster = points[labels == lab]
            if len(cluster) < self.min_cluster_size:
                continue   # rejette les petits clusters = bruit
            for px, py in cluster:
                self.voxels.add((
                    int(np.floor(px / self.voxel_size)),
                    int(np.floor(py / self.voxel_size)),
                ))

    def cluster(self, points: np.ndarray) -> np.ndarray:
        """Clustering séquentiel angulaire (adapté du TP4).

        Les points sont triés par angle autour du robot, puis un nouveau cluster
        démarre dès qu'un saut de distance > cluster_dist apparaît entre voisins.
        """
        n = len(points)
        if n < 2:
            return np.zeros(n, dtype=int)

        order = np.argsort(np.arctan2(points[:, 1] - self.y, points[:, 0] - self.x))
        sorted_pts = points[order]

        labels_sorted = np.zeros(n, dtype=int)
        cur = 1
        labels_sorted[0] = cur
        for i in range(1, n):
            if np.linalg.norm(sorted_pts[i] - sorted_pts[i - 1]) > self.cluster_dist:
                cur += 1
            labels_sorted[i] = cur

        labels = np.zeros(n, dtype=int)
        labels[order] = labels_sorted
        return labels

    # ------------------------------------------------------------------ #
    # Publications                                                        #
    # ------------------------------------------------------------------ #
    def publish_cloud(self, pub, points, stamp):
        if len(points) == 0:
            return
        header = Header(stamp=stamp, frame_id="odom")
        pub.publish(make_pointcloud2(header, points[:, 0], points[:, 1]))

    def publish_map(self):
        if len(self.voxels) < 5:
            return
        xs, ys = [], []
        for vx, vy in self.voxels:
            xs.append((vx + 0.5) * self.voxel_size)
            ys.append((vy + 0.5) * self.voxel_size)
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="odom")
        self.pub_map.publish(make_pointcloud2(header, xs, ys))

    def publish_arrow_markers(self):
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="odom")
        self.pub_arrows.publish(
            make_markers(header, self.arrow_points, self.arrow_cols, scale=0.12, ns="arrows")
        )


def main(args=None):
    rclpy.init(args=args)
    node = Mapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
