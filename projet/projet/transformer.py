#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2

from .utils import make_pointcloud2, declare_param


class Transformer(Node):
    def __init__(self):
        super().__init__("transformer")
        self.pub = self.create_publisher(PointCloud2, "points", 10)
        self.pub_pos = self.create_publisher(PointStamped, "robot_position", 10)
        # Le bag publie /scan en BEST_EFFORT : le subscriber doit avoir le même QoS,
        # sinon (RELIABLE par défaut) la souscription est incompatible et ne reçoit rien.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, "scan", self.callback, qos)

        # Positions (x, y) des 3 amers réfléchissants dans le référentiel fixe du labyrinthe.
        # La méthode de triangulation utilise les distances mesurées vers ces 3 points connus
        # pour calculer la position du robot par trilatération.
        # À calibrer selon la disposition réelle des marqueurs dans le labyrinthe.
        declare_param(self, "landmark_1", [0.0, 0.0])
        declare_param(self, "landmark_2", [2.0, 0.0])
        declare_param(self, "landmark_3", [1.0, 1.5])

    def callback(self, msg: LaserScan):
        x = []
        y = []
        intensities = []
        ranges_valid = []
        angles_valid = []

        for i, theta in enumerate(np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)):
            if msg.ranges[i] > 0.1:
                x.append(msg.ranges[i] * np.cos(theta))
                y.append(msg.ranges[i] * np.sin(theta))
                intensities.append(msg.intensities[i])
                ranges_valid.append(msg.ranges[i])
                angles_valid.append(theta)

        self.pub.publish(make_pointcloud2(msg.header, x, y, intensities))

        # Sélection des 3 amers et trilatération de la position du robot.
        idx = self.select_landmarks(intensities, angles_valid)
        if idx is not None:
            d1 = ranges_valid[idx[0]]
            d2 = ranges_valid[idx[1]]
            d3 = ranges_valid[idx[2]]

            rx, ry = self.triangulate(d1, d2, d3)

            pos = PointStamped()
            pos.header = msg.header
            pos.point.x = float(rx)
            pos.point.y = float(ry)
            self.pub_pos.publish(pos)

    def select_landmarks(self, intensities: list, angles: list) -> list | None:
        """Sélectionne les 3 meilleurs amers en combinant réflectivité et répartition angulaire.

        Le scan est divisé en 3 secteurs de 120° (0°-120°, 120°-240°, 240°-360°).
        Dans chaque secteur, on retient le point le plus réfléchissant.

        Cette double contrainte garantit :
          - une bonne répartition angulaire (les 3 points ne sont pas tous du même côté),
            ce qui conditionne bien la matrice A de la trilatération et évite la singularité,
          - la meilleure réflectivité disponible dans chaque direction.

        Retourne None si un secteur est vide (trilatération impossible).
        """
        intensities = np.array(intensities)
        # Ramène tous les angles dans [0, 2π] pour un découpage uniforme
        angles = np.array(angles) % (2 * np.pi)

        sector_size = 2 * np.pi / 3  # 120° par secteur
        best_indices = []

        for s in range(3):
            lo = s * sector_size
            hi = (s + 1) * sector_size
            in_sector = np.where((angles >= lo) & (angles < hi))[0]

            if len(in_sector) == 0:
                self.get_logger().warn(f"Secteur {s} vide — trilatération impossible")
                return None

            # Meilleur point du secteur = celui avec l'intensité maximale
            best_indices.append(in_sector[np.argmax(intensities[in_sector])])

        return best_indices

    def triangulate(self, d1: float, d2: float, d3: float) -> tuple[float, float]:
        """Calcule la position (rx, ry) du robot par trilatération à partir de 3 distances.

        Les distances d1, d2, d3 sont issues de la mesure LiDAR selon l'une des deux
        méthodes physiques suivantes (réalisées par le capteur) :

          - Temps de vol  : d = c * t / 2
              c = 3e8 m/s (vitesse de la lumière), t = durée aller-retour de l'impulsion laser

          - Déphasage     : d = (phi * c) / (4 * pi * f)
              phi = déphasage entre l'onde émise et l'onde reçue, f = fréquence de modulation

        La trilatération résout le système des 3 équations de cercles centrés sur les amers :
            (rx - x1)² + (ry - y1)² = d1²
            (rx - x2)² + (ry - y2)² = d2²
            (rx - x3)² + (ry - y3)² = d3²

        En soustrayant l'équation 1 aux équations 2 et 3, on obtient un système linéaire A·r = b :
            A = 2 * [[x2-x1, y2-y1],
                     [x3-x1, y3-y1]]
            b = [d1²-d2² - x1²+x2² - y1²+y2²,
                 d1²-d3² - x1²+x3² - y1²+y3²]
        """
        x1, y1 = self.landmark_1
        x2, y2 = self.landmark_2
        x3, y3 = self.landmark_3

        A = 2.0 * np.array([
            [x2 - x1, y2 - y1],
            [x3 - x1, y3 - y1],
        ])
        b = np.array([
            d1**2 - d2**2 - x1**2 + x2**2 - y1**2 + y2**2,
            d1**2 - d3**2 - x1**2 + x3**2 - y1**2 + y3**2,
        ])

        rx, ry = np.linalg.solve(A, b)
        return rx, ry


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(Transformer())
    except KeyboardInterrupt:
        pass
