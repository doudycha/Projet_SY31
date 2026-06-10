#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
try:
    from turtlebot3_msgs.msg import SensorState
    TURTLEBOT3_MSGS_AVAILABLE = True
except ImportError:
    TURTLEBOT3_MSGS_AVAILABLE = False

from .utils import make_pointcloud2


class Pipeline(Node):
    """Nœud de fusion des 3 capteurs : caméra, LiDAR et odométrie.

    Sources :
      - (x, y)  : encodeurs via /sensor_state  → même méthode que callback_enco dans odompose.py
      - θ       : gyroscope via /imu            → même méthode que callback_gyro dans odompose.py
      - points  : LiDAR filtré via /points_filtered
      - couleur : caméra via /detections

    Les points LiDAR sont mis dans le référentiel global par une matrice de transformation
    homogène 3×3 combinant la rotation θ (gyroscope) et la translation (x, y) (encodeurs).
    """

    # Constantes robot — identiques à odompose.py
    N = 4096
    L = 80e-3
    r = 33e-3

    def __init__(self):
        super().__init__("pipeline")

        # --- État odométrique ---
        # Position (x, y) calculée depuis les encodeurs (callback_enco)
        self.x_odom = 0.0
        self.y_odom = 0.0
        self.O_odom = 0.0   # angle intégré depuis les encodeurs (pour la mise à jour de x, y)
        self.v      = 0.0   # vitesse linéaire courante, partagée avec le gyroscope

        # Angle θ calculé depuis le gyroscope (callback_gyro)
        self.O_gyro = 0.0

        # Encodeurs précédents
        self.prev_left_encoder  = 0.0
        self.prev_right_encoder = 0.0

        # Drapeaux : indiquent que les deux capteurs ont envoyé au moins un message
        self.enco_ready = False
        self.gyro_ready = False

        # Points accumulés dans le référentiel global
        self.map_x = []
        self.map_y = []
        self.map_i = []

        # Publishers
        self.pub_map = self.create_publisher(PointCloud2, "map_points", 10)

        # Subscribers — 3 capteurs
        if TURTLEBOT3_MSGS_AVAILABLE:
            self.sub_enco = self.create_subscription(
                SensorState, "/sensor_state", self.callback_enco, 10
            )
        else:
            self.get_logger().warn("turtlebot3_msgs non disponible — position encodeurs désactivée, (x,y) fixé à (0,0)")
        self.sub_gyro = self.create_subscription(
            Imu, "/imu", self.callback_gyro, 10
        )
        self.sub_points = self.create_subscription(
            PointCloud2, "points_filtered", self.callback_points, 10
        )
        self.sub_detections = self.create_subscription(
            Image, "detections", self.callback_detections, 10
        )

    # ------------------------------------------------------------------ #
    # Helper                                                               #
    # ------------------------------------------------------------------ #

    def dt_from_stamp(self, stamp, field: str) -> float:
        t = stamp.sec + stamp.nanosec / 1e9
        dt = t - getattr(self, field) if hasattr(self, field) else 0.0
        setattr(self, field, t)
        return dt

    # ------------------------------------------------------------------ #
    # Callbacks odométrie                                                  #
    # ------------------------------------------------------------------ #

    def callback_enco(self, sensor_state):
        """Calcule (x, y) du robot depuis les encodeurs.

        Méthode identique à callback_enco dans odompose.py.
        La position est intégrée avec O_odom (angle encodeurs),
        et self.v est partagé avec callback_gyro.
        """
        dq_left  = sensor_state.left_encoder  - self.prev_left_encoder
        dq_right = sensor_state.right_encoder - self.prev_right_encoder
        self.prev_left_encoder  = sensor_state.left_encoder
        self.prev_right_encoder = sensor_state.right_encoder

        dt = self.dt_from_stamp(sensor_state.header.stamp, "prev_enco_t")
        if dt <= 0:
            return

        phi_right = dq_right * 2 * np.pi / (dt * self.N)
        phi_left  = dq_left  * 2 * np.pi / (dt * self.N)
        v_right   = self.r * phi_right
        v_left    = self.r * phi_left
        self.v    = (v_right + v_left) / 2
        w         = self.r * (phi_right - phi_left) / (2 * self.L)

        self.x_odom = self.x_odom + self.v * dt * np.cos(self.O_odom)
        self.y_odom = self.y_odom + self.v * dt * np.sin(self.O_odom)
        self.O_odom = self.O_odom + w * dt

        self.enco_ready = True

    def callback_gyro(self, gyro: Imu):
        """Calcule θ du robot depuis le gyroscope.

        Méthode identique à callback_gyro dans odompose.py.
        self.v est mis à jour par callback_enco.
        """
        dt = self.dt_from_stamp(gyro.header.stamp, "prev_gyro_t")
        if dt <= 0:
            return

        z_angular_velocity = gyro.angular_velocity.z
        self.O_gyro = self.O_gyro + z_angular_velocity * dt

        self.gyro_ready = True

    # ------------------------------------------------------------------ #
    # Callback caméra                                                      #
    # ------------------------------------------------------------------ #

    def callback_detections(self, msg: Image):
        """Reçoit les détections couleur de la caméra (rouge/bleu).
        Extensible pour associer les détections colorées à des positions de la carte.
        """
        pass

    # ------------------------------------------------------------------ #
    # Callback principal : transformation LiDAR → référentiel global      #
    # ------------------------------------------------------------------ #

    def callback_points(self, msg: PointCloud2):
        """Transforme les points LiDAR dans le référentiel global et accumule la carte.

        Matrice de transformation homogène 3×3 :

            T = [ cos(θ)  -sin(θ)  x ]
                [ sin(θ)   cos(θ)  y ]
                [   0        0     1 ]

        où θ vient du gyroscope (O_gyro) et (x, y) viennent des encodeurs (x_odom, y_odom).

        Pour chaque point robot (x_r, y_r) :
            [ x_g ]   [ x_r ]
            [ y_g ] = T · [ y_r ]
            [  1  ]   [  1  ]
        """
        if not self.gyro_ready:
            return

        theta  = self.O_gyro
        x_odom = self.x_odom
        y_odom = self.y_odom

        # Décode les points LiDAR filtrés en tableau numpy [[x, y, intensité], ...]
        points = read_points_numpy(msg, ["x", "y", "intensity"])
        if len(points) == 0:
            return

        # Matrice de transformation homogène 3×3
        T = np.array([
            [np.cos(theta), -np.sin(theta), x_odom],
            [np.sin(theta),  np.cos(theta), y_odom],
            [0.0,            0.0,           1.0   ],
        ])

        # Coordonnées homogènes des points robot : [[x_r, y_r, 1], ...]
        ones = np.ones(len(points))
        points_h = np.column_stack([points[:, 0], points[:, 1], ones])

        # Application de la transformation : T · p pour chaque point
        transformed = (T @ points_h.T).T

        x_global = transformed[:, 0]
        y_global = transformed[:, 1]

        # Accumulation dans la carte globale
        self.map_x.extend(x_global.tolist())
        self.map_y.extend(y_global.tolist())
        self.map_i.extend(points[:, 2].tolist())

        map_header = msg.header
        map_header.frame_id = "odom"
        self.pub_map.publish(
            make_pointcloud2(map_header, self.map_x, self.map_y, self.map_i)
        )


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(Pipeline())
    except KeyboardInterrupt:
        pass
