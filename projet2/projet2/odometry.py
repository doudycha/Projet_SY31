#!/usr/bin/env python3
"""Odométrie encodeurs + gyromètre (sujet 1.2 — Cartographie guidée).

Le robot maintient son odométrie à partir :
  - des encodeurs (/sensor_state) → vitesse linéaire v
  - du gyromètre  (/imu)          → vitesse angulaire ω puis orientation θ

Cette pose (x, y, θ) est ensuite réutilisée par le nœud `mapper` pour accumuler
les points LiDAR dans un repère fixe.

Modèle cinématique (identique au TP3) :
    φ_roue = dq · 2π / (N · dt)          vitesse angulaire de chaque roue
    v      = r · (φ_g + φ_d) / 2         vitesse linéaire du robot
    θ     += ω_z · dt                    orientation intégrée depuis le gyro
    x     += v · dt · cos(θ)
    y     += v · dt · sin(θ)

Publications :
  /odom_est   nav_msgs/Odometry      pose estimée (x, y, θ) + vitesses
  /trajectory sensor_msgs/PointCloud2 trace de la trajectoire (pour RViz)
  TF odom → base_link                 pour afficher le repère robot dans RViz
"""
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from tf2_ros import TransformBroadcaster

try:
    from turtlebot3_msgs.msg import SensorState
except ImportError as e:  # pragma: no cover
    raise ImportError(
        "turtlebot3_msgs introuvable. Installer : "
        "sudo apt-get install -y ros-jazzy-turtlebot3-msgs"
    ) from e

from .utils import declare_param, make_pointcloud2


class Odometry2D(Node):
    # Constantes robot TurtleBot3 (identiques au TP3)
    ENCODER_RESOLUTION = 4096      # impulsions/tour
    WHEEL_RADIUS = 0.033           # m
    WHEEL_SEPARATION = 0.160       # m (2L)

    def __init__(self):
        super().__init__("odometry")

        # Pose et vitesse courantes
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0               # vitesse linéaire (calculée par les encodeurs)

        # Mémoire des messages précédents
        self.prev_left = None
        self.prev_right = None
        self.prev_enc_t = None
        self.prev_imu_t = None

        # Trace de la trajectoire (liste de [x, y])
        self.traj_x = []
        self.traj_y = []

        # Sous-échantillonnage de la trajectoire (1 point tous les `traj_decim`)
        declare_param(self, "traj_decimation", 5)
        self._imu_count = 0

        # Publications
        self.pub_odom = self.create_publisher(Odometry, "/odom_est", 10)
        self.pub_traj = self.create_publisher(PointCloud2, "/trajectory", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Souscriptions : encodeurs (v) et gyro (θ)
        self.sub_enc = self.create_subscription(
            SensorState, "/sensor_state", self.callback_encoders, 10
        )
        self.sub_imu = self.create_subscription(
            Imu, "/imu", self.callback_gyro, 50
        )

        self.get_logger().info("Nœud odometry démarré (encodeurs + gyromètre)")

    # ------------------------------------------------------------------ #
    # Encodeurs : calcul de la vitesse linéaire v                         #
    # ------------------------------------------------------------------ #
    def callback_encoders(self, msg: SensorState):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Premier message : on initialise sans calculer
        if self.prev_left is None:
            self.prev_left = msg.left_encoder
            self.prev_right = msg.right_encoder
            self.prev_enc_t = t
            return

        dt = t - self.prev_enc_t
        if dt <= 0:
            return

        dq_left = msg.left_encoder - self.prev_left
        dq_right = msg.right_encoder - self.prev_right
        self.prev_left = msg.left_encoder
        self.prev_right = msg.right_encoder
        self.prev_enc_t = t

        # Impulsions → angle de rotation de chaque roue
        ang_left = dq_left * 2.0 * np.pi / self.ENCODER_RESOLUTION
        ang_right = dq_right * 2.0 * np.pi / self.ENCODER_RESOLUTION

        # Angle → vitesse linéaire de chaque roue, puis moyenne
        v_left = ang_left * self.WHEEL_RADIUS / dt
        v_right = ang_right * self.WHEEL_RADIUS / dt
        self.v = (v_left + v_right) / 2.0

    # ------------------------------------------------------------------ #
    # Gyromètre : intégration de θ et de la position                      #
    # ------------------------------------------------------------------ #
    def callback_gyro(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.prev_imu_t is None:
            self.prev_imu_t = t
            return

        dt = t - self.prev_imu_t
        self.prev_imu_t = t
        if dt <= 0:
            return

        # Orientation par intégration de la vitesse angulaire autour de z
        w = msg.angular_velocity.z
        self.theta += w * dt

        # Position par intégration de la vitesse linéaire (issue des encodeurs)
        self.x += self.v * dt * np.cos(self.theta)
        self.y += self.v * dt * np.sin(self.theta)

        self.publish_odometry(msg.header.stamp, w)
        self.publish_tf(msg.header.stamp)
        self.update_trajectory(msg.header)

    # ------------------------------------------------------------------ #
    # Publications                                                        #
    # ------------------------------------------------------------------ #
    def publish_odometry(self, stamp, w):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        # Quaternion d'une rotation pure autour de z (yaw = θ)
        odom.pose.pose.orientation.z = np.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = np.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = w
        self.pub_odom.publish(odom)

    def publish_tf(self, stamp):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = np.sin(self.theta / 2.0)
        tf.transform.rotation.w = np.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(tf)

    def update_trajectory(self, header):
        self._imu_count += 1
        if self._imu_count % max(1, self.traj_decimation) != 0:
            return
        self.traj_x.append(self.x)
        self.traj_y.append(self.y)

        header.frame_id = "odom"
        self.pub_traj.publish(make_pointcloud2(header, self.traj_x, self.traj_y))


def main(args=None):
    rclpy.init(args=args)
    node = Odometry2D()
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
