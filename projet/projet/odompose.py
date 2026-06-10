#!/usr/bin/env python3
# Adapté de tp3/tp3/odom2pose.py

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
try:
    from turtlebot3_msgs.msg import SensorState
    TURTLEBOT3_MSGS_AVAILABLE = True
except ImportError:
    TURTLEBOT3_MSGS_AVAILABLE = False
from transforms3d.euler import euler2quat


class OdomPose(Node):
    # Constants
    ENCODER_RESOLUTION = 4096
    WHEEL_RADIUS = 0.033
    WHEEL_SEPARATION = 0.160
    MAG_OFFSET = np.pi / 2.0 - 0.07

    def __init__(self):
        super().__init__("odom_to_pose")

        # Variables
        self.x_odom, self.y_odom, self.O_odom = 0.0, 0.0, 0.0
        self.x_gyro, self.y_gyro, self.O_gyro = 0.0, 0.0, 0.0
        self.x_magn, self.y_magn, self.O_magn = 0.0, 0.0, 0.0
        self.prev_left_encoder = 0.0
        self.prev_right_encoder = 0.0
        self.v = 0.0

        # Publishers
        self.pub_enco = self.create_publisher(PoseStamped, "/pose_enco", 10)
        self.pub_gyro = self.create_publisher(PoseStamped, "/pose_gyro", 10)
        self.pub_magn = self.create_publisher(PoseStamped, "/pose_magn", 10)

        # Subscribers
        self.sub_gyro = self.create_subscription(Imu, "/imu", self.callback_gyro, 10)
        if TURTLEBOT3_MSGS_AVAILABLE:
            self.sub_enco = self.create_subscription(
                SensorState, "/sensor_state", self.callback_enco, 10
            )
        else:
            self.get_logger().warn("turtlebot3_msgs non disponible — callback encodeurs désactivé")
        self.sub_magn = self.create_subscription(
            MagneticField, "/magnetic_field", self.callback_magn, 10
        )

    @staticmethod
    def coordinates_to_message(x: float, y: float, O: float, t: Time) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = t
        msg.header.frame_id = "odom"
        msg.pose.position.x = x
        msg.pose.position.y = y
        [
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
        ] = euler2quat(0.0, 0.0, O)
        return msg

    def dt_from_stamp(self, stamp: Time, field: str) -> float:
        t = stamp.sec + stamp.nanosec / 1e9
        dt = t - getattr(self, field) if hasattr(self, field) else 0.0
        setattr(self, field, t)
        return dt

    def callback_enco(self, sensor_state: SensorState):
        N = 4096
        L = 80*(10**(-3))
        r = 33*(10**(-3))
        dq_left = sensor_state.left_encoder - self.prev_left_encoder
        dq_right = sensor_state.right_encoder - self.prev_right_encoder
        self.prev_left_encoder = sensor_state.left_encoder
        self.prev_right_encoder = sensor_state.right_encoder

        dt = self.dt_from_stamp(sensor_state.header.stamp, "prev_enco_t")
        if dt <= 0:
            return

        phi_right = dq_right*2*np.pi/(dt*N)
        phi_left = dq_left*2*np.pi/(dt*N)
        v_right = r*phi_right
        v_left = r*phi_left
        self.v = (v_right+v_left)/2
        w = r*(phi_right-phi_left)/(2*L)

        self.x_odom = self.x_odom + self.v*dt*np.cos(self.O_odom)
        self.y_odom = self.y_odom + self.v*dt*np.sin(self.O_odom)
        self.O_odom = self.O_odom + w*dt

        self.pub_enco.publish(
            OdomPose.coordinates_to_message(
                self.x_odom, self.y_odom, self.O_odom, sensor_state.header.stamp
            )
        )

    def callback_gyro(self, gyro: Imu):
        dt = self.dt_from_stamp(gyro.header.stamp, "prev_gyro_t")
        if dt <= 0:
            return

        z_angular_velocity = gyro.angular_velocity.z
        self.O_gyro = self.O_gyro + z_angular_velocity*dt
        self.x_gyro = self.x_gyro + self.v*dt*np.cos(self.O_gyro)
        self.y_gyro = self.y_gyro + self.v*dt*np.sin(self.O_gyro)

        self.pub_gyro.publish(
            OdomPose.coordinates_to_message(
                self.x_gyro, self.y_gyro, self.O_gyro, gyro.header.stamp
            )
        )

    def callback_magn(self, magnetic_field: MagneticField):
        dt = self.dt_from_stamp(magnetic_field.header.stamp, "prev_magn_t")
        if dt <= 0:
            return

        self.O_magn = np.arctan2(magnetic_field.magnetic_field.y, magnetic_field.magnetic_field.x) - OdomPose.MAG_OFFSET
        self.x_magn = self.x_magn + self.v*dt*np.cos(self.O_magn)
        self.y_magn = self.y_magn + self.v*dt*np.sin(self.O_magn)

        self.pub_magn.publish(
            OdomPose.coordinates_to_message(
                self.x_magn, self.y_magn, self.O_magn, magnetic_field.header.stamp
            )
        )


def main(args=None):
    try:
        rclpy.init(args=args)
        node = OdomPose()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
