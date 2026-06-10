#!/usr/bin/env python3
"""Affichage de la consigne de direction pour l'opérateur (sujet 1.2).

Reçoit /direction_cmd (String : "left", "right", "none") et publie une image
avec une flèche correspondante sur /direction_display, visualisable dans
rqt_image_view (plus robuste que cv2.imshow sous WSL).

    left  → flèche gauche  (rouge)
    right → flèche droite  (bleu)
    none  → tout droit     (vert)
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class DirectionDisplay(Node):
    def __init__(self):
        super().__init__("direction_display")
        self.bridge = CvBridge()
        self.direction = "none"

        self.pub = self.create_publisher(Image, "/direction_display", 10)
        self.create_subscription(String, "/direction_cmd", self.callback, 10)
        # Rafraîchit l'affichage à ~10 Hz, indépendamment du flux caméra
        self.create_timer(0.1, self.render)

        self.get_logger().info("Nœud direction_display démarré (voir /direction_display)")

    def callback(self, msg: String):
        self.direction = msg.data

    def render(self):
        img = np.zeros((300, 300, 3), dtype=np.uint8)
        cx, cy, L = 150, 150, 80

        if self.direction == "left":
            start, end, color, txt = (cx + L, cy), (cx - L, cy), (0, 0, 255), "GAUCHE"
        elif self.direction == "right":
            start, end, color, txt = (cx - L, cy), (cx + L, cy), (255, 0, 0), "DROITE"
        else:
            start, end, color, txt = (cx, cy + L), (cx, cy - L), (0, 255, 0), "TOUT DROIT"

        cv2.arrowedLine(img, start, end, color, 14, tipLength=0.35)
        cv2.putText(img, txt, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        self.pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = DirectionDisplay()
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
