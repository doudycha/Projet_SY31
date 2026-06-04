#! /usr/bin/env python3

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


class Detector(Node):
    def __init__(self):
        super().__init__("detector")

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, "detections", 10)
        self.sub = self.create_subscription(CompressedImage, "turtlecam/image_raw/compressed", self.callback, 10)

    def callback(self, msg: CompressedImage):
        """Process the images going on image_raw/compressed"""

        # Convert CompressedImage -> OpenCV
        try:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if img is None:
                self.get_logger().warn("Failed to decode compressed image")
                return
        except Exception as e:
            self.get_logger().warn(f"Compressed->OpenCV {e}")
            return

        img_out = self.detect(img)

        # Convert OpenCV -> ROS
        try:
            format = "bgr8" if img_out.ndim == 3 else "mono8"
            msg_out = self.bridge.cv2_to_imgmsg(img_out, format)
        except CvBridgeError as e:
            self.get_logger().warn(f"ROS->OpenCV {e}")
            return

        self.pub.publish(msg_out)

    def detect(self, img: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # --- Détection rouge ---
        # Le rouge occupe deux zones dans l'espace HSV (autour de 0° et 180°)
        lower1 = np.array([0, 100, 100])
        upper1 = np.array([10, 255, 255])

        lower2 = np.array([170, 100, 100])
        upper2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)

        mask_red = cv2.bitwise_or(mask1, mask2)

        contours_red, hierarchy = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours_red:
            max_contour = max(contours_red, key=cv2.contourArea)
            M = cv2.moments(max_contour)
            area = cv2.contourArea(max_contour)
            perimeter = cv2.arcLength(max_contour, True)
            epsilon = 0.1 * cv2.arcLength(max_contour, True)
            approx = cv2.approxPolyDP(max_contour, epsilon, True)

        cv2.drawContours(img, contours_red, -1, (0, 255, 0), 3)

        # --- Détection bleu ---
        lower_blue = np.array([100, 100, 50])
        upper_blue = np.array([130, 255, 255])

        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        contours_blue, hierarchy = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours_blue:
            max_contour = max(contours_blue, key=cv2.contourArea)
            M = cv2.moments(max_contour)
            area = cv2.contourArea(max_contour)
            perimeter = cv2.arcLength(max_contour, True)
            epsilon = 0.1 * cv2.arcLength(max_contour, True)
            approx = cv2.approxPolyDP(max_contour, epsilon, True)

        cv2.drawContours(img, contours_blue, -1, (0, 255, 255), 3)

        return img


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
