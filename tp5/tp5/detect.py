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

        # TODO: Determine min and max pixel values
        #self.mask_min = np.array([0, 0, 0], dtype=np.uint8)
        #self.mask_max = np.array([255, 255, 255], dtype=np.uint8)
        #Mask min and max for red color
        self.mask_min = np.array([55, 45, 100], dtype=np.uint8)
        self.mask_max = np.array([85, 75, 137], dtype=np.uint8)

        #self.sub = self.create_subscription(Image, "turtlecam/image_rect", self.callback, 10)
        #Pour le proet, on utilise l'image non rectifiée
        self.sub = self.create_subscription(CompressedImage, "turtlecam/image_raw", self.callback, 10)

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
        # TODO: Filter pixels based on their value
        # Hint: Use cv2.inRange
        #mask = cv2.inRange(img, self.mask_min, self.mask_max)

        #Test mask avec hsv
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower1 = np.array([0, 100, 100])
        upper1 = np.array([10, 255, 255])

        lower2 = np.array([170, 100, 100])
        upper2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)

        mask = cv2.bitwise_or(mask1, mask2)

        #result = cv2.bitwise_and(img, img, mask=mask)
                #In order to only show the detected pixels, we can use bitwise_and to keep only the pixels that are within the mask
        #img = cv2.bitwise_and(img, img, mask=mask)
        #return img

        
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            #Calcul du moments et de l'aire du contour
            M = cv2.moments(max_contour)
            area = cv2.contourArea(max_contour)
            perimeter = cv2.arcLength(max_contour, True)
            epsilon = 0.1*cv2.arcLength(max_contour,True)
            approx = cv2.approxPolyDP(max_contour,epsilon,True)
        
        
        #cx = int(M['m10']/M['m00'])
        #cy = int(M['m01']/M['m00'])

        

        
        cv2.drawContours(img, contours, -1, (0,255,0), 3)
        #contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #cont = cv2.drawContours(img, contours, -1, (0,255,0), 3)
        #Détection de la forme du contour exacte
        

        #return mask
        #print(f"Area: {area}")
        #print(f"Moments: {M}")

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
