#!/usr/bin/env python3
"""Détection et clustering des flèches colorées (sujet 1.2).

Le robot indique à l'opérateur de tourner à gauche ou à droite grâce à des
flèches de couleur détectées par la caméra :

    flèche ROUGE → tourner à GAUCHE
    flèche BLEUE → tourner à DROITE

Conformément à l'énoncé, les flèches détectées sont **regroupées en clusters,
puis chaque cluster est représenté par un point** (son centroïde).

Le bag ne contient que le flux compressé `/turtlecam/image_raw/compressed`
(JPEG). On le décode directement avec cv2.imdecode, sans cv_bridge pour l'entrée
ni nœud décompresseur séparé.

Publications :
  /detections     Image    image annotée (contours + centroïdes des clusters)
  /direction_cmd  String   "left" (rouge), "right" (bleu) ou "none"
  /arrow_points   Image    n'est pas un nuage : voir /detections ; le comptage
                           de clusters par couleur est loggué et annoté.
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from .utils import declare_param

# Convention couleur → direction (modifiable si la consigne s'inverse)
RED_DIRECTION = "left"
BLUE_DIRECTION = "right"


class ArrowDetector(Node):
    def __init__(self):
        super().__init__("arrow_detector")

        self.bridge = CvBridge()

        # Paramètres réglables à chaud
        declare_param(self, "min_area", 600.0)        # aire min d'un contour (px²)
        declare_param(self, "cluster_dist", 60.0)     # distance max intra-cluster (px)
        declare_param(self, "use_roi", True)          # restreindre à la zone centrale

        # Plages HSV
        # Rouge : deux intervalles (la teinte rouge chevauche 0° et 180°)
        self.red_lo1 = np.array([0, 100, 100], np.uint8)
        self.red_hi1 = np.array([10, 255, 255], np.uint8)
        self.red_lo2 = np.array([160, 100, 100], np.uint8)
        self.red_hi2 = np.array([180, 255, 255], np.uint8)
        # Bleu
        self.blue_lo = np.array([90, 80, 50], np.uint8)
        self.blue_hi = np.array([135, 255, 255], np.uint8)

        # Publications
        self.pub_img = self.create_publisher(Image, "/detections", 10)
        self.pub_cmd = self.create_publisher(String, "/direction_cmd", 10)

        # Le flux compressé est publié en RELIABLE dans le bag ; on garde un QoS
        # tolérant pour fonctionner aussi bien en bag qu'en direct sur le robot.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            CompressedImage, "/turtlecam/image_raw/compressed", self.callback, qos
        )

        self.get_logger().info("Nœud arrow_detector démarré (rouge=gauche, bleu=droite)")

    # ------------------------------------------------------------------ #
    def callback(self, msg: CompressedImage):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn("Décodage image compressée impossible")
            return

        # Région d'intérêt centrale : réduit les fausses détections sur les bords
        h, w = img.shape[:2]
        if self.use_roi:
            ox, oy = int(w * 0.20), int(h * 0.15)
            roi_box = (ox, oy, w - ox, h - oy)
        else:
            roi_box = (0, 0, w, h)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask_red = cv2.bitwise_or(
            cv2.inRange(hsv, self.red_lo1, self.red_hi1),
            cv2.inRange(hsv, self.red_lo2, self.red_hi2),
        )
        mask_blue = cv2.inRange(hsv, self.blue_lo, self.blue_hi)

        # Nettoyage morphologique (supprime le bruit poivre-et-sel)
        kernel = np.ones((5, 5), np.uint8)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

        # Clustering → un centroïde par flèche, pour chaque couleur
        red_pts = self.cluster_centroids(mask_red, roi_box)
        blue_pts = self.cluster_centroids(mask_blue, roi_box)

        # Annotation
        if self.use_roi:
            cv2.rectangle(img, roi_box[:2], roi_box[2:], (0, 255, 0), 1)
        self.draw_centroids(img, red_pts, (0, 0, 255), "L")    # rouge → gauche
        self.draw_centroids(img, blue_pts, (255, 0, 0), "R")   # bleu  → droite

        # Décision de direction : la couleur au plus gros cluster l'emporte
        direction = self.decide_direction(red_pts, blue_pts)
        cv2.putText(img, f"DIR: {direction}  (R={len(red_pts)} B={len(blue_pts)})",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        self.pub_cmd.publish(String(data=direction))
        out = self.bridge.cv2_to_imgmsg(img, "bgr8")
        out.header = msg.header
        self.pub_img.publish(out)

    # ------------------------------------------------------------------ #
    # Clustering : regroupe les contours proches et renvoie un centroïde   #
    # (et l'aire totale) par cluster.                                      #
    # ------------------------------------------------------------------ #
    def cluster_centroids(self, mask, roi_box):
        x0, y0, x1, y1 = roi_box
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Centroïde + aire de chaque contour assez grand et dans la ROI
        blobs = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area:
                continue
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cx = M["m10"] / M["m00"]
            cy = M["m01"] / M["m00"]
            if not (x0 <= cx <= x1 and y0 <= cy <= y1):
                continue
            blobs.append((cx, cy, area))

        if not blobs:
            return []

        # Regroupement spatial simple : deux blobs à moins de cluster_dist px
        # appartiennent au même cluster (union par proximité).
        centers = np.array([[b[0], b[1]] for b in blobs])
        areas = np.array([b[2] for b in blobs])
        labels = self.merge_by_distance(centers, self.cluster_dist)

        clusters = []
        for lab in np.unique(labels):
            sel = labels == lab
            w = areas[sel]
            # Centroïde du cluster pondéré par l'aire des blobs
            cx = float(np.average(centers[sel, 0], weights=w))
            cy = float(np.average(centers[sel, 1], weights=w))
            clusters.append((cx, cy, float(w.sum())))
        return clusters

    @staticmethod
    def merge_by_distance(centers: np.ndarray, max_dist: float) -> np.ndarray:
        """Étiquette les points : même label si reliés par des sauts < max_dist."""
        n = len(centers)
        labels = -np.ones(n, dtype=int)
        cur = 0
        for i in range(n):
            if labels[i] != -1:
                continue
            # Parcours en largeur depuis le point i
            stack = [i]
            labels[i] = cur
            while stack:
                j = stack.pop()
                d = np.linalg.norm(centers - centers[j], axis=1)
                for k in np.where((d < max_dist) & (labels == -1))[0]:
                    labels[k] = cur
                    stack.append(k)
            cur += 1
        return labels

    def decide_direction(self, red_pts, blue_pts):
        red_area = max((p[2] for p in red_pts), default=0.0)
        blue_area = max((p[2] for p in blue_pts), default=0.0)
        if red_area == 0 and blue_area == 0:
            return "none"
        return RED_DIRECTION if red_area >= blue_area else BLUE_DIRECTION

    @staticmethod
    def draw_centroids(img, clusters, color, label):
        for cx, cy, _ in clusters:
            cv2.circle(img, (int(cx), int(cy)), 8, color, -1)
            cv2.putText(img, label, (int(cx) + 10, int(cy)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


def main(args=None):
    rclpy.init(args=args)
    node = ArrowDetector()
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
