#!/usr/bin/env python3
"""Helpers communs au package projet2.

Regroupe :
  - declare_param   : déclaration d'un paramètre ROS2 réglable à chaud
  - make_pointcloud2: construction d'un message PointCloud2 depuis des tableaux
  - make_markers    : construction d'un MarkerArray RViz (points/sphères colorés)

Base reprise de tp4/utils.py (PointCloud2) et étendue pour le projet.
"""
from typing import Any, Iterable

import numpy as np
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.parameter_service import Parameter, SetParametersResult
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

# Champs d'un PointCloud2 : x, y, z, intensité (réflectivité LiDAR ou 0)
PC2FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]


def declare_param(node: Node, name: str, default_value: Any) -> None:
    """Déclare un paramètre ROS2 et l'expose comme attribut du nœud.

    Un callback met à jour l'attribut quand le paramètre change à l'exécution
    (ros2 param set ...), ce qui permet le réglage à chaud des seuils.
    """
    def callback(params: Iterable[Parameter]) -> SetParametersResult:
        for param in params:
            node.get_logger().info(f"Paramètre '{param.name}' = {param.value}")
            setattr(node, param.name, param.value)
        return SetParametersResult(successful=True)

    if len(node._on_set_parameters_callbacks) < 1:
        node.add_on_set_parameters_callback(callback)

    node.declare_parameter(name, default_value)
    setattr(node, name, node.get_parameter(name).value)


def make_pointcloud2(header: Header, x, y, i=None, z=None) -> PointCloud2:
    """Construit un PointCloud2 depuis des tableaux x, y (+ intensité, + z optionnels)."""
    x = np.asarray(x, dtype=np.float32)
    y = np.asarray(y, dtype=np.float32)
    z = np.zeros(len(x), dtype=np.float32) if z is None else np.asarray(z, dtype=np.float32)
    i = np.zeros(len(x), dtype=np.float32) if i is None else np.asarray(i, dtype=np.float32)

    assert len(x) == len(y) == len(z) == len(i), (
        f"Tailles incohérentes: x={len(x)}, y={len(y)}, z={len(z)}, i={len(i)}"
    )

    points = np.vstack((x, y, z, i)).T
    return create_cloud(header, PC2FIELDS, points)


def make_markers(header: Header, points, colors, scale: float = 0.08,
                 ns: str = "arrows") -> MarkerArray:
    """Construit un MarkerArray de sphères, une par point.

    Args:
        points : liste de (x, y) ou (x, y, z)
        colors : liste de (r, g, b) dans [0, 1], une couleur par point
        scale  : diamètre des sphères (m)
        ns     : namespace des markers
    """
    markers = MarkerArray()
    # Efface les markers du cycle précédent pour éviter les fantômes
    markers.markers.append(Marker(header=header, ns=ns, action=Marker.DELETEALL))

    for idx, (pt, col) in enumerate(zip(points, colors)):
        m = Marker(header=header, ns=ns, id=idx + 1, action=Marker.ADD)
        m.type = Marker.SPHERE
        m.pose.position.x = float(pt[0])
        m.pose.position.y = float(pt[1])
        m.pose.position.z = float(pt[2]) if len(pt) > 2 else 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = float(scale)
        m.color = ColorRGBA(r=float(col[0]), g=float(col[1]), b=float(col[2]), a=1.0)
        markers.markers.append(m)

    return markers
