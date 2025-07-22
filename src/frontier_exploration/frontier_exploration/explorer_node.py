import rclpy
from rclpy.node import Node
import math

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np

class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer_node')

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/frontier_markers',
            10
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.robot_position = None
        self.threshold = 0.7

    def update_robot_position(self):
        try:
            now = rclpy.time.Time()
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                now,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            self.robot_position = (
                transform.transform.translation.x,
                transform.transform.translation.y
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("No se pudo obtener la posición del robot.")
            self.robot_position = None

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin

        data = np.array(msg.data).reshape((height, width))

        frontier_points = []

        self.update_robot_position()
        if self.robot_position is None:
            return  # no hay posición del robot, salimos

        robot_x, robot_y = self.robot_position

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if data[y, x] == 0:  # celda libre
                    neighbors = data[y - 1:y + 2, x - 1:x + 2]
                    if -1 in neighbors:  # al menos un vecino desconocido
                        wx = origin.position.x + (x + 0.5) * resolution
                        wy = origin.position.y + (y + 0.5) * resolution
                        frontier_points.append((wx, wy))
                        if math.hypot(wx - robot_x, wy - robot_y) < self.threshold:
                            continue

        self.publish_markers(frontier_points)

    def publish_markers(self, points):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "frontiers"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1 
        marker.scale.y = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        for (x, y) in points:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)

        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
