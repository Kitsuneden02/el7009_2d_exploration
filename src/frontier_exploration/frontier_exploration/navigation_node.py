import rclpy
from rclpy.node import Node
import math
from tf_transformations import quaternion_from_euler 
import tf2_ros

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class FrontierNavigation(Node):
    def __init__(self):
        super().__init__('frontier_client_node')

        self.subscription = self.create_subscription(
            MarkerArray,
            '/frontier_markers',
            self.frontier_callback,
            10
        )

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.robot_x = 0.0
        self.robot_y = 0.0

        self.goal_sent = False
        self.goal_pub = False
        self.goal_x = 0.0
        self.goal_y = 0.0

        self.failed_points = []
        self.fail_dist_threshold = 0.4  # metros
        self.succeful_dist_threshold = 0.3  # metros

        self.initial_min_travel_distance = 0.7
        self.min_travel_distance = self.initial_min_travel_distance
        self.min_travel_distance_cap = 0.4

        self.succeful_points = []

        self.tries = 1
        self.max_tries = 3

        self.get_logger().info("FrontierClient listo.")

    def is_fail_point(self, x, y):
        for px, py in self.failed_points:
            if math.hypot(x - px, y - py) < self.fail_dist_threshold:
                return True
        return False

    def is_succeful_point(self, x, y):
        for px, py in self.succeful_points:
            if math.hypot(x - px, y - py) < self.succeful_dist_threshold:
                return True
        return False

    def frontier_callback(self, msg):
        if self.goal_sent:
            return  # esperar a que termine antes de enviar otra meta

        # obtener posición actual del robot
        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            self.robot_x = trans.transform.translation.x
            self.robot_y = trans.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f"No se pudo obtener la transformación: {e}")
            return

        min_dist = float('inf')
        target_point = None

        for point in msg.markers[0].points:
            if self.is_fail_point(point.x, point.y):
                self.get_logger().info(f"Frontera ({point.x:.2f}, {point.y:.2f}) descartada: intento fallido anterior.")
                continue
            if self.is_succeful_point(point.x, point.y):
                #self.get_logger().info(f"Frontera ({point.x:.2f}, {point.y:.2f}) descartada: punto ya recorrido.")
                continue

            dist = math.hypot(point.x - self.robot_x, point.y - self.robot_y)
            if dist < min_dist and dist > self.min_travel_distance:
                min_dist = dist
                target_point = point
                if self.min_travel_distance != self.initial_min_travel_distance:
                    self.min_travel_distance = self.initial_min_travel_distance
                    self.get_logger().info(f"Distancia minima vuelve a su valor original: {self.initial_min_travel_distance}")

        if target_point is None:
            self.get_logger().info(f"No se encontraron fronteras. Intento {self.tries+1}/{self.max_tries}.")
            self.tries+=1
            if self.tries == self.max_tries:
                if self.min_travel_distance > self.min_travel_distance_cap:
                    self.min_travel_distance = self.min_travel_distance - 0.1
                    self.get_logger().info(f"Intentos máximos superados, disminuyendo la distancia mínima de desplazamiento a {self.min_travel_distance}m.")
                    self.tries = 1
                else:
                    self.get_logger().info("Ya no es posible encontrar más fronteras. Finalizando exploración.")
                    self.destroy_node()
            return
        
        self.send_goal(target_point)

    def send_goal(self, point):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('Servidor de navegación no disponible.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = point.x
        goal_msg.pose.pose.position.y = point.y
        self.goal_x = point.x
        self.goal_y = point.y

        dx = point.x - self.robot_x
        dy = point.y - self.robot_y
        yaw = math.atan2(dy, dx)
        q = quaternion_from_euler(0, 0, yaw)

        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self.get_logger().info(f"Enviando objetivo a: x={point.x:.2f}, y={point.y:.2f}")
        self.tries = 1
        self.goal_sent = True

        self._send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        # goal pose para rviz, meramente visual
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.goal_pub.publish(goal_msg.pose)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('La meta fue rechazada.')
            self.goal_sent = False
            return

        self.get_logger().info('Meta aceptada. Esperando resultado...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        try:
            result = future.result().result
            error_code = result.error_code

            if error_code == 0:
                self.get_logger().info("Meta alcanzada con éxito.")
                self.succeful_points.append((self.goal_x, self.goal_y))
            else:
                try:
                    trans = self.tf_buffer.lookup_transform(
                        'map',
                        'base_link',
                        rclpy.time.Time()
                    )
                    robot_x = trans.transform.translation.x
                    robot_y = trans.transform.translation.y
                    distance = math.hypot(self.goal_x - robot_x, self.goal_y - robot_y)

                    if distance < 0.4:
                        self.get_logger().warn(f"El robot no alcanzó el objetivo (error={error_code}), pero está muy cerca ({distance:.2f} m). Se considera exitoso.")
                        self.succeful_points.append((self.goal_x, self.goal_y))
                    else:
                        self.get_logger().warn(f"¡El robot no pudo alcanzar el objetivo! Código de error: {error_code}")
                        self.failed_points.append((self.goal_x, self.goal_y))
                except Exception as e:
                    self.get_logger().warn(f"Error al verificar distancia al objetivo: {e}")
                    self.failed_points.append((self.goal_x, self.goal_y))

        except Exception as e:
            self.get_logger().error(f"Error al obtener resultado de navegación: {e}")

        self.goal_sent = False


def main(args=None):
    rclpy.init(args=args)
    node = FrontierNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
