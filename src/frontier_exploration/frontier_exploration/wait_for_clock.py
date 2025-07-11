import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

class ClockWaiter(Node):
    def __init__(self):
        super().__init__('clock_waiter')
        self.subscription = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10)
        self.clock_received = False

    def clock_callback(self, msg):
        if not self.clock_received:
            self.get_logger().info("Clock message received. Ready to launch SLAM Toolbox.")
            self.clock_received = True
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ClockWaiter()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
