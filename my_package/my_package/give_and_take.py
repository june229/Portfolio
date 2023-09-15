import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')

        # Publisher to initiate the DetectBall process
        self.start_pub = self.create_publisher(Int32, "/start_detection", 10)
        self.start_pub2 = self.create_publisher(Int32, "/start_detection2", 10)

        # Subscriber to listen for error messages from DetectBall
        self.error_sub = self.create_subscription(Int32, "/error_status", self.error_callback, 10)

        # Initiate the DetectBall process
        start_msg = Int32()
        start_msg.data = 1
        self.start_pub.publish(start_msg)
        self.start_pub2.publish(start_msg)
        self.get_logger().info('Sent start message to DetectBall node.')

    def error_callback(self, msg):
        if msg.data == 1:
            self.get_logger().warn('Received error from DetectBall node. Please check.')
        elif msg.data == 2:
            self.get_logger().warn('Mission, Complete!')
            end_msg = Int32()
            end_msg.data = 2
            self.start_pub.publish(end_msg)
            self.start_pub2.publish(end_msg)

def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
