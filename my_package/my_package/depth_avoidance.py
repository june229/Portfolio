import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ObstacleAvoidanceTurtle(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_turtle')
        self.qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', self.qos)
        self.create_subscription(Image, '/camera/depth/image_rect_raw', self.depth_image_callback, self.qos)
        self.create_timer(0.1, self.pubmessage)
        self.vel = 0.2
        self.min_obstacle_distance = 1.0  # meters
        self.obstacle_detected = False
        self.cv_bridge = CvBridge()

    def depth_image_callback(self, msg):
        depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        threshold = (depth_image < self.min_obstacle_distance) & (depth_image > 0)
        if np.any(threshold):
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def pubmessage(self):
        msg = Twist()
        if not self.obstacle_detected:
            msg.linear.x = self.vel
            msg.angular.z = 0.0
        else:
            msg.linear.x = 0.0
            msg.angular.z = 1.0  # 왼쪽으로 회전

        self.pub.publish(msg)
        self.get_logger().info(f'Sending message: [{msg}]')

def main():
    rclpy.init()
    node = ObstacleAvoidanceTurtle()
    try:
        rclpy.spin(node)
    except:
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
