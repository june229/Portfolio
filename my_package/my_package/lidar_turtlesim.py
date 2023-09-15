import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)

    def laser_callback(self, msg):
        front = min(msg.ranges[90:270]) # Front readings (90 to 270 degrees)
        front_left = min(msg.ranges[45:90]) # Front left readings (45 to 90 degrees)
        front_right = min(msg.ranges[270:315]) # Front right readings (270 to 315 degrees)

        threshold_distance = 0.5 # Change according to your requirement
        twist = Twist()

        if front > threshold_distance:
            # Move forward if the path is clear
            twist.linear.x = 0.2
            twist.angular.z = 0.0
        elif front_left > front_right:
            # Turn right if there's more space on the left side
            twist.linear.x = 0.0
            twist.angular.z = -0.5
        else:
            # Turn left if there's more space on the right side
            twist.linear.x = 0.0
            twist.angular.z = 0.5

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = ObstacleAvoidance()
    rclpy.spin(obstacle_avoidance)
    obstacle_avoidance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
