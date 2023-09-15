import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10)
        self.subscription  # prevent unused variable warning

    def lidar_callback(self, msg):
        # process lidar scan data
        ranges = msg.ranges

        # simple logic: if an obstacle is detected within 1m in front, stop
        if min(ranges) < 1.0:
            velocity_command = Twist()
            velocity_command.linear.x = 0.0  # stop
            self.publisher_.publish(velocity_command)

        else:
            velocity_command = Twist()
            velocity_command.linear.x = 0.5  # move forward with 0.5 m/s
            self.publisher_.publish(velocity_command)


def main(args=None):
    rclpy.init(args=args)

    obstacle_avoidance_node = ObstacleAvoidanceNode()

    rclpy.spin(obstacle_avoidance_node)

    obstacle_avoidance_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
