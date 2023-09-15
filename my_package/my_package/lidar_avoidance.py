import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.twist = Twist()

    def laser_callback(self, msg):
        # 라이다 데이터 처리
        front = min(min(msg.ranges[:45]), min(msg.ranges[315:]), msg.range_max)
        left = min(min(msg.ranges[45:135]), msg.range_max)
        right = min(min(msg.ranges[225:315]), msg.range_max)

        # 장애물 회피 알고리즘 구현 (간단한 방향 전환)
        if front > 1.0:
            # 전방에 장애물이 없으면 직진
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0.0
        elif left > right:
            # 왼쪽에 더 많은 공간이 있으면 왼쪽으로 회전
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.5
        else:
            # 오른쪽에 더 많은 공간이 있으면 오른쪽으로 회전
            self.twist.linear.x = 0.0
            self.twist.angular.z = -0.5

        self.cmd_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = ObstacleAvoidance()
    rclpy.spin(obstacle_avoidance)
    obstacle_avoidance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
