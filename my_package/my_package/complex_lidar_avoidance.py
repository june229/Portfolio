import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import tf_transformations

class VFHObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('vfh_obstacle_avoidance')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.twist = Twist()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.goal_x = 5.0
        self.goal_y = 0.0

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = tf_transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def laser_callback(self, msg):
        # Check if the goal has been reached
        distance_to_goal = np.sqrt((self.goal_x - self.x)**2 + (self.goal_y - self.y)**2)
        if distance_to_goal < 0.1:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_pub.publish(self.twist)
            return

        # 라이다 데이터 처리
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # 벡터 필드 히스토그램 알고리즘
                # 안전 거리 설정
        safe_distance = 0.3

        # 임계값 설정
        threshold = 1.0

        # 안전 거리 이상의 거리값만 추출
        mask = ranges > safe_distance
        masked_ranges = ranges[mask]
        masked_angles = angles[mask]

        # 조향각 초기화
        steering_angle = 0.0

        # 거리값과 각도값을 zip하여 반복문 실행
        for r, a in zip(masked_ranges, masked_angles):
            # 거리값이 임계값보다 작으면 조향각 계산
            if r < threshold:
                steering_angle += a / r

        # 최대 각속도 설정
        max_angular_velocity = 3.0

        # 조향각 범위 제한
        steering_angle = max(min(steering_angle, max_angular_velocity), -max_angular_velocity)

        # Calculate the direction to the goal
        goal_angle = np.arctan2(self.goal_y - self.y, self.goal_x - self.x) - self.yaw

        # 회피 알고리즘 결과에 따라 속도 및 회전 속도 설정
        self.twist.linear.x = 0.5
        self.twist.angular.z = -steering_angle + goal_angle

        self.cmd_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    vfh_obstacle_avoidance = VFHObstacleAvoidance()
    rclpy.spin(vfh_obstacle_avoidance)
    vfh_obstacle_avoidance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

