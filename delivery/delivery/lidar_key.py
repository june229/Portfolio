import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from std_msgs.msg import String


class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.publisher = self.create_publisher(String, 'obstacle_detected', 10)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.obstacle_detected = False

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.dt = timer_period
        self.theta = 0.0
        self.radius = 1.0
        self.omega = 0.5
        self.last_trajectory_msg = TrajectorySetpoint()

    def laser_callback(self, msg):
        # 감지된 거리 정보를 가져옴
        #front_distances = msg.ranges[0:45] + msg.ranges[315:359]  # 전방 각도 범위: 0-45도, 315-359도
        left_distances = msg.ranges[15:135]  # 좌측 각도 범위: 45-135도
        right_distances = msg.ranges[225:344]  # 우측 각도 범위: 225-315도

        # 임계값을 설정하여 감지 여부 확인
        threshold = 1.0  # 임계값 설정
        #front_detected = any(d < threshold for d in front_distances)
        left_detected = any(d < threshold for d in left_distances)
        right_detected = any(d < threshold for d in right_distances)

        # 감지된 부분을 메시지로 전송
        #if front_detected:
        #    self.publish_obstacle_detected('Front')
        if left_detected:
            self.publish_obstacle_detected('Left')
        if right_detected:
            self.publish_obstacle_detected('Right')
    def publish_obstacle_detected(self, direction):
        msg = String()
        msg.data = f'Obstacle detected on {direction}'
        self.publisher.publish(msg)

    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)
        
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            trajectory_msg = TrajectorySetpoint()
            
            if not self.obstacle_detected:
                trajectory_msg.position[0] = self.radius * np.cos(self.theta)
                trajectory_msg.position[1] = self.radius * np.sin(self.theta)
                trajectory_msg.position[2] = -1.0
                trajectory_msg.yaw = np.arctan2(np.cos(self.theta), -np.sin(self.theta))
                
                self.theta = self.theta + self.omega * self.dt
            else:
                self.get_logger().info("Obstacle detected. Avoiding obstacle.")
                # Adjust trajectory to avoid the obstacle
                avoidance_radius = 2.0  # Adjust as needed
                trajectory_msg.position[0] = avoidance_radius * np.cos(self.theta)
                trajectory_msg.position[1] = avoidance_radius * np.sin(self.theta)
                trajectory_msg.position[2] = -1.0
                trajectory_msg.yaw = np.arctan2(np.cos(self.theta), -np.sin(self.theta))
                
                self.theta = self.theta + self.omega * self.dt
            
            self.last_trajectory_msg = trajectory_msg
            self.publisher_trajectory.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
