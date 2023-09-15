import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus, VehicleAttitudeSetpoint


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
        self.publisher_attitude_setpoint = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.obstacle_detected = False
        self.obstacle_closed = False
        self.warning = False

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.dt = timer_period
        self.theta = 0.0
        self.radius = 1.0
        self.omega = 0.5
        self.last_trajectory_msg = TrajectorySetpoint()
        self.altitude = -1.0
        self.a=0
        self.b=1.0
        self.c=0

    def laser_callback(self, msg):
        front = min(min(msg.ranges[:45]), min(msg.ranges[315:]), msg.range_max)

        if 5.0 < front < 7.0:
            self.obstacle_closed = True
        elif 4.0 < front < 5.0:
            self.obstacle_detected =True
        elif front > 4.0:
            self.warning = True
        else:
            self.obstacle_detected = False
            self.obstacle_closed = False
            self.warning = False

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
            attitude_setpoint = VehicleAttitudeSetpoint()

            if  not self.obstacle_detected:
                # 지속적 전진비행
                self.a += self.b * (self.dt / 8) #default 값 : 4
                current_position = np.array([float(self.a), self.last_trajectory_msg.position[1], self.altitude],dtype=np.float32)
                desired_position = np.array([current_position[0] + float(self.a), current_position[1], current_position[2]],dtype=np.float32)
                trajectory_msg.position = desired_position
                trajectory_msg.yaw = 0.0

                self.theta = self.theta + self.omega * self.dt
            
            elif self.obstacle_closed:
                self.get_logger().info("Obstacle closed. Starting avoidance.")
                self.a += self.b * (self.dt / 1600) #default 값 : 8
                self.c += self.b * (self.dt / 2000) # -는 왼쪽, +는 오른쪽
                # 점진적 전진 비행 및 장애물 회피
                current_position = np.array([self.last_trajectory_msg.position[0], self.last_trajectory_msg.position[1], self.last_trajectory_msg.position[2]],dtype=np.float32)
                desired_position = np.array([current_position[0] + float(self.a), current_position[1] + float(self.c), current_position[2]],dtype=np.float32)  # Move to the right (positive y-direction)
                trajectory_msg.position = desired_position
                trajectory_msg.yaw = self.last_trajectory_msg.yaw
                
            elif self.warning:
                self.get_logger().info("Obstacle too close. Moving back.")
                self.a += self.b * (self.dt / 1600) #default 값 : 8
                self.c += self.b * (self.dt / 2000) # -는 왼쪽, +는 오른쪽
                # 점진적 전진 비행 및 장애물 회피
                current_position = np.array([self.last_trajectory_msg.position[0], self.last_trajectory_msg.position[1], self.last_trajectory_msg.position[2]],dtype=np.float32)
                desired_position = np.array([current_position[0] - float(self.a), current_position[1] + float(self.c), current_position[2]],dtype=np.float32)  # Move to the right (positive y-direction)
                trajectory_msg.position = desired_position
                trajectory_msg.yaw = self.last_trajectory_msg.yaw
                
            elif self.obstacle_detected:
                self.get_logger().info("Obstacle detected. Avoiding obstacle.")
                self.c += self.b * (self.dt / 2000) # -는 왼쪽, +는 오른쪽
                # 장애물회피
                current_position = np.array([self.last_trajectory_msg.position[0], self.last_trajectory_msg.position[1], self.last_trajectory_msg.position[2]],dtype=np.float32)
                desired_position = np.array([current_position[0], current_position[1] + float(self.c), current_position[2]],dtype=np.float32)  # Move to the right (positive y-direction)
                trajectory_msg.position = desired_position
                trajectory_msg.yaw = self.last_trajectory_msg.yaw

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
