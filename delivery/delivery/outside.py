import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus, VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleCommand

class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile)

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.obstacle_detected = False
        self.obstacle_closed = False
        self.status_dangered = False

        self.takeoff_altitude = -1.5
        self.current_altitude = 0.0
        self.takeoff_complete = False
        self.armed = False
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.stop_duration = 5.0  # seconds
        self.stop_state = False
        self.stop_start_time = None
        self.stop_timer = 0.0
        self.yaw=0.0
        self.dt = timer_period
        self.theta = 0.0
        self.radius = 1.0
        self.omega = 0.5
        self.last_trajectory_msg = TrajectorySetpoint()
        self.a=0
        self.b=1.0
        self.c=0
        
    def local_position_callback(self, msg):
        self.current_altitude = msg.z

    def laser_callback(self, msg):
        front = min(min(msg.ranges[:45]), min(msg.ranges[315:]), msg.range_max)

        if 0.50 < front < 0.70:             #0.50 < front < 0.70:   5.0 < front < 7.0: 
            self.obstacle_closed = True
            self.obstacle_detected =False
            self.status_dangered = False
        elif 0.40 < front < 0.50:          #0.40 < front < 0.50:   4.0 < front < 5.0:   
            self.obstacle_detected =True
            self.obstacle_closed = False
            self.status_dangered = False
        elif front < 0.35:              #front < 0.35:         front < 3.5:  
            self.status_dangered = True
            self.obstacle_detected =False
            self.obstacle_closed = False
        else :
            self.status_dangered = False
            self.obstacle_detected = False
            self.obstacle_closed = False
            
        

    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.nav_state = msg.nav_state

    def cmdloop_callback(self):
        if not self.armed:
            self.arm()
            return
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            trajectory_msg = TrajectorySetpoint()
            if not self.takeoff_complete:
                trajectory_msg.position[0] = 0
                trajectory_msg.position[1] = 0
                trajectory_msg.position[2] = self.takeoff_altitude
                trajectory_msg.yaw = self.yaw 

                if self.current_altitude <= self.takeoff_altitude + 0.5:
                    self.takeoff_complete = True
                    self.stop_start_time = self.get_clock().now().seconds_nanoseconds()[0]  # Record stop start time

            elif not self.stop_state:
                trajectory_msg.position[0] = 0
                trajectory_msg.position[1] = 0
                trajectory_msg.position[2] = self.takeoff_altitude
                trajectory_msg.yaw = self.yaw 

                # Check if stop duration has passed
                self.stop_timer = self.get_clock().now().seconds_nanoseconds()[0] - self.stop_start_time
                if self.stop_timer >= self.stop_duration:
                    self.stop_state = True
                    self.land_state = True

            elif  not self.obstacle_detected and not self.obstacle_closed and not self.status_dangered:
                # 지속적 전진비행
                self.a += self.b * (self.dt / 2) #default 값 : 4
                current_position = np.array([float(self.a), self.last_trajectory_msg.position[1], self.takeoff_altitude],dtype=np.float32)
                desired_position = np.array([float(self.a), float(self.c), current_position[2]],dtype=np.float32)
                trajectory_msg.position = desired_position
                trajectory_msg.yaw = 0.0

                self.theta = self.theta + self.omega * self.dt
            
            elif self.obstacle_closed:
                self.get_logger().info("Obstacle closed. Starting avoidance.")
                self.a += self.b * (self.dt / 4) #default 값 : 8
                self.c -= self.b * (self.dt / 2.5) # -는 왼쪽, +는 오른쪽
                # 점진적 전진 비행 및 장애물 회피
                current_position = np.array([self.last_trajectory_msg.position[0], self.last_trajectory_msg.position[1], self.last_trajectory_msg.position[2]],dtype=np.float32)
                desired_position = np.array([float(self.a), float(self.c), current_position[2]],dtype=np.float32)  # Move to the right (positive y-direction)
                trajectory_msg.position = desired_position
                trajectory_msg.yaw = self.last_trajectory_msg.yaw
                
            elif self.obstacle_detected:
                self.get_logger().info("Obstacle detected. Avoiding obstacle.")
                self.c -= self.b * (self.dt / 2.5) # -는 왼쪽, +는 오른쪽
                # 장애물회피
                current_position = np.array([self.last_trajectory_msg.position[0], self.last_trajectory_msg.position[1], self.last_trajectory_msg.position[2]],dtype=np.float32)
                desired_position = np.array([float(self.a), float(self.c), current_position[2]],dtype=np.float32)  # Move to the right (positive y-direction)
                trajectory_msg.position = desired_position
                trajectory_msg.yaw = self.last_trajectory_msg.yaw
                
            elif self.status_dangered:
            # 점진적 후진비행 및 장애물 회피
                self.get_logger().info("Vehicle is dangerous. Moving back.")
                self.a -= self.b * (self.dt / 4) #default 값 : 4
                self.c -= self.b * (self.dt / 2.5) # -는 왼쪽, +는 오른쪽
                current_position = np.array([self.last_trajectory_msg.position[0], self.last_trajectory_msg.position[1], self.last_trajectory_msg.position[2]],dtype=np.float32)
                desired_position = np.array([float(self.a), float(self.c), current_position[2]],dtype=np.float32)
                trajectory_msg.position = desired_position
                trajectory_msg.yaw = self.last_trajectory_msg.yaw

            self.last_trajectory_msg = trajectory_msg
            self.publisher_trajectory.publish(trajectory_msg)
            
    def arm(self):
        vehicle_command = VehicleCommand()
        vehicle_command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        vehicle_command.param1 = 1.0  # 1 to arm, 0 to disarm
        vehicle_command.target_system = 1  # System ID
        vehicle_command.target_component = 1  # Component ID
        vehicle_command.timestamp = int(Clock().now().nanoseconds / 1000)
        self.publisher_vehicle_command.publish(vehicle_command)


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
