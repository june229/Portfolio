import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleCommand
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

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
        self.start_pub = self.create_publisher(Int32, "/start_detection", 10)
        self.start_pub2 = self.create_publisher(Int32, "/start_detection2", 10)
        self.error_sub = self.create_subscription(Int32, "/error_status", self.error_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.takeoff_altitude = -4.0
        self.current_altitude = 0.0
        self.takeoff_complete = False
        self.armed = False
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.dt = timer_period
        self.land_state = False
        self.stop_duration = 5.0  # seconds
        self.stop_state = False
        self.stop_start_time = None
        self.stop_timer = 0.0
        self.yaw=3.14   # 90:360 = 1.5711:1.5711*4 // cw, ccw = (-)
        self.Mission_start = False
        self.last_cmd_vel = Twist()
        self.a=0
        self.a_a=0
        self.b=1.0
        self.c=0
        self.c_c=0
        self.last_trajectory_msg = TrajectorySetpoint()
        self.z_1=0.0
        self.msg_data = 0
        
    def cmd_vel_callback(self, msg):
        self.last_cmd_vel = msg

    def calculate_x(self, yaw, distance):
        yaw_rad = np.radians(yaw)
        x = distance * np.cos(yaw_rad)

        return x

    def calculate_y(self, yaw, distance):
        yaw_rad = np.radians(yaw)
        y = distance * np.sin(yaw_rad)

        return y
    
    def local_position_callback(self, msg):
        self.current_altitude = msg.z

    def vehicle_status_callback(self, msg):
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.nav_state = msg.nav_state

    def cmdloop_callback(self):
        if not self.armed:
            self.arm()
            return
    
        if self.land_state:
            self.land()
            return

        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
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
                    #self.land_state = True
                    
            elif not self.Mission_start :
                trajectory_msg = TrajectorySetpoint()
                yaw=self.a_a *60
                x = self.calculate_x(- yaw, self.a) #* (self.dt / 4) 
                y = self.calculate_y(- yaw, self.a) #* (self.dt / 4) 
                self.a = self.last_cmd_vel.linear.x * (self.dt / 1) 
                self.a_a += self.last_cmd_vel.angular.z * (self.dt / 1.027)#burger        ##waffle1)#1.03)      
                current_position = np.array([self.last_trajectory_msg.position[0], self.last_trajectory_msg.position[1], self.takeoff_altitude],dtype=np.float32)
                desired_position = np.array([self.last_trajectory_msg.position[0] + x, self.last_trajectory_msg.position[1] + y, self.last_trajectory_msg.position[2]],dtype=np.float32)
                trajectory_msg.position = desired_position
                trajectory_msg.yaw = - self.a_a
                print(self.a)
                print(-self.a_a)
                start_msg = Int32()
                start_msg.data = 1
                self.start_pub.publish(start_msg)
                self.start_pub2.publish(start_msg)
                self.get_logger().info('Mission start!')
                if self.msg_data == 1:
                    self.z_1 += self.dt / 8000
                    trajectory_msg.position[2]=self.last_trajectory_msg.position[2]+self.z_1
                elif self.msg_data == 2:
                    trajectory_msg.position[2]=self.last_trajectory_msg.position[2]
                elif self.msg_data == 3:
                    self.a = -0.26 * (self.dt / 1) 
                    desired_position = np.array([self.last_trajectory_msg.position[0] + x, self.last_trajectory_msg.position[1] + y, current_position[2]],dtype=np.float32)
                    self.stop_timer = self.get_clock().now().seconds_nanoseconds()[0] - self.stop_start_time
                    if self.stop_timer >= self.stop_duration:
                        self.stop_state = True
                        self.Mission_start = True
                        self.land_state = True
                        start_msg.data =2
                        self.start_pub.publish(start_msg)
                        self.start_pub2.publish(start_msg)

            self.publisher_trajectory.publish(trajectory_msg)
            self.last_trajectory_msg = trajectory_msg
            
    def error_callback(self, msg):
        self.msg_data = msg.data
        if msg.data == 1:
            self.get_logger().warn('Received error from DetectBall node. Please check.')
        elif msg.data == 2:
            self.get_logger().warn('Mission, Complete!')
            end_msg = Int32()
            end_msg.data = 2
            self.start_pub.publish(end_msg)
            self.start_pub2.publish(end_msg)

    def arm(self):
        vehicle_command = VehicleCommand()
        vehicle_command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        vehicle_command.param1 = 1.0  # 1 to arm, 0 to disarm
        vehicle_command.target_system = 1  # System ID
        vehicle_command.target_component = 1  # Component ID
        vehicle_command.timestamp = int(Clock().now().nanoseconds / 1000)
        self.publisher_vehicle_command.publish(vehicle_command)

    def land(self):
        vehicle_command = VehicleCommand()
        vehicle_command.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        vehicle_command.param5 = 0.0  # Latitude
        vehicle_command.param6 = 0.0  # Longitude
        vehicle_command.param7 = self.current_altitude  # Altitude
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
