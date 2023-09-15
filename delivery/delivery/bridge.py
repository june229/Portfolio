import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

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
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.dt = timer_period
        self.last_trajectory_msg = TrajectorySetpoint()
        self.last_cmd_vel = Twist()
        self.a=0
        self.a_a=0
        self.b=1.0
        self.c=0
        self.c_c=0

    def cmd_vel_callback(self, msg):
        self.last_cmd_vel = msg

    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        #print("NAV_STATUS: ", msg.nav_state)
        #print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=True
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            trajectory_msg = TrajectorySetpoint()
            yaw=self.a_a *60
            x = self.calculate_x(- yaw, self.a) #* (self.dt / 4) 
            y = self.calculate_y(- yaw, self.a) #* (self.dt / 4) 

            self.a = self.last_cmd_vel.linear.x * (self.dt / 1) 
            self.a_a += self.last_cmd_vel.angular.z * (self.dt / 1.027)#burger        ##waffle1)#1.03)
                        
            current_position = np.array([self.last_trajectory_msg.position[0], self.last_trajectory_msg.position[1], -1.0],dtype=np.float32)
            desired_position = np.array([self.last_trajectory_msg.position[0] + x, self.last_trajectory_msg.position[1] + y, current_position[2]],dtype=np.float32)
            trajectory_msg.position = desired_position
            trajectory_msg.yaw = - self.a_a
            print(self.a)
            print(-self.a_a)
            
        
            self.last_trajectory_msg = trajectory_msg
            self.publisher_trajectory.publish(trajectory_msg)
            
    def calculate_x(self, yaw, distance):
        yaw_rad = np.radians(yaw)
        x = distance * np.cos(yaw_rad)

        return x

    def calculate_y(self, yaw, distance):
        yaw_rad = np.radians(yaw)
        y = distance * np.sin(yaw_rad)

        return y


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
