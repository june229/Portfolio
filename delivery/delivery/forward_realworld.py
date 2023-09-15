import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
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
        
        self.stop_duration = 5.0  # seconds
        self.stop_state = False
        self.stop_start_time = None
        self.stop_timer = 0.0

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

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.dt = timer_period
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_heading = 0.0
        self.current_speed = 0.0
        self.current_altitude = 0.0
        self.takeoff_altitude = -5.0
        self.takeoff_complete = False
        self.armed = False
        self.forward_speed = 1.0  # meters per second

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.armed = msg.arming_state ==VehicleStatus.ARMING_STATE_ARMED
    
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
                self.current_altitude -= self.dt * self.current_speed
                if self.current_altitude <= self.takeoff_altitude:
                    self.takeoff_complete = True
                    self.current_altitude = self.takeoff_altitude
                    self.stop_start_time = self.get_clock().now().seconds_nanoseconds()[0]  # Record stop start time
                    self.current_speed = 0.0

            if self.takeoff_complete:
                if not self.stop_state:
                    # Check if stop duration has passed
                    self.stop_timer = self.get_clock().now().seconds_nanoseconds()[0] - self.stop_start_time
                    if self.stop_timer >= self.stop_duration:
                        self.stop_state = True

                if self.stop_state:
                    # Calculate the desired position for forward flight
                    desired_position = np.array([self.current_position[0], self.current_position[1] + 10.0, self.current_altitude])
                    distance_to_goal = np.linalg.norm(desired_position - self.current_position)

                    # Check if the goal has been reached
                    if distance_to_goal > 1.0:
                        # Calculate the desired heading
                        desired_heading = np.arctan2(desired_position[1] - self.current_position[1], desired_position[0] - self.current_position[0])

                        # Calculate the desired velocity
                        desired_velocity = (desired_position - self.current_position) / distance_to_goal * self.forward_speed

                        # Update the current position, heading, and speed
                        self.current_position += desired_velocity * self.dt
                        self.current_heading = desired_heading
                        self.current_speed = self.forward_speed

                    else:
                        # Stop the vehicle when the goal is reached
                        self.current_speed = 0.0

            # Set the trajectory message
            trajectory_msg.position[0] = self.current_position[0]
            trajectory_msg.position[1] = self.current_position[1]
            trajectory_msg.position[2] = self.current_altitude
            trajectory_msg.yaw = self.current_heading
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