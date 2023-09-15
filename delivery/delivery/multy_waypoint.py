import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from scipy.spatial.transform import Rotation

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleAttitude


class OffboardControl(Node):

    def __init__(self,system_id):
        super().__init__('offboard_control_node_{}'.format(system_id))
        self.system_id = system_id
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, 
            '/px4_{}/fmu/out/vehicle_attitude'.format(self.system_id), 
            self.attitude_callback, 
            qos_profile)
                
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/px4_{}/fmu/out/vehicle_local_position'.format(self.system_id),
            self.local_position_callback,
            qos_profile)

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/px4_{}/fmu/out/vehicle_status'.format(self.system_id),
            self.vehicle_status_callback,
            qos_profile)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/px4_{}/fmu/in/vehicle_command'.format(self.system_id), qos_profile)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/px4_{}/fmu/in/offboard_control_mode'.format(self.system_id), qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/px4_{}/fmu/in/trajectory_setpoint'.format(self.system_id), qos_profile)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        # Add stop duration and stop state variables
        self.stop_duration = 5.0  # seconds
        self.stop_state = False
        self.stop_start_time = None
        self.stop_timer = 0.0

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.dt = timer_period
        self.forward_speed = 1.0  # meters per second
        self.forward_position = 0.0
        self.takeoff_altitude = -5.0
        self.current_altitude = 0.0
        self.takeoff_complete = False
        self.armed = False
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = [np.array([0.0, 0.0, -5.0]), np.array([50.0, 53.0, -7.0]), np.array([100.0, 53.0, -5.0])]
        self.current_waypoint_index = 0
        self.direction = 1
        self.finished = False
        self.land_state = False
        self.current_yaw = None
        self.initial_yaw = None
        
    def attitude_callback(self, msg):
        # Convert the quaternion to Euler angles
        quat = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]
        r = Rotation.from_quat(quat)
        euler_angles = r.as_euler('zyx')
        self.current_yaw = euler_angles[0]

        if self.initial_yaw is None:
            self.initial_yaw = self.current_yaw
        
    def local_position_callback(self, msg):
        self.current_position = np.array([msg.x, msg.y, msg.z])

        
    def is_waypoint_reached(self, current_position, waypoint, threshold=0.5):
        distance = np.linalg.norm(current_position - waypoint)
        return distance < threshold

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
        
        if self.finished:
            if not self.land_state:
                # Start landing sequence
                self.land_state = True
                self.landing_start_time = self.get_clock().now().seconds_nanoseconds()[0]
            else:
                # Check if landing duration has passed
                landing_timer = self.get_clock().now().seconds_nanoseconds()[0] - self.landing_start_time
                if landing_timer >= self.stop_duration:
                    self.land()
        
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)
        
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position[0] = self.waypoints[self.current_waypoint_index][0]
            trajectory_msg.position[1] = self.waypoints[self.current_waypoint_index][1]
            trajectory_msg.position[2] = self.waypoints[self.current_waypoint_index][2]
            # Calculate the difference between the current yaw and the initial yaw
            if self.current_yaw is not None and self.initial_yaw is not None:
                # Calculate the difference between the current yaw and the initial yaw
                yaw_diff = self.initial_yaw - self.current_yaw

                if self.current_waypoint_index + self.direction < len(self.waypoints) and self.current_waypoint_index + self.direction >= 0:
                    dx = self.waypoints[self.current_waypoint_index + self.direction][0] - self.waypoints[self.current_waypoint_index][0]
                    dy = self.waypoints[self.current_waypoint_index + self.direction][1] - self.waypoints[self.current_waypoint_index][1]

                    # Calculate the yaw angle using the arctangent function
                    yaw_angle = np.arctan2(dy, dx)

                    # Apply the yaw difference to the calculated yaw angle
                    yaw_angle += yaw_diff

                    # Assign the calculated yaw angle to trajectory_msg.yaw
                    trajectory_msg.yaw = yaw_angle

                self.publisher_trajectory.publish(trajectory_msg)
            
            if not self.takeoff_complete:
                self.current_altitude -= self.dt * self.forward_speed
                if self.current_altitude <= self.takeoff_altitude:
                    self.takeoff_complete = True
                    self.current_altitude = self.takeoff_altitude
                    self.stop_start_time = self.get_clock().now().seconds_nanoseconds()[0]  # Record stop start time

            if self.takeoff_complete:
                if not self.stop_state:
                    # Check if stop duration has passed
                    self.stop_timer = self.get_clock().now().seconds_nanoseconds()[0] - self.stop_start_time
                    if self.stop_timer >= self.stop_duration:
                        self.stop_state = True

                if self.stop_state:
                    self.forward_position += self.forward_speed * self.dt

            if self.is_waypoint_reached(self.current_position, self.waypoints[self.current_waypoint_index]):
                if self.direction == 1: # Forward direction
                    if self.current_waypoint_index == len(self.waypoints) - 1: # At the last waypoint
                        self.direction = -1 # Change to backward direction
                    else:
                        self.current_waypoint_index += 1
                else: # Backward direction
                    if self.current_waypoint_index == 0: # At the first waypoint
                        #self.direction = 1 # Change to forward direction
                        self.finished = True
                    else:
                        self.current_waypoint_index -= 1
            

            if self.current_waypoint_index + self.direction < len(self.waypoints) and self.current_waypoint_index + self.direction >= 0:
                dx = self.waypoints[self.current_waypoint_index + self.direction][0] - self.waypoints[self.current_waypoint_index][0]
                dy = self.waypoints[self.current_waypoint_index + self.direction][1] - self.waypoints[self.current_waypoint_index][1]

                # Calculate the yaw angle using the arctangent function
                yaw_angle = np.arctan2(dy, dx)

                # Assign the calculated yaw angle to trajectory_msg.yaw
                trajectory_msg.yaw = yaw_angle

            self.publisher_trajectory.publish(trajectory_msg)
        
    def arm(self):
        vehicle_command = VehicleCommand()
        vehicle_command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        vehicle_command.param1 = 1.0  # 1 to arm, 0 to disarm
        vehicle_command.target_system = self.system_id + 1 # System ID
        vehicle_command.target_component = 1  # Component ID
        vehicle_command.timestamp = int(Clock().now().nanoseconds / 1000)
        self.publisher_vehicle_command.publish(vehicle_command)
        
    def disarm(self):
        vehicle_command = VehicleCommand()
        vehicle_command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        vehicle_command.param1 = 0.0  # 1 to arm, 0 to disarm
        vehicle_command.target_system = self.system_id + 1 # System ID
        vehicle_command.target_component = 1  # Component ID
        vehicle_command.timestamp = int(Clock().now().nanoseconds / 1000)
        self.publisher_vehicle_command.publish(vehicle_command)

    def land(self):
        vehicle_command = VehicleCommand()
        vehicle_command.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        vehicle_command.param5 = self.current_position[0]  # Latitude
        vehicle_command.param6 = self.current_position[1]  # Longitude
        vehicle_command.param7 = self.current_altitude  # Altitude
        vehicle_command.target_system = self.system_id + 1 # System ID
        vehicle_command.target_component = 1  # Component ID
        vehicle_command.timestamp = int(Clock().now().nanoseconds / 1000)
        self.publisher_vehicle_command.publish(vehicle_command)
        self.disarm()


def main(args=None):
    rclpy.init(args=args)

    drone1 = OffboardControl(1)
    drone2 = OffboardControl(2)
    drone3 = OffboardControl(3)
    drone4 = OffboardControl(4)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(drone1)
    executor.add_node(drone2)
    executor.add_node(drone3)
    executor.add_node(drone4)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        drone1.destroy_node()
        drone2.destroy_node()
        drone3.destroy_node()
        drone4.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
