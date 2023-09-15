import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleCommand

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class OffboardControl(Node):
    # ... (existing code) ...

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

        self.takeoff_altitude = -3.0
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

        # Add the following lines to initialize PID controllers for x, y, and z positions
        self.pid_x = PIDController(0.1, 0.01, 0.05)  # Set appropriate PID gains
        self.pid_y = PIDController(0.1, 0.01, 0.05)  # Set appropriate PID gains
        self.pid_z = PIDController(0.1, 0.01, 0.05)  # Set appropriate PID gains

        self.current_x = 0.0
        self.current_y = 0.0
        self.desired_x = 0.0
        self.desired_y = 0.0

    def local_position_callback(self, msg):
        self.current_altitude = msg.z
        self.current_x = msg.x
        self.current_y = msg.y
        
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
                self.desired_x = 0
                self.desired_y = 0
                trajectory_msg.position[2] = self.takeoff_altitude
                trajectory_msg.yaw = 130.0  # Drone faces north(0)

                if self.current_altitude <= self.takeoff_altitude + 0.5:
                    self.takeoff_complete = True
                    self.stop_start_time = self.get_clock().now().seconds_nanoseconds()[0]  # Record stop start time

            elif not self.stop_state:
                # Modify the desired x and y positions for the drone to move
                self.desired_x = 5.0
                self.desired_y = 5.0
                trajectory_msg.position[2] = self.takeoff_altitude
                trajectory_msg.yaw = 130.0  # Drone faces east(90)

                # Check if stop duration has passed
                self.stop_timer = self.get_clock().now().seconds_nanoseconds()[0] - self.stop_start_time
                if self.stop_timer >= self.stop_duration:
                    self.stop_state = True
                    self.land_state = True

            # Add PID control for x, y positions
            error_x = self.desired_x - self.current_x
            error_y = self.desired_y - self.current_y

            control_x = self.pid_x.compute(error_x, self.dt)
            control_y = self.pid_y.compute(error_y, self.dt)

            trajectory_msg.position[0] = control_x
            trajectory_msg.position[1] = control_y

            self.publisher_trajectory.publish(trajectory_msg)

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
