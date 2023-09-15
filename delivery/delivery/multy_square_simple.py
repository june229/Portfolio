import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleCommand

class OffboardControl(Node):
    
    completed_drones = 0

    def __init__(self, system_id, target_position):
        super().__init__('offboard_control_node_{}'.format(system_id))
        self.system_id = system_id
        self.target_position = target_position

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

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

        self.takeoff_altitude = -10.0
        self.current_altitude = 0.0
        self.takeoff_complete = False
        self.armed = False
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.dt = timer_period
        self.land_state = False
        self.stop_duration = 5.0  # seconds
        self.stop_start_time = None
        self.stop_timer = 0.0
        self.phase = 0

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
                trajectory_msg.yaw = 90.0  # Drone faces north(0)

                if self.current_altitude <= self.takeoff_altitude + 0.5:
                    self.takeoff_complete = True
                    self.stop_start_time = self.get_clock().now().seconds_nanoseconds()[0]  # Record stop start time

            elif self.phase == 0:
                self.stop_timer = self.get_clock().now().seconds_nanoseconds()[0] - self.stop_start_time
                if self.stop_timer >= self.stop_duration:
                    self.phase = 1
                    self.stop_start_time = self.get_clock().now().seconds_nanoseconds()[0]

            elif self.phase == 1:
                trajectory_msg.position[0] = self.target_position[0]
                trajectory_msg.position[1] = self.target_position[1]
                trajectory_msg.position[2] = self.takeoff_altitude
                trajectory_msg.yaw = 90.0

                self.stop_timer = self.get_clock().now().seconds_nanoseconds()[0] - self.stop_start_time
                if self.stop_timer >= self.stop_duration:
                    self.phase = 2
                    self.stop_start_time = self.get_clock().now().seconds_nanoseconds()[0]

            elif self.phase == 2:
                self.land_state = True

            self.publisher_trajectory.publish(trajectory_msg)

    def arm(self):
        vehicle_command = VehicleCommand()
        vehicle_command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        vehicle_command.param1 = 1.0  # 1 to arm, 0 to disarm
        vehicle_command.target_system = self.system_id + 1 # System ID
        vehicle_command.target_component = 1  # Component ID
        vehicle_command.timestamp = int(Clock().now().nanoseconds / 1000)
        self.publisher_vehicle_command.publish(vehicle_command)

    def land(self):
        vehicle_command = VehicleCommand()
        vehicle_command.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        vehicle_command.param5 = 0.0  # Latitude
        vehicle_command.param6 = 0.0  # Longitude
        vehicle_command.param7 = self.current_altitude  # Altitude
        vehicle_command.target_system = self.system_id + 1 # System ID
        vehicle_command.target_component = 1  # Component ID
        vehicle_command.timestamp = int(Clock().now().nanoseconds / 1000)
        self.publisher_vehicle_command.publish(vehicle_command)

def main(args=None):
    rclpy.init(args=args)

    drone1 = OffboardControl(1, [5.0, 0.0])
    drone2 = OffboardControl(2, [5.0, 5.0])
    drone3 = OffboardControl(3, [0.0, 5.0])
    drone4 = OffboardControl(4, [0.0, 0.0])

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




# 각 대형에 도착하면 신호를 보내고 신호가 모이면 동시 착륙