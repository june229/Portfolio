import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleCommand, SensorGps
from std_msgs.msg import Float64MultiArray

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
        
        '''self.gps_sub = self.create_subscription(
            SensorGps,
            '/fmu/out/vehicle_gps_position',
            self.vehicle_gps_callback,
            qos_profile)
        
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'gps_goal',
            self.gps_goal_callback,
            qos_profile)'''
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        self.takeoff_altitude = -2.0
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
        self.waypoint1_lat = None
        self.waypoint1_lon = None
        self.waypoint2_lat = None
        self.waypoint2_lon = None
        self.waypoint_reached = False
        self.waypoint_returned = False
        self.waypoint = [-53.26890000021933, 3.1967999995003993, self.takeoff_altitude]  # [x, y, z]
        #                                        x
        #                                        -
        #                                  -y  - - -  y
        #                                        -
        #                                       -x
        self.return_point = [0.0, 0.0, self.takeoff_altitude]
        self.x=0
        self.y=0
        self.r_x=self.waypoint[0]
        self.r_y=self.waypoint[1]
        self.r_yaw_x=-self.waypoint[0]
        self.r_yaw_y=-self.waypoint[1]
        self.speed = 5000 # 기준값 5000, 값이 커지면 속도가 느려짐(적정값 -30,100기준, 거리가 줄어들면 기준값 낮추기)
        
    '''def gps_goal_callback(self, msg):
        # 수신한 메시지 처리
        self.waypoint1_lat = msg.data[0]
        self.waypoint1_lon = msg.data[1]
        self.waypoint2_lat = msg.data[2]
        self.waypoint2_lon = msg.data[3]

        # 수신한 값을 사용하여 원하는 작업 수행
        print("Received GPS Goals:")
        print("Waypoint 1: Lat={}, Lon={}".format(self.waypoint1_lat, self.waypoint1_lon))
        print("Waypoint 2: Lat={}, Lon={}".format(self.waypoint2_lat, self.waypoint2_lon))


    def vehicle_gps_callback(self, msg):
        print('현재위치')
        print(msg.lat/10000000)
        print(msg.lon/10000000)
        if (self.waypoint1_lat is not None and self.waypoint1_lon is not None 
            and self.waypoint1_lat is not None and self.waypoint1_lon is not None):
            print('목표지점')
            print(self.waypoint1_lat, 'degree')
            print(self.waypoint1_lon, 'degree')
            print((self.waypoint1_lat - (msg.lat/10000000))*111000, 'm') # 나누기 천만
            print((self.waypoint1_lon - (msg.lon/10000000))*111000, 'm\n')
            self.waypoint=[(self.waypoint1_lat - (msg.lat/10000000))*111000,
                           (self.waypoint1_lon - (msg.lon/10000000))*111000,self.takeoff_altitude]'''
    
    def laser_callback(self, msg):
        front = min(min(msg.ranges[:45]), min(msg.ranges[315:]), msg.range_max)

        if front < 3.0:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

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
                trajectory_msg.position[0] = 0
                trajectory_msg.position[1] = 0
                trajectory_msg.position[2] = self.takeoff_altitude
                trajectory_msg.yaw = 0.0

                if self.current_altitude <= self.takeoff_altitude + 0.5:
                    self.takeoff_complete = True
                    self.stop_start_time = self.get_clock().now().seconds_nanoseconds()[0]  # Record stop start time

            elif not self.stop_state:
                trajectory_msg.position[0] = 0
                trajectory_msg.position[1] = 0
                trajectory_msg.position[2] = self.takeoff_altitude
                trajectory_msg.yaw = 0.0

                # Check if stop duration has passed
                self.stop_timer = self.get_clock().now().seconds_nanoseconds()[0] - self.stop_start_time
                if self.stop_timer >= self.stop_duration:
                    self.stop_state = True

                    
            elif not self.waypoint_reached:
                #값이 -이면 +, +이면 +를 한다.
                self.x += self.waypoint[0] / self.speed # 값이 커지면 속도가 느려진다.(5000)
                print(self.x)
                if self.waypoint[0] > 0:
                    if self.x >= self.waypoint[0]:
                        self.x = self.waypoint[0]
                elif self.waypoint[0] < 0:
                    if self.x <= self.waypoint[0]:
                        self.x = self.waypoint[0]
                self.y += self.waypoint[1] / self.speed
                print(self.y)
                if self.waypoint[1] > 0:
                    if self.y >= self.waypoint[1]:
                        self.y = self.waypoint[1]
                elif self.waypoint[1] < 0:
                    if self.y <= self.waypoint[1]:
                        self.y = self.waypoint[1]
                trajectory_msg.position[0] = self.x #self.waypoint[0]
                trajectory_msg.position[1] = self.y #self.waypoint[1]
                trajectory_msg.position[2] = self.waypoint[2]
                trajectory_msg.yaw = self.calculate_yaw(self.x, self.y) * 0.01745667

                # Check if the drone is close to the waypoint
                if (abs(self.current_x - self.waypoint[0]) <= 0.5 and
                    abs(self.current_y - self.waypoint[1]) <= 0.5):
                    if self.stop_start_time is None:
                        self.stop_start_time = self.get_clock().now().seconds_nanoseconds()[0]  # Record stop start time
                    
                    self.stop_timer = self.get_clock().now().seconds_nanoseconds()[0] - self.stop_start_time
                    if self.stop_timer >= self.stop_duration:
                        self.waypoint_reached = True
                else:
                    self.stop_start_time = None

            elif self.waypoint_reached:
                self.land_state = True

            self.publisher_trajectory.publish(trajectory_msg)
            
    def calculate_yaw(self, x, y):
        yaw_rad = np.arctan2(y, x)
        yaw_deg = np.degrees(yaw_rad)

        return yaw_deg

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
