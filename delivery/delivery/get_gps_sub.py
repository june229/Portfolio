import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus, SensorGps
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

        
        self.gps_sub = self.create_subscription(
            SensorGps,
            '/fmu/out/vehicle_gps_position',
            self.vehicle_gps_callback,
            qos_profile)
        
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'gps_goal',
            self.gps_goal_callback,
            qos_profile)
        
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.waypoint1_lat = None
        self.waypoint1_lon = None
        self.waypoint2_lat = None
        self.waypoint2_lon = None
        self.x=0.0
        self.y=0.0

    def gps_goal_callback(self, msg):
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
            print('wp1')
            print(self.waypoint1_lat, 'degree')
            print(self.waypoint1_lon, 'degree')
            print((self.waypoint1_lat - (msg.lat/10000000))*111000, 'm') # 나누기 천만
            print((self.waypoint1_lon - (msg.lon/10000000))*111000, 'm')
            print('wp2')
            print(self.waypoint2_lat, 'degree')
            print(self.waypoint2_lon, 'degree')
            print((self.waypoint2_lat - (msg.lat/10000000))*111000, 'm') # 나누기 천만
            print((self.waypoint2_lon - (msg.lon/10000000))*111000, 'm\n')



def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
