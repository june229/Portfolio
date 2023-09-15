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
from std_msgs.msg import Float64MultiArray, String

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
        self.publisher = self.create_publisher(Float64MultiArray, 'ned_goal', qos_profile)
        self.publisher_wp1 = self.create_publisher(String, 'wp1', qos_profile)
        self.publisher_wp2 = self.create_publisher(String, 'wp2', qos_profile)
        self.publisher_rt1 = self.create_publisher(String, 'rt1', qos_profile)
        self.publisher_rt2 = self.create_publisher(String, 'rt2', qos_profile)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        self.b=1.0
        self.obstacle_detected = False
        self.obstacle_closed = False
        self.status_dangered = False
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
        self.waypoint1_reached = False
        self.waypoint2_reached = False
        self.waypoint1_returned = False
        self.waypoint2_returned = False
        self.waypoint1_lat = None
        self.waypoint1_lon = None
        self.waypoint2_lat = None
        self.waypoint2_lon = None
        #--------------------------------------------------------------------------------
        self.waypoint1 = [15.839699999759205, 11.677200000804078, self.takeoff_altitude]
        self.waypoint2 = [51.470699999711655, 13.120200000315663, self.takeoff_altitude]
        #--------------------------------------------------------------------------------
        self.return_point = [0.0, 0.0, self.takeoff_altitude]
        self.waypoint = [-3.0, 10.0, self.takeoff_altitude]  # [x, y, z]
        #                                        x
        #                                        -
        #                                  -y  - - -  y
        #                                        -
        #                                       -x
        self.x_1=0
        self.x_2=self.waypoint1[0]
        self.y_1=0
        self.y_2=self.waypoint1[1]
        self.r_x_1=self.waypoint1[0]
        self.r_y_1=self.waypoint1[1]
        self.r_yaw_x_1=-self.return_point[0]
        self.r_yaw_y_1=-self.return_point[1]
        self.r_x_2=self.waypoint2[0]
        self.r_y_2=self.waypoint2[1]
        self.r_yaw_x_2=-self.waypoint1[0]
        self.r_yaw_y_2=-self.waypoint1[1]
        self.speed = 2500 # 기준값 5000, 값이 커지면 속도가 느려짐(적정값 -30,100기준, 거리가 줄어들면 기준값 낮추기)
        goal = Float64MultiArray()
        goal.data = [self.waypoint1[0],self.waypoint1[1],self.waypoint2[0],self.waypoint2[1]]
        self.publisher.publish(goal)
        
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
            wp1_x=(self.waypoint1_lat - (msg.lat/10000000))*111000
            wp1_y=(self.waypoint1_lon - (msg.lon/10000000))*111000
            wp2_x=(self.waypoint2_lat - (msg.lat/10000000))*111000
            wp2_y=(self.waypoint2_lon - (msg.lon/10000000))*111000
            self.waypoint1 = [wp1_x,wp1_y,self.takeoff_altitude]
            self.waypoint2 = [wp2_x,wp2_y,self.takeoff_altitude]
            goal = Float64MultiArray()
            goal.data = [wp1_x,wp1_y,wp2_x,wp2_y]
            self.publisher.publish(goal)'''

    def laser_callback(self, msg):
        front = min(min(msg.ranges[:45]), min(msg.ranges[315:]), msg.range_max)

        if 5.0 < front < 7.0:            #0.50 < front < 0.70:   5.0 < front < 7.0: 
            self.obstacle_closed = True
            self.obstacle_detected =False
            self.status_dangered = False
        elif 4.0 < front < 5.0:          #0.40 < front < 0.50:   4.0 < front < 5.0:   
            self.obstacle_detected =True
            self.obstacle_closed = False
            self.status_dangered = False
        elif front < 3.8:                 #front < 0.35:         front < 3.5:  
            self.status_dangered = True
            self.obstacle_detected =False
            self.obstacle_closed = False
        else :
            self.status_dangered = False
            self.obstacle_detected = False
            self.obstacle_closed = False

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
                
            elif self.obstacle_closed:
                if -22.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 22.5:
                    self.x_1 += self.b * (self.dt / 10) #default 값 : 8
                    self.y_1 += self.b * (self.dt / 10) # -는 왼쪽, +는 오른쪽
                elif (22.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 67.5) or (-292.5 > self.last_trajectory_msg.yaw / 0.01745667 >= -337.5):
                    self.y_1 += self.b * (self.dt / 14.14) # -는 왼쪽, +는 오른쪽
                elif (67.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 112.5) or (-247.5 > self.last_trajectory_msg.yaw / 0.01745667 >= -292.5):
                    self.x_1 -= self.b * (self.dt / 10) #default 값 : 8
                    self.y_1 += self.b * (self.dt / 10) # -는 왼쪽, +는 오른쪽
                elif (112.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 157.5) or (-202.5 > self.last_trajectory_msg.yaw / 0.01745667 >= -247.5):
                    self.x_1 -= self.b * (self.dt / 14.14) #default 값 : 8
                elif (157.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 202.5) or (-157.5 > self.last_trajectory_msg.yaw / 0.01745667 >= -202.5):
                    self.x_1 -= self.b * (self.dt / 10) #default 값 : 8
                    self.y_1 -= self.b * (self.dt / 10) # -는 왼쪽, +는 오른쪽
                elif (202.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 247.5) or (-112.5 > self.last_trajectory_msg.yaw / 0.01745667 >= -157.5):
                    self.y_1 -= self.b * (self.dt / 14.14) # -는 왼쪽, +는 오른쪽
                elif (247.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 292.5) or (-67.5 > self.last_trajectory_msg.yaw / 0.01745667 >= -112.5):
                    self.x_1 += self.b * (self.dt / 10) #default 값 : 8
                    self.y_1 -= self.b * (self.dt / 10) # -는 왼쪽, +는 오른쪽
                elif (292.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 337.5) or (-22.5 > self.last_trajectory_msg.yaw / 0.01745667 >= -67.5):
                    self.x_1 += self.b * (self.dt / 14.14) #default 값 : 8
                self.get_logger().info("Obstacle closed. Starting avoidance.")
                # 점진적 전진 비행 및 장애물 회피
                desired_position = np.array([float(self.x_1), float(self.y_1), self.last_trajectory_msg.position[2]],dtype=np.float32)  # Move to the right (positive y-direction)
                trajectory_msg.position = desired_position
                trajectory_msg.yaw = self.last_trajectory_msg.yaw
                
            elif self.obstacle_detected:
                if -22.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 22.5:
                    self.y_1 += self.b * (self.dt / 10) # -는 왼쪽, +는 오른쪽
                elif (22.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 67.5) or (-292.5 > self.last_trajectory_msg.yaw / 0.01745667 >= -337.5):
                    self.x_1 -= self.b * (self.dt / 7.07) #default 값 : 8
                    self.y_1 += self.b * (self.dt / 7.07) # -는 왼쪽, +는 오른쪽
                elif (67.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 112.5) or (-247.5 > self.last_trajectory_msg.yaw / 0.01745667 >= -292.5):
                    self.x_1 -= self.b * (self.dt / 10) #default 값 : 8
                elif (112.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 157.5) or (-202.5 > self.last_trajectory_msg.yaw / 0.01745667 >= -247.5):
                    self.x_1 -= self.b * (self.dt / 7.07) #default 값 : 8
                    self.y_1 -= self.b * (self.dt / 7.07) # -는 왼쪽, +는 오른쪽
                elif (157.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 202.5) or (-157.5 > self.last_trajectory_msg.yaw / 0.01745667 >= -202.5):
                    self.y_1 -= self.b * (self.dt / 10) # -는 왼쪽, +는 오른쪽
                elif (202.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 247.5) or (-112.5 > self.last_trajectory_msg.yaw / 0.01745667 >= -157.5):
                    self.x_1 += self.b * (self.dt / 7.07) #default 값 : 8
                    self.y_1 -= self.b * (self.dt / 7.07) # -는 왼쪽, +는 오른쪽
                elif (247.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 292.5) or (-67.5 > self.last_trajectory_msg.yaw / 0.01745667 >= -112.5):
                    self.x_1 += self.b * (self.dt / 10) #default 값 : 8
                elif (292.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 337.5) or (-22.5 > self.last_trajectory_msg.yaw / 0.01745667 >= -67.5):
                    self.x_1 += self.b * (self.dt / 7.07) #default 값 : 8
                    self.y_1 += self.b * (self.dt / 7.07) # -는 왼쪽, +는 오른쪽
                self.get_logger().info("Obstacle detected. Avoiding obstacle.")
                # 장애물회피
                desired_position = np.array([float(self.x_1), float(self.y_1), self.last_trajectory_msg.position[2]],dtype=np.float32)  # Move to the right (positive y-direction)
                trajectory_msg.position = desired_position
                trajectory_msg.yaw = self.last_trajectory_msg.yaw
                
            elif self.status_dangered:
                if -22.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 22.5:
                    self.x_1 -= self.b * (self.dt / 10) #default 값 : 4
                    self.y_1 += self.b * (self.dt / 10) # -는 왼쪽, +는 오른쪽
                elif (22.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 67.5) or (-292.5 > self.last_trajectory_msg.yaw / 0.01745667 >= -337.5):
                    self.x_1 -= self.b * (self.dt / 14.14) #default 값 : 4
                elif (67.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 112.5) or (-247.5 > self.last_trajectory_msg.yaw / 0.01745667 >= -292.5):
                    self.x_1 -= self.b * (self.dt / 10) #default 값 : 4
                    self.y_1 -= self.b * (self.dt / 10) # -는 왼쪽, +는 오른쪽
                elif (112.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 157.5) or (-202.5 > self.last_trajectory_msg.yaw / 0.01745667 >= -247.5):
                    self.x_1 -= self.b * (self.dt / 7.07) #default 값 : 8
                    self.y_1 -= self.b * (self.dt / 7.07) # -는 왼쪽, +는 오른쪽
                elif (157.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 202.5) or (-157.5 > self.last_trajectory_msg.yaw / 0.01745667 >= -202.5):
                    self.x_1 += self.b * (self.dt / 10) #default 값 : 4
                    self.y_1 -= self.b * (self.dt / 10) # -는 왼쪽, +는 오른쪽
                elif (202.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 247.5) or (-112.5 > self.last_trajectory_msg.yaw / 0.01745667 >= -157.5):
                    self.x_1 += self.b * (self.dt / 14.14) #default 값 : 4
                elif (247.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 292.5) or (-67.5 > self.last_trajectory_msg.yaw / 0.01745667 >= -112.5):
                    self.x_1 += self.b * (self.dt / 10) #default 값 : 4
                    self.y_1 += self.b * (self.dt / 10) # -는 왼쪽, +는 오른쪽
                elif (292.5 < self.last_trajectory_msg.yaw / 0.01745667 <= 337.5) or (-22.5 > self.last_trajectory_msg.yaw / 0.01745667 >= -67.5):
                    self.y_1 += self.b * (self.dt / 14.14) # -는 왼쪽, +는 오른쪽
            # 점진적 후진비행 및 장애물 회피
                self.get_logger().info("Vehicle is dangerous. Moving back.")
                desired_position = np.array([float(self.x_1), float(self.y_1), self.last_trajectory_msg.position[2]],dtype=np.float32)
                trajectory_msg.position = desired_position
                trajectory_msg.yaw = self.last_trajectory_msg.yaw
                    
            elif not self.waypoint1_reached:
                #값이 -이면 +, +이면 +를 한다.
                self.x_1 += self.waypoint1[0] / self.speed # 값이 커지면 속도가 느려진다.(5000)
                print(self.x_1)
                if self.waypoint1[0] > 0:
                    if self.x_1 >= self.waypoint1[0]:
                        self.x_1 = self.waypoint1[0]
                elif self.waypoint1[0] < 0:
                    if self.x_1 <= self.waypoint1[0]:
                        self.x_1 = self.waypoint1[0]
                self.y_1 += self.waypoint1[1] / self.speed
                print(self.y_1)
                if self.waypoint1[1] > 0:
                    if self.y_1 >= self.waypoint1[1]:
                        self.y_1 = self.waypoint1[1]
                elif self.waypoint1[1] < 0:
                    if self.y_1 <= self.waypoint1[1]:
                        self.y_1 = self.waypoint1[1]
                trajectory_msg.position[0] = self.x_1 #self.waypoint[0]
                trajectory_msg.position[1] = self.y_1 #self.waypoint[1]
                trajectory_msg.position[2] = self.waypoint1[2]
                trajectory_msg.yaw = self.calculate_yaw(self.x_1, self.y_1) * 0.01745667 #0.01745667 = 1도

                # Check if the drone is close to the waypoint
                if (abs(self.current_x - self.waypoint1[0]) <= 0.5 and
                    abs(self.current_y - self.waypoint1[1]) <= 0.5):
                    if self.stop_start_time is None:
                        self.stop_start_time = self.get_clock().now().seconds_nanoseconds()[0]  # Record stop start time
                    
                    self.stop_timer = self.get_clock().now().seconds_nanoseconds()[0] - self.stop_start_time
                    if self.stop_timer >= self.stop_duration:
                        self.waypoint1_reached = True
                        wp1 = String()
                        wp1.data = 'Waypoint1 is passed!'
                        self.publisher_wp1.publish(wp1)
                else:
                    self.stop_start_time = None
                    
            elif not self.waypoint2_reached:
                aim_x=self.waypoint2[0] - self.waypoint1[0]
                #값이 -이면 +, +이면 +를 한다.
                self.x_1 += aim_x / self.speed # 값이 커지면 속도가 느려진다.(5000)
                print(self.x_1)
                if aim_x > 0:
                    if self.x_1 >= self.waypoint2[0]:
                        self.x_1 = self.waypoint2[0]
                elif aim_x < 0:
                    if self.x_1 <= self.waypoint2[0]:
                        self.x_1 = self.waypoint2[0]
                aim_y=self.waypoint2[1] - self.waypoint1[1]
                self.y_1 += aim_y / self.speed
                print(self.y_1)
                if aim_y > 0:
                    if self.y_1 >= self.waypoint2[1]:
                        self.y_1 = self.waypoint2[1]
                elif aim_y < 0:
                    if self.y_1 <= self.waypoint2[1]:
                        self.y_1 = self.waypoint2[1]
                trajectory_msg.position[0] = self.x_1 #self.waypoint[0]
                trajectory_msg.position[1] = self.y_1 #self.waypoint[1]
                trajectory_msg.position[2] = self.waypoint2[2]
                trajectory_msg.yaw = self.calculate_yaw(aim_x, aim_y) * 0.01745667

                # Check if the drone is close to the waypoint
                if (abs(self.current_x - self.waypoint2[0]) <= 0.5 and
                    abs(self.current_y - self.waypoint2[1]) <= 0.5):
                    if self.stop_start_time is None:
                        self.stop_start_time = self.get_clock().now().seconds_nanoseconds()[0]  # Record stop start time
                    
                    self.stop_timer = self.get_clock().now().seconds_nanoseconds()[0] - self.stop_start_time
                    if self.stop_timer >= self.stop_duration:
                        self.waypoint2_reached = True
                        wp2 = String()
                        wp2.data = 'Waypoint2 is passed!'
                        self.publisher_wp2.publish(wp2)
                else:
                    self.stop_start_time = None

            elif not self.waypoint1_returned:
                aim_x=self.waypoint2[0] - self.waypoint1[0]
                #복귀 위해 마이너스(-)
                self.x_1 -= aim_x / self.speed # 값이 커지면 속도가 느려진다.(5000)
                print(self.x_1)
                if aim_x > 0:
                    if self.x_1 <= self.waypoint1[0]:
                        self.x_1 = self.waypoint1[0]
                elif aim_x < 0:
                    if self.x_1 >= self.waypoint1[0]:
                        self.x_1 = self.waypoint1[0]
                aim_y=self.waypoint2[1] - self.waypoint1[1]
                self.y_1 -= aim_y / self.speed
                print(self.y_1)
                if aim_y > 0:
                    if self.y_1 <= self.waypoint1[1]:
                        self.y_1 = self.waypoint1[1]
                elif aim_y < 0:
                    if self.y_1 >= self.waypoint1[1]:
                        self.y_1 = self.waypoint1[1]
                trajectory_msg.position[0] = self.x_1  #self.waypoint1[0]
                trajectory_msg.position[1] = self.y_1  #self.waypoint1[1]
                trajectory_msg.position[2] = self.waypoint1[2]
                trajectory_msg.yaw = self.calculate_yaw(-aim_x, -aim_y) * 0.01745667

                # Check if the drone is close to the return point
                if (abs(self.current_x - self.waypoint1[0]) <= 0.5 and
                    abs(self.current_y - self.waypoint1[1]) <= 0.5):
                    if self.stop_start_time is None:
                        self.stop_start_time = self.get_clock().now().seconds_nanoseconds()[0]  # Record stop start time                    
                    self.stop_timer = self.get_clock().now().seconds_nanoseconds()[0] - self.stop_start_time
                    if self.stop_timer >= self.stop_duration:
                        self.waypoint1_returned = True
                        rt1 = String()
                        rt1.data = 'Returnpoint1 is passed!'
                        self.publisher_rt1.publish(rt1)
                else:
                    self.stop_start_time = None
                    
            elif not self.waypoint2_returned:
                aim_x=self.waypoint1[0] - self.return_point[0]
                #값이 -이면 +, +이면 +를 한다.
                self.x_1 -= aim_x / self.speed # 값이 커지면 속도가 느려진다.(5000)
                print(self.x_1)
                if aim_x > 0:
                    if self.x_1 <= self.return_point[0]:
                        self.x_1 = self.return_point[0]
                elif aim_x < 0:
                    if self.x_1 >= self.return_point[0]:
                        self.x_1 = self.return_point[0]
                aim_y=self.waypoint1[1] - self.return_point[1]
                self.y_1 -= aim_y / self.speed
                print(self.y_1)
                if aim_y > 0:
                    if self.y_1 <= self.return_point[1]:
                        self.y_1 = self.return_point[1]
                elif aim_y < 0:
                    if self.y_1 >= self.return_point[1]:
                        self.y_1 = self.return_point[1]
                trajectory_msg.position[0] = self.x_1  #self.return_point[0]
                trajectory_msg.position[1] = self.y_1  #self.return_point[1]
                trajectory_msg.position[2] = self.return_point[2]
                trajectory_msg.yaw = self.calculate_yaw(-aim_x, -aim_y) * 0.01745667

                # Check if the drone is close to the return point
                if (abs(self.current_x - self.return_point[0]) <= 0.5 and
                    abs(self.current_y - self.return_point[1]) <= 0.5):
                    if self.stop_start_time is None:
                        self.stop_start_time = self.get_clock().now().seconds_nanoseconds()[0]  # Record stop start time                    
                    self.stop_timer = self.get_clock().now().seconds_nanoseconds()[0] - self.stop_start_time
                    if self.stop_timer >= self.stop_duration:
                        self.waypoint2_returned = True
                        rt2 = String()
                        rt2.data = 'Returnpoint2 is passed!'
                        self.publisher_rt2.publish(rt2)
                else:
                    self.stop_start_time = None

            elif self.waypoint2_returned:
                self.land_state = True

            self.last_trajectory_msg = trajectory_msg
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
