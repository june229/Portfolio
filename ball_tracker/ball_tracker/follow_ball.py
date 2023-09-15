# Copyright 2023 Josh Newans
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Int32

class FollowBall(Node):

    def __init__(self):
        super().__init__('follow_ball')
        self.subscription = self.create_subscription(
            Point,
            '/detected_ball',
            self.listener_callback,
            10)
        self.pub = self.create_publisher(Int32, "led_out", 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        

        self.start_sub = self.create_subscription(Int32, "/start_detection", self.start_callback, 10)
        self.process_images = False  # Initially set to False
        # 오류 상태를 위한 발행자 생성
        self.error_pub = self.create_publisher(Int32, "/error_status", 10)
        self.last_lost_time = None
        self.last_large_dist_time = None
        self.last_direction_switch_time = time.time()
        self.direction_switch_interval = 2.0  # 2 seconds
        self.search_left = True
        self.state = 0  # 0: Left 2s, 1: Right 2s, 2: Right 2s, 3: Left 2s, 4: Left 2s



        self.declare_parameter("rcv_timeout_secs", 1.0)
        self.declare_parameter("angular_chase_multiplier", 0.7)
        self.declare_parameter("forward_chase_speed", 0.1)
        self.declare_parameter("search_angular_speed", 0.5)
        self.declare_parameter("max_size_thresh", 0.1) # default = 0.1
        self.declare_parameter("filter_value", 0.9)


        self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
        self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
        self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
        self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
        self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
        self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value


        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.target_val = 0.0
        self.target_dist = 0.0
        self.lastrcvtime = time.time() - 10000
    def start_callback(self, msg):
        if msg.data == 1:
            self.process_images = True  # Allow image processing
            #print('start')
        elif msg.data == 2:
            self.process_images = False
        else:
            self.process_images = False
            #print('false')

    def timer_callback(self):
        if self.process_images:
            msg = Twist()
            if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
                self.get_logger().info('Target: {}'.format(self.target_val))
                print(self.target_dist)
                self.last_lost_time = None
                if (self.target_dist < self.max_size_thresh):
                    msg.linear.x = self.forward_chase_speed
                    print('hmm')
                    self.state = 0
                    error_msg = Int32()
                    error_msg.data = 2
                    self.error_pub.publish(error_msg)
                elif (self.target_dist >= self.max_size_thresh):
                    msg.linear.x = 0.0
                    print('Stop!!')
                    self.state = 0
                    # Check if this is the first time this condition is met
                    if self.last_large_dist_time is None:
                        self.last_large_dist_time = time.time()
                    # Check if 5 seconds have passed since the condition first became true
                    elif (time.time() - self.last_large_dist_time > 5.0):
                        fire_msg = Int32()
                        fire_msg.data = 1
                        self.pub.publish(fire_msg)
                    # Check if 10 seconds have passed since the condition first became true
                    elif (time.time() - self.last_large_dist_time > 10.0):
                        ending_msg = Int32()
                        ending_msg.data = 3  
                        self.error_pub.publish(ending_msg)
                        
                        self.get_logger().warn('Target distance has been large for over 10 seconds!')
                        self.last_large_dist_time = None  # Reset the time to prevent continuous publishing

                msg.angular.z = -self.angular_chase_multiplier*self.target_val
            else:
                self.get_logger().info('Target lost')
                
                if time.time() - self.last_direction_switch_time > self.direction_switch_interval:
                    self.state += 1
                    if self.state > 4:
                        self.state = 1

                    self.last_direction_switch_time = time.time()
                    error_msg = Int32()
                    error_msg.data = 1  # 예: 1은 공을 찾지 못하는 오류를 나타냅니다.
                    self.error_pub.publish(error_msg)

                if self.state == 1 or self.state == 4 :
                    msg.angular.z = -self.search_angular_speed  # 왼쪽으로 회전
                elif self.state == 0:
                    msg.angular.z = -self.search_angular_speed / 2
                else:
                    msg.angular.z = self.search_angular_speed   # 오른쪽으로 회전

                
                    
            self.publisher_.publish(msg)


    def listener_callback(self, msg):
        f = self.filter_value
        self.target_val = self.target_val * f + msg.x * (1-f)
        self.target_dist = self.target_dist * f + msg.z * (1-f)
        self.lastrcvtime = time.time()
        # self.get_logger().info('Received: {} {}'.format(msg.x, msg.y))


def main(args=None):
    rclpy.init(args=args)
    follow_ball = FollowBall()
    rclpy.spin(follow_ball)
    follow_ball.destroy_node()
    rclpy.shutdown()
