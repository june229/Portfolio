import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import time
import math
from tf2_ros import TransformException
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np

class MoveToGoal(Node):
    def __init__(self):
        super().__init__('move_to_goal')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # wait for /goal_pose subscription to be ready
        while not self.publisher_.get_subscription_count():
            self.get_logger().info('waiting for /goal_pose subscriber...')
            rclpy.spin_once(self)

        # set intermediate goal positions
        self.intermediate_goals = [
            {'x': 0.0, 'y': 20.77, 'theta': 0.0},
            {'x': 0.0, 'y': 22.0, 'theta': 0.0},
            {'x': 0.0, 'y': 23.13, 'theta': 0.0}
        ]

        # set final goal position
        self.final_goal = {'x': 0.0, 'y': 45.0, 'theta': 0.0}

        # set tolerance for goal position
        self.position_tolerance = 0.5

        # set current position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # publish goal positions
        self.publish_goals()

    def is_goal_reached(self, goal):
        distance = math.sqrt((self.current_x - goal['x']) ** 2 + (self.current_y - goal['y']) ** 2)
        return distance < self.position_tolerance

    def publish_goals(self):
        for goal in self.intermediate_goals:
            self.publish_goal(goal)

            while not self.is_goal_reached(goal):
                rclpy.spin_once(self)

        self.publish_goal(self.final_goal)

    def publish_goal(self, goal):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        theta = goal['theta']
        pose_base = np.array([[goal['x']], [goal['y']], [0], [1]])
        pose_map = np.array([
            [math.cos(theta), -math.sin(theta), 0, goal['x']],
            [math.sin(theta), math.cos(theta), 0, goal['y']],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]) @ pose_base
        
        msg.pose.position.x = pose_map[0][0]
        msg.pose.position.y = pose_map[1][0]
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(theta/2)
        msg.pose.orientation.w = math.cos(theta/2)
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent goal position: x={goal["x"]}, y={goal["y"]}')


    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_theta = msg.pose.pose.orientation.z
        self.get_logger().info(f'Current position: x={self.current_x}, y={self.current_y}')

def main(args=None):
    rclpy.init(args=args)
    move_to_goal = MoveToGoal()
    rclpy.spin(move_to_goal)
    move_to_goal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
