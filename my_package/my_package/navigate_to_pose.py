import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class MoveToGoal(Node):
    def __init__(self):
        super().__init__('move_to_goal')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # wait for /goal_pose subscription to be ready
        while not self.publisher_.get_subscription_count():
            self.get_logger().info('waiting for /goal_pose subscriber...')
            rclpy.spin_once(self)

        # set goal position
        self.goal_x = 2.0 #음수
        self.goal_y = 1.0 #음수
        self.goal_theta = 0.0

        # publish goal position
        self.publish_goal()

    def publish_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.goal_x
        msg.pose.position.y = self.goal_y
        msg.pose.orientation.z = self.goal_theta
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent goal position: x={self.goal_x}, y={self.goal_y}')

    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_theta = msg.pose.pose.orientation.z
        self.get_logger().info(f'Current position: x={current_x}, y={current_y}')

def main(args=None):
    rclpy.init(args=args)
    move_to_goal = MoveToGoal()
    rclpy.spin(move_to_goal)
    move_to_goal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
