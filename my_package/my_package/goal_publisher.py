import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.goal_publisher = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.timer = self.create_timer(1.0, self.publish_goal)  # 1초마다 publish_goal 함수를 호출

    def publish_goal(self):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()  # 현재 시간을 목표의 타임스탬프로 설정
        goal.header.frame_id = 'map'
        goal.pose.position.x = 1.65
        goal.pose.position.y = 0.6
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        self.goal_publisher.publish(goal)
        self.get_logger().info('Published goal pose')

def main(args=None):
    rclpy.init(args=args)
    goal_publisher = GoalPublisher()
    rclpy.spin(goal_publisher)
    goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
