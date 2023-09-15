import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtlebotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtlebot3_diff_drive', 10)

    def send_velocity_command(self, linear_x, angular_z):
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_x
        cmd_vel.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    turtlebot_controller = TurtlebotController()
    turtlebot_controller.send_velocity_command(0.5, 0.0)  # 터틀봇에게 전진 속도를 전달
    rclpy.spin(turtlebot_controller)

if __name__ == '__main__':
    main()
