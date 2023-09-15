import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_geometry_msgs import do_transform_pose

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.waypoints = [
            Pose(position=Point(x=1.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
            Pose(position=Point(x=1.0, y=1.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
            Pose(position=Point(x=0.0, y=1.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
            Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        ]
        self.current_waypoint_index = 0

    def odom_callback(self, msg):
        if self.current_waypoint_index >= len(self.waypoints):
            return

        current_pose = msg.pose.pose
        target_pose = self.waypoints[self.current_waypoint_index]

        distance = math.sqrt((target_pose.position.x - current_pose.position.x) ** 2 +
                             (target_pose.position.y - current_pose.position.y) ** 2)

        if distance < 0.1:  # 0.1m 이내에 도달하면 다음 목표점으로 이동
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info("모든 경로점에 도착했습니다.")
                return

        try:
            transform = self.tf_buffer.lookup_transform("base_link", "odom", rclpy.time.Time())
            target_pose_transformed = do_transform_pose(target_pose, transform)
        except (TransformException):
            self.get_logger().warning("Transform 실패")
            return

        angle_to_target = math.atan2(target_pose_transformed.position.y, target_pose_transformed.position.x)

        angular_speed = 1.0 * angle_to_target
        linear_speed = 0.5 * distance

        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    waypoint_navigator = WaypointNavigator()
    rclpy.spin(waypoint_navigator)
    waypoint_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

