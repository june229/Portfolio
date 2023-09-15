import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class GPSGoalPublisher(Node):
    def __init__(self):
        super().__init__('gps_goal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # 토픽 발행자 생성
        self.publisher = self.create_publisher(Float64MultiArray, 'gps_goal', qos_profile)
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'ned_goal',
            self.ned_goal_callback,
            qos_profile)
        self.subscription = self.create_subscription(
            String,
            'wp1',
            self.wp1_pass,
            qos_profile)
        self.subscription = self.create_subscription(
            String,
            'wp2',
            self.wp2_pass,
            qos_profile)
        self.subscription = self.create_subscription(
            String,
            'rt1',
            self.rt1_pass,
            qos_profile)
        self.subscription = self.create_subscription(
            String,
            'rt2',
            self.rt2_pass,
            qos_profile)
        
    def ned_goal_callback(self,msg):
        self.waypoint1_lat = msg.data[0]
        self.waypoint1_lon = msg.data[1]
        self.waypoint2_lat = msg.data[2]
        self.waypoint2_lon = msg.data[3]

        # 수신한 값을 사용하여 원하는 작업 수행
        print("\nReceived NED Goals:")
        print("Waypoint 1: X={}, Y={}".format(self.waypoint1_lat, self.waypoint1_lon))
        print("Waypoint 2: X={}, Y={}".format(self.waypoint2_lat, self.waypoint2_lon))
    
    def wp1_pass(self,msg):
        passed1 = msg.data
        print('------------------------------------------------------')
        print('{}'.format(passed1))
        print('{}'.format(passed1))
        print('{}'.format(passed1))
        print('{}'.format(passed1))
        print('{}'.format(passed1))
        print('{}'.format(passed1))
        print('{}'.format(passed1))
        print('{}'.format(passed1))
        print('{}'.format(passed1))
        print('{}'.format(passed1))
        print("\nReceived NED Goals:")
        print("Waypoint 2: X={}, Y={}".format(self.waypoint2_lat, self.waypoint2_lon))
        
    def wp2_pass(self,msg):
        passed2 = msg.data
        print('------------------------------------------------------')
        print('{}'.format(passed2))
        print('{}'.format(passed2))
        print('{}'.format(passed2))
        print('{}'.format(passed2))
        print('{}'.format(passed2))
        print('{}'.format(passed2))
        print('{}'.format(passed2))
        print('{}'.format(passed2))
        print('{}'.format(passed2))
        print('{}'.format(passed2))
        print("\nReceived NED Goals:")
        print("Waypoint 1: X={}, Y={}".format(self.waypoint1_lat, self.waypoint1_lon))
    def rt1_pass(self,msg):
        passed3 = msg.data
        print('------------------------------------------------------')
        print('{}'.format(passed3))
        print('{}'.format(passed3))
        print('{}'.format(passed3))
        print('{}'.format(passed3))
        print('{}'.format(passed3))
        print('{}'.format(passed3))
        print('{}'.format(passed3))
        print('{}'.format(passed3))
        print('{}'.format(passed3))
        print('{}'.format(passed3))
        print("\nReturning to home")
    def rt2_pass(self,msg):
        passed4 = msg.data
        print('------------------------------------------------------')
        print('{}'.format(passed4))
        print('{}'.format(passed4))
        print('{}'.format(passed4))
        print('{}'.format(passed4))
        print('{}'.format(passed4))
        print('{}'.format(passed4))
        print('{}'.format(passed4))
        print('{}'.format(passed4))
        print('{}'.format(passed4))
        print('{}'.format(passed4))
    
    def publish_gps_goal(self):
        # 위도 경도 입력 받기
        msg = Float64MultiArray()
        waypoint1_lat = float(input("WP1, 위도(lat)를 입력하세요: "))
        waypoint1_lon = float(input("WP1, 경도(lon)를 입력하세요: "))
        waypoint2_lat = float(input("WP2, 위도(lat)를 입력하세요: "))
        waypoint2_lon = float(input("WP2, 경도(lon)를 입력하세요: "))
        

        msg.data = [waypoint1_lat, waypoint1_lon, waypoint2_lat, waypoint2_lon]
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    gps_goal_publisher = GPSGoalPublisher()
    gps_goal_publisher.publish_gps_goal()
    rclpy.spin(gps_goal_publisher)
    gps_goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
