import rclpy
from rclpy.node import Node    #rclpy.node는 rclpy라는 폴더안에 node라는 파일
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile


class M_turtle(Node): #상속을 받기위해 가로 안에 부모 class를 작성한다.
  def __init__(self): #상속을 하기 위해서는 이를 작성해야 한다.
    super().__init__('move_turtle2') # mpub는 토픽의 이름을 설정한 것이다.
    self.qos = QoSProfile(depth = 10)
    self.pub1 = self.create_publisher(Twist, 'turtle1/cmd_vel', self.qos) #message는 통신하는 토픽의 이름으로 pub과 sub의 내용이 동일하다.
    self.pub2 = self.create_publisher(Twist, 'turtle2/cmd_vel', self.qos)
    self.pub3 = self.create_publisher(Twist, 'turtle3/cmd_vel', self.qos)
    self.pub4 = self.create_publisher(Twist, 'turtle4/cmd_vel', self.qos)
    self.pub5 = self.create_publisher(Twist, 'turtle5/cmd_vel', self.qos)
    #self.pub2 = self.create_publisher(String, 'message2', 10)
    self.create_timer(0.1, self.pubmessage)
    #self.create_timer(0.1, self.pubtime)
    self.vel = 0.0
    #self.time=0

  def pubmessage(self): # class 안에서 함수를 정의할 때는 'self' 인자를 주어야 한다.
    msg1 = Twist() # String은 하나의 타입이다. 이 타입을 msg에 적용한다.
    msg1.linear.x = self.vel # Twist의 변수 //x는 속도
    msg1.linear.y = 0.0
    msg1.linear.z = 0.0
    msg1.angular.x = 0.0
    msg1.angular.y = 0.0
    msg1.angular.z = 0.5 # rad/s
    self.pub1.publish(msg1)
    self.get_logger().info(f'Sending message: [{msg1}]')
    self.vel += 0.3
    if self.vel > 3.0:
      self.vel = 0.0

    msg2 = Twist() # String은 하나의 타입이다. 이 타입을 msg에 적용한다.
    msg2.linear.x = self.vel # Twist의 변수 //x는 속도
    msg2.linear.y = 0.0
    msg2.linear.z = 0.0
    msg2.angular.x = 0.0
    msg2.angular.y = 0.0
    msg2.angular.z = 0.5 # rad/s
    self.pub2.publish(msg2)
    self.get_logger().info(f'Sending message: [{msg2}]')
    self.vel += 0.3
    if self.vel > 3.0:
      self.vel = 0.0

    msg3 = Twist() # String은 하나의 타입이다. 이 타입을 msg에 적용한다.
    msg3.linear.x = self.vel # Twist의 변수 //x는 속도
    msg3.linear.y = 0.0
    msg3.linear.z = 0.0
    msg3.angular.x = 0.0
    msg3.angular.y = 0.0
    msg3.angular.z = 0.5 # rad/s
    self.pub3.publish(msg3)
    self.get_logger().info(f'Sending message: [{msg3}]')
    self.vel += 0.3
    if self.vel > 3.0:
      self.vel = 0.0

    msg4 = Twist() # String은 하나의 타입이다. 이 타입을 msg에 적용한다.
    msg4.linear.x = self.vel # Twist의 변수 //x는 속도
    msg4.linear.y = 0.0
    msg4.linear.z = 0.0
    msg4.angular.x = 0.0
    msg4.angular.y = 0.0
    msg4.angular.z = 0.5 # rad/s
    self.pub4.publish(msg4)
    self.get_logger().info(f'Sending message: [{msg4}]')
    self.vel += 0.3
    if self.vel > 3.0:
      self.vel = 0.0

    msg5 = Twist() # String은 하나의 타입이다. 이 타입을 msg에 적용한다.
    msg5.linear.x = self.vel # Twist의 변수 //x는 속도
    msg5.linear.y = 0.0
    msg5.linear.z = 0.0
    msg5.angular.x = 0.0
    msg5.angular.y = 0.0
    msg5.angular.z = 0.5 # rad/s
    self.pub5.publish(msg5)
    self.get_logger().info(f'Sending message: [{msg5}]')
    self.vel += 0.3
    if self.vel > 3.0:
      self.vel = 0.0



def main():
  rclpy.init() # ros middleware(DDS전체)를 불러온다.
  node = M_turtle()
  try:
    rclpy.spin(node)#효율적인 반복구문 - spin
  except:
    node.destroy_node()


if __name__ == '__main__': #함수가 main함수일 때만 main 함수를 실행하여라.
  main()

