import rclpy #rclpy는 패키지 이름이다.
from rclpy.node import Node    #rclpy.node는 rclpy라는 패키지 안에 node라는 파일에서 Node라는 class를 불러옴
from rclpy.qos import QoSProfile
from std_msgs.msg import String

class M_sub(Node): #상속을 받기위해 가로 안에 부모 class를 작성한다.
  def __init__(self): #상속을 하기 위해서는 이를 작성해야 한다.
    super().__init__('msub') # mpub는 토픽의 이름을 설정한 것이다.
    self.qos = QoSProfile(depth = 10)
    self.sub = self.create_subscription(String, 'message', self.messagesub, self.qos)

  def messagesub(self, msg): # class 안에서 함수를 정의할 때는 'self' 인자를 주어야 한다.
    self.get_logger().info('incoming message : [' + msg.data)

def main():
  rclpy.init() # ros middleware(DDS전체)를 불러온다.
  node = M_sub()
  try:
    rclpy.spin(node)#효율적인 반복구문 - spin
  except:
    node.destroy_node()


if __name__ == '__main__': #함수가 main함수일 때만 main 함수를 실행하여라.
  main()

