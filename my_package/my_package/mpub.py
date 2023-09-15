import rclpy
from rclpy.node import Node    #rclpy.node는 rclpy라는 폴더안에 node라는 파일
from std_msgs.msg import String

class M_pub(Node): #상속을 받기위해 가로 안에 부모 class를 작성한다.
  def __init__(self): #상속을 하기 위해서는 이를 작성해야 한다.
    super().__init__('mpub') # mpub는 토픽의 이름을 설정한 것이다.
    self.pub = self.create_publisher(String, 'message', 10) #message는 통신하는 토픽의 이름으로 pub과 sub의 내용이 동일하다.
    self.pub2 = self.create_publisher(String, 'message2', 10)
    self.create_timer(1, self.pubmessage)
    self.create_timer(0.1, self.pubtime)
    self.count=0
    self.time=0

  def pubmessage(self): # class 안에서 함수를 정의할 때는 'self' 인자를 주어야 한다.
    msg = String() # String은 하나의 타입이다. 이 타입을 msg에 적용한다.
    msg.data = f'hello, world : {self.count}'
    self.pub.publish(msg)
    self.get_logger().info(f'Sending message: [{msg.data}]')
    self.count += 1

  def pubtime(self):
    tmsg = String()
    tmsg.data = f'{self.time}s'
    self.pub2.publish(tmsg)
    self.time += 0.1


def main():
  rclpy.init() # ros middleware(DDS전체)를 불러온다.
  node = M_pub()
  try:
    rclpy.spin(node)#효율적인 반복구문 - spin
  except:
    node.destroy_node()


if __name__ == '__main__': #함수가 main함수일 때만 main 함수를 실행하여라.
  main()

