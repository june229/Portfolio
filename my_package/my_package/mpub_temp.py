import rclpy
from rclpy.node import Node    #rclpy.node는 rclpy라는 폴더안에 node라는 파일
from std_msgs.msg import String

class M_pub(Node): #상속을 받기위해 가로 안에 부모 class를 작성한다.
  def __init__(self): #상속을 하기 위해서는 이를 작성해야 한다.
    super().__init__('mpub')

def main():
  rclpy.init()
  node = M_pub()
  pub = node.create_publisher(String, 'messagepub', 10)
  msg = String()
  msg.data = 'hello, world'
  while True :
    pub.publish(msg)
  node.destroy_node()


if __name__ == '__main__': #함수가 main함수일 때만 main 함수를 실행하여라.
  main()

