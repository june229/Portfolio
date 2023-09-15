import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from turtlesim.msg import Pose

class M_turtle(Node):
  def __init__(self):
    super().__init__('move_turtle')
    self.qos = QoSProfile(depth = 10)
    self.pub = self.create_publisher(Twist, 'cmd_vel', self.qos)
    self.sub = self.create_subscription(Pose, '/turtle1/pose', self.subpose, self.qos)
    self.create_timer(0.1, self.pubmessage)
    self.vel = 0.0
    self.x = 0.0
    self.y = 0.0
    self.theta = 0.0
    self.phase = 0
    self.pi = 3.141592

  def pubmessage(self):
    msg = Twist()

    if self.phase == 0:
      if (self.theta > self.pi/2-0.1) and (self.theta < self.pi/2+0.1):
        msg.linear.x = 1.0
        msg.angular.z = 0.0
        if self.y > 10:
          self.phase = 1
      else :
        msg.linear.x = 0.0
        msg.angular.z = 0.3
      self.pub.publish(msg)

    if self.phase == 1:
      if (self.theta > 0-0.1) and (self.theta < 0+0.1):
        msg.linear.x = 1.0
        msg.angular.z = 0.0
        if self.x > 10:
          self.phase = 2
      else :
        msg.linear.x = 0.0
        msg.angular.z = -1.0
      self.pub.publish(msg)

    if self.phase == 2:
      if (self.theta > -self.pi/2-0.1) and (self.theta < -self.pi/2+0.1):
        msg.linear.x = 1.0
        msg.angular.z = 0.0
        if self.y < 1:
          self.phase = 3
      else :
        msg.linear.x = 0.0
        msg.angular.z = -1.0
      self.pub.publish(msg)

    if self.phase == 3:
      if (self.theta < -self.pi+0.1):
        msg.linear.x = 1.0
        msg.angular.z = 0.0
        if self.x < 1:
          self.phase = 4
      else :
        msg.linear.x = 0.0
        msg.angular.z = -1.0
      self.pub.publish(msg)

    if self.phase == 4:
      msg.linear.x = 0.0
      msg.angular.z = 0.0
      self.pub.publish(msg)
      print('complete')
      self.phase = 0

  def subpose(self, msg):
    self.x = msg.x
    self.y = msg.y
    self.theta = msg.theta
    print(self.x, self.y, self.theta)

def main():
  rclpy.init()
  node = M_turtle()
  try:
    rclpy.spin(node)
  except:
    node.destroy_node()

if __name__ == '__main__':
  main()
