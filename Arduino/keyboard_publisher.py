import rospy
from std_msgs.msg import Byte
from std_msgs.msg import Int32
from std_msgs.msg import String

def keyboard_publisher():
    rospy.init_node('keyboard_publisher', anonymous=True)
    pub = rospy.Publisher('led_out', Byte, queue_size=10)

    def chatter_callback(msg):
        rospy.loginfo("\n\n%s", msg.data)
        rospy.signal_shutdown("Message received, shutting down...")

    rospy.Subscriber('/chatter', String, chatter_callback)

    while not rospy.is_shutdown():
        key = input("0:stop\n1:delever\n2:return\nEnter a number (0-2): ")
        try:
            data = int(key)
            if 0 <= data <= 2:
                msg = Byte()
                msg.data = data
                pub.publish(msg)
            else:
                rospy.logwarn("Invalid input. Enter a number between 0 and 2.")
        except ValueError:
            rospy.logwarn("Invalid input. Enter a number.")

if __name__ == '__main__':
    try:
        keyboard_publisher()
    except rospy.ROSInterruptException:
        pass

