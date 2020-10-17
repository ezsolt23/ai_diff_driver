#!../venv/bin/python3

# Import required Python code.
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class TestDataProviderNode(object):

    def __init__(self):
        rate = 2
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.pub_odom    = rospy.Publisher("/odometry/filtered", Odometry, queue_size=10)
        
        rospy.Timer(rospy.Duration(1.0 / rate), self.timer_callback)

    def timer_callback(self, _event):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2
        cmd_vel.angular.z = 0.0

        self.pub_cmd_vel.publish(cmd_vel)
        self.publishOdom()

    def publishOdom(self):
        odom = Odometry()
        odom.twist.twist.linear.x= 0.3
        odom.twist.twist.angular.z = 0.3

        self.pub_odom.publish(odom)


# Main function.
if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node("pytalker")
    # Go to class functions that do all the heavy lifting.
    try:
        TestDataProviderNode()
    except rospy.ROSInterruptException:
        pass
    # Allow ROS to go to all callbacks.
    rospy.spin()