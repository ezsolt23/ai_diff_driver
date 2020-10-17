#!../venv/bin/python3

# Import required Python code.
import rospy
from std_msgs.msg import Float32

class TestDataProviderNode(object):

    def __init__(self):
        rate = 10
        self.pub_l = rospy.Publisher("/cmd_wheel_current_l", Float32, queue_size=10)
        self.pub_r = rospy.Publisher("/cmd_wheel_current_r", Float32, queue_size=10)
        
        rospy.Timer(rospy.Duration(1.0 / rate), self.timer_callback)

    def timer_callback(self, _event):
        f = Float32(2.0)

        self.pub_l.publish(f)
        self.pub_r.publish(f)


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