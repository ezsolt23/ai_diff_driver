# -*- coding: utf-8 -*-
"""AI wheel driver node"""

# Import required Python code.
import rospy
import sys
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from pprint import pprint
#from train import Trainer
import numpy as np
from qlearning import Trainer

class AiDriverNode(object):

    last_odom = None
    last_vel = None

    wheel_current_l = 0
    wheel_current_r = 0


    def __init__(self):
        # Initialize the node and name it.
        rospy.init_node("ai_motor_driver")
        self.setupListeners()
        self.pub_wheel_current_l = rospy.Publisher("cmd_wheel_current_l", Float32, queue_size=10)
        self.pub_wheel_current_r = rospy.Publisher("cmd_wheel_current_r", Float32, queue_size=10)

    def ready(self):
        return True

    def vel_callback(self, vel):
        """Handle subscriber data."""
        # Simply print out values in our custom message.

        #rospy.loginfo("Cmd_vel: Linear velocity: %.2f  Angular velocity: %.2f ", vel.linear.x, vel.angular.z)
        self.last_vel = vel
        #self.calcReward()

    def odom_callback(self, odom):
        """Handle subscriber data."""
        # Simply print out values in our custom message.

        #rospy.loginfo("Odom: Linear velocity: %.2f  Angular velocity: %.2f ", odom.twist.twist.linear.x, odom.twist.twist.angular.z)
        self.last_odom = odom.twist.twist
        #self.calcReward()

    def sendCurrent(self, current_left, current_right):
        self.wheel_current_l = current_left
        self.wheel_current_r = current_right
        self.pub_wheel_current_l.publish(current_left)
        self.pub_wheel_current_r.publish(current_right)

    def setupListeners(self):
        """Configure subscriber."""
        self.vel_subscriber = rospy.Subscriber("/cmd_vel", Twist, self.vel_callback, queue_size=2)
        self.odom_subscriber = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback, queue_size=2)



# Main function.
if __name__ == "__main__":
    ros_node = AiDriverNode()
    #trainer = Trainer(ros_node, collect_mode=False)
    #trainer.loop()

    trainer = Trainer()
    trainer.run(ros_node)

