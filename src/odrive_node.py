#!./venv/bin/python3
# -*- coding: utf-8 -*-
"""Odrive current ROS driver"""

from std_msgs.msg import Float32
import rospy
import odrive
from odrive.enums import *

class ODriveNode(object):

    MAX_CURRENT = 5.0
    MIN_CURRENT = -5.0

    def __init__(self):
        rospy.init_node("ai_odrive_node")
        self.connect()
        self.sub_l = rospy.Subscriber("cmd_wheel_current_l", Float32, self.l_callback, queue_size=2)
        self.sub_r = rospy.Subscriber("cmd_wheel_current_r", Float32, self.r_callback, queue_size=2)

    def __del__(self):
        self.driver.axis0.controller.current_setpoint = 0
        self.driver.axis1.controller.current_setpoint = 0
        self.driver.axis0.requested_state = AXIS_STATE_IDLE
        self.driver.axis1.requested_state = AXIS_STATE_IDLE
        rospy.loginfo("ODrive disconnected.")
    
    def connect(self):
        rospy.loginfo("Connecting to ODrive..")
        self.driver = odrive.find_any()
        rospy.loginfo("ODrive connected.")
        self.driver.axis0.controller.current_setpoint = 0
        self.driver.axis1.controller.current_setpoint = 0
        self.driver.axis0.requested_state = CTRL_MODE_CURRENT_CONTROL
        self.driver.axis1.requested_state = CTRL_MODE_CURRENT_CONTROL
        self.driver.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.driver.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.driver.axis0.controller.vel_setpoint = 0
        self.driver.axis1.controller.vel_setpoint = 0

        rospy.loginfo("ODrive engaged.")
        
    def spin(self):
        rospy.spin()

    def l_callback(self, current):
        if current.data > self.MAX_CURRENT:
            current.data = self.MAX_CURRENT

        if current.data < self.MIN_CURRENT:
            current.data = self.MIN_CURRENT

        rospy.loginfo("Sending currrent to left motor: %.2f", current.data)
        self.driver.axis0.controller.vel_setpoint = current.data

    def r_callback(self, current):
        if current.data > self.MAX_CURRENT:
            current.data = self.MAX_CURRENT

        if current.data < self.MIN_CURRENT:
            current.data = self.MIN_CURRENT

        rospy.loginfo("Sending currrent to right motor: %.2f", current.data)
        self.driver.axis1.controller.vel_setpoint = current.data


if __name__ == '__main__':
    try:
        node = ODriveNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass