#!./python
# -*- coding: utf-8 -*-
"""Odrive current ROS driver"""

from std_msgs.msg import Float32
import rospy
import pprint
from nav_msgs.msg import Odometry
import os
import time
from slack import WebClient
from slack.errors import SlackApiError

class VirtualFenceNode(object):

    initial_x = None
    initial_y = None

    fence_width = 0.5
    fence_length = 0.5

    def __init__(self):
        rospy.init_node("virtual_fence_node")
        self.odom_subscriber = rospy.Subscriber("/odometry/filtered", Odometry, self.odomCallback, queue_size=2)
        self.pub_wheel_current_l = rospy.Publisher("cmd_wheel_current_l", Float32, queue_size=10)
        self.pub_wheel_current_r = rospy.Publisher("cmd_wheel_current_r", Float32, queue_size=10)

        rospy.Timer(rospy.Duration(1.0), self.printDebugInfos)
        rospy.loginfo("Virtual fence starting...")

    def sendCurrent(self, current_left, current_right):
        self.pub_wheel_current_l.publish(current_left)
        self.pub_wheel_current_r.publish(current_right)

    def spin(self):
        rospy.spin()

    def odomCallback(self, odom):
        if self.initial_x == None:
            self.initial_x = odom.pose.pose.position.x
            self.initial_y = odom.pose.pose.position.y

        self.x = self.initial_x - odom.pose.pose.position.x
        self.y = self.initial_y - odom.pose.pose.position.y

        if abs(self.x) > self.fence_width or abs(self.y) > self.fence_length:
            rospy.loginfo("Fence reached!!!!")
            os.system('killall -9 ai_driver_node.py')
            rospy.loginfo("ai_driver_node killed.")
            self.sendCurrent(0,0)
            time.sleep(3)
            self.sendCurrent(0,0)
            rospy.loginfo("Sent stop to wheels.")
            rospy.signal_shutdown("Exiting.")
            os.system("curl -X POST -H 'Content-type: application/json' --data '{\"text\":\"Virtual fence reached!\"}' https://hooks.slack.com/services/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")

    def printDebugInfos(self, event):
        rospy.loginfo("Pos x: %.2f Pos y: %.2f", self.x, self.y)

if __name__ == '__main__':
    try:
        node = VirtualFenceNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass