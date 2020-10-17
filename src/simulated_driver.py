#!./python
# -*- coding: utf-8 -*-
"""AI wheel driver node simulator for offline training"""

#from train import Trainer
import numpy as np
from qlearning import Trainer

class AiDriverSimulator(object):

    last_odom = None
    last_vel = None

    wheel_current_l = 0
    wheel_current_r = 0

    def __init__(self):
        self.last_odom.linear.x = 0.0
        self.last_odom.angular.z = 0.0

    def sendCurrent(self, current_left, current_right):
        self.wheel_current_l = current_left
        self.wheel_current_r = current_right



# Main function.
if __name__ == "__main__":
    ros_node = AiDriverSimulator()
    trainer = Trainer()
    trainer.run(ros_node)

