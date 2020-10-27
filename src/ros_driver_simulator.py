#!./venv/bin/python3
# -*- coding: utf-8 -*-
"""AI wheel driver node simulator for offline training"""

import numpy as np


class RosDriverSimulator(object):

    last_odom = None
    last_vel = None

    wheel_current_l = 0.0
    wheel_current_r = 0.0
    current_l_avg = 0.0
    current_r_avg = 0.0
    averages = []

    def __init__(self):
        self.x = 0.0
        self.z = 0.0

    def getLinearSpeed(self):
        return self.x

    def getAngularSpeed(self):
        return self.z

    def ready(self):
        return True

    def reset(self):
        self.x = 0.0
        self.z = 0.0
        self.averages.clear()
        self.current_l_avg = 0
        self.current_r_avg = 0

    def sendCurrent(self, current_left, current_right):
        self.wheel_current_l = current_left
        self.wheel_current_r = current_right
        self.calcVelocities()
        self.calcAverages()

    def calcAverages(self):
        self.averages.append((self.wheel_current_l, self.wheel_current_r))

        if (len(self.averages) > 5):
            self.averages.pop(0)

        l = 0.0
        r = 0.0
        for item in self.averages:
            l += item[0]
            r += item[1]

        self.current_l_avg = l/len(self.averages)
        self.current_r_avg = r/len(self.averages)

    def calcVelocities(self):
        l = self.current_l_avg
        r = self.current_r_avg
        if abs(l) < 0.5:
            l = 0.0

        if abs(r) < 0.5:
            r = 0.0

        if abs(l) < 0.5 and abs(r) < 0.5:
            self.x = 0.0
            self.z = 0.0
            return

        tyre_circumference = 0.67   # m
        wheel_track = 0.35   # m

        if (l - r) != 0:
            s = tyre_circumference * (l+r) / ((r - l) * 2.0)
        else:
            s = l * tyre_circumference

        w = tyre_circumference * ((r-l) / wheel_track) * 0.1
        self.x = s
        self.z = w
