# -*- coding: utf-8 -*-
"""AI wheel driver node"""

import csv
import math
import random
import time
import timeit

import numpy as np
from tensorflow.keras.layers import Activation, Dense, Flatten, Input
from tensorflow.keras.models import Sequential
from tensorflow.keras.optimizers import Adam

import exceptions


class RobotEnv(object):

    current_left = 0.0
    current_right = 0.0
    reward = 0.0
    error = 0
    linear_diff = 0
    angular_diff = 0
    time_penalty = 0
    steps = 0
    sum_reward = 0.0
    target_x = 0.0
    target_z = 0.0

    def __init__(self, ros_node):
        self.ros_node = ros_node
        self.start_time = timeit.default_timer()
        #self.rate = rospy.Rate(5)
        self.reset()
        self.calcReward()

        #rospy.Timer(rospy.Duration(1.0), self.printDebugInfos)

    def setTarget(self, x, z):
        self.target_x = x
        self.target_z = z

    def step(self, action):
        """Run one timestep of the environment's dynamics.
        Accepts an action and returns a tuple (observation, reward, done, info).
        # Arguments
            action (object): An action provided by the environment.
        # Returns
            observation (object): Agent's observation of the current environment.
            reward (float) : Amount of reward returned after previous action.
            done (boolean): Whether the episode has ended, in which case further step() calls will return undefined results.
            info (dict): Contains auxiliary diagnostic information (helpful for debugging, and sometimes learning).
        """

        self.steps += 1
        self.sendWheelsCurrent(action)
        # self.rate.sleep()

        self.time_penalty -= 0.001
        if self.ros_node.getLinearSpeed() == 0:
            self.time_penalty -= 0.1

        reward = self.getReward()
        self.sum_reward += reward

        done = False
        if self.actual_target_dist > self.origo_target_dist * 1.1:
            self.reward = -200
            reward = self.reward
            done = True

        if self.target_z == 0 and abs(self.ros_node.getAngularSpeed()) > 1.0:
            self.reward = -200
            reward = self.reward
            done = True

        if self.ros_node.getLinearSpeed() > 0:
            reward += 1

        observation = self.getObservation()
        info = {}
        return (observation, reward, done, info)

    def reset(self):
        """
        Resets the state of the environment and returns an initial observation.
        # Returns/home/ezsolt/.ros/rtabmap.db
            observation (object): The initial observation of the space. Initial reward is assumed to be 0.
        """

        previous_reward = self.reward
        previous_speed = self.ros_node.getLinearSpeed()
        previous_x = self.target_x
        previous_z = self.target_z

        print("Reward: %.2f\tX: %.2f\tZ: %.2f\tX speed: %.2f Z speed:%.2f" %
              (previous_reward, previous_x, previous_z, previous_speed, self.ros_node.getAngularSpeed()))

        self.ros_node.reset()
        self.sum_reward = 0
        self.reward = 0
        self.steps = 0
        self.time_penalty = 0
        self.current_left = 0
        self.current_right = 0
        self.sendWheelsCurrent(0)
        # time.sleep(2)
        self.start_time = timeit.default_timer()

        # Set up new random target speed
        # self.ros_node.last_vel.linear.x
        #self.target_x = random.uniform(0.3, 1.0)
        #self.target_x *= random.choice([-1, 1])
        self.target_x = 1.0
        # self.ros_node.last_vel.angular.z
        self.target_z = 0
        #self.target_z = random.uniform(-0.5, 0.5)

        if abs(self.target_x) < 0.15:
            self.target_x = 0.0

        if abs(self.target_z < 0.15):
            self.target_z = 0.0

        observation = self.getObservation()
        reward = self.reward
        return observation

    def getReward(self):
        reward = self.calcReward()
        #self.reward = self.scale(-24, 1, reward + self.time_penalty)
        self.reward = reward + self.time_penalty
        return self.reward

    def getObservation(self):
        while not self.ros_node.ready():
            print("Res: ", self.scale(-1, 0, -1.5))

            print("Waiting for odom to become available..")
            time.sleep(1)

        data = [
            self.ros_node.averages[0][0],
            self.ros_node.averages[0][1],

            self.ros_node.averages[1][0],
            self.ros_node.averages[1][1],

            self.ros_node.averages[2][0],
            self.ros_node.averages[2][1],

            self.ros_node.averages[3][0],
            self.ros_node.averages[3][1],

            self.ros_node.averages[4][0],
            self.ros_node.averages[4][1],

            self.ros_node.getLinearSpeed(),
            self.ros_node.getAngularSpeed(),
            self.target_x,
            self.target_z,
            self.ros_node.wheel_current_l,
            self.ros_node.wheel_current_r
        ]

        state = np.array(data)

        # print(state)
        # print(state.shape)
        return state

    def floatToVect(self, min, max, value, steps):
        if value < min:
            value = min
        if value > max:
            value = max

        # if value < min or value > max:
        #    error = exceptions.Error("Index out of range. min: %.2f max %.2f value: %.2f", min, max, value)
        #    raise error

        nb_ranges = round((max - min) // steps)
        result = np.zeros(nb_ranges)

        index = round((value - min) // steps)-1
        result[index] = 1
        return result

    def calcReward(self):
        # ros_node.last_odom == None: #or self.last_vel == None:
        if not self.ros_node.ready():
            print("Waiting for ROS node to become ready.")
            return 0

        actual_x = self.ros_node.getLinearSpeed()  # last_odom.linear.x
        actual_z = self.ros_node.getAngularSpeed()  # last_odom.angular.z

        target_x = self.target_x
        target_z = self.target_z

        actual_target_dist = self.distance(
            actual_x, actual_z, target_x, target_z)
        origo_target_dist = self.distance(0, 0, target_x, target_z)
        self.actual_target_dist = actual_target_dist
        self.origo_target_dist = origo_target_dist

        # if origo_target_dist < 0.1 and actual_target_dist > 0.1:
        #    return -1

        if origo_target_dist == 0 and actual_target_dist < 0.05:
            return 1

        if actual_target_dist > origo_target_dist:
            return -1 * abs(actual_target_dist)

        penalty = actual_target_dist/origo_target_dist

        return 1-penalty

    def distance(self, x1, y1, x2, y2):
        return math.sqrt(pow(x1-x2, 2) + pow(y1-y2, 2))

    def printDebugInfos(self, event):
        print("Linear: %.2f - %.2f     Angular: %.2f - %.2f   O-Target dist: %.2f     A-target dist: %.2f   Reward: %.2f       L: %.2f    R: %.2f ",
              self.target_x, self.ros_node.getLinearSpeed(), self.target_z, self.ros_node.getAngularSpeed(), self.origo_target_dist, self.actual_target_dist, self.reward, self.current_left, self.current_right)

    def scale(self, min, max, value):
        if value > max:
            return 1
        if value < min:
            return -1
        return (1 - ((value - min) / (max - min)) * 2)

    def render(self, mode='human', close=False):
        print("", end="")

    def close(self):
        print("Close")

    def sendWheelsCurrent(self, action):
        # sendCurrent (left, right)
        inc = 0.3
        if action == 0:
            self.current_left += inc
        if action == 1:
            self.current_left += inc
        if action == 2:
            self.current_right += inc
        if action == 3:
            self.current_left += inc
            self.current_right += inc
        if action == 4:
            self.current_left -= inc
        if action == 5:
            self.current_right -= inc
        if action == 6:
            self.current_left -= inc
            self.current_right -= inc
        if action == 7:
            self.current_left += inc
            self.current_right -= inc
        if action == 8:
            self.current_left -= inc
            self.current_right += inc
        if action == 9:
            self.current_left += inc * 2
        if action == 10:
            self.current_right += inc * 2
        if action == 11:
            self.current_left -= inc * 2
        if action == 12:
            self.current_right -= inc * 2

        #print("{0:2d}".format(action), end=" ")

        self.ros_node.sendCurrent(self.current_left, self.current_right)
