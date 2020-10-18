# -*- coding: utf-8 -*-
"""AI wheel driver node"""

import numpy as np

from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Activation, Input, Flatten
from tensorflow.keras.optimizers import Adam

from rl.agents.dqn import DQNAgent
from rl.policy import BoltzmannQPolicy
from rl.memory import SequentialMemory
import rl.core
import rospy
import timeit
import time
import random
import csv
import math

class RobotEnv(rl.core.Env):

    current_left  = 0.0
    current_right = 0.0
    reward = 0.0
    error = 0
    linear_diff = 0
    angular_diff = 0
    time_penalty = 0
    steps = 0
    sum_reward = 0.0

    def __init__(self, ros_node):
        self.ros_node = ros_node
        self.start_time=timeit.default_timer()
        self.rate = rospy.Rate(5)
        self.reset()
        self.calcReward()

        #rospy.Timer(rospy.Duration(1.0), self.printDebugInfos)

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

        self.steps +=1
        self.sendWheelsCurrent(action)
        #self.rate.sleep()
        done = False
        if self.steps == 50:
            done = True

        self.time_penalty -= 0.001
        reward = self.getReward()
        self.sum_reward += reward

        observation = self.getObservation()
        info = {}
        return (observation, reward, done, info)


    def reset(self):
        """
        Resets the state of the environment and returns an initial observation.
        # Returns/home/ezsolt/.ros/rtabmap.db
            observation (object): The initial observation of the space. Initial reward is assumed to be 0.
        """
        rospy.loginfo("          ----------           RESET   %.2f", self.sum_reward)

        self.sum_reward = 0
        self.reward = 0
        self.steps = 0
        self.time_penalty = 0
        self.current_left = 0
        self.current_right = 0
        self.sendWheelsCurrent(0)
        #time.sleep(2)
        self.start_time=timeit.default_timer()

        # Set up new random target speed
        self.target_x = random.uniform(-0.5, 0.5)# self.ros_node.last_vel.linear.x 
        self.target_z = random.uniform(-0.5, 0.5) # self.ros_node.last_vel.angular.z

        if abs(self.target_x) < 0.15:
            self.target_x = 0.0

        if abs(self.target_z < 0.15):
            self.target_z = 0.0

        observation = self.getObservation()
        reward = self.getReward()
        return observation

    def getReward(self):
        reward = self.calcReward()
        self.reward = reward + self.time_penalty
        return reward

    def getObservation(self):
        while not self.ros_node.ready():
            print("Res: ", self.scale(-1,0,-1.5))

            rospy.loginfo("Waiting for odom to become available..")
            time.sleep(1)    
        data =  [
            self.scale(-1, 1, self.ros_node.getLinearSpeed()),  #last_odom.linear.x), 
            self.scale(-1, 1, self.ros_node.getAngularSpeed()), #last_odom.angular.z),
            self.scale(-1, 1, self.target_x),
            self.scale(-1, 1, self.target_z),
            self.scale(-5, 5, self.ros_node.wheel_current_l),
            self.scale(-5, 5, self.ros_node.wheel_current_r)
            ]
        state = np.array([data])
        return state

    def calcReward(self):
        if not self.ros_node.ready(): #ros_node.last_odom == None: #or self.last_vel == None:
            rospy.loginfo("Waiting for rospy node to become ready.")
            return 0

        actual_x = self.ros_node.getLinearSpeed() #last_odom.linear.x
        actual_z = self.ros_node.getAngularSpeed() #last_odom.angular.z

        target_x = self.target_x
        target_z = self.target_z

        actual_target_dist = self.distance(actual_x, actual_z, target_x, target_z)
        origo_target_dist = self.distance(0,0,target_x,target_z)
        self.actual_target_dist = actual_target_dist
        self.origo_target_dist = origo_target_dist

        #if origo_target_dist < 0.1 and actual_target_dist > 0.1:
        #    return -1

        if origo_target_dist == 0 and actual_target_dist < 0.05:
            return 1

        if actual_target_dist > origo_target_dist:
            return -1 * self.scale(0,2, actual_target_dist)

        penalty = actual_target_dist/origo_target_dist

        return self.scale(-1,0, -penalty)

    def distance(self, x1,y1,x2,y2):
        return math.sqrt(pow(x1-x2, 2) + pow(y1-y2, 2) )

    def printDebugInfos(self, event):
        rospy.loginfo("Linear: %.2f - %.2f     Angular: %.2f - %.2f   O-Target dist: %.2f     A-target dist: %.2f   Reward: %.2f       L: %.2f    R: %.2f ", 
            self.target_x, self.ros_node.getLinearSpeed(), self.target_z, self.ros_node.getAngularSpeed(), self.origo_target_dist, self.actual_target_dist, self.reward, self.current_left, self.current_right)

    def scale(self, min, max, value):
        if value > max:
            return 1
        if value < min:
            return 0
        return (value - min) / (max - min)

    def render(self, mode='human', close=False):
        print(".", end="")

    def close(self):
        rospy.loginfo("Close")

    def sendWheelsCurrent(self, action):
        # sendCurrent (left, right)
        inc = 0.3
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


class Trainer(object):

    def run(self, ros_node):
        env = RobotEnv(ros_node)

        nb_actions = 13
        input_size = 6

        model = Sequential()
        model.add(Flatten(input_shape=(1,1,input_size,)))
        #model.add(Input(shape=(input_size,)))
        model.add(Dense(32))
        model.add(Activation('relu'))
        model.add(Dense(32))
        model.add(Activation('relu'))
        model.add(Dense(32))
        model.add(Activation('relu'))
        model.add(Dense(nb_actions))
        model.add(Activation('linear'))
        print(model.summary())

        # Finally, we configure and compile our agent. You can use every built-in tensorflow.keras optimizer and
        # even the metrics!
        memory = SequentialMemory(limit=50000, window_length=1)
        policy = BoltzmannQPolicy()
        agent = DQNAgent(model=model, nb_actions=nb_actions, memory=memory, nb_steps_warmup=60,
                    target_model_update=1e-2, policy=policy)
        #agent = DDPGAgent(model=model, nb_actions=nb_actions, memory=memory, nb_steps_warmup=60,
        #            target_model_update=1e-2, policy=policy)

        agent.compile(Adam(lr=1e-3), metrics=['mae'])
        #agent.load_weights('qlearning_weights.h5f')

        # Okay, now it's time to learn something! We visualize the training here for show, but this
        # slows down training quite a lot. You can always safely abort the training prematurely using
        # Ctrl + C.

        for i in range(1000000):
            history_callback = agent.fit(env, nb_steps=100, visualize=True, verbose=0)
            #self.history = history_callback.history["loss"]
            agent.save_weights('qlearning_weights.h5f', overwrite=True)
            #self.saveLossHistory()

        env.reset()

    def saveLossHistory(self):
        with open('loss_data.csv', 'w') as lf:
            wr = csv.writer(lf)
            for loss_item in self.history.history:
                wr.writerow(loss_item)
