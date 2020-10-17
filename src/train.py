import numpy as np
import random
from neural_net import neural_net, LossHistory
import timeit
import rospy
from pprint import pprint
import time
import pickle
import csv

class Trainer(object):

    BUFFER_SIZE = 5000
    BATCH_SIZE = 64
    GAMMA = 0.9  # Forgetting.
    epsilon = 1.0
    loss_log = []
    training_active = False
    training_epoches = 0
    collect_mode = False
    collect_counter = 0
    bad_actions_in_a_row = 0

    current_left  = 0.0
    current_right = 0.0
    start_random = True

    reward = 0.0

    def __init__(self,ros_node, collect_mode = False):
        self.model = neural_net() #'saved-models/model-1-8000.h5')
        #self.start_random = False

        self.ros_node = ros_node
        self.replay = []

        time.sleep(2)
        self.old_state = self.getState()
        self.old_action = self.getAction()

        if collect_mode == True:
            rospy.loginfo("Collect mode activted. No actual learning will happen. I only save data to file.")
            self.collect_mode = True

        rate = 10
        rospy.Timer(rospy.Duration(1.0 / rate), self.train)
        self.start_time=timeit.default_timer()

    def train(self, event):
        with open('STATE.txt') as f: s = f.read()
        if s == 'p':
            rospy.loginfo("Training paused for 5 seconds")
            time.sleep(5)
            return

        if self.training_active == True :
            rospy.loginfo("Training overrun.")
            return

        self.training_active = True

        new_action = self.getAction()
        self.sendWheelsCurrent(new_action)

        # Use old action and new state for training as 
        # it needs time for the action to take effect"""
        state = self.old_state
        action = self.old_action
        new_state = self.getState()
        reward = self.getReward()


        self.replay.append((state, action, reward, new_state))
        # If we've stored enough in our buffer, pop the oldest.
        if len(self.replay) > self.BUFFER_SIZE:
            self.replay.pop(0)

        if len(self.replay) < self.BATCH_SIZE:
            self.finishTraining(new_action, new_state)
            return

        if self.collect_mode == True:
            self.finishTraining(new_action, new_state)
            self.collect_counter +=1
            if self.collect_counter < 100:
                return
            self.collect_counter = 0
            self.saveHistoryData()
            return

        minibatch = random.sample(self.replay, self.BATCH_SIZE)
        x_train, y_train = self.prepareBatch(minibatch)

        history = LossHistory()
        self.model.fit(
            x_train, y_train, batch_size=self.BATCH_SIZE,
            verbose=0, epochs=1, callbacks=[history]
        )
        self.loss_log.append(history.losses)

        self.training_epoches +=1

        if reward < 0:
            self.reset()

        # if we run out of time, we reset
        if timeit.default_timer()-self.start_time > 10:
            self.reset()

        if self.training_epoches % 2000 == 0:
            self.model.save_weights('saved-models/model-1-' +
                               str(self.training_epoches) + '.h5',
                               overwrite=True)
            self.saveLosses()
            rospy.loginfo("Saving model.")

        print(self.training_epoches , end = ",")
        #rospy.loginfo("Training finished for epoch %d", self.training_epoches)
        self.finishTraining(new_action, new_state)

    def saveLosses(self):
        with open('loss_data.csv', 'w') as lf:
            wr = csv.writer(lf)
            for loss_item in self.loss_log:
                wr.writerow(loss_item)

    def saveHistoryData(self):
        filename = 'history_data.dat'
        f = open(filename,'w')
        str = repr(self.replay)
        f.write(str)
        f.close()
        rospy.loginfo("Data saved.")

    def finishTraining(self, new_action, new_state):
        self.old_action = new_action
        self.old_state = new_state
        self.training_active = False
        self.reward -= 1

    def sendWheelsCurrent(self, action):
        # sendCurrent (left, right)
        inc = 0.4
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

        self.ros_node.sendCurrent(self.current_left, self.current_right)

    def getReward(self):
        reward = self.ros_node.reward

        
        return reward

    def reset(self):
        # Reset
        self.reward = 0
        self.current_left = 0
        self.current_right = 0
        rospy.loginfo("          ----------           RESET")
        self.sendWheelsCurrent(0)
        time.sleep(1)
        self.start_time=timeit.default_timer()


    def getAction(self):
        # Actions:
        #      left     right 
        # 0:   0        0
        # 1:   +0.2     0
        # 2:   0        +0.2
        # 3:   +0.2     +0.2
        # 4:   -0.2     0
        # 5:   0        -0.2
        # 6:   -0.2     -0.2
        #    

        if self.start_random and (self.training_epoches < 100 or random.random() < self.epsilon): 
            #or self.bad_actions_in_a_row > 10:
            action = np.random.randint(0, 9)  # random
            return action
        
        # Decrement epsilon over time.
        if self.training_epoches > 100:
            self.epsilon -= (1.0/4000)

        qval = self.model.predict(self.getState(), batch_size=1)
        action = (np.argmax(qval))
        return action

    def getState(self):
        while self.ros_node.last_odom == None:
            time.sleep(1)    
        data =  [
            10.0+self.ros_node.last_odom.linear.x, 
            10.0+self.ros_node.last_odom.angular.z,
            10.0+self.ros_node.last_vel.linear.x,
            10.0+self.ros_node.last_vel.angular.z,
            10.0+self.ros_node.wheel_current_l,
            10.0+self.ros_node.wheel_current_r
            ]
        state = np.array([data])
        return state

    def loop(self):
        rospy.spin()


    def prepareBatch(self, minibatch):
        # by Microos, improve this batch processing function 
        #   and gain 50~60x faster speed (tested on GTX 1080)
        #   significantly increase the training FPS
        
        # instead of feeding data to the model one by one, 
        #   feed the whole batch is much more efficient

        mb_len = len(minibatch)

        old_states = np.zeros(shape=(mb_len, 6))
        actions = np.zeros(shape=(mb_len,))
        rewards = np.zeros(shape=(mb_len,))
        new_states = np.zeros(shape=(mb_len, 6))

        for i, m in enumerate(minibatch):
            old_state_m, action_m, reward_m, new_state_m = m
            old_states[i, :] = old_state_m[...]
            actions[i] = action_m
            rewards[i] = reward_m
            new_states[i, :] = new_state_m[...]

        old_qvals = self.model.predict(old_states, batch_size=mb_len)
        new_qvals = self.model.predict(new_states, batch_size=mb_len)

        maxQs = np.max(new_qvals, axis=1)
        y = old_qvals
        non_term_inds = np.where(rewards < 0)[0]
        term_inds = np.where(rewards >= 0)[0]

        y[non_term_inds, actions[non_term_inds].astype(int)] = rewards[non_term_inds] + (self.GAMMA * maxQs[non_term_inds])
        y[term_inds, actions[term_inds].astype(int)] = rewards[term_inds]

        X_train = old_states
        y_train = y
        return X_train, y_train

