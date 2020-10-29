
import time

import sac_actor
from robot_env import RobotEnv
from ros_driver_simulator import RosDriverSimulator
from sac_agent import Agent

alpha = 0.01
tau = 0.01
batchsize = 64

# Environment
ros_node = RosDriverSimulator()
env = RobotEnv(ros_node)
input_dim = 16
output_dim = 13

# Agent
lr, gamma = 3*10**-4, 0.99
clipnorm, verbose = False, False
agent = Agent(input_dim, output_dim, lr, gamma, tau, alpha, clipnorm, verbose)
agent.memory_size = batchsize
agent.batchsize = batchsize

# Train
EPISODES = 10**4
scores = []
t1 = time.time()
for e in range(1, EPISODES+1):
    state = env.reset()
    state = agent.make_tensor(state)
    reward_sum = 0
    done = False
    while not done:

        # Do main step
        # env.render()
        action = agent.act(state)
        next_state, reward, done, _ = env.step(action)
        reward_sum += reward
        next_state = agent.make_tensor(next_state)
        # want to remember state as a vec
        agent.remember(state[0], action, reward, next_state[0], done)
        state = next_state
        if e >= 2:
            agent.learn()
