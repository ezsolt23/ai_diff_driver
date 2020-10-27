from robot_env import RobotEnv
from ros_driver_simulator import RosDriverSimulator

ros_node = RosDriverSimulator()
env = RobotEnv(ros_node)

env.reset()
env.setTarget(0.3, 0)


def step(action):
    env.step(action)
    r = env.getReward()
    print(" Linear vel: %.2f Angular vel: %.2f  Reward: %.2f " %
          (ros_node.getLinearSpeed(), ros_node.getAngularSpeed(), r))


for i in range(8):
    step(3)  # incrrease l and r

for i in range(4):
    step(6)  # incrrease l and r

for i in range(4):
    step(3)  # incrrease l and r

for i in range(10):
    step(0)  # incrrease l and r
