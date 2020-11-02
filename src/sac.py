
from robot_env import RobotEnv
from ros_driver_simulator import RosDriverSimulator
from sac_sac import *

ros_node = RosDriverSimulator()
env = RobotEnv(ros_node)

network_params = {
    'hidden_sizes': [64, 64],
    'activation': 'relu',
    'policy': kl_policy
}

rl_params = {
    # env params
    # 'env_name':'FrozenLake-v0',
    # 'env_name': 'CartPole-v1',
    # 'env_name':'Taxi-v2',
    # 'env_name':'MountainCar-v0',
    # 'env_name':'Acrobot-v1',
    # 'env_name':'LunarLander-v2',

    # control params
    'seed': int(1),
    'epochs': int(50),
    'steps_per_epoch': 2000,
    'replay_size': 100000,
    'batch_size': 256,
    'start_steps': 4000,
    'max_ep_len': 500,
    'save_freq': 5,
    'render': False,

    # rl params
    'gamma': 0.99,
    'polyak': 0.995,
    'lr': 0.0003,
    'state_hist_n': 1,
    'grad_clip_val': None,

    # entropy params
    'alpha': 'auto',
    'target_entropy_start': 0.3,  # proportion of max_entropy
    'target_entropy_stop': 0.3,
    'target_entropy_steps': 1e5,
}

saved_model_dir = './saved_models'
logger_kwargs = setup_logger_kwargs(
    exp_name='sac_discrete_' + rl_params['env_name'], seed=rl_params['seed'], data_dir=saved_model_dir, datestamp=False)

sac(lambda: env, actor_critic=mlp_actor_critic,
    logger_kwargs=logger_kwargs,
    network_params=network_params,
    rl_params=rl_params)
