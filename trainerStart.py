from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3 import PPO
import torch
import subprocess
import os

NUM_ROBOTS = 8



def make_env(rank):
    def _init():
        # Start Webots without blocking
        webots_process = subprocess.Popen(
            ["webots", f"--port={4444+rank}", f"{__file__[:-17]}worlds/inference.wbt"]
        )

        # Start controller
        webots_controller = os.path.join(os.environ["WEBOTS_HOME"], "webots-controller")
        controller_process = subprocess.Popen(
            [webots_controller, f"--port={4444+rank}", f"{__file__[:-17]}controllers/noonRobotTrainer/noonRobotTrainer.py"]
        )
        
        return env
    return _init


policy_kwargs = dict(
    net_arch=dict(pi=[64, 64], vf=[128, 64]),
    activation_fn=torch.nn.LeakyReLU
)

env = SubprocVecEnv([make_env(i) for i in range(NUM_ROBOTS)])

model : PPO = PPO("MlpPolicy",verbose=1,policy_kwargs=policy_kwargs,env=env, device="cpu")

model.learn(1000000)