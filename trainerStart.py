from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3 import PPO
import torch
import subprocess
import os
import socket
import gymnasium as gym
import numpy as np
from typing import Tuple, Dict, Any

NUM_ROBOTS = 1


HOST = "127.0.0.1"
PORT = 9876

observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(29,), dtype=np. float32)


def make_env(rank):
    thisSocket : socket.socket
    def reset(seed=None, options=None) -> tuple[list[float], dict]:
        print("RESET ALREADY YOU CUNT")
        return [0], {}
    
    def step(action: np.ndarray) -> Tuple[list[float], float, bool, bool, Dict[str, Any]]:
        
        print("STEP ALREADY YOU CUNT")
        return [0], 0, False, False, {}
    
    def _init():
        env : gym.Env = gym.Env()
        
        env.action_space = gym.spaces.Box(low=0, high=1,shape=(18,), dtype=np.float32)

        env.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(29,), dtype=np. float32)
        
        env.reset = reset
        
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

