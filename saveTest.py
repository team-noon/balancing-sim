import torch
from stable_baselines3 import SAC
from envUtil import init_env
import gymnasium as gym
import numpy as np
from typing import Dict, Tuple, Any

policy_kwargs = dict(
    net_arch=dict(pi=[256, 256, 256, 256], vf=[128, 128, 128]),
    activation_fn=torch.nn.LeakyReLU
)

class DummyEnv(gym.Env):
        
        def __init__(self):
            super().__init__()
            self.action_space = gym.spaces.Box(low=0.0, high=1.0, shape=(1,), dtype=np.float32)
            self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(1,), dtype=np.float32)

        def reset(self, seed=None, options=None) -> tuple[list[float], dict]:
            
            
            return [0], {}

        def step(self, action : np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
            
         
            info = {}
            return [0], 0, False, False, info
        
env = DummyEnv()
        
policy_kwargs = dict(
    net_arch=dict(pi=[2, 2], qf=[2, 2]),
    activation_fn=torch.nn.LeakyReLU
)

model = SAC("MlpPolicy",
    env=env,
    device="cpu",
    verbose=1,
    learning_rate=4e-4,
    n_steps=4096,
    batch_size=256,
    gamma=0.9,
    ent_coef=0.065,
    use_sde=True,
    sde_sample_freq=1,
    policy_kwargs=policy_kwargs
)

