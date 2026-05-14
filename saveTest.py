import torch
from stable_baselines3 import SAC
import gymnasium as gym
import numpy as np
from typing import Dict, Tuple, Any
from util import exportModel


def xor(a: np.float32, b: np.float32) -> np.float32:
    if((a == 0 and b==0) or (a!= 0 and b!= 0)):
        return 0
    return 1

def other(a: np.float32) -> np.float32:
    if(a != 0):
        return 0
    return 1




cases = [[0,0], [0,1],[1,0], [1,1]]

class DummyEnv(gym.Env):
        curCase : int = 0
        
        def __init__(self):
            super().__init__()
            self.observation_space = gym.spaces.Box(low=0, high=1, shape=(2,), dtype=np.float32)
            self.action_space = gym.spaces.Box(low=0.0, high=1.0, shape=(1,), dtype=np.float32)

        def reset(self, seed=None, options=None) -> tuple[list[float], dict]:
            
            
            return cases[self.curCase % 4], {}

        def step(self, action : np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
            reward = -abs(action[0] - xor(cases[(self.curCase) ][0], cases[(self.curCase) ][1]))
                         
            
            #if(action[0] > 0.6 and xor(cases[(self.curCase) ][0], cases[(self.curCase) ][1]) == 1):
            #    reward = 1
            #    
            #if(action[0] > 0.4 or action[0] < 0.8):
            #    reward = (10*(action[0]-0.4)-1) * (xor(cases[(self.curCase) ][0], cases[(self.curCase) ][1]) *2 -1)
            #    
            #if(action[0] < 0.4 and xor(cases[(self.curCase) ][0], cases[(self.curCase) ][1]) == 0):
            #    reward = 1
         
            self.curCase = np.random.choice([0,1,2,3])

            info = {}
            return cases[self.curCase ], reward, False, True, info
        
env = DummyEnv()
        
policy_kwargs = dict(
    net_arch=dict(pi=[2], qf=[1]),
    activation_fn=torch.nn.LeakyReLU
)

model = SAC("MlpPolicy",
    env=env,
    verbose=1,
    learning_rate=0.02,
    learning_starts=50,
    gamma=1,
    use_sde=False,
    policy_kwargs=policy_kwargs,
    gradient_steps=50,
    train_freq=1
)

model.replay_buffer.handle_timeout_termination = False

for obs in cases:
    
    
    model.replay_buffer.add(done=np.array([True]), action=np.array([xor(obs[0], obs[1])], np.float32), obs=np.array(obs, np.float32), infos={}, next_obs=np.array(obs, np.float32), reward=10)
        
        
    

model.learn(10)

model.train(50, 132)




print("\n=== XOR TEST ===")

for obs in cases:
    obs_array = np.array(obs, dtype=np.float32)

    action, _ = model.predict(obs_array, deterministic=True)

    predicted = 1 if action[0] > 0.5 else 0
    expected = xor(obs[0], obs[1])

    print(
        f"input={obs} "
        f"raw_action={action[0]:.4f} "
        f"predicted={predicted} "
        f"expected={expected}"
    )
    
exportModel(model, "test")