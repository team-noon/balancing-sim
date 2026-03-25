import gymnasium as gym
import numpy as np
import subprocess
import os
import socket
from typing import Dict, Any, Tuple
from util import recv_exact

subProcesses : list[subprocess.Popen[bytes]] = []

def init_env(rank: int, BASEPORT : int, socketsRef : list[socket.socket], numEnvs : int, numRobotsInEnv : int, Debug : bool):
    """
    Create and return an env instance for SubprocVecEnv.
    Keep this function top-level so it's picklable for spawn/forkserver.
    """
    class DummyEnv(gym.Env):
        thisRank : int
        sockets : list[socket.socket]
        
        def __init__(self, rank):
            super().__init__()
            self.rank = rank
            self.action_space = gym.spaces.Box(low=0.0, high=1.0, shape=(18,), dtype=np.float32)
            self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(198,), dtype=np.float32)
            self.thisRank = rank
            self.sockets = socketsRef

        def reset(self, seed=None, options=None) -> tuple[list[float], dict]:
            self.sockets[self.rank].sendall(b"r")
            
            
            data = recv_exact(self.sockets[self.rank], 198 * 4)
            
            
            return np.frombuffer(data, dtype=np.float32), {}

        def step(self, action : np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
            
            self.sockets[self.rank].sendall(b"s" + action.tobytes())
            
            data = recv_exact(self.sockets[self.rank], 198 * 4 + 4 + 1 + 1)
                
            
            obs = np.frombuffer(data[0:198*4], dtype=np.float32)
            reward = np.frombuffer(data[198*4:198*4+4], dtype=np.float32)[0]
            terminated = bool(data[198*4+4])
            truncated = bool(data[198*4+5])
        
            
            info = {}
            return obs, reward, terminated, truncated, info

    env = DummyEnv(rank)

    if rank % numRobotsInEnv == 0:
        
        # Start Webots and the controller — log output to files for debugging
        try:
            #webots_cmd = ["xvfb-run","--auto-servernum","webots","--batch", "--no-rendering" , "--mode=fast" , f"--port={BASEPORT - 1 - int(rank / numRobotsInEnv)}", f"{os.path.dirname(__file__)}/worlds/train.wbt"]
            webots_cmd=["webots","--batch","--no-rendering", "--mode=fast" , f"--port={BASEPORT - 1 - int(rank / numRobotsInEnv)}", f"{os.path.dirname(__file__)}/worlds/train.wbt"] # ONLY FUR DEBUGGING
            if(Debug):
                
                webots_cmd=["webots","--batch" , "--mode=fast" , f"--port={BASEPORT - 1 - int(rank / numRobotsInEnv)}", f"{os.path.dirname(__file__)}/worlds/trainDebug.wbt"] # ONLY FUR DEBUGGING
            
            webots_proc = subprocess.Popen(webots_cmd)


            ctrl_cmd = [f"{os.environ['WEBOTS_HOME']}/webots-controller", f"--port={BASEPORT - 1 - int(rank / numRobotsInEnv)}", f"{os.path.dirname(__file__)}/controllers/trainer/trainer.py", f"{int(rank / numRobotsInEnv)}", f"{numRobotsInEnv}", f"{Debug}"]  
            controller_proc = subprocess.Popen(ctrl_cmd)
            subProcesses.append(webots_proc)
            subProcesses.append(controller_proc)

        except Exception as e:
            # If launching webots fails, raise so SubprocVecEnv can detect crash
            raise RuntimeError(f"Failed to launch Webots or controller for rank {rank}: {e}")
    

    

    return env


def cleanUp():
    for process in subProcesses:
        process.terminate()
    os.system("pkill -9 webots")
    os.system("pkill -9 webots-bin")
    os.system("pkill -9 python3")
    