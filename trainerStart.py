if __name__ == "__main__":
    import socket
    import gymnasium as gym
    import numpy as np
    from typing import Tuple, Dict, Any
    import multiprocessing as mp
    import functools
    import atexit
    import signal
    from envUtil import init_env, cleanUp
    import sys
    from multiprocessing import Manager
    from util import exportONNX
    import datetime

    NUM_ENVS = int(sys.argv[1])
    NUM_ROBOTS_IN_ENV = int(sys.argv[2])


    if(not NUM_ENVS or not NUM_ROBOTS_IN_ENV):
        raise "You need to pass how many robots to start"

    HOST = "127.0.0.1"
    BASEPORT = 9876


    observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(29,), dtype=np. float32)

    manager = Manager()
    sockets : list[socket.socket] = manager.list()
    
    server_sockets = []
    for i in range(NUM_ROBOTS_IN_ENV * NUM_ENVS):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        srv.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 256*1024)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 256*1024)
        srv.setsockopt(socket.IPPROTO_TCP, 12, 1)

        srv.bind((HOST, BASEPORT + i))
        srv.listen()
        server_sockets.append(srv)
    
    # CLEANUP
    atexit.register(cleanUp)
    def signal_handler(sig, frame):
        cleanUp()
        raise SystemExit("Exiting due to signal")
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    from stable_baselines3.common.vec_env import SubprocVecEnv
    from stable_baselines3 import PPO
    import torch
    startTime = datetime.datetime.now().__str__()
    
    print("WAAAH",startTime)
    
    
    policy_kwargs = dict(
        net_arch=dict(pi=[64, 64], vf=[128, 64]),
        activation_fn=torch.nn.LeakyReLU
    )
    
    try:
        mp.set_start_method("fork", force=True)
    except RuntimeError:
        # start method already set; ignore
        pass
    
    env_fns : list[functools.partial] = []
    

    env_fns = [functools.partial(init_env, i, BASEPORT, sockets, NUM_ENVS, NUM_ROBOTS_IN_ENV) for i in range(NUM_ROBOTS_IN_ENV * NUM_ENVS)]
    
    
    env = SubprocVecEnv(env_fns)
    
    i :int = 0
    while(sockets.__len__() != NUM_ROBOTS_IN_ENV * NUM_ENVS):
        

        print(f"Listening on {BASEPORT + i }")
        conn, addr = server_sockets[i].accept()
        print("YIPPI")
        sockets.append(conn)
        i+= 1
        
    
    model : PPO = PPO("MlpPolicy",verbose=1,policy_kwargs=policy_kwargs,env=env, device="cpu")
    
    while True:
        model.learn(1000000)
        
        exportONNX(model, startTime)

