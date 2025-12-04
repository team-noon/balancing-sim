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

    if(sys.argv.__len__() < 4):
        raise "at least 3 arguments are needed, 1: NUM ENVS 2: NUM ROBOTS/ENV 3: CONTINUE?"

    NUM_ENVS = int(sys.argv[1])
    NUM_ROBOTS_IN_ENV = int(sys.argv[2])
    arg = sys.argv[3].strip()
    if arg.lower() == "true":
        CONTINUE = True
    else:
        try:
            CONTINUE = float(arg) > 0
        except ValueError:
            CONTINUE = False


    if(not NUM_ENVS or not NUM_ROBOTS_IN_ENV):
        raise "You need to pass how many envs and robots to start"

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
        
        
    from stable_baselines3.common.vec_env import SubprocVecEnv
    from stable_baselines3 import PPO
    import torch
        
    model : PPO
    startTime = datetime.datetime.now().__str__()
    
    # CLEANUP
    atexit.register(cleanUp)
    def signal_handler(sig, frame):
        if(model):
            exportONNX(model, startTime)
        cleanUp()
        raise SystemExit("Exiting due to signal")
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    

    
    
    
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
        

        
        conn, addr = server_sockets[i].accept()
        print(f"{i} / {NUM_ROBOTS_IN_ENV * NUM_ENVS} robot connected with port: {BASEPORT + i}")
        sockets.append(conn)
        i+= 1
        
    
        
    if not CONTINUE:
        model : PPO = PPO("MlpPolicy",verbose=1,policy_kwargs=policy_kwargs,env=env, device="cpu")
        
    else:
        model = PPO.load("./models/continue", env=env, policy_kwargs=policy_kwargs, device="cpu")
        print("imported model to continue training")
    
    while True:
        model.learn(1000000)
        
        exportONNX(model, startTime)

