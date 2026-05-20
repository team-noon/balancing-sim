if __name__ == "__main__":
    import socket
    import gymnasium as gym
    import numpy as np
    import multiprocessing as mp
    import functools
    import atexit
    import signal
    from envUtil import init_env, cleanUp
    import sys
    from multiprocessing import Manager
    from util import exportModel
    import datetime
    import os
    
    DEBUG : bool =False
    
    NUM_ENVS : int
    NUM_ROBOTS_IN_ENV : int
    CONTINUE :bool
    SAVE_INTERVAL : int

    if(sys.argv.__len__() == 2):
        if(sys.argv[1].strip().lower() == "true"):
            DEBUG =True
            NUM_ENVS = 1
            NUM_ROBOTS_IN_ENV = 2
            SAVE_INTERVAL = 1000
            CONTINUE = False
            
            print(f"\033[93m Warning: you are using debug mode, no saves will be done \033[0m")


    if(sys.argv.__len__() < 5 and not DEBUG):
        raise "at least 4 arguments are needed, 1: NUM ENVS 2: NUM ROBOTS/ENV 3: SAVE INTERVAL 4: CONTINUE?"



    if(not DEBUG):
        NUM_ENVS  = int(sys.argv[1])
        NUM_ROBOTS_IN_ENV : int= int(sys.argv[2])
        SAVE_INTERVAL : int= int(sys.argv[3])
        arg = sys.argv[4].strip()
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

    manager = Manager()
    sockets : list[socket.socket] = manager.list()
    
    server_sockets = []
    for i in range(NUM_ROBOTS_IN_ENV * NUM_ENVS):
        SOCK_PATH = f"/tmp/noon_robot_{i}.sock"
        try:
            os.unlink(SOCK_PATH)
        except FileNotFoundError:
            pass
        
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        #srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        #srv.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        #srv.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 256*1024)
        #srv.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 256*1024)
        #srv.setsockopt(socket.IPPROTO_TCP, 12, 1)

        srv.bind(SOCK_PATH)
        #srv.bind((HOST, BASEPORT + i))
        
        srv.listen()
        server_sockets.append(srv)
        
        
    from stable_baselines3.common.vec_env import SubprocVecEnv
    from stable_baselines3 import SAC
    import torch
        
    model : SAC
    startTime = datetime.datetime.now().__str__()
    
    # CLEANUP
    atexit.register(cleanUp)
    def signal_handler(sig, frame):
        if(model and not DEBUG):
            exportModel(model, startTime)
        cleanUp()
        raise SystemExit("Exiting due to signal")
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)


    # pins the process to 1 core
    #os.sched_setaffinity(0, {0})
    
    
    
    policy_kwargs = dict(
        net_arch=dict(pi=[256, 256, 256, 256], qf=[128, 128, 128]),
        activation_fn=torch.nn.ReLU
    )
    
    try:
        mp.set_start_method("fork", force=True)
    except RuntimeError:
        # start method already set; ignore
        pass
    
    env_fns : list[functools.partial] = []
    

    env_fns = [functools.partial(init_env, i, BASEPORT, sockets, NUM_ENVS, NUM_ROBOTS_IN_ENV, DEBUG) for i in range(NUM_ROBOTS_IN_ENV * NUM_ENVS)]
    
    
    env = SubprocVecEnv(env_fns)
    
    i :int = 0
    while(sockets.__len__() != NUM_ROBOTS_IN_ENV * NUM_ENVS):
        

        
        conn, _ = server_sockets[i].accept()
        print(f"{i+1} / {NUM_ROBOTS_IN_ENV * NUM_ENVS} robot connected with port: {BASEPORT + i}")
        sockets.append(conn)
        i+= 1
        
    
        
    if not CONTINUE:
        model = SAC(
            "MlpPolicy",
            env=env,
            verbose=1,

            learning_rate=4e-4,
            n_steps=4096,
            batch_size=2048,
            
            learning_starts=0,
            gradient_steps=10,
            train_freq=10,
            
            # CHANGE THIS ITS ONLY SO LOW SO I CAN TEST ON MY SHITTY LAPTOP
            buffer_size=1000,

            gamma=0.85,


            ent_coef=0.065,

            use_sde=True,
            sde_sample_freq=1,


            policy_kwargs=policy_kwargs,
        )
        
        print("SOFT WARMUP STARTING")
        print(model.replay_buffer.size())
        
        done_warmup = False
        
        restart = True
        
        obs : np.ndarray= np.zeros(231, dtype=np.float32)
        next_obs : np.ndarray = np.zeros(231, dtype=np.float32)
        
        print(str(sockets[0].recv(3)))
        
        # WARM STARTUP
        while(not done_warmup):
            if(restart):
                data = sockets[0].recv(231 * 4)

                obs = np.frombuffer(data, dtype=np.float32)
                
                restart = False
                
                print(obs)
                
                del data
                
            print("asd2")
            
            data = sockets[0].recv(231 * 4 + 4 + 1 + 1 + 1 + 18 * 4)
            
            
            print(data.__len__())
            
            next_obs = np.frombuffer(data[0:231*4], dtype=np.float32)
            
            print(next_obs)
            
            print(np.frombuffer(data[231*4:231*4+4], dtype=np.float32))
            
            reward = np.frombuffer(data[231*4:231*4+4], dtype=np.float32)[0]
            terminated = bool(data[231*4+4])
            truncated = bool(data[231*4+5])
            done_warmup = bool(data[231*4+6])
            
            action = np.frombuffer(data[231*4+7:231*4+7+18* 4], dtype=np.float32)
            
            if(terminated or truncated):
                restart = True
               
            info = np.array([]) 
            if(truncated):
                info = np.array([])
            
            model.replay_buffer.add(action=action, done=np.array([truncated or terminated]), infos=info, next_obs=next_obs, obs=obs, reward=reward)
            
            pass
        
        print("SOFT WARMUP DONE")
        print(model.replay_buffer.size())
        
        model.learn(0)
        
        for i in range(256):
            model.train(256, 2048)
        
    else:
        model = SAC.load("./models/continue", env=env, policy_kwargs=policy_kwargs, device="cpu")
        print("imported model to continue training")
    
    while True:
        model.train
        model = model.learn(SAVE_INTERVAL)
        

        
        if(not DEBUG):
            exportModel(model, startTime)

