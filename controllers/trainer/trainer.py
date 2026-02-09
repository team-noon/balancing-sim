"""noonRobotTrainer controller."""

import math
from controller import Supervisor, InertialUnit, Gyro, Accelerometer, Node
from typing import List, Tuple, Dict, Any
import numpy as np

import socket
import sys
import os

controller_dir = os.path.dirname(__file__)
shared_dir = os.path.join(controller_dir, '..')
sys.path.append(os.path.abspath(shared_dir))
from classes import BodyPartData, MotorData
from initScripts import InitBodyParts, InitMotors
from parameters import brushlessSpeed, brushlessTorque, servoSpeed, servoTorque


HOST = "127.0.0.1"
BASEPORT = 9876

robot = Supervisor()
worldInfoInfoField = robot.getFromDef("WorldInfo").getField("info")
timestep = int(robot.getBasicTimeStep())

if robot.getName() == "trainer":
    if sys.argv.__len__() == 4:
        env_num = int(sys.argv[1])
        NUM_ROBOTS = int(sys.argv[2])
        
        DEBUG = False
        
        if(sys.argv[3].strip().lower()=="true"):
            DEBUG=True
        
        worldInfoInfoField.insertMFString(0, f"{env_num}")
        worldInfoInfoField.insertMFString(1, f"{NUM_ROBOTS}")
        worldInfoInfoField.insertMFString(2, f"{DEBUG}")


        if(DEBUG):
            print("Starting world in DEBUG mode")


        

        robot.getFromDef("TRAINER").getField("count").setSFInt32(NUM_ROBOTS)

        robot.simulationReset()
        

    robot.getSelf().remove()
    robot.step(timestep)
    exit()


robotSelf : Node = robot.getSelf()

if(robot.getName() != "trainer" and  robot.getName() != "noonRobot"):

    robotSelf = robot.getFromDef("TRAINER").getFromProtoDef(f"NOONROBOT_{robot.getName()}")



# PARAMETERS

maxSteps = 50000 # MAX STEPS AN INSTANCE CAN LIVE

# REWARD PARAMETERS
uprightRewardWeight = 1.2

movementPenaltyWeight = 0.02

turnRateRewardWeight = 4
walkSpeedRewardWeight = 4

verticalMovementPenaltyWeight = 0.2
sideMovementPenaltyWeight = 0.2



BodyParts: List[BodyPartData] = InitBodyParts(robotSupervisor=robot, timestep=timestep)


motors: List[MotorData] = InitMotors(timestep=timestep)

basePrevActions: np.ndarray = np.zeros((18,), dtype=np.float32)

i = 0

while(i < motors.__len__()):
    basePrevActions[i] = motors[i].defaultPos
    i+=1




gyro = Gyro(name="BODY_GYRO", sampling_period=timestep)
gyro.enable(timestep)
accelerometer = Accelerometer(name="BODY_ACCELEROMETER",sampling_period=timestep)
accelerometer.enable(timestep)
inertialUnit = InertialUnit(name="BODY_INERTIALUNIT", sampling_period=timestep)
inertialUnit.enable(timestep)

turnRate = 0
walkSpeed = 0
    
lastObs: list[float] = []
lastLastObs : list[float]= []
            
def getObservationSpace() -> np.ndarray:
    global lastObs, lastLastObs, turnRate, walkSpeed
    ret : list[float]= []
    for motor in motors:
        if motor.currentPos and motor.positionSensor:
            ret.append(motor.positionSensor.getValue())
        else:
            ret.append(motor.motor.getTargetPosition())
    
    
    
    ret.extend(gyro.getValues())
    ret.extend(accelerometer.getValues())
    ret.extend(inertialUnit.getRollPitchYaw())
    
    tObs = ret.copy()
    
    ret.extend(lastObs)
    ret.extend(lastLastObs)
    
    lastLastObs = lastObs.copy()
    lastObs= tObs.copy()
    
    ret.extend([turnRate, walkSpeed])
    return np.asarray(ret, dtype=np.float32)


stepsSinceReset = 0



prevActions: np.ndarray = basePrevActions.copy()

def step(action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
    global prevActions, stepsSinceReset, turnRate, walkSpeed
    
    reward = 0
    terminated = False
    truncated = False
    
    robot.step(timestep)

    
    # applies the actions to the motors
    
    i = 0
    for curAction in action:
        curAction = np.clip(curAction, 0.0, 1.0)
        pos = ((curAction) * ((motors[i].maxPos) - (motors[i].minPos))) + motors[i].minPos
        
        motors[i].motor.setPosition(pos)
        
        if(motors[i].currentPos):
            motors[i].motor.setVelocity(brushlessSpeed)
            motors[i].motor.setAcceleration(10)
            motors[i].motor.setAvailableTorque(brushlessTorque/1000)
        else:
            motors[i].motor.setVelocity(servoSpeed)
            motors[i].motor.setAcceleration(10)
            motors[i].motor.setAvailableTorque(servoTorque/1000)

        reward -= movementPenaltyWeight*abs(curAction - prevActions[i])
        
        i+=1
        
    prevActions = action.copy()
    
    # checks if any parts of the body that shouldnt be is touching the floor
    for bodyPart in BodyParts:
        touch = bodyPart.touchSensor.getValue()
        if touch != 0 and bodyPart.doneOnTouch:
            terminated = True
            reward = -10
            break
        
        if touch !=0 and bodyPart.touchReward:
            bodyPart.lastTouched = stepsSinceReset
            reward += bodyPart.touchReward
            
        if touch == 0 and bodyPart.noTouchReward and stepsSinceReset - bodyPart.lastTouched > bodyPart.noTouchRewardDelay:
            reward += bodyPart.noTouchReward
            
            
        
    # truncates the robot if it reaches a specified limit of steps
    if(stepsSinceReset >= maxSteps):
        truncated=True
        
            
    # calcualate reward based on how upright it is *

    bodyRot =  robotSelf.getOrientation()
    reward += max(-1, bodyRot[8] * uprightRewardWeight) # beatufiul line of code
    
    #print("rotation reward: ", max(-1, 1 - math.acos(bodyRot[8]) * uprightRewardWeight))
    
    
    # calculate reward based on turnspeed 
    vel = robotSelf.getVelocity()
    
    bodyAngVelocity = vel[3:]
    reward += max(-1, 1 - abs(turnRate - bodyAngVelocity[2]) * turnRateRewardWeight)
    #print("rotation: ", bodyAngVelocity[2], "reward:", max(-1, 1 - abs(turnRate - bodyAngVelocity[2]) * turnRateRewardWeight))
    
    
    # calculate reward based on walkspeed
    bodyLinVelocityVector = vel[:3]
    bodyVelocityMagnitude = bodyLinVelocityVector[0]*bodyRot[1] + bodyLinVelocityVector[1]*bodyRot[4] + bodyLinVelocityVector[2]*bodyRot[7]
    reward += max(-1, 1 - abs(walkSpeed - bodyVelocityMagnitude) * walkSpeedRewardWeight)
    
    # small penalty for vertical motion
    reward -= verticalMovementPenaltyWeight * abs(bodyLinVelocityVector[2])
    
    # small penatly for side motion
    bodySideVelocityMagnitude = bodyLinVelocityVector[0]*bodyRot[0] + bodyLinVelocityVector[1]*bodyRot[3] + bodyLinVelocityVector[2]*bodyRot[6]
    reward -= sideMovementPenaltyWeight * abs(bodySideVelocityMagnitude)
    
    #print("velocity: ", bodyVelocityMagnitude, "reward:", max(-1, 1 - abs(walkSpeed - bodyVelocityMagnitude) * turnRateRewardWeight))
    

    
    observation = getObservationSpace()
    
    stepsSinceReset+=1
    

    info = {}
    return observation, reward, terminated, truncated, info

robotSelf.saveState(robotSelf.getDef())

def reset(seed=None, options=None)-> tuple[np.ndarray, dict]:
    global stepsSinceReset, prevActions
    
    global motors, BodyParts, turnRate, walkSpeed

    prevActions = basePrevActions.copy()
    
    robotSelf.loadState(robotSelf.getDef())    
    
    stepsSinceReset = 0
    
    obs = getObservationSpace()
    
    # increment size
    step_size = 0.01  

    # ----- walk speed range: -0.05 to +0.20 -----
    walk_min = -0.05
    walk_max =  0.20
    walk_steps = int((walk_max - walk_min) / step_size) + 1
    walkSpeed = np.random.choice([walk_min + i * step_size for i in range(walk_steps)])

    # ----- turn rate range: -0.20 to +0.20 -----
    turn_min = -0.20
    turn_max =  0.20
    turn_steps = int((turn_max - turn_min) / step_size) + 1
    turnRate = np.random.choice([turn_min + i * step_size for i in range(turn_steps)])

    info = {}
    return obs, info


# INFERENCE
if(robotSelf.getField("inference").getSFBool()): 
    from stable_baselines3 import PPO
    import gymnasium as gym
    env = gym.Env()

    env.action_space = gym.spaces.Box(low=0, high=1,shape=(18,), dtype=np.float32)

    env.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(101,), dtype=np.float32)
    env.reset = reset
    env.step = step
    

    
    model : PPO
    try:
        model = PPO.load("../../models/continue", env=env, device="cpu")
        print("Sucessfully imported PPO for inference")


        model.policy.eval()
        obs, info = env.reset()
        while True:
            # model.predict already runs under torch.no_grad internally
            # ensure observation is a numpy array (stable-baselines3 expects ndarray)
            obs_array = np.asarray(obs, dtype=np.float32)
            action, _ = model.predict(obs_array, deterministic=True)   
                        
            obs, reward, terminated, truncated, info = env.step(action)
            if (terminated or truncated):
                obs, info = env.reset()
    except:
        
        raise Exception("cant run inference, there is no model, put one into the models folder as continue.zip")

rank = int(worldInfoInfoField.getMFString(0)) * int(worldInfoInfoField.getMFString(1)) + int(robotSelf.getField("name").getSFString())

SOCK_PATH = f"/tmp/noon_robot_{rank}.sock"


thisSocket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
#thisSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

#thisSocket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
#thisSocket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 256*1024)
#thisSocket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 256*1024)

while True:
    try:
        thisSocket.connect(SOCK_PATH)
        #thisSocket.connect((HOST, BASEPORT + rank))
        break
    except:
        pass
    

def recv_exact(sock, n):
    return sock.recv(n)


#import os

# pins the process to 1 core
#os.sched_setaffinity(0, {1+math.floor(rank/3)})


DEBUG = worldInfoInfoField.getMFString(2).lower() == "true"
IS_DEBUG_MASTER = DEBUG and rank == 0


import time
from collections import defaultdict





obs_buffer = np.empty(101, dtype=np.float32)
action_buffer = np.empty(18, dtype=np.float32)

obs_buffer, info = reset()

if(IS_DEBUG_MASTER):
    
    timing_acc = defaultdict(float)
    timing_count = 0
    
    PRINT_EVERY = 200  # steps
    while True:
        t0 = time.perf_counter()

        data = recv_exact(thisSocket, 1)


        t_recv_cmd = time.perf_counter()

        if data == b'r':
            t_reset_start = time.perf_counter()

            obs, info = reset()


            t_reset_done = time.perf_counter()

            thisSocket.sendall(obs.tobytes())


            t_send = time.perf_counter()

            timing_acc["recv_cmd"] += t_recv_cmd - t0
            timing_acc["reset"] += t_reset_done - t_reset_start
            timing_acc["send"] += t_send - t_reset_done
            timing_count += 1

        elif data == b's':

            t_recv_action_start = time.perf_counter()

            action_buffer = np.frombuffer(
                recv_exact(thisSocket, 18 * 4), dtype=np.float32
            )


            t_recv_action_done = time.perf_counter()
            t_step_start = time.perf_counter()

            obs_buffer, reward, terminated, truncated, info = step(action_buffer)


            t_step_done = time.perf_counter()
            t_pack_start = time.perf_counter()

            packet = (
                memoryview(obs_buffer).tobytes()
                + np.float32(reward).tobytes()
                + np.int8(terminated).tobytes()
                + np.int8(truncated).tobytes()
            )


            t_pack_done = time.perf_counter()
            t_send_start = time.perf_counter()

            thisSocket.sendall(packet)

            
            t_send_done = time.perf_counter()
            timing_acc["recv_cmd"] += t_recv_cmd - t0
            timing_acc["recv_action"] += t_recv_action_done - t_recv_action_start
            timing_acc["step"] += t_step_done - t_step_start
            timing_acc["pack"] += t_pack_done - t_pack_start
            timing_acc["send"] += t_send_done - t_send_start
            timing_count += 1

        if  timing_count >= PRINT_EVERY:
            print("\n=== DEBUG TIMINGS (avg over", timing_count, "steps) ===")
            for k, v in timing_acc.items():
                print(f"{k:12s}: {(v / timing_count)*1000:.3f} ms")
            print("========================================\n")

            timing_acc.clear()
            timing_count = 0
else:
    while True:

        data = recv_exact(thisSocket, 1)

        if data == b'r':

            obs, info = reset()

            thisSocket.sendall(obs.tobytes())


    

        elif data == b's':

            action_buffer = np.frombuffer(
                recv_exact(thisSocket, 18 * 4), dtype=np.float32
            )




            obs_buffer, reward, terminated, truncated, info = step(action_buffer)



            packet = (
                memoryview(obs_buffer).tobytes()
                + np.float32(reward).tobytes()
                + np.int8(terminated).tobytes()
                + np.int8(truncated).tobytes()
            )

            thisSocket.sendall(packet)



