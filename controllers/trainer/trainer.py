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
from trainParams import maxTime, movementPenaltyWeight, sideMovementPenaltyWeight, turnRateRewardWeight, uprightRewardWeight, verticalMovementPenaltyWeight, walkSpeedRewardWeight, targetPenaltyWeight
from patternGenerator import patternGenerator, pattern

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
    
lastObs: list[List[float]] = [[0 for _ in range(66)], [0 for _ in range(66)]]

thisPatternGenerator = patternGenerator()
            
timeSinceReset = 0            

def getObservationSpace() -> np.ndarray:
    global lastObs, turnRate, walkSpeed, thisPatternGenerator, timeSinceReset, motors
    ret : list[float]= []
    for motor in motors:
        if motor.currentPos and motor.positionSensor:
            ret.append(motor.positionSensor.getValue())
        else:
            ret.append(motor.motor.getTargetPosition())
    
    
    
    ret.extend(gyro.getValues())
    ret.extend(accelerometer.getValues())
    rot = inertialUnit.getRollPitchYaw()
    ret.extend(rot)
    
    pat : pattern= thisPatternGenerator.evaluatePattern(motors=motors, rot=rot, timestep=timeSinceReset)
    
    ret.extend(pat.values)
    ret.extend(pat.mask)
    
    ret.extend([turnRate, walkSpeed, pat.walkMask])
    
    lastObs.append(ret.copy())
    
    ret.extend(lastObs[0])
    ret.extend(lastObs[1])
    
    lastObs.pop(0)
    
    
    return np.asarray(ret, dtype=np.float32)




DEBUG = False
IS_DEBUG_MASTER = False


INFERENCE =robotSelf.getField("inference").getSFBool()



if(not INFERENCE):
    rank = int(worldInfoInfoField.getMFString(0)) * int(worldInfoInfoField.getMFString(1)) + int(robotSelf.getField("name").getSFString())
    DEBUG = worldInfoInfoField.getMFString(2).lower() == "true"
    IS_DEBUG_MASTER = DEBUG and rank == 0

prevActions: np.ndarray = basePrevActions.copy()

def step(action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
    global prevActions, timeSinceReset, turnRate, walkSpeed, motors
    
    reward = 0
    terminated = False
    truncated = False
    
    robot.step(timestep)

    pat = thisPatternGenerator.evaluatePattern(
        motors=motors,
        rot=inertialUnit.getRollPitchYaw(),
        timestep=timeSinceReset
    )
    if(IS_DEBUG_MASTER or INFERENCE):
        print("\n--- STEP DEBUG ---")
        print(f"Step: {timeSinceReset}")

    # applies the actions to the motors
    for i, curAction in enumerate(action):
        motors[i].setMotor(curAction)

        # target pattern penalty
        if pat.mask[i]:
            penalty = targetPenaltyWeight * abs(curAction - pat.values[i])
            reward -= penalty
            if(IS_DEBUG_MASTER or INFERENCE):
                print(f"[Motor {i}] Target penalty: -{penalty:.4f}")

        # movement smoothness penalty
        penalty = movementPenaltyWeight * abs(curAction - prevActions[i])
        reward -= penalty
        if(IS_DEBUG_MASTER or INFERENCE):
            print(f"[Motor {i}] Movement penalty: -{penalty:.4f}")

    prevActions = action.copy()
    
    # body part contacts
    for bodyPart in BodyParts:
        touch = bodyPart.touchSensor.getValue()

        if touch != 0 and bodyPart.doneOnTouch:
            terminated = True
            reward = -10
            if(IS_DEBUG_MASTER or INFERENCE):
                print(f"[{bodyPart.name}] TERMINATION touch! Reward set to -10")
            break
        
        if touch != 0 and bodyPart.touchReward:
            bodyPart.lastTouched = timeSinceReset
            reward += bodyPart.touchReward
            if(IS_DEBUG_MASTER or INFERENCE):
                print(f"[{bodyPart.name}] Touch reward: +{bodyPart.touchReward}")
            
        if touch == 0 and bodyPart.noTouchReward and bodyPart.noTouchRewardDelay:
            if timeSinceReset - bodyPart.lastTouched > bodyPart.noTouchRewardDelay:
                reward += bodyPart.noTouchReward
                if(IS_DEBUG_MASTER or INFERENCE):
                    print(f"[{bodyPart.name}] No-touch reward: +{bodyPart.noTouchReward}")

    # truncation
    if timeSinceReset >= maxTime:
        truncated = True
        if(IS_DEBUG_MASTER or INFERENCE):
            print("[Truncation] Max steps reached")

    # upright reward
    bodyRot = robotSelf.getOrientation()
    upright_reward = max(-1, bodyRot[8] * uprightRewardWeight)
    reward += upright_reward
    if(IS_DEBUG_MASTER or INFERENCE):
        print(f"[Upright] Reward: {upright_reward:.4f}")
    
    vel = robotSelf.getVelocity()
    bodyLinVelocityVector = vel[:3]
    
    if(pat.walkMask):
        # angular velocity reward
        
        bodyAngVelocity = vel[3:]
        turn_reward = max(-1, 1 - abs(turnRate - bodyAngVelocity[2]) * turnRateRewardWeight)
        reward += turn_reward
        if(IS_DEBUG_MASTER or INFERENCE):
            print(f"[Turn Rate] Reward: {turn_reward:.4f} (actual: {bodyAngVelocity[2]:.4f})")

        # forward velocity reward
        # COMMENTED OUT FOR NOW
        # 
        # bodyVelocityMagnitude = (
        #     bodyLinVelocityVector[0]*bodyRot[1] +
        #     bodyLinVelocityVector[1]*bodyRot[4] +
        #     bodyLinVelocityVector[2]*bodyRot[7]
        # )
        # walk_reward = max(-1, 1 - abs(walkSpeed - bodyVelocityMagnitude) * walkSpeedRewardWeight)
        # reward += walk_reward
        # if(IS_DEBUG_MASTER or INFERENCE):
        #   print(f"[Walk Speed] Reward: {walk_reward:.4f} (actual: {bodyVelocityMagnitude:.4f})")

    # vertical movement penalty
    vertical_penalty = verticalMovementPenaltyWeight * abs(bodyLinVelocityVector[2])
    reward -= vertical_penalty
    if(IS_DEBUG_MASTER or INFERENCE):
        print(f"[Vertical Movement] Penalty: -{vertical_penalty:.4f}")

    # side movement penalty
    bodySideVelocityMagnitude = (
        bodyLinVelocityVector[0]*bodyRot[0] +
        bodyLinVelocityVector[1]*bodyRot[3] +
        bodyLinVelocityVector[2]*bodyRot[6]
    )
    side_penalty = sideMovementPenaltyWeight * abs(bodySideVelocityMagnitude)
    reward -= side_penalty
    if(IS_DEBUG_MASTER or INFERENCE):
        print(f"[Side Movement] Penalty: -{side_penalty:.4f}")

        print(f"TOTAL REWARD: {reward:.4f}")
        print("--- END STEP ---\n")

    observation = getObservationSpace()
    timeSinceReset += timestep

    info = {}
    return observation, reward, terminated, truncated, info

robotSelf.saveState(robotSelf.getDef())

def reset(seed=None, options=None)-> tuple[np.ndarray, dict]:
    global timeSinceReset, prevActions
    
    global motors, BodyParts, turnRate, walkSpeed

    prevActions = basePrevActions.copy()
    
    robotSelf.loadState(robotSelf.getDef())    
    
    timeSinceReset = 0
    
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
if(INFERENCE): 
    from stable_baselines3 import PPO
    import gymnasium as gym
    env = gym.Env()

    env.action_space = gym.spaces.Box(low=0, high=1,shape=(18,), dtype=np.float32)

    env.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(198,), dtype=np.float32)
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
    



#import os

# pins the process to 1 core
#os.sched_setaffinity(0, {1+math.floor(rank/3)})





import time
from collections import defaultdict





obs_buffer = np.empty(198, dtype=np.float32)
action_buffer = np.empty(18, dtype=np.float32)

obs_buffer, info = reset()

if(IS_DEBUG_MASTER):
    
    timing_acc = defaultdict(float)
    timing_count = 0
    
    PRINT_EVERY = 400  # steps
    while True:
        t0 = time.perf_counter()

        data = thisSocket.recv(1)


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
                thisSocket.recv(18 * 4), dtype=np.float32
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

        data = thisSocket.recv( 1)

        if data == b'r':

            obs, info = reset()

            thisSocket.sendall(obs.tobytes())


    

        elif data == b's':

            action_buffer = np.frombuffer(
                thisSocket.recv( 18 * 4), dtype=np.float32
            )




            obs_buffer, reward, terminated, truncated, info = step(action_buffer)



            packet = (
                memoryview(obs_buffer).tobytes()
                + np.float32(reward).tobytes()
                + np.int8(terminated).tobytes()
                + np.int8(truncated).tobytes()
            )

            thisSocket.sendall(packet)



