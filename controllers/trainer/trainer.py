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
from trainParams import maxTime, movementPenaltyWeight, sideMovementPenaltyWeight, turnRateRewardWeight, uprightRewardWeight, verticalMovementPenaltyWeight, jerkPenalty, walkSpeedRewardWeight, terminationPenalty, targetRewardFalloff, maxTargetReward, targetThreshold, armTargetThreshhold, maxStillnessReward, stillnessRewardFalloff, upsideDownPenalty
from patternGenerator import patternGenerator, pattern, patternTypes
from util import threshold, isArmNum

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

    
lastObs: list[List[float]] =  [[0 for a in range(77)] for b in range(2)]

thisPatternGenerator = patternGenerator(motors, timestep, True)
            
timeSinceReset = 0            

def getObservationSpace() -> np.ndarray:
    global lastObs, thisPatternGenerator, timeSinceReset, motors
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
    
    pat : pattern= thisPatternGenerator.evaluatePattern(rot=rot)
    
    ret.extend(pat.values)
    ret.extend(pat.mask)
    
    ret.extend([pat.turnRate,pat.standMode, pat.walkMode, pat.kickMode, pat.animateMode, pat.uprightReward, pat.turnRateReward, pat.walkSpeedReward, pat.verticalPenalty, pat.sidePenalty, pat.stillnessReward, pat.canTouchGround, pat.touchReward, pat.noTouchReward])
    
    lastObs.append(ret.copy())
    
    ret.extend(lastObs[1])
    ret.extend(lastObs[0])
    
    lastObs.pop(0)
    
    
    return np.asarray(ret, dtype=np.float32)




DEBUG = False
IS_DEBUG_MASTER = False


INFERENCE =robotSelf.getField("inference").getSFBool()

TOTAL_REWARD = 0

if(not INFERENCE):
    rank = int(worldInfoInfoField.getMFString(0)) * int(worldInfoInfoField.getMFString(1)) + int(robotSelf.getField("name").getSFString())
    DEBUG = worldInfoInfoField.getMFString(2).lower() == "true"
    IS_DEBUG_MASTER = DEBUG and rank == 0

prevActions: np.ndarray = basePrevActions.copy()

prevBodyVelocities : np.ndarray = np.zeros(10, dtype=np.float32)
prevAngularVelocities : np.ndarray= np.zeros(10, dtype=np.float32)

def step(action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
    global prevActions, timeSinceReset, motors, TOTAL_REWARD, thisPatternGenerator, prevAngularVelocities, prevBodyVelocities
    
    reward = 0
    terminated = False
    truncated = False
    
    robot.step(timestep)

    pat = thisPatternGenerator.evaluatePattern(
        rot=inertialUnit.getRollPitchYaw()
    )
        
    
    if(IS_DEBUG_MASTER or INFERENCE):
        print("\n--- STEP DEBUG ---")
        print(f"Step: {timeSinceReset}")

    # applies the actions to the motors
    for i, curAction in enumerate(action):
        motors[i].setMotor(curAction)

        # target pattern reward
        if pat.mask[i] == 1:
            
            targetReward = maxTargetReward-targetRewardFalloff *abs(curAction - pat.values[i])  + (armTargetThreshhold if isArmNum(i) else targetThreshold) * targetRewardFalloff
            
            if(threshold(curAction, pat.values[i], armTargetThreshhold if isArmNum(i) else targetThreshold)):
                targetReward = maxTargetReward
                
            
            
            reward += targetReward
            if(IS_DEBUG_MASTER or INFERENCE):
                print(f"[Motor {i}] Target reward: {targetReward:.4f}")

        # dont penalize for moving when its told to move
        if pat.mask[i] != 1:
            # movement smoothness penalty
            penalty = movementPenaltyWeight * abs(curAction - prevActions[i])
            reward -= penalty
            if(IS_DEBUG_MASTER or INFERENCE):
                print(f"[Motor {i}] Movement penalty: -{penalty:.4f}")

    prevActions = action.copy()
    


    # truncation
    if timeSinceReset >= maxTime:
        truncated = True
        if(IS_DEBUG_MASTER or INFERENCE):
            print("[Truncation] Max steps reached")

    # upright reward
    bodyRot = robotSelf.getOrientation()
    uprightReward =  bodyRot[8] * uprightRewardWeight
    if(bodyRot[8] < 0):
        uprightReward += upsideDownPenalty
    uprightReward *= pat.uprightReward
    reward += uprightReward
    if(IS_DEBUG_MASTER or INFERENCE):
        print(f"[Upright] Reward: {uprightReward:.4f}")
    
    vel = robotSelf.getVelocity()
    bodyLinVelocityVector = vel[:3]
    
    
    # angular velocity reward
    
    bodyAngVelocity = vel[3:]
    
    prevAngularVelocities = np.roll(prevAngularVelocities, -1)
    prevAngularVelocities[-1] = bodyAngVelocity[2]
    
    turnRate = prevAngularVelocities.mean() * 0.75 + bodyAngVelocity[2] * 0.25
    turnReward = max(-1, 1 - abs(pat.turnRate- turnRate) * turnRateRewardWeight)
    turnReward *= pat.turnRateReward
    reward += turnReward 
    if(IS_DEBUG_MASTER or INFERENCE):
        print(f"[Turn Rate] Reward: {turnReward:.4f} (actual: {bodyAngVelocity[2]:.4f})")
        
    # angular velocity jerk penalty
    angVelocityJerk = prevAngularVelocities[-1] - 2 * prevAngularVelocities[-2] + prevAngularVelocities[-3]
    
    reward -= (angVelocityJerk**2) * jerkPenalty
    
    # forward velocity reward
    
    bodyVelocityMagnitude = (
        bodyLinVelocityVector[0]*bodyRot[1] +
        bodyLinVelocityVector[1]*bodyRot[4] +
        bodyLinVelocityVector[2]*bodyRot[7]
    )
    
    prevBodyVelocities = np.roll(prevBodyVelocities, -1)
    prevBodyVelocities[-1] = bodyVelocityMagnitude
    
    avgBodyVelocity = prevBodyVelocities.mean() * 0.75 + bodyVelocityMagnitude * 0.25
    
    #walk speed maximalization reward
    walkReward = avgBodyVelocity * walkSpeedRewardWeight
    walkReward *=pat.walkSpeedReward
    reward += walkReward 
    if(IS_DEBUG_MASTER or INFERENCE):
      print(f"[Walk Speed] Reward: {walkReward:.4f} (actual: {bodyVelocityMagnitude:.4f})")
    
    #walk speed minimalization reward
    stillnessReward = maxStillnessReward - stillnessRewardFalloff * abs(avgBodyVelocity)
    stillnessReward *= pat.stillnessReward
    reward += stillnessReward
    
    linVelocityJerk = prevBodyVelocities[-1] - 2 * prevBodyVelocities[-2] + prevBodyVelocities[-3]
    reward -= (linVelocityJerk**2) * jerkPenalty

    # vertical movement penalty
    verticalPenalty = verticalMovementPenaltyWeight * abs(bodyLinVelocityVector[2])
    verticalPenalty *= pat.verticalPenalty
    reward -= verticalPenalty 
    if(IS_DEBUG_MASTER or INFERENCE):
        print(f"[Vertical Movement] Penalty: -{verticalPenalty:.4f}")

    # side movement penalty
    bodySideVelocityMagnitude = (
        bodyLinVelocityVector[0]*bodyRot[0] +
        bodyLinVelocityVector[1]*bodyRot[3] +
        bodyLinVelocityVector[2]*bodyRot[6]
    )
    sidePenalty = sideMovementPenaltyWeight * abs(bodySideVelocityMagnitude)
    sidePenalty *= pat.sidePenalty
    reward -= sidePenalty
    if(IS_DEBUG_MASTER or INFERENCE):
        print(f"[Side Movement] Penalty: -{sidePenalty:.4f}")

        
        
    # body part contacts
    for bodyPart in BodyParts:
        touch = bodyPart.touchSensor.getValue()

        if touch != 0 and bodyPart.doneOnTouch and pat.canTouchGround == False:
            terminated = True
            reward = terminationPenalty
            if(IS_DEBUG_MASTER or INFERENCE):
                print(f"[{bodyPart.name}] TERMINATION touch! Reward set to {terminationPenalty}")
            break
        
        if touch != 0 and bodyPart.touchReward and pat.touchReward:
            bodyPart.lastTouched = timeSinceReset
            reward += bodyPart.touchReward
            if(IS_DEBUG_MASTER or INFERENCE):
                print(f"[{bodyPart.name}] Touch reward: +{bodyPart.touchReward}")
            
        if touch == 0 and bodyPart.noTouchReward and bodyPart.noTouchRewardDelay and pat.noTouchReward:
            if timeSinceReset - bodyPart.lastTouched > bodyPart.noTouchRewardDelay:
                reward += bodyPart.noTouchReward
                if(IS_DEBUG_MASTER or INFERENCE):
                    print(f"[{bodyPart.name}] No-touch reward: +{bodyPart.noTouchReward}")
                    
                    
    if(IS_DEBUG_MASTER or INFERENCE):
        TOTAL_REWARD += reward
        print(f"TOTAL REWARD: {reward:.4f}")
        print("--- END STEP ---\n")

    observation = getObservationSpace()
    timeSinceReset += timestep

    info = {}
    return observation, reward, terminated, truncated, info

robotSelf.saveState(robotSelf.getDef())

def reset(seed=None, options=None)-> tuple[np.ndarray, dict]:
    global timeSinceReset, prevActions, TOTAL_REWARD, prevAngularVelocities, prevBodyVelocities
    
    global motors, BodyParts, thisPatternGenerator

    prevActions = basePrevActions.copy()
    
    prevAngularVelocities =np.zeros(10, dtype=np.float32)
    prevBodyVelocities =np.zeros(10, dtype=np.float32)
    
    
    robotSelf.loadState(robotSelf.getDef())    
    
    timeSinceReset = 0
    
    obs = getObservationSpace()
    
    toDo = np.random.choice([patternTypes.walk, patternTypes.stand])
    
    if(toDo == patternTypes.walk):
        thisPatternGenerator.setWalkMode()
        turnRate = 0 if np.random.choice(["straight","turn","turn","turn"]) == "straight" else np.random.choice([-2 + i * 0.1 for i in range(41)])
        thisPatternGenerator.patternWalkParameters.turnRate = turnRate
    elif(toDo == patternTypes.stand):
        thisPatternGenerator.setStandMode()
        turnRate = 0 if np.random.choice(["straight","turn"]) == "straight" else np.random.choice([-2 + i * 0.1 for i in range(41)])
        thisPatternGenerator.patternStandParameters.turnRate = turnRate


    if(IS_DEBUG_MASTER or INFERENCE):
        print(f"\033[92m🔥 Episode Total Reward: {TOTAL_REWARD:.4f} 🔥\033[0m")
        TOTAL_REWARD = 0
        
    
    info = {}
    return obs, info


# INFERENCE
if(INFERENCE): 
    from stable_baselines3 import PPO
    import gymnasium as gym
    env = gym.Env()

    env.action_space = gym.spaces.Box(low=0, high=1,shape=(18,), dtype=np.float32)

    env.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(231,), dtype=np.float32)
    env.reset = reset
    env.step = step
    

    
    model : PPO
    try:
        model = PPO.load("../../models/continue", env=env, device="cpu")
        print("Sucessfully imported PPO for inference")


        model.policy.eval()
        model.policy.set_training_mode(False)
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


# WARM STARTUP
if(rank == 0):
    done_warmup= False
    obs = np.empty(231, dtype=np.float32)
    action = np.empty(18, dtype=np.float32)
    truncated = False
    terminated = False
    reward = 0.0
    
    def sendObs():
        thisSocket.sendall(memoryview(obs).tobytes())
    
    def sendStep():
        packet=(memoryview(obs).tobytes() + np.float32(reward).tobytes() + np.int8(terminated).tobytes() + np.int8(truncated).tobytes() + np.int8(done_warmup).tobytes()+ memoryview(action).tobytes())
        
        thisSocket.sendall(packet)
    
    thisPatternGenerator.setWalkMode()
    
    obs = reset()
    sendObs()
    
    # walk
    while not (truncated or terminated):
        pat = thisPatternGenerator.evaluatePattern(
            rot=inertialUnit.getRollPitchYaw(),
            updateTimeStep=False
        )
        
        action = np.array(pat.values, dtype=np.float32)
        
        obs, reward, terminated, truncated, infos = step(action=action)
        
        sendStep()
    
    # stand
    
    obs = reset()
    terminated = False
    truncated = False
    sendObs()
    
    thisPatternGenerator.setStandMode()
    thisPatternGenerator.patternStandParameters.turnRate = 0
    
    
    while not (truncated or terminated):
        pat = thisPatternGenerator.evaluatePattern(
            rot=inertialUnit.getRollPitchYaw(),
            updateTimeStep=False
        )
        
        action = np.array(pat.values, dtype=np.float32)
        
        obs, reward, terminated, truncated, infos = step(action=action)
        
        
        
        sendStep()
        
    # stand + turn right
    
    obs = reset()
    terminated = False
    truncated = False
    sendObs()
    
    thisPatternGenerator.setStandMode()
    thisPatternGenerator.patternStandParameters.turnRate = 2
    
    
    
    while not (truncated or terminated):
        pat = thisPatternGenerator.evaluatePattern(
            rot=inertialUnit.getRollPitchYaw(),
            updateTimeStep=False
        )
        
        action = np.array(pat.values, dtype=np.float32)
        
        obs, reward, terminated, truncated, infos = step(action=action)
        
        sendStep()
    
    # stand + turn left slower
    
    obs = reset()
    terminated = False
    truncated = False
    sendObs()
    
    thisPatternGenerator.setStandMode()
    thisPatternGenerator.patternStandParameters.turnRate = -1
    
    
    
    while not (truncated or terminated):
        pat = thisPatternGenerator.evaluatePattern(
            rot=inertialUnit.getRollPitchYaw(),
            updateTimeStep=False
        )
        
        action = np.array(pat.values, dtype=np.float32)
        
        obs, reward, terminated, truncated, infos = step(action=action)
        
        if(terminated or truncated):
            done_warmup = True
        
        sendStep()


obs_buffer = np.empty(231, dtype=np.float32)
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



