"""noonRobotTrainer controller."""

import math
import datetime
from controller import Supervisor, InertialUnit, Gyro, Accelerometer, Node
from typing import List, Tuple, Dict, Any
import numpy as np
from classes import BodyPartData, MotorData
from initScripts import InitBodyParts, InitMotors
import socket
import sys
import time




HOST = "127.0.0.1"
BASEPORT = 9876

robot = Supervisor()
worldInfoInfoField = robot.getFromDef("WorldInfo").getField("info")
timestep = int(robot.getBasicTimeStep())

if sys.argv.__len__() == 3:
    env_num = int(sys.argv[1])
    NUM_ROBOTS = int(sys.argv[2])
    
    print("END MY SUFFERING PLEASE")
    
    print(robot.getName(), robot.getSelf().getTypeName())
    
    worldInfoInfoField.insertMFString(0, f"{env_num}")
    worldInfoInfoField.insertMFString(1, f"{NUM_ROBOTS}")
    
    robot.getFromDef("TRAINER").getField("count").setSFInt32(NUM_ROBOTS)
    print(robot.step(timestep))
    robot.getSelf().remove()


    exit()


robotSelf : Node = robot.getSelf()

print(robot.getSelf().getTypeName())

# PARAMETERS

maxSteps = 20000 # MAX STEPS AN INSTANCE CAN LIVE

uprightRewardWeight = 1
maxUprightReward = 1

movementPenaltyWeight = 0.03

turnRateRewardWeight = 1
maxTurnRateReward = 1

walkSpeedRewardWeight = 1
maxWalkSpeedReward = 1

servoTorque = 10 * 10 * 9.81 # in mNm
servoSpeed = 39/50 * math.pi # in rad/sec

brushlessTorque = 13750 / 3 # in mNm
brushlessSpeed = 2 * math.pi # in rad/sec


BodyParts: List[BodyPartData] = InitBodyParts(robotSupervisor=robot, timestep=timestep)


motors: List[MotorData] = InitMotors(timestep=timestep)


gyro = Gyro(name="BODY_GYRO", sampling_period=timestep)
gyro.enable(timestep)
accelerometer = Accelerometer(name="BODY_ACCELEROMETER",sampling_period=timestep)
accelerometer.enable(timestep)
inertialUnit = InertialUnit(name="BODY_INERTIALUNIT", sampling_period=timestep)
inertialUnit.enable(timestep)



turnRate = 0
walkSpeed = 0
            
def getObservationSpace() -> np.ndarray:
    ret : list[float]= []
    for motor in motors:
        if motor.currentPos and motor.positionSensor:
            ret.append(motor.positionSensor.getValue())
        else:
            ret.append(motor.motor.getTargetPosition())
    
    ret.extend(gyro.getValues())
    ret.extend(accelerometer.getValues())
    ret.extend(inertialUnit.getRollPitchYaw())
    
    ret.extend([turnRate, walkSpeed])
    return np.asarray(ret, dtype=np.float32)


stepsSinceReset = 0


prevActions: np.ndarray = np.zeros((18,), dtype=np.float32)

def step(action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
    global prevActions, stepsSinceReset
    
    reward = 1
    terminated = False
    truncated = False
    
    robot.step(timestep)


    
    
    # applies the actions to the motors
    
    i = 0
    for curAction in action:
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

        reward -= movementPenaltyWeight*((curAction - prevActions[i]) ** 2)
        
        i+=1
        
    prevActions = action
    
    # checks if any parts of the body that shouldnt be is touching the floor
    for bodyPart in BodyParts:
        touch = bodyPart.touchSensor.getValue()
        if touch != 0 and bodyPart.doneOnTouch:
            terminated = True
            reward = -10
            break
        
    # truncates the robot if it reaches a specified limit of steps
    if(stepsSinceReset >= maxSteps):
        truncated=True
            
    # calcualate reward based on how upright it is *
    bodyRot =  inertialUnit.getRollPitchYaw()
    reward += max(-2, maxUprightReward - (abs(bodyRot[0]) + abs(bodyRot[1])) * uprightRewardWeight)
    
    # calculate reward based on turnspeed 
    bodyAngVelocity = BodyParts[0].angularVelocityField.getSFVec3f()
    reward += max(-2, maxTurnRateReward - abs(turnRate - bodyAngVelocity[2]) * turnRateRewardWeight)
    
    # calculate reward based on walkspeed
    bodyLinVelocityVector = BodyParts[0].linearVelocityField.getSFVec3f()
    bodyVelocityMagnitude = math.sqrt( bodyLinVelocityVector[0] ** 2 + bodyLinVelocityVector[1] ** 2) # calc the velocity that we care about (we dont care about the z component)
    reward += max(-2, maxWalkSpeedReward - abs(walkSpeed - bodyVelocityMagnitude) * turnRateRewardWeight)
    

    
    observation = getObservationSpace()
    
    stepsSinceReset+=timestep
    

    info = {}
    return observation, reward, terminated, truncated, info

robotSelf.saveState(robotSelf.getDef())

def reset(seed=None, options=None)-> tuple[np.ndarray, dict]:
    global stepsSinceReset
    
    global motors, BodyParts
    
    robotSelf.loadState(robotSelf.getDef())    
    
    stepsSinceReset = 0
    
    obs = getObservationSpace()
        
    info = {}
    return obs, info


# INFERENCE
if(robotSelf.getField("inference").getSFBool()): 
    from stable_baselines3 import PPO
    import gymnasium as gym
    env = gym.Env()

    env.action_space = gym.spaces.Box(low=0, high=1,shape=(18,), dtype=np.float32)

    env.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(29,), dtype=np.float32)
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
            if terminated or truncated:
                obs, info = env.reset()
    except:
        
        raise Exception("cant run inference, there is no model, put one into the models folder as continue.zip")

rank = int(worldInfoInfoField.getMFString(0)) * int(worldInfoInfoField.getMFString(1)) + int(robotSelf.getField("name").getSFString())


thisSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

while True:
    try:
        thisSocket.connect((HOST, BASEPORT + rank))
        break
    except:
        pass
    



obs, info = reset()
# TRAINING LOOP
while True:
    data = thisSocket.recv(1)
    if data == b'r':
        obs, info = reset()
        packet = obs.tobytes()
        
        thisSocket.sendall(packet)
    elif data== b's':
        actionData = thisSocket.recv(18*4)
        actionToTake = np.frombuffer(actionData, dtype=np.float32)
        observation, reward, terminated, truncated, info= step(actionToTake)
        obs = observation.astype(np.float32)
        reward32 = np.float32(reward)
        terminated8 = np.int8(terminated)
        truncated8 = np.int8(truncated)

        # Build a single contiguous byte buffer
        packet = obs.tobytes() + reward32.tobytes() + terminated8.tobytes() + truncated8.tobytes()

        thisSocket.sendall(packet)
        
        
    pass




