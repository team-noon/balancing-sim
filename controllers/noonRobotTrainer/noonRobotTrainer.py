"""noonRobotTrainer controller."""

import math
import datetime
from controller import Supervisor, InertialUnit, Gyro, Accelerometer, Node
from typing import List, Tuple, Dict, Any
import numpy as np
from classes import BodyPartData, MotorData
from initScripts import InitBodyParts, InitMotors
import socket


# PARAMETERS

maxSteps = 20000 # MAX STEPS AN INSTANCE CAN LIVE

uprightRewardWeight = 1
maxUprightReward = 1

movementPenaltyWeight = 0.03

turnRateRewardWeight = 1
maxTurnRateReward = 1

walkSpeedRewardWeight = 1
maxWalkSpeedReward = 1

servoTorque = 10 * 2.2 * 9.81 # in mNm
servoSpeed = 39/50 * math.pi # in rad/sec

brushlessTorque = 13750 / 3 # in mNm
brushlessSpeed = 2 * math.pi # in rad/sec

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

BodyParts: List[BodyPartData] = InitBodyParts(robotSupervisor=robot, timestep=timestep)


motors: List[MotorData] = InitMotors(timestep=timestep)

worldInfoTitleField = robot.getFromDef("WorldInfo").getField("title")
worldInfoTitleField.setSFString(datetime.datetime.now().__str__())


gyro = Gyro(name="BODY_GYRO", sampling_period=timestep)
gyro.enable(timestep)
accelerometer = Accelerometer(name="BODY_ACCELEROMETER",sampling_period=timestep)
accelerometer.enable(timestep)
inertialUnit = InertialUnit(name="BODY_INERTIALUNIT", sampling_period=timestep)
inertialUnit.enable(timestep)



turnRate = 0
walkSpeed = 0
            
def getObservationSpace() -> list[float]:
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
    return ret



stepsSinceReset = 0


prevActions: np.ndarray = np.zeros((18,), dtype=np.float32)

def step(action: np.ndarray) -> Tuple[list[float], float, bool, bool, Dict[str, Any]]:
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

robot.getSelf().saveState(robot.getSelf().getDef())

def reset(seed=None, options=None)-> tuple[list[float], dict]:
    global stepsSinceReset
    #for bodyPart in BodyParts:
    #    bodyPart.angularVelocityField.setSFVec3f([0,0,0])
    #    bodyPart.linearVelocityField.setSFVec3f([0,0,0])
    #    bodyPart.transField.setSFVec3f(bodyPart.startingPosition)
    #    bodyPart.rotField.setSFRotation(bodyPart.startingRotation)
    #    
    #    bodyPart.node.setVelocity([0,0,0])
    #    bodyPart.node.resetPhysics()
    #    
    #    robot.getSelf().saveState()
    #    
    #
    #    
    #for motor in motors:
    #    motor.motor.setPosition(0)
    
    global motors, BodyParts
    
    robot.getSelf().loadState(robot.getSelf().getDef())    
    
    stepsSinceReset = 0
    
    obs = getObservationSpace()
        
    info = {}
    return obs, info




# INFERENCE
if(robot.getSelf().getField("inference").getSFBool()): 
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
    
serverSocket = socket.socket()


obs, info = reset()
# TRAINING LOOP
while True:
    
    pass




