"""noonRobotTrainer controller."""

# PARAMETERS

maxSteps = 10000 # MAX STEPS AN INSTANCE CAN LIVE




import datetime
import torch
from controller import Supervisor, InertialUnit, Gyro, Accelerometer, Node
from typing import List, Tuple, Dict, Any
import gymnasium as gym
from stable_baselines3 import PPO
import numpy as np
from classes import BodyPartData, MotorData
from initScripts import InitBodyParts, InitMotors
from util import exportONNX

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

env = gym.Env()

env.action_space = gym.spaces.Box(low=0, high=1,shape=(18,), dtype=np.float32)

env.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(29,), dtype=np.float32)

stepsSinceReset = 0


prevActions: np.ndarray = np.zeros(env.observation_space.shape, dtype=np.float32)

def step(action: np.ndarray) -> Tuple[list[float], float, bool, bool, Dict[str, Any]]:
    reward = 1
    terminated = False
    truncated = False
    


    
    
    # applies the actions to the motors
    
    i = 0
    for curAction in action:
        pos = ((curAction) * ((motors[i].maxPos) - (motors[i].minPos))) + motors[i].minPos
        motors[i].motor.setPosition(pos)
        motors[i].motor.setVelocity(3.0)
        motors[i].motor.setAcceleration(5)

        
        i+=1
    
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
            
    
    
    robot.step(timestep)
    
    observation = getObservationSpace()
    

    info = {}
    return observation, reward, terminated, truncated, info

env.step = step

robot.getSelf().saveState(robot.getSelf().getDef())

def reset(seed=None, options=None):
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
    
    BodyParts = InitBodyParts(robotSupervisor=robot, timestep=timestep)

    motors = InitMotors(timestep=timestep)
    
    obs = getObservationSpace()
    i = 0
    for t in obs:
        obs[i] = 0
        i+=1
        
    info = {}
    return obs, info

env.reset = reset

policy_kwargs = dict(
    net_arch=dict(pi=[64, 64], vf=[128, 64]),
    activation_fn=torch.nn.LeakyReLU
)

model = PPO("MlpPolicy", env, verbose=1, policy_kwargs=policy_kwargs, device="cpu")
model.learn(100)

exportONNX(model, robot.getSelf().getDef(), worldInfoTitleField)


while robot.step(timestep) != -1:
    #print("stepping cause i dont know waht the fuck to do")
    pass
