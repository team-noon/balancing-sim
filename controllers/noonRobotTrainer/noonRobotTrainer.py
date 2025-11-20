"""noonRobotTrainer controller."""

import torch
from controller import Supervisor, InertialUnit, Gyro, Accelerometer, Node
from typing import List, Tuple, Dict, Any
import gymnasium as gym
from stable_baselines3 import PPO
import numpy as np
from classes import BodyPartData, MotorData
from initScripts import InitBodyParts, InitMotors

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

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


def step(action: np.ndarray) -> Tuple[list[float], float, bool, bool, Dict[str, Any]]:
    # perform one simulation step, apply action to motors, read sensors, compute reward/termination
    reward = 1
    terminated = False
    truncated = False

    i = 0
    
    for curAction in action:
        pos = ((curAction) * ((motors[i].maxPos) - (motors[i].minPos))) + motors[i].minPos
        #motors[i].motor.setForce(0.1)
        #motors[i].motor.setVelocity(1.0)
        motors[i].motor.setPosition(pos)
        motors[i].motor.setVelocity(1.0)

        
        i+=1
    
    for bodyPart in BodyParts:
        touch = bodyPart.touchSensor.getValue()
        if touch != 0 and bodyPart.doneOnTouch:
            terminated = True
            
    
    
    robot.step(timestep)
    
    observation = getObservationSpace()
    

    info = {}
    return observation, reward, terminated, truncated, info

env.step = step

def reset(seed=None, options=None):
    obs = env.observation_space.sample()
    info = {}
    return obs, info

env.reset = reset

policy_kwargs = dict(
    net_arch=dict(pi=[64, 64], vf=[128, 64]),
    activation_fn=torch.nn.LeakyReLU
)

model = PPO("MlpPolicy", env, verbose=1, policy_kwargs=policy_kwargs, device="cpu")
model.learn(10000)

while robot.step(timestep) != -1:
    print("stepping cause i dont know waht the fuck to do")
    pass
