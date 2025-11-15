"""noonRobotTrainer controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import torch
from controller import  Supervisor, InertialUnit, Gyro, Accelerometer
from types import SimpleNamespace
import gymnasium as gym
import sys
from stable_baselines3 import PPO
import numpy as np


robot = Supervisor()

timestep = int(robot.getBasicTimeStep())


# USE currentPos False for every motor that we just assumes is instantenious (alias for the shitty servo motors we will use)

joints = SimpleNamespace(
    asymmetric=[
        # SimpleNamespace(name="HEAD", axis=[SimpleNamespace(axis="Z", currentPos=False)])
    ],
    symmetric=[
        SimpleNamespace(
            name="SHOULDER",
            axis=[
                SimpleNamespace(axis="Y", currentPos=False),
                SimpleNamespace(axis="Z", currentPos=False)
            ]
        ),
        SimpleNamespace(
            name="ELBOW",
            axis=[
                SimpleNamespace(axis="Y", currentPos=False)
            ]
        ),
        SimpleNamespace(
            name="HIP",
            axis=[
                SimpleNamespace(axis="X", currentPos=True),
                SimpleNamespace(axis="Y", currentPos=False),
                SimpleNamespace(axis="Z", currentPos=False)
            ]
        ),
        SimpleNamespace(
            name="KNEE",
            axis=[
                SimpleNamespace(axis="X", currentPos=True)
            ]
        ),
        SimpleNamespace(
            name="FOOT",
            axis=[
                SimpleNamespace(axis="X", currentPos=True),
                SimpleNamespace(axis="Y", currentPos=False)
            ]
        )
    ]
)




motors = []

# INIT SYMMETRIC
directions = ["LEFT", "RIGHT"]

for direction in directions:
    for joint in joints.symmetric:
        for axis in joint.axis:
            data = SimpleNamespace()
            data.motor = robot.getDevice(f"RM_{axis.axis}_{direction}_{joint.name}")

            if axis.currentPos:
                posSens = robot.getDevice(f"PS_{axis.axis}_{direction}_{joint.name}")
                posSens.enable(timestep)
                data.positionSensor = posSens

            data.currentPos = axis.currentPos
            motors.append(data)
            
# INIT ASYMMETRIC
for joint in joints.asymmetric:
    for axis in joint.axis:
        data = SimpleNamespace()
        data.motor = robot.getDevice(f"RM_{axis.axis}_{joint.name}")

        if axis.currentPos:
            posSens = robot.getDevice(f"PS_{axis.axis}_{joint.name}")
            posSens.enable(timestep)
            data.positionSensor = posSens
        
        data.motor.setPosition(0)

        data.currentPos = axis.currentPos
        motors.append(data)
        
gyro = Gyro(name="BODY_GYRO", sampling_period=timestep)
gyro.enable(timestep)
accelerometer = Accelerometer(name="BODY_ACCELEROMETER",sampling_period=timestep)
accelerometer.enable(timestep)
inertialUnit = InertialUnit(name="BODY_INERTIALUNIT", sampling_period=timestep)
inertialUnit.enable(timestep)


turnRate = 0
walkSpeed = 0
            
def getObservationSpace() -> list[float]:
    ret = []
    for motor in motors:
        if motor.currentPos:
            ret.append(motor.positionSensor.getValue())
        else:
            ret.append(motor.motor.getTargetPosition())
    
    ret.extend(gyro.getValues())
    ret.extend(accelerometer.getValues())
    ret.extend(inertialUnit.getRollPitchYaw())
    
    ret.extend([turnRate, walkSpeed])


    return ret

print(getObservationSpace())

env = gym.Env()

env.action_space = gym.spaces.Box(low=-1, high=1,shape=(18,), dtype=np.float32)

env.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(29,), dtype=np.float32)

def step(action):
    # perform one simulation step, apply action to motors, read sensors, compute reward/termination
    # (fill in your action -> motor commands here)
    # Example: advance simulation and read all position sensors as observation
    i = 0
    
    for curAction in action:
        motors[i].motor.setPosition(curAction)
        
        i+=1
        
    
    
    robot.step(timestep)
    
    observation = getObservationSpace()
    
    reward = 0.0
    
    terminated = False
    truncated = False
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

print("asd")
model = PPO("MlpPolicy", env, verbose=1, policy_kwargs=policy_kwargs)
model.learn(10000)

while robot.step(timestep) != -1:
    print("stepping cause i dont know waht the fuck to do")
    pass
