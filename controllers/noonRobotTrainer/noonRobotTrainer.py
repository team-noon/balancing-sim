"""noonRobotTrainer controller."""

import torch
from controller import Supervisor, InertialUnit, Gyro, Accelerometer, Motor, PositionSensor, Node, Field, TouchSensor
from dataclasses import dataclass
from typing import Optional, List, Tuple, Dict, Any, Union
import gymnasium as gym
from stable_baselines3 import PPO
import numpy as np

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Define data structures using dataclasses
@dataclass
class BodyPart:
    name: str
    doneOnTouch : bool

@dataclass
class BodyPartCollection:
    asymmetric: List[BodyPart]
    symmetric: List[BodyPart]

@dataclass
class BodyPartData:
    node: Node
    startingPosition: List[float]
    transField: Field
    linearVelocityField : Field
    angularVelocityField : Field
    touchSensor : TouchSensor
    doneOnTouch : bool

@dataclass
class JointAxis:
    axis: str
    currentPos: bool

@dataclass
class Joint:
    name: str
    axes: List[JointAxis]

@dataclass
class JointCollection:
    asymmetric: List[Joint]
    symmetric: List[Joint]

@dataclass
class MotorData:
    motor: Motor
    currentPos: bool
    positionSensor: Optional[PositionSensor] = None
    
# CHATGPT WROTE THIS FUNCTION
def GetNodeByName(root_node, target_name)-> Optional[Node]:
    # If this node has a name field, check it
    
    name_field = root_node.getField("name")
    if name_field:
        print("THERE IS A NAMEFIELD : ", name_field.getSFString())
        if name_field.getSFString() == target_name:
            return root_node

    # Search children fields
    for field_name in ["children", "endPoint"]:
        field = root_node.getField(field_name)
        if field is None:
            print("WHAAAT THERE IS NO FUCKING ", field_name)
            continue

        if field.getTypeName() == "MFNode":
            for i in range(field.getCount()):
                result = GetNodeByName(field.getMFNode(i), target_name)
                if result:
                    return result

        elif field.getTypeName() == "SFNode":
            child = field.getSFNode()
            if child:
                result = GetNodeByName(child, target_name)
                if result:
                    return result

    return None

# Initialize body parts
bodyPartList = BodyPartCollection(
    asymmetric=[
        #BodyPart(name="body", doneOnTouch=True),
        #BodyPart(name="head", doneOnTouch=True)
    ],
    symmetric=[
        #BodyPart(name="upper_arm", doneOnTouch=True),
        #BodyPart(name="lower_arm", doneOnTouch=True),
        #BodyPart(name="hand", doneOnTouch=True),
        #BodyPart(name="upper_leg", doneOnTouch=True),
        #BodyPart(name="lower_leg", doneOnTouch=True),
        #BodyPart(name="foot", doneOnTouch=False)
    ]
)

print(GetNodeByName(robot.getRoot(), "body"))

BodyParts: List[BodyPartData] = []

# Initialize body part data (you'll need to implement this part)
for bodyPart in bodyPartList.asymmetric:
    # You'll need to get the actual node and field references here
    thisNode = GetNodeByName(robot.getSelf(), bodyPart.name)
    
    if thisNode is None:
        print("AAAAAAH", bodyPart.name)
        continue
        
    transField = thisNode.getField("translation")
    linearVelocityField = thisNode.getField("linearVelocity")
    angularVelocityField = thisNode.getField("angularVelocity")
    
    touchSens = TouchSensor(f"TS_{bodyPart.name.upper()}")
    touchSens.enable(timestep)
    
    BodyParts.append(BodyPartData(node=thisNode, startingPosition=transField.getSFVec3f(), transField=transField, linearVelocityField=linearVelocityField, angularVelocityField=angularVelocityField, touchSensor=touchSens, doneOnTouch=bodyPart.doneOnTouch))


directions = ["left", "right"]
for direction in directions:
    for bodyPart in bodyPartList.symmetric:
        
        thisNode = GetNodeByName(robot.getSelf(), f"{direction}_{bodyPart.name}")
        
        if thisNode is None:
            print("AAAAAAH", f"{direction}_{bodyPart.name}")
            continue
        transField = thisNode.getField("translation")
        linearVelocityField = thisNode.getField("linearVelocity")
        angularVelocityField = thisNode.getField("angularVelocity")

        touchSens = TouchSensor(f"TS_{direction.upper}_{bodyPart.name.upper()}")
        touchSens.enable(timestep)
    
        BodyParts.append(BodyPartData(node=thisNode, startingPosition=transField.getSFVec3f(), transField=transField, linearVelocityField=linearVelocityField, angularVelocityField=angularVelocityField, touchSensor=touchSens, doneOnTouch=bodyPart.doneOnTouch))

# Define joints
joints = JointCollection(
    asymmetric=[
        # Joint(name="HEAD", axes=[JointAxis(axis="Z", currentPos=False)])
    ],
    symmetric=[
        Joint(
            name="SHOULDER",
            axes=[
                JointAxis(axis="Y", currentPos=False),
                JointAxis(axis="Z", currentPos=False)
            ]
        ),
        Joint(
            name="ELBOW",
            axes=[
                JointAxis(axis="Y", currentPos=False)
            ]
        ),
        Joint(
            name="HIP",
            axes=[
                JointAxis(axis="X", currentPos=True),
                JointAxis(axis="Y", currentPos=False),
                JointAxis(axis="Z", currentPos=False)
            ]
        ),
        Joint(
            name="KNEE",
            axes=[
                JointAxis(axis="X", currentPos=True)
            ]
        ),
        Joint(
            name="FOOT",
            axes=[
                JointAxis(axis="X", currentPos=True),
                JointAxis(axis="Y", currentPos=False)
            ]
        )
    ]
)

motors: List[MotorData] = []

# INIT SYMMETRIC JOINTS
directions = ["LEFT", "RIGHT"]

for direction in directions:
    for joint in joints.symmetric:
        for axis in joint.axes:
            motor_name = f"RM_{axis.axis}_{direction}_{joint.name}"
            data = MotorData(
                motor=Motor(motor_name),
                currentPos=axis.currentPos
            )

            if axis.currentPos:
                pos_sens_name = f"PS_{axis.axis}_{direction}_{joint.name}"
                pos_sens = PositionSensor(pos_sens_name)
                pos_sens.enable(timestep)
                data.positionSensor = pos_sens
                
            data.motor.setPosition(0.0)
            data.motor.setVelocity(2.0)
            data.motor.setAcceleration(2.0)

            motors.append(data)
            
# INIT ASYMMETRIC JOINTS
for joint in joints.asymmetric:
    for axis in joint.axes:
        motor_name = f"RM_{axis.axis}_{joint.name}"
        data = MotorData(
            motor=Motor(motor_name),
            currentPos=axis.currentPos
        )

        if axis.currentPos:
            pos_sens_name = f"PS_{axis.axis}_{joint.name}"
            pos_sens = PositionSensor(pos_sens_name)
            pos_sens.enable(timestep)
            data.positionSensor = pos_sens
        
        data.motor.setPosition(0.0)
        data.motor.setVelocity(2.0)
        data.motor.setAcceleration(2.0)

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
    # (fill in your action -> motor commands here)
    # Example: advance simulation and read all position sensors as observation
    i = 0
    
    for curAction in action:
        pos = ((curAction) * ((motors[i].motor.getMaxPosition()) - (motors[i].motor.getMinPosition()))) + motors[i].motor.getMinPosition()
        motors[i].motor.setForce(1)
        motors[i].motor.setVelocity(2.0)
        motors[i].motor.setAcceleration(2.0)
        motors[i].motor.setPosition(pos/1.2)

        
        i+=1
    
    for bodyPart in BodyParts:
        touch = bodyPart.touchSensor.getValue()
        if touch != 0:
            print(bodyPart.node.getField("name").getSFString())
    
    
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

model = PPO("MlpPolicy", env, verbose=1, policy_kwargs=policy_kwargs, device="cpu")
model.learn(10000)

while robot.step(timestep) != -1:
    print("stepping cause i dont know waht the fuck to do")
    pass
