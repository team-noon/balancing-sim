from typing import List, Optional
from dataclasses import dataclass
from controller import Node, Field, TouchSensor, Motor, PositionSensor
import torch

# Define data structures using dataclasses
@dataclass
class BodyPart:
    name: str
    doneOnTouch : bool
    
    touchReward : Optional[float]
    noTouchRewardDelay : Optional[int]
    noTouchReward : Optional[float]

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
    rotField : Field
    startingRotation : List[float]
    touchSensor : TouchSensor
    doneOnTouch : bool
    lastTouched : int
    
    touchReward : Optional[float]
    noTouchRewardDelay : Optional[int]
    noTouchReward : Optional[float]

@dataclass
class JointAxis:
    axis: str
    currentPos: bool
    minPos : float
    maxPos : float

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
    minPos : float
    maxPos : float
    positionSensor: Optional[PositionSensor] = None
    
    
