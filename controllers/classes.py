from typing import List, Optional
from dataclasses import dataclass
from controller import Node, Field, TouchSensor, Motor, PositionSensor
from enum import Enum
from parameters import brushlessSpeed, brushlessTorque, servoSpeed, servoTorque

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

    isFlipped : bool = False
    
    defaultPosition : Optional[float] = None

@dataclass
class Joint:
    name: str
    axes: List[JointAxis]

@dataclass
class JointCollection:
    asymmetric: List[Joint]
    symmetric: List[Joint]

class setMotorTypes(Enum):
    normalised = 0 # 0 - 1
    angle = 1 # angle that it will set it to
    

@dataclass
class MotorData:
    motor: Motor
    currentPos: bool
    minPos : float
    maxPos : float
    defaultPos : float
    
    positionSensor: Optional[PositionSensor] = None
    flipped : bool = False
    
    
    def setMotor(self,pos: float, type : setMotorTypes = setMotorTypes.normalised) -> float:
        angle = self.defaultPos
        if(type == setMotorTypes.normalised):
            angle = self.getAngleFromNormal(pos)
            
        elif(type == setMotorTypes.angle):
            angle = min(self.maxPos, max(self.minPos, pos))
            
            
        self.motor.setPosition(angle)
        # set motor type and attributes
        if(self.currentPos):
            self.motor.setVelocity(brushlessSpeed)
            self.motor.setAcceleration(20)
            self.motor.setAvailableTorque(brushlessTorque/1000)
        else:
            self.motor.setVelocity(servoSpeed)
            self.motor.setAcceleration(20)
            self.motor.setAvailableTorque(servoTorque/1000)
            
        return angle
    
    def getAngleFromNormal(self, pos : float)->float:
        clampedpos = min(1, max(0, pos))
        
        angle = ((clampedpos) * ((self.maxPos) - (self.minPos))) + self.minPos
        if(self.flipped):
            angle =  self.maxPos - ((clampedpos) *((self.maxPos) - (self.minPos)))
            
        return angle
    
    #cat gpt wrotre this func
    def getNormalFromAngle(self, ang : float) -> float:
        clampedpos = 0.5
        if self.flipped:
            clampedpos = (self.maxPos - ang) / (self.maxPos - self.minPos)
        else:
            clampedpos = (ang - self.minPos) / (self.maxPos - self.minPos)
        return min(1, max(0, clampedpos))
            
        
            
        
        


""" 
@dataclass
class KeyFrameAxis:
    Time : int
    pos : float
    torque : float
    
class KeyFrames:
    x : List[KeyFrameAxis]
    xMotorNum : int
    y : List[KeyFrameAxis]
    yMotorNum : int
    z: List[KeyFrameAxis]
    zMotorNum : int
    
@dataclass
class symmetricKeyFrame:
    L : KeyFrames
    R : KeyFrames


@dataclass
class KeyFrameCollection:
    neck : KeyFrames
    shoulder : symmetricKeyFrame
    elbow : symmetricKeyFrame
    hip : symmetricKeyFrame
    knee : symmetricKeyFrame
    ankle : symmetricKeyFrame
"""
    
