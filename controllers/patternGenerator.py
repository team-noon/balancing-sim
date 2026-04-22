from enum import Enum
from dataclasses import dataclass
from typing import List, Tuple, Optional
from math import pi, sin
from parameters import HipXMotorNum, HipYMotorNum, KneeXMotorNum, AnkleXMotornum, AnkleYMotornum
from classes import MotorData
from walkPattern import evaluateWalk, walkParameters

from animationPattern import initAnimations, evaluateAnimation


class patternTypes(Enum):
    walk = 0
    stand = 1
    animate = 2
    kick = 3
    

    
@dataclass
class pattern:
    mask : List[float] # WHERE MASK IS 1, USE THAT NUMBER
    values : List[float]
    uprightReward : bool
    turnRateReward : bool
    walkSpeedReward : bool
    verticalPenalty : bool
    sidePenalty : bool
    stillnessReward : bool
    canTouchGround : bool
    touchReward : bool
    noTouchReward : bool
    


class patternGenerator:
    currentPattern :patternTypes = patternTypes.walk
    
    patternWalkParameters = walkParameters() 
    
    currentAnimation : int= -1 # -1 =no animation
    
    animations : List[dict]= []
    
    animationPointers : dict = {}
    
    timeSinceChange : int= 0
    
    motors : List[MotorData] = []
    
    def __init__(self, motors : List[MotorData]) -> None:
        self.motors = motors
        pass
    
    def setAnimationById(self, newAnimId : int)-> None:
        if(newAnimId >= 0 and newAnimId < self.animations.__len__()):
            self.currentPattern = patternTypes.animate
            self.currentAnimation = newAnimId
            self.timeSinceChange = 0
            
    def setAnimationByName(self, newAnimName : str)-> None:
        anim = self.animationPointers.get(newAnimName)
        if anim is not None:
            self.setAnimationById(anim)

    def setWalkMode(self)-> None:
        self.currentAnimation = -1
        self.currentPattern = patternTypes.walk
        self.timeSinceChange=0
        
    def setStandMode(self) -> None:
        self.currentAnimation = -1
        self.currentPattern = patternTypes.stand
        self.timeSinceChange= 0
    
    def setKickMode(self) -> None:
        self.currentAnimation = -1
        self.currentPattern = patternTypes.kick
        self.timeSinceChange=0
    
    
    def evaluatePattern(self, timestep : int, rot: Optional[Tuple[float, float, float]] = None) -> pattern:
        self.timeSinceChange += timestep
        
        if(self.currentPattern == patternTypes.walk and rot is not None):
            return evaluateWalk(self.patternWalkParameters, max(0, self.timeSinceChange), rot, self.motors)
        elif(self.currentPattern == patternTypes.animate and self.currentAnimation != -1):
            return evaluateAnimation(self.animations[self.currentAnimation], self.timeSinceChange, self.motors)

        return pattern([-1 for _ in range(18)],[-1 for _ in range(18)])