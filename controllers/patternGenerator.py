from enum import Enum
from dataclasses import dataclass
from typing import List, Tuple
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
    


class patternGenerator:
    currentPattern :patternTypes = patternTypes.walk
    
    patternWalkParameters = walkParameters() 
    
    currentAnimation : int= -1 # -1 =no animation
    
    animations : List[dict]= []
    
    animationPointers : dict = {}
    
    timeSinceChange = 0
    
    def __init__(self) -> None:
        
        pass
        
    
    def evaluatePattern(self, timestep : int, rot: Tuple[float, float, float], motors : List[MotorData]) -> pattern:
        
        if(self.currentPattern == patternTypes.walk):
            return evaluateWalk(self.patternWalkParameters, max(0, timestep - self.timeSinceChange), rot, motors)
        elif(self.currentPattern == patternTypes.animate):
            return evaluateAnimation()