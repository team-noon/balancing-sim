from enum import Enum
from typing import List, Tuple, Optional
from dataclasses import dataclass

class patternTypes(Enum):
    walk = 0
    stand = 1
    animate = 2
    kick = 3

@dataclass
class pattern:
    mask : List[float] # WHERE MASK IS 1, USE THAT NUMBER
    values : List[float]
    
    turnRate : float= 0
    
    walkMode : bool = False
    standMode : bool = False
    animateMode : bool = False
    kickMode : bool = False
    
    uprightReward : bool = False
    turnRateReward : bool   = False
    walkSpeedReward : bool = False
    verticalPenalty : bool = False
    sidePenalty : bool = False
    stillnessReward : bool = False
    canTouchGround : bool = False
    touchReward : bool = False
    noTouchReward : bool = False






from math import pi, sin
from parameters import HipXMotorNum, HipYMotorNum, KneeXMotorNum, AnkleXMotornum, AnkleYMotornum
from classes import MotorData
from walkPattern import evaluateWalk, walkParameters

from animationPattern import initAnimations, evaluateAnimation

from standPattern import evaluateStand, standParameters

from parameters import ShoulderYMotornum, ShoulderZMotornum, ElbowZMotornum


    

    


class patternGenerator:
    currentPattern :patternTypes = patternTypes.walk
    
    patternWalkParameters = walkParameters() 
    
    patternStandParameters = standParameters()
    
    currentAnimation : int= -1 # -1 =no animation
    
    animations : List[dict]= []
    
    animationPointers : dict = {}
    
    timeSinceChange : int= 0
    
    motors : List[MotorData] = []
    
    trainMode : bool = False
    
    timeStep :int = 0
    
    def __init__(self, motors : List[MotorData], timeStep, train : bool = False) -> None:
        self.motors = motors
        
        self.trainMode = train
        
        self.timeStep = timeStep
        
        animInit = initAnimations(train)
        self.animations = animInit[0]
        self.animationPointers = animInit[1]
        
        
        
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
            print("setting anim to: " + newAnimName)

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
    
    
    def evaluatePattern(self, rot: Optional[Tuple[float, float, float]] = None) -> pattern:
        self.timeSinceChange += self.timeStep
        
        retPat = pattern([-1 for _ in range(18)],[-1 for _ in range(18)])
        
        def DefArm():
            retPat.mask[ShoulderYMotornum] = 1
            retPat.values[ShoulderYMotornum] = self.motors[ShoulderYMotornum].defaultPos
            retPat.mask[ShoulderZMotornum] = 1
            retPat.values[ShoulderZMotornum] = self.motors[ShoulderZMotornum].defaultPos
            retPat.mask[ElbowZMotornum] = 1
            retPat.values[ElbowZMotornum] = self.motors[ElbowZMotornum].defaultPos
            
            offset = 9
            
            retPat.mask[ShoulderYMotornum + offset] = 1
            retPat.values[ShoulderYMotornum] = self.motors[ShoulderYMotornum+ offset].defaultPos
            retPat.mask[ShoulderZMotornum+ offset] = 1
            retPat.values[ShoulderZMotornum+ offset] = self.motors[ShoulderZMotornum+ offset].defaultPos
            retPat.mask[ElbowZMotornum+ offset] = 1
            retPat.values[ElbowZMotornum+ offset] = self.motors[ElbowZMotornum+ offset].defaultPos
        
        if(self.currentPattern == patternTypes.walk and rot is not None):
            retPat = evaluateWalk(self.patternWalkParameters, max(0, self.timeSinceChange), rot, self.motors)
            
            if(self.patternWalkParameters.defaultArmPos):
                DefArm()
        

        elif(self.currentPattern == patternTypes.animate and self.currentAnimation != -1):
            retPat = evaluateAnimation(self.animations[self.currentAnimation], self.timeSinceChange, self.motors)
        elif(self.currentPattern == patternTypes.stand and rot is not None):
            retPat = evaluateStand(self.patternStandParameters, max(0, self.timeSinceChange), rot, self.motors)
            if(self.patternStandParameters.defaultArmPos):
                DefArm()

        return retPat