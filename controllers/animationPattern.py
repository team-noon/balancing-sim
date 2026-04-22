from patternGenerator import pattern
from typing import List, Tuple
from classes import MotorData

def remove_json_comments(text: str) -> str:
    lines = text.split("\n")

    i = 0
    while i < len(lines):
        j = 0
        while j < len(lines[i]) - 1:
            if lines[i][j] == "/" and lines[i][j + 1] == "/":
                lines[i] = lines[i][:j]
                break
            j += 1
        i += 1

    return "\n".join(lines)

# first is the actual list of 
def initAnimations() -> Tuple[List[dict], dict]:
    
    return ([],{})


def stepAxis(motorNumber : int, axisKeyFrames : list[dict], curTime : int, motors : List[MotorData]) -> float:
    lastTime : int = 0
    lastPos : float= motors[motorNumber].defaultPos

    nextTime :int = 0
    nextPos : float= motors[motorNumber].defaultPos
    
    if(len(axisKeyFrames) > 0):
        nextTime = axisKeyFrames[0]["time"]
        nextPos = axisKeyFrames[0]["pos"]
    
    i : int = 0
    
    while i < len(axisKeyFrames):
        if(axisKeyFrames[i]["time"] <= curTime):
            lastTime = axisKeyFrames[i]["time"]
            lastPos = axisKeyFrames[i]["pos"]
        
            nextTime = lastTime
            nextPos = lastPos
            
            
            if(i < len(axisKeyFrames) -1):
                nextTime = axisKeyFrames[i+1]["time"]
                nextPos = axisKeyFrames[i+1]["pos"]

            
        i+=1
        
    curAction = 0
    
    if(nextPos < 0 or nextPos>1):
        nextPos = motors[motorNumber].defaultPos
    if(lastPos < 0 or lastPos>1):
        lastPos = motors[motorNumber].defaultPos
        
    if nextTime == lastTime:
        curAction = nextPos
    else:
        curAction = lastPos + (nextPos - lastPos) * ((curTime-lastTime) / (nextTime-lastTime))
    
    curAction = min(1, max(0, curAction))
    return curAction
    

    
symmetricList = ["shoulder", "elbow", "hip", "knee", "ankle"]
asymmetricList = []#["neck"]



def evaluateAnimation(keyframes : dict, curTime : int, motors : List[MotorData]) -> pattern:
    mask: List[float] = [-1 for _ in range(18)]
    values: List[float] = [-1 for _ in range(18)]
    
    i = 0
    while i < symmetricList.__len__():
        if(keyframes[symmetricList[i]]["L"]["xMotorNum"] != -1):
            mask[keyframes[symmetricList[i]]["L"]["xMotorNum"]] = 1
            values[keyframes[symmetricList[i]]["L"]["xMotorNum"]] = stepAxis(keyframes[symmetricList[i]]["L"]["xMotorNum"], keyframes[symmetricList[i]]["L"]["x"], curTime, motors)
        if(keyframes[symmetricList[i]]["L"]["yMotorNum"] != -1):
            mask[keyframes[symmetricList[i]]["L"]["yMotorNum"]] = 1
            values[keyframes[symmetricList[i]]["L"]["yMotorNum"]] = stepAxis(keyframes[symmetricList[i]]["L"]["yMotorNum"], keyframes[symmetricList[i]]["L"]["y"], curTime, motors)
        if(keyframes[symmetricList[i]]["L"]["zMotorNum"] != -1):
            mask[keyframes[symmetricList[i]]["L"]["zMotorNum"]] = 1
            values[keyframes[symmetricList[i]]["L"]["zMotorNum"]] = stepAxis(keyframes[symmetricList[i]]["L"]["zMotorNum"], keyframes[symmetricList[i]]["L"]["z"], curTime, motors)
            
        if(keyframes[symmetricList[i]]["R"]["xMotorNum"] != -1):
            mask[keyframes[symmetricList[i]]["R"]["xMotorNum"]] = 1
            values[keyframes[symmetricList[i]]["R"]["xMotorNum"]] = stepAxis(keyframes[symmetricList[i]]["R"]["xMotorNum"], keyframes[symmetricList[i]]["R"]["x"], curTime, motors)
        if(keyframes[symmetricList[i]]["R"]["yMotorNum"] != -1):
            mask[keyframes[symmetricList[i]]["R"]["yMotorNum"]] = 1
            values[keyframes[symmetricList[i]]["R"]["yMotorNum"]] = stepAxis(keyframes[symmetricList[i]]["R"]["yMotorNum"], keyframes[symmetricList[i]]["R"]["y"], curTime, motors)
        if(keyframes[symmetricList[i]]["R"]["zMotorNum"] != -1):
            mask[keyframes[symmetricList[i]]["R"]["zMotorNum"]] = 1
            values[keyframes[symmetricList[i]]["R"]["zMotorNum"]] = stepAxis(keyframes[symmetricList[i]]["R"]["zMotorNum"], keyframes[symmetricList[i]]["R"]["z"], curTime, motors)
        i+=1
        
    i = 0
        
    while i < asymmetricList.__len__():
        if(keyframes[asymmetricList[i]]["xMotorNum"] != -1):
            mask[keyframes[asymmetricList[i]]["xMotorNum"]] = 1
            values[keyframes[asymmetricList[i]]["xMotorNum"]] = stepAxis(keyframes[asymmetricList[i]]["xMotorNum"], keyframes[asymmetricList[i]]["x"], curTime, motors)
        if(keyframes[asymmetricList[i]]["yMotorNum"] != -1):
            mask[keyframes[asymmetricList[i]]["yMotorNum"]] = 1
            values[keyframes[asymmetricList[i]]["yMotorNum"]] = stepAxis(keyframes[asymmetricList[i]]["yMotorNum"], keyframes[asymmetricList[i]]["y"], curTime, motors)
        if(keyframes[asymmetricList[i]]["zMotorNum"] != -1):
            mask[keyframes[asymmetricList[i]]["zMotorNum"]] = 1
            values[keyframes[asymmetricList[i]]["zMotorNum"]] = stepAxis(keyframes[asymmetricList[i]]["zMotorNum"], keyframes[asymmetricList[i]]["z"], curTime, motors)
            
        i+=1
    
    
    return pattern(mask=mask, values=values)
    
    