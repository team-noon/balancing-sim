from patternGenerator import pattern
from typing import List, Tuple

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


def stepAxis(motorNumber : int, axisKeyFrames : list[dict]):
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
    

    
    



def evaluateAnimation() -> pattern:
    mask: List[float] = [-1 for _ in range(18)]
    values: List[float] = [-1 for _ in range(18)]
    
    
    
    return pattern(mask=mask, values=values)
    
    