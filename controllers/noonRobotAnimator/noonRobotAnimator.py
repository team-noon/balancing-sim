import os
import sys
from types import SimpleNamespace

controller_dir = os.path.dirname(__file__)
shared_dir = os.path.join(controller_dir, '..')
sys.path.append(os.path.abspath(shared_dir))

from initScripts import InitMotors
from controller import Supervisor
from parameters import brushlessSpeed, brushlessTorque, servoSpeed, servoTorque
import json

keyframes : dict

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


with open(os.path.join(controller_dir, "keyframes.jsonc"), "r") as jsonfile:
    data = json.loads(remove_json_comments(jsonfile.read()))
    keyframes = data

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

motors = InitMotors(timestep, True)

symmetricList = ["shoulder", "elbow", "hip", "knee", "ankle"]
asymmetricList = ["neck"]


curTime : int =0

def evalTorque(axis: dict):
    torque = axis.get("torque", 1)
    if 0 <= torque <= 1:
        return torque
    else:
        return 1

def stepAxis(motorNumber : int, axisKeyFrames : list[dict]):
    lastTime :int = 0
    lastPos : float= motors[motorNumber].defaultPos
    lastTorque : float = 1
    
    nextTime :int = 0
    nextPos : float= motors[motorNumber].defaultPos
    nextTorque : float = 1
    
    if(len(axisKeyFrames) > 0):
        nextTime = axisKeyFrames[0]["time"]
        nextPos = axisKeyFrames[0]["pos"]
        nextTorque = evalTorque(axisKeyFrames[0])
    
    i : int = 0
    
    while i < len(axisKeyFrames):
        if(axisKeyFrames[i]["time"] <= curTime):
            lastTime = axisKeyFrames[i]["time"]
            lastPos = axisKeyFrames[i]["pos"]
            lastTorque = evalTorque(axisKeyFrames[i])
        
            nextTime = lastTime
            nextPos = lastPos
            nextTorque = lastTorque
            
            
            if(i < len(axisKeyFrames) -1):
                nextTime = axisKeyFrames[i+1]["time"]
                nextPos = axisKeyFrames[i+1]["pos"]
                nextTorque = evalTorque(axisKeyFrames[i+1])

            
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

    nextTorque =min(1, max(0, nextTorque))
    
    pos = ((curAction) * ((motors[motorNumber].maxPos) - (motors[motorNumber].minPos))) + motors[motorNumber].minPos
    
    motors[motorNumber].motor.setPosition(pos)
    
    if(motors[motorNumber].currentPos):
        motors[motorNumber].motor.setVelocity(brushlessSpeed)
        motors[motorNumber].motor.setAcceleration(10)
        motors[motorNumber].motor.setAvailableTorque(nextTorque*brushlessTorque/1000)
    else:
        motors[motorNumber].motor.setVelocity(servoSpeed)
        motors[motorNumber].motor.setAcceleration(10)
        motors[motorNumber].motor.setAvailableTorque(nextTorque*servoTorque/1000)
    
    
    
    
    
    
    


while robot.step(timestep) != -1:
    curTime += timestep
    i : int = 0
    
    while i < symmetricList.__len__():
        if(keyframes[symmetricList[i]]["L"]["xMotorNum"] != -1):
            stepAxis(keyframes[symmetricList[i]]["L"]["xMotorNum"], keyframes[symmetricList[i]]["L"]["x"])
        if(keyframes[symmetricList[i]]["L"]["yMotorNum"] != -1):
            stepAxis(keyframes[symmetricList[i]]["L"]["yMotorNum"], keyframes[symmetricList[i]]["L"]["y"])
        if(keyframes[symmetricList[i]]["L"]["zMotorNum"] != -1):
            stepAxis(keyframes[symmetricList[i]]["L"]["zMotorNum"], keyframes[symmetricList[i]]["L"]["z"])
            
        if(keyframes[symmetricList[i]]["R"]["xMotorNum"] != -1):
            stepAxis(keyframes[symmetricList[i]]["R"]["xMotorNum"], keyframes[symmetricList[i]]["R"]["x"])
        if(keyframes[symmetricList[i]]["R"]["yMotorNum"] != -1):
            stepAxis(keyframes[symmetricList[i]]["R"]["yMotorNum"], keyframes[symmetricList[i]]["R"]["y"])
        if(keyframes[symmetricList[i]]["R"]["zMotorNum"] != -1):
            stepAxis(keyframes[symmetricList[i]]["R"]["zMotorNum"], keyframes[symmetricList[i]]["R"]["z"])
        i+=1
        
    i = 0
        
    while i < asymmetricList.__len__():
        if(keyframes[asymmetricList[i]]["xMotorNum"] != -1):
            stepAxis(keyframes[asymmetricList[i]]["xMotorNum"], keyframes[asymmetricList[i]]["x"])
        if(keyframes[asymmetricList[i]]["yMotorNum"] != -1):
            stepAxis(keyframes[asymmetricList[i]]["yMotorNum"], keyframes[asymmetricList[i]]["y"])
        if(keyframes[asymmetricList[i]]["zMotorNum"] != -1):
            stepAxis(keyframes[asymmetricList[i]]["zMotorNum"], keyframes[asymmetricList[i]]["z"])
        i+=1
    
    
    
