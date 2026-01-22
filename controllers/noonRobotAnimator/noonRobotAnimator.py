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


with open("keyframes.json", "r") as jsonfile:
    data = json.loads(jsonfile)
    keyframes = data

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

motors = InitMotors(timestep, True)

symmetricList = ["shoulder", "elbow", "hip", "knee", "ankle"]
asymmetricList = ["neck"]


curTime : int =0

def stepAxis(motorNumber : int, axisKeyFrames : list[dict]):
    lastTime :int = 0
    lastPos : float= 0.0
    lastTorque : float = 0.0
    
    nextTime :int = 0
    nextPos : float= 0.0
    nextTorque : float = 0.0
    
    if(axisKeyFrames.count() > 0):
        nextTime = axisKeyFrames[0]["time"]
        nextPos = axisKeyFrames[0]["pos"]
        nextTorque = axisKeyFrames[0]["torque"]
    
    i : int = 0
    
    while i < axisKeyFrames.count():
        if(axisKeyFrames[i]["time"] >= curTime):
            lastTime = axisKeyFrames[i]["time"]
            lastPos = axisKeyFrames[i]["pos"]
            lastTorque = axisKeyFrames[i]["torque"]
            
            nextTime = lastTime
            nextPos = lastPos
            nextTorque = lastTorque
            
            if(i < axisKeyFrames.count -1):
                nextTime = axisKeyFrames[i+1]["time"]
                nextPos = axisKeyFrames[i+1]["pos"]
                nextTorque = axisKeyFrames[i+1]["torque"]
            
        i+=1
        
    
        
    
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
    i : int = 0
    
    while i < symmetricList.__len__():
        if(keyframes[symmetricList[i]]["x"] and keyframes[symmetricList[i]]["xMotorNum"]):
            stepAxis(keyframes[symmetricList[i]]["xMotorNum"], keyframes[symmetricList[i]]["x"])
        if(keyframes[symmetricList[i]]["y"] and keyframes[symmetricList[i]]["yMotorNum"]):
            stepAxis(keyframes[symmetricList[i]]["yMotorNum"], keyframes[symmetricList[i]]["y"])
        if(keyframes[symmetricList[i]]["z"] and keyframes[symmetricList[i]]["zMotorNum"]):
            stepAxis(keyframes[symmetricList[i]]["zMotorNum"], keyframes[symmetricList[i]]["z"])
        i+=1
        
    i = 0
        
    while i < asymmetricList.__len__():
        if(keyframes[asymmetricList[i]]["x"] and keyframes[asymmetricList[i]]["xMotorNum"]):
            stepAxis(keyframes[asymmetricList[i]]["xMotorNum"], keyframes[asymmetricList[i]]["x"])
        if(keyframes[asymmetricList[i]]["y"] and keyframes[asymmetricList[i]]["yMotorNum"]):
            stepAxis(keyframes[asymmetricList[i]]["yMotorNum"], keyframes[asymmetricList[i]]["y"])
        if(keyframes[asymmetricList[i]]["z"] and keyframes[asymmetricList[i]]["zMotorNum"]):
            stepAxis(keyframes[asymmetricList[i]]["zMotorNum"], keyframes[asymmetricList[i]]["z"])
        i+=1
    
    
    
