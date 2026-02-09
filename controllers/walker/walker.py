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
from math import pi, sin

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

motors = InitMotors(timestep)

symmetricList = ["shoulder", "elbow", "hip", "knee", "ankle"]


motors[0].motor.setPosition(((0) * ((motors[0].maxPos) - (motors[0].minPos))) + motors[0].minPos)
motors[9].motor.setPosition(((0) * ((motors[9].maxPos) - (motors[9].minPos))) + motors[9].minPos)

walkSpeed = 0.1


HipXMotorNum = 3

HipXPhase  = 0
HipXWeight = 0.2
HipXOffset = 0.5


KneeXMotorNum = 6

KneeXPhase  = pi / 2
KneeXWeight = 0.2
KneeXOffset = 0.8


AnkleXMotornum = 7

AnkleXPhase  = pi
AnkleXWeight = 0.4
AnkleXOffset = 0.5


curTime : int =0

while robot.step(timestep) != -1:
    curTime += timestep
    
    motors[HipXMotorNum].motor.setPosition((min(1, max(0, sin((curTime/1000)*walkSpeed + HipXPhase)*HipXWeight + HipXOffset)) * ((motors[HipXMotorNum].maxPos) - (motors[HipXMotorNum].minPos))) + motors[HipXMotorNum].minPos)
    
    motors[HipXMotorNum + 9].motor.setPosition((min(1, max(0, sin((curTime/1000)*walkSpeed + HipXPhase + pi)*HipXWeight + HipXOffset)) * ((motors[HipXMotorNum+9].maxPos) - (motors[HipXMotorNum+9].minPos))) + motors[HipXMotorNum+9].minPos)
    
    motors[KneeXMotorNum].motor.setPosition((min(1, max(0, sin((curTime/1000)*walkSpeed + KneeXPhase)*KneeXWeight + KneeXOffset)) * ((motors[KneeXMotorNum].maxPos) - (motors[KneeXMotorNum].minPos))) + motors[KneeXMotorNum].minPos)
    
    motors[KneeXMotorNum + 9].motor.setPosition((min(1, max(0, sin((curTime/1000)*walkSpeed + KneeXPhase + pi)*KneeXWeight + KneeXOffset)) * ((motors[KneeXMotorNum+9].maxPos) - (motors[KneeXMotorNum+9].minPos))) + motors[KneeXMotorNum+9].minPos)
    
    motors[AnkleXMotornum].motor.setPosition((min(1, max(0, sin((curTime/1000)*walkSpeed + AnkleXPhase)*AnkleXWeight + AnkleXOffset)) * ((motors[AnkleXMotornum].maxPos) - (motors[AnkleXMotornum].minPos))) + motors[AnkleXMotornum].minPos)
    
    motors[AnkleXMotornum + 9].motor.setPosition((min(1, max(0, sin((curTime/1000)*walkSpeed + AnkleXPhase + pi)*AnkleXWeight + AnkleXOffset)) * ((motors[AnkleXMotornum+9].maxPos) - (motors[AnkleXMotornum+9].minPos))) + motors[AnkleXMotornum+9].minPos)
    
    
    i : int = 0
    

        
    
    
    
