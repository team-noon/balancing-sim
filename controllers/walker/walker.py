import os
import sys
from types import SimpleNamespace

controller_dir = os.path.dirname(__file__)
shared_dir = os.path.join(controller_dir, '..')
sys.path.append(os.path.abspath(shared_dir))

from initScripts import InitMotors
from controller import Supervisor, InertialUnit
from math import pi, sin

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

motors = InitMotors(timestep)

symmetricList = ["shoulder", "elbow", "hip", "knee", "ankle"]


motors[0].motor.setPosition(((0) * ((motors[0].maxPos) - (motors[0].minPos))) + motors[0].minPos)
motors[9].motor.setPosition(((0) * ((motors[9].maxPos) - (motors[9].minPos))) + motors[9].minPos)

inertialUnit = InertialUnit(name="BODY_INERTIALUNIT", sampling_period=timestep)
inertialUnit.enable(timestep)

walkSpeed = 1


HipXMotorNum = 3

HipXPhase  = 0
HipXWeight = 0.4
HipXOffset = 0.5
HipMin = 0.45
HipMax = 0.6



HipYMotorNum = 4

HipYPhase  = 0
HipYWeight = 0.15
HipYOffset = 0.5
HipYMin = 0.40
HipYMax = 0.6

KneeXMotorNum = 6

KneeXPhase  = 0
KneeXWeight = 0.1
KneeXOffset = 0.5
KneeXMin = 0.4
KneeXMax = 0.5


AnkleXMotornum = 7

AnkleYMotornum = 8


curTime : int =0


while robot.step(timestep) != -1:
    curTime += timestep
    
    rot = inertialUnit.getRollPitchYaw()
    
    hipXAngle = (min(HipMax, max(HipMin, sin((curTime/1000)*walkSpeed + HipXPhase)*HipXWeight + HipXOffset)) * ((motors[HipXMotorNum].maxPos) - (motors[HipXMotorNum].minPos))) + motors[HipXMotorNum].minPos
    
    motors[HipXMotorNum].motor.setPosition(hipXAngle)
    
    
    hipYAngle = (min(HipYMax, max(HipYMin,sin((curTime/1000)*walkSpeed + HipYPhase)*HipYWeight + HipYOffset))* ((motors[HipYMotorNum].maxPos) - (motors[HipYMotorNum].minPos)))+ motors[HipYMotorNum].minPos
    
    motors[HipYMotorNum].motor.setPosition(hipYAngle)

    kneeXAngle=(min(KneeXMax, max(KneeXMin, sin((curTime/1000)*walkSpeed + KneeXPhase)*KneeXWeight + KneeXOffset)) * ((motors[KneeXMotorNum].maxPos) - (motors[KneeXMotorNum].minPos))) + motors[KneeXMotorNum].minPos
    
    motors[KneeXMotorNum].motor.setPosition(kneeXAngle)
    
    motors[AnkleXMotornum].motor.setPosition(min(motors[AnkleXMotornum].maxPos, max(motors[AnkleXMotornum].minPos, -hipXAngle - kneeXAngle - rot[0])))
    
    motors[AnkleYMotornum].motor.setPosition(min(motors[AnkleYMotornum].maxPos,max(-hipYAngle, motors[AnkleYMotornum].minPos)))
    
    #other leg
    
    otherHipXAngle = (min(HipMax, max(HipMin, sin((curTime/1000)*walkSpeed + HipXPhase + pi)*HipXWeight + HipXOffset)) * ((motors[HipXMotorNum+9].maxPos) - (motors[HipXMotorNum+9].minPos))) + motors[HipXMotorNum+9].minPos
    
    motors[HipXMotorNum + 9].motor.setPosition(otherHipXAngle)
    
    otherHipYAngle = (min(HipYMax, max(HipYMin,sin((curTime/1000)*walkSpeed + HipYPhase)*HipYWeight + HipYOffset))* ((motors[HipYMotorNum+9].maxPos) - (motors[HipYMotorNum+9].minPos)))+ motors[HipYMotorNum+9].minPos
    
    motors[HipYMotorNum + 9].motor.setPosition(otherHipYAngle)
    
    otherKneeXAngle = (min(KneeXMax, max(KneeXMin, sin((curTime/1000)*walkSpeed + KneeXPhase + pi)*KneeXWeight + KneeXOffset)) * ((motors[KneeXMotorNum+9].maxPos) - (motors[KneeXMotorNum+9].minPos))) + motors[KneeXMotorNum+9].minPos
    
    motors[KneeXMotorNum + 9].motor.setPosition(otherKneeXAngle)
    
    
    motors[AnkleXMotornum + 9].motor.setPosition(min(max(-otherKneeXAngle + otherHipXAngle - rot[0], motors[AnkleXMotornum + 9].minPos),motors[AnkleXMotornum + 9].maxPos))#)
    
    motors[AnkleYMotornum + 9].motor.setPosition(min(max(otherHipYAngle, motors[AnkleYMotornum + 9].minPos),motors[AnkleYMotornum + 9].maxPos))
    
