import os
import sys
from types import SimpleNamespace

controller_dir = os.path.dirname(__file__)
shared_dir = os.path.join(controller_dir, '..')
sys.path.append(os.path.abspath(shared_dir))

from initScripts import InitMotors
from controller import Supervisor, InertialUnit
from math import pi, sin
from classes import setMotorTypes
from patternGenerator import patternGenerator

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

motors = InitMotors(timestep)


inertialUnit = InertialUnit(name="BODY_INERTIALUNIT", sampling_period=timestep)
inertialUnit.enable(timestep)

thisPatternGenerator = patternGenerator()


curTime : int =0

while robot.step(timestep) != -1:
    curTime += timestep
    
    rot = inertialUnit.getRollPitchYaw()
    
    pats = thisPatternGenerator.evaluatePattern(timestep=curTime, rot=rot ,motors=motors)
    
    for i in range(18):
        if(pats.mask[i] == True):
            motors[i].setMotor(pats.values[i])
        

    