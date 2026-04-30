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
from patternGenerator import patternGenerator

#keyframes : dict


'''with open(os.path.join(controller_dir, "keyframes.jsonc"), "r") as jsonfile:
    data = json.loads(remove_json_comments(jsonfile.read()))
    keyframes = data'''

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

motors = InitMotors(timestep)


thisPatternGenerator = patternGenerator(motors, timestep)

thisPatternGenerator.setAnimationByName("sweep")




while robot.step(timestep) != -1:
    pat = thisPatternGenerator.evaluatePattern()
    for i in range(18):
        if(pat.mask[i] == 1):
            motors[i].setMotor(pat.values[i])
            print(motors[i].name, pat.values[i])
   
   
    

    
    
    
