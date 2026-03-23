from enum import Enum

class patternTypes(Enum):
    walk = 0
    animate = 1
    kick = 2

def evaluateWalk():
    print("walk")



class patternGenerator:
    currentPattern = 0