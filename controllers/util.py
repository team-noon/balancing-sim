from parameters import ShoulderYMotornum, ShoulderZMotornum, ElbowZMotornum

def threshold(val : float, thresholdValue : float, threshold : float) -> bool:
    return abs(val - thresholdValue) <= abs(threshold)

def isArmNum(motorNum : int) -> bool:
    if motorNum in [ShoulderZMotornum, ShoulderZMotornum + 9, ShoulderYMotornum, ShoulderYMotornum +9, ElbowZMotornum, ElbowZMotornum +9]:
        return True
    return False