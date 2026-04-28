from patternGenerator import pattern
from parameters import HipXMotorNum, HipYMotorNum, KneeXMotorNum, AnkleXMotornum, AnkleYMotornum
from classes import MotorData

from math import sin, pi

from typing import Tuple, List
from dataclasses import dataclass


@dataclass
class walkParameters:
    walkSpeed = 4
    
    HipXPhase  = 0
    HipXWeight = 0.15
    HipXOffset = 0.5
    HipXMin = 0.35
    HipXMax = 0.55

    HipYPhase  = 0
    HipYWeight = 0.05
    HipYOffset = 0.5
    HipYMin = 0.47
    HipYMax = 0.53

    KneeXPhase  = 0.3
    KneeXWeight = 0.3
    KneeXOffset = 0.5
    KneeXMin = 0.36
    KneeXMax = 0.5

def evaluateWalk(
    params: walkParameters,
    timestep: int,
    rot: Tuple[float, float, float],
    motors : List[MotorData]
) -> pattern:
    mask: List[float] = [0 for _ in range(18)]
    values: List[float] = [-1 for _ in range(18)]
    
    t = timestep / 1000

    # ================= LEFT LEG =================
    
    # --- HIP X ---
    mask[HipXMotorNum] = 1
    values[HipXMotorNum] = min(params.HipXMax, max(params.HipXMin,
        sin(t * params.walkSpeed + params.HipXPhase) * params.HipXWeight + params.HipXOffset))
    HipXAngle = motors[HipXMotorNum].getAngleFromNormal(values[HipXMotorNum])

    # --- HIP Y ---
    mask[HipYMotorNum] = 1
    values[HipYMotorNum] = min(params.HipYMax, max(params.HipYMin,
        sin(t * params.walkSpeed + params.HipYPhase) * params.HipYWeight + params.HipYOffset))
    HipYAngle = motors[HipYMotorNum].getAngleFromNormal(values[HipYMotorNum])

    # --- KNEE X ---
    mask[KneeXMotorNum] = 1
    values[KneeXMotorNum] = min(params.KneeXMax, max(params.KneeXMin,
        sin(t * params.walkSpeed + params.KneeXPhase) * params.KneeXWeight + params.KneeXOffset))
    KneeXAngle = motors[KneeXMotorNum].getAngleFromNormal(values[KneeXMotorNum])

    # --- ANKLE X ---
    mask[AnkleXMotornum] = 1
    ankleXAngle = -HipXAngle - KneeXAngle - rot[0]
    values[AnkleXMotornum] = motors[AnkleXMotornum].getNormalFromAngle(ankleXAngle)

    # --- ANKLE Y ---
    mask[AnkleYMotornum] = 1
    values[AnkleYMotornum] = motors[AnkleYMotornum].getNormalFromAngle(HipYAngle)


    # ================= RIGHT LEG =================
    
    offset = 9  # motor index offset

    # --- HIP X ---
    mask[HipXMotorNum + offset] = 1
    values[HipXMotorNum + offset] = min(params.HipXMax, max(params.HipXMin,
        sin(t * params.walkSpeed + params.HipXPhase + pi) * params.HipXWeight + params.HipXOffset))
    otherHipXAngle = motors[HipXMotorNum + offset].getAngleFromNormal(values[HipXMotorNum + offset])

    # --- HIP Y ---
    mask[HipYMotorNum + offset] = 1
    values[HipYMotorNum + offset] = min(params.HipYMax, max(params.HipYMin,
        sin(t * params.walkSpeed + params.HipYPhase) * params.HipYWeight + params.HipYOffset))
    otherHipYAngle = motors[HipYMotorNum + offset].getAngleFromNormal(values[HipYMotorNum + offset])

    # --- KNEE X ---
    mask[KneeXMotorNum + offset] = 1
    values[KneeXMotorNum + offset] = min(params.KneeXMax, max(params.KneeXMin,
        sin(t * params.walkSpeed + params.KneeXPhase + pi) * params.KneeXWeight + params.KneeXOffset))
    otherKneeXAngle = motors[KneeXMotorNum + offset].getAngleFromNormal(values[KneeXMotorNum + offset])

    # --- ANKLE X ---
    mask[AnkleXMotornum + offset] = 1
    otherAnkleXAngle = -otherKneeXAngle + otherHipXAngle - rot[0]
    values[AnkleXMotornum + offset] = motors[AnkleXMotornum + offset].getNormalFromAngle(otherAnkleXAngle)

    # --- ANKLE Y ---
    mask[AnkleYMotornum + offset] = 1
    values[AnkleYMotornum + offset] = motors[AnkleYMotornum + offset].getNormalFromAngle(-otherHipYAngle)

    return pattern(mask=mask, values=values)