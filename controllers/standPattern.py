from patternGenerator import pattern
from parameters import HipXMotorNum, HipYMotorNum, KneeXMotorNum, AnkleXMotornum, AnkleYMotornum
from classes import MotorData

from math import sin, pi

from typing import Tuple, List
from dataclasses import dataclass


@dataclass
class standParameters:
    turnRate: float = 0
    
    defaultArmPos : float = True # sets the arms to their default position so that the NN doesnt do weird shit
    
    walkSpeed : float= 5.5
    
    HipXPhase  = 0
    HipXWeight = 0.25
    HipXOffset = 0.5
    HipXMin = 0.3
    HipXMax = 0.51

    HipYPhase  = 0
    HipYWeight = 0.05
    HipYOffset = 0.50
    HipYMin = 0.47
    HipYMax = 0.53

def evaluateStand(
    params: standParameters,
    timestep: int,
    rot: Tuple[float, float, float],
    motors : List[MotorData]
) -> pattern:
    mask: List[float] = [0 for _ in range(18)]
    values: List[float] = [-1 for _ in range(18)]
    
    if(params.turnRate == 0):
        offset = 9  # right leg offset

        # ================= LEFT LEG =================

        # HIP X
        mask[HipXMotorNum] = 1
        values[HipXMotorNum] = motors[HipXMotorNum].defaultPos

        # HIP Y
        mask[HipYMotorNum] = 1
        values[HipYMotorNum] = motors[HipYMotorNum].defaultPos


        # KNEE X
        mask[KneeXMotorNum] = 1
        values[KneeXMotorNum] = motors[KneeXMotorNum].defaultPos

        # ANKLE X
        mask[AnkleXMotornum] = 1
        values[AnkleXMotornum] = motors[AnkleXMotornum].defaultPos

        # ANKLE Y
        mask[AnkleYMotornum] = 1
        values[AnkleYMotornum] = motors[AnkleYMotornum].defaultPos

        # ================= RIGHT LEG =================

        # HIP X
        mask[HipXMotorNum + offset] = 1
        values[HipXMotorNum + offset] = motors[HipXMotorNum + offset].defaultPos

        # HIP Y
        mask[HipYMotorNum + offset] = 1
        values[HipYMotorNum + offset] = motors[HipYMotorNum + offset].defaultPos

        # KNEE X
        mask[KneeXMotorNum + offset] = 1
        values[KneeXMotorNum + offset] = motors[KneeXMotorNum + offset].defaultPos

        # ANKLE X
        mask[AnkleXMotornum + offset] = 1
        values[AnkleXMotornum + offset] = motors[AnkleXMotornum + offset].defaultPos

        # ANKLE Y
        mask[AnkleYMotornum + offset] = 1
        values[AnkleYMotornum + offset] = motors[AnkleYMotornum + offset].defaultPos

    else:
        t = timestep / 1000

        # ================= LEFT LEG =================

        # --- HIP X ---
        mask[HipXMotorNum] = 1
        values[HipXMotorNum] = min(params.HipXMax, max(params.HipXMin,
            sin(t * params.walkSpeed + params.HipXPhase) * params.HipXWeight + params.HipXOffset))
        hipXAngle = motors[HipXMotorNum].getAngleFromNormal(values[HipXMotorNum])

        # --- HIP Y ---
        mask[HipYMotorNum] = 1
        values[HipYMotorNum] = min(params.HipYMax, max(params.HipYMin,
            sin(t * params.walkSpeed + params.HipYPhase) * params.HipYWeight + params.HipYOffset))
        hipYAngle = motors[HipYMotorNum].getAngleFromNormal(values[HipYMotorNum])

        # --- KNEE X ---
        mask[KneeXMotorNum] = 1
        kneeXAngle = -hipXAngle
        values[KneeXMotorNum] = motors[KneeXMotorNum].getNormalFromAngle(kneeXAngle)

        # --- ANKLE X ---
        mask[AnkleXMotornum] = 1
        ankleXAngle = -hipXAngle - kneeXAngle - rot[0]
        values[AnkleXMotornum] = motors[AnkleXMotornum].getNormalFromAngle(ankleXAngle)

        # --- ANKLE Y ---
        mask[AnkleYMotornum] = 1
        values[AnkleYMotornum] = motors[AnkleYMotornum].getNormalFromAngle(hipYAngle)


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
        otherKneeXAngle = otherHipXAngle
        values[KneeXMotorNum + offset] = motors[KneeXMotorNum + offset].getNormalFromAngle(otherKneeXAngle)


        # --- ANKLE X ---
        mask[AnkleXMotornum + offset] = 1
        otherAnkleXAngle = -otherKneeXAngle + otherHipXAngle - rot[0]
        values[AnkleXMotornum + offset] = motors[AnkleXMotornum + offset].getNormalFromAngle(otherAnkleXAngle)

        # --- ANKLE Y ---
        mask[AnkleYMotornum + offset] = 1
        values[AnkleYMotornum + offset] = motors[AnkleYMotornum + offset].getNormalFromAngle(-otherHipYAngle)

    return pattern(mask=mask, values=values, canTouchGround=False, noTouchReward=True, sidePenalty=True, stillnessReward=True, touchReward=True, turnRateReward=True, uprightReward=True, verticalPenalty=True, walkSpeedReward=False, turnRate = params.turnRate)