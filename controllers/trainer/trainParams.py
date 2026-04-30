# PARAMETERS

maxTime = 35000 # MAX STEPS AN INSTANCE CAN LIVE

# REWARD PARAMETERS
uprightRewardWeight = 2

movementPenaltyWeight = 0.07

maxTargetReward = 2.5
targetRewardFalloff = 20
targetThreshold = 0.05 # the value it is allowed to differ from the target

armTargetThreshhold = 0.15

turnRateRewardWeight = 7
walkSpeedRewardWeight = 8

verticalMovementPenaltyWeight = 0.3
sideMovementPenaltyWeight = 0.3

terminationPenalty = -700

maxStillnessReward = 15
stillnessRewardFalloff = 100

jerkPenalty = 3