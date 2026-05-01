# PARAMETERS

maxTime = 35000 # MAX STEPS AN INSTANCE CAN LIVE

# REWARD PARAMETERS
uprightRewardWeight = 4

movementPenaltyWeight = 0.07

maxTargetReward = 2.5
targetRewardFalloff = 20
targetThreshold = 0.05 # the value it is allowed to differ from the target

armTargetThreshhold = 0.15

turnRateRewardWeight = 6
walkSpeedRewardWeight = 5

verticalMovementPenaltyWeight = 3
sideMovementPenaltyWeight = 1

terminationPenalty = -700

maxStillnessReward = 15
stillnessRewardFalloff = 100

jerkPenalty = 3

upsideDownPenalty = -500