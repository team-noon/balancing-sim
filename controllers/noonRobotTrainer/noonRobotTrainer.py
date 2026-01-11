"""noonRobotTrainer controller."""

import math
from controller import Supervisor, InertialUnit, Gyro, Accelerometer, Node
from typing import List, Tuple, Dict, Any
import numpy as np
from classes import BodyPartData, MotorData
from initScripts import InitBodyParts, InitMotors
import socket
import sys



HOST = "127.0.0.1"
BASEPORT = 9876

robot = Supervisor()
worldInfoInfoField = robot.getFromDef("WorldInfo").getField("info")
timestep = int(robot.getBasicTimeStep())

if robot.getName() == "trainer":
    if sys.argv.__len__() == 3:
        env_num = int(sys.argv[1])
        NUM_ROBOTS = int(sys.argv[2])

        worldInfoInfoField.insertMFString(0, f"{env_num}")
        worldInfoInfoField.insertMFString(1, f"{NUM_ROBOTS}")




        

        robot.getFromDef("TRAINER").getField("count").setSFInt32(NUM_ROBOTS)
        robot.simulationReset()

    robot.getSelf().remove()
    robot.step(timestep)
    exit()


robotSelf : Node = robot.getSelf()

if(robot.getName() != "trainer" and  robot.getName() != "noonRobot"):

    robotSelf = robot.getFromDef("TRAINER").getFromProtoDef(f"NOONROBOT_{robot.getName()}")



# PARAMETERS

maxSteps = 40000 # MAX STEPS AN INSTANCE CAN LIVE

# REWARD PARAMETERS
uprightRewardWeight = 1

movementPenaltyWeight = 0.02

turnRateRewardWeight = 10
walkSpeedRewardWeight = 10


# MOTOR PARAMETERS
servoTorque = 20 * 10 * 9.81 # in mNm
servoSpeed = 39/50 * math.pi # in rad/sec

brushlessTorque =  30000 #13750 / 3 # in mNm
brushlessSpeed = 3 * math.pi # in rad/sec


BodyParts: List[BodyPartData] = InitBodyParts(robotSupervisor=robot, timestep=timestep)


motors: List[MotorData] = InitMotors(timestep=timestep)



gyro = Gyro(name="BODY_GYRO", sampling_period=timestep)
gyro.enable(timestep)
accelerometer = Accelerometer(name="BODY_ACCELEROMETER",sampling_period=timestep)
accelerometer.enable(timestep)
inertialUnit = InertialUnit(name="BODY_INERTIALUNIT", sampling_period=timestep)
inertialUnit.enable(timestep)

turnRate = 0.3
walkSpeed = 0.3
            
def getObservationSpace() -> np.ndarray:
    ret : list[float]= []
    for motor in motors:
        if motor.currentPos and motor.positionSensor:
            ret.append(motor.positionSensor.getValue())
        else:
            ret.append(motor.motor.getTargetPosition())
    
    ret.extend(gyro.getValues())
    ret.extend(accelerometer.getValues())
    ret.extend(inertialUnit.getRollPitchYaw())
    
    ret.extend([turnRate, walkSpeed])
    return np.asarray(ret, dtype=np.float32)


stepsSinceReset = 0


prevActions: np.ndarray = np.zeros((18,), dtype=np.float32)

def step(action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
    global prevActions, stepsSinceReset, turnRate, walkSpeed
    
    reward = 0
    terminated = False
    truncated = False
    
    robot.step(timestep)


    
    
    # applies the actions to the motors
    
    i = 0
    for curAction in action:
        pos = ((curAction) * ((motors[i].maxPos) - (motors[i].minPos))) + motors[i].minPos
        motors[i].motor.setPosition(pos)
        
        if(motors[i].currentPos):
            motors[i].motor.setVelocity(brushlessSpeed)
            motors[i].motor.setAcceleration(10)
            motors[i].motor.setAvailableTorque(brushlessTorque/1000)
        else:
            motors[i].motor.setVelocity(servoSpeed)
            motors[i].motor.setAcceleration(10)
            motors[i].motor.setAvailableTorque(servoTorque/1000)

        reward -= movementPenaltyWeight*((curAction - prevActions[i]) ** 2)
        
        i+=1
        
    prevActions = action
    
    # checks if any parts of the body that shouldnt be is touching the floor
    for bodyPart in BodyParts:
        touch = bodyPart.touchSensor.getValue()
        if touch != 0 and bodyPart.doneOnTouch:
            terminated = True
            reward = -10
            break
        
        if touch !=0 and bodyPart.touchReward:
            bodyPart.lastTouched = stepsSinceReset
            reward += bodyPart.touchReward
            
        if touch == 0 and bodyPart.noTouchReward and stepsSinceReset - bodyPart.lastTouched > bodyPart.noTouchRewardDelay:
            reward += bodyPart.noTouchReward
            
            
        
    # truncates the robot if it reaches a specified limit of steps
    if(stepsSinceReset >= maxSteps):
        truncated=True
        
            
    # calcualate reward based on how upright it is *
    
    
    
    bodyRot =  robotSelf.getOrientation()
    reward += max(-1, bodyRot[8] * uprightRewardWeight) # beatufiul line of code
    
    #print("rotation reward: ", max(-1, 1 - math.acos(bodyRot[8]) * uprightRewardWeight))
    
    
    # calculate reward based on turnspeed 
    vel = robotSelf.getVelocity()
    
    bodyAngVelocity = vel[3:]
    reward += max(-1, 1 - abs(turnRate - bodyAngVelocity[2]) * turnRateRewardWeight)
    #print("rotation: ", bodyAngVelocity[2], "reward:", max(-1, 1 - abs(turnRate - bodyAngVelocity[2]) * turnRateRewardWeight))
    
    
    # calculate reward based on walkspeed
    bodyLinVelocityVector = vel[:3]
    bodyVelocityMagnitude = bodyLinVelocityVector[0]*bodyRot[3] + bodyLinVelocityVector[1]*bodyRot[4] 
    reward += max(-1, 1 - abs(walkSpeed - bodyVelocityMagnitude) * turnRateRewardWeight)
    
    #print("velocity: ", bodyVelocityMagnitude, "reward:", max(-1, 1 - abs(walkSpeed - bodyVelocityMagnitude) * turnRateRewardWeight))
    

    
    observation = getObservationSpace()
    
    stepsSinceReset+=timestep
    

    info = {}
    return observation, reward, terminated, truncated, info

robotSelf.saveState(robotSelf.getDef())

def reset(seed=None, options=None)-> tuple[np.ndarray, dict]:
    global stepsSinceReset, prevActions
    
    global motors, BodyParts, turnRate, walkSpeed

    prevActions = np.zeros((18,), dtype=np.float32)
    
    robotSelf.loadState(robotSelf.getDef())    
    
    stepsSinceReset = 0
    
    obs = getObservationSpace()
    
    # increment size
    step_size = 0.01  

    # ----- walk speed range: -0.05 to +0.20 -----
    walk_min = -0.05
    walk_max =  0.20
    walk_steps = int((walk_max - walk_min) / step_size) + 1
    walkSpeed = np.random.choice([walk_min + i * step_size for i in range(walk_steps)])

    # ----- turn rate range: -0.20 to +0.20 -----
    turn_min = -0.20
    turn_max =  0.20
    turn_steps = int((turn_max - turn_min) / step_size) + 1
    turnRate = np.random.choice([turn_min + i * step_size for i in range(turn_steps)])

        
    info = {}
    return obs, info


# INFERENCE
if(robotSelf.getField("inference").getSFBool()): 
    from stable_baselines3 import PPO
    import gymnasium as gym
    env = gym.Env()

    env.action_space = gym.spaces.Box(low=0, high=1,shape=(18,), dtype=np.float32)

    env.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(29,), dtype=np.float32)
    env.reset = reset
    env.step = step
    

    
    model : PPO
    try:
        model = PPO.load("../../models/continue", env=env, device="cpu")
        print("Sucessfully imported PPO for inference")


        model.policy.eval()
        obs, info = env.reset()
        while True:
            # model.predict already runs under torch.no_grad internally
            # ensure observation is a numpy array (stable-baselines3 expects ndarray)
            obs_array = np.asarray(obs, dtype=np.float32)
            action, _ = model.predict(obs_array, deterministic=True)
            i = 0
            while(i < action.__len__()):
               #action[i] = 0
               i += 1
            obs, reward, terminated, truncated, info = env.step(action)
            if False and terminated or truncated:
                obs, info = env.reset()
    except:
        
        raise Exception("cant run inference, there is no model, put one into the models folder as continue.zip")

rank = int(worldInfoInfoField.getMFString(0)) * int(worldInfoInfoField.getMFString(1)) + int(robotSelf.getField("name").getSFString())


thisSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

thisSocket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

while True:
    try:
        thisSocket.connect((HOST, BASEPORT + rank))
        break
    except:
        pass
    






obs_buffer = np.empty(29, dtype=np.float32)
action_buffer = np.empty(18, dtype=np.float32)

obs_buffer, info = reset()

# TRAINING LOOP
while True:
    data = thisSocket.recv(1)
    if data == b'r':
        obs, info = reset()
        packet = obs.tobytes()
        
        thisSocket.sendall(packet)
    elif data== b's':
        action_buffer = np.frombuffer(thisSocket.recv(18*4), dtype=np.float32)
        obs_buffer, reward, terminated, truncated, info= step(action_buffer)
        
        reward32 = np.float32(reward)
        terminated8 = np.int8(terminated)
        truncated8 = np.int8(truncated)

        # Build a single contiguous byte buffer
        packet = memoryview(obs_buffer).tobytes() + reward32.tobytes() + terminated8.tobytes() + truncated8.tobytes()

        thisSocket.sendall(packet)
        
        
    pass




