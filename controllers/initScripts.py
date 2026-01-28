from classes import *
from controller import Supervisor

# MOTOR INITITATION ----------

headJoint : Joint = Joint(name="HEAD", axes=[JointAxis(axis="Z", currentPos=False, minPos=-45, maxPos= 45)])

# Define joints for motor initiation
joints = JointCollection(
    asymmetric=[
        # Joint(name="HEAD", axes=[JointAxis(axis="Z", currentPos=False)])
    ],
    symmetric=[
        Joint(
            name="SHOULDER",
            axes=[
                JointAxis(axis="Y", currentPos=False, minPos=-75, maxPos=75),
                JointAxis(axis="Z", currentPos=False, minPos=-75, maxPos=75)
            ]
        ),
        Joint(
            name="ELBOW",
            axes=[
                JointAxis(axis="Z", currentPos=False, minPos=-5, maxPos=150)
            ]
        ),
        Joint(
            name="HIP",
            axes=[
                JointAxis(axis="X", currentPos=True, minPos=-30 , maxPos=120),
                JointAxis(axis="Y", currentPos=False,minPos=-45 , maxPos=45),
                JointAxis(axis="Z", currentPos=False,minPos=-20 , maxPos=20)
            ]
        ),
        Joint(
            name="KNEE",
            axes=[
                JointAxis(axis="X", currentPos=True, minPos=-130, maxPos=5)
            ]
        ),
        Joint(
            name="FOOT",
            axes=[
                JointAxis(axis="X", currentPos=True , minPos=-30, maxPos=45),
                JointAxis(axis="Y", currentPos=False, minPos=-20, maxPos=20)
            ]
        )
    ]
)

def InitMotors(timestep : float, loadHead : bool = False) -> List[MotorData]:
    # INIT SYMMETRIC JOINTS
    motors : List[MotorData] = []
    directions = ["LEFT", "RIGHT"]
    
    if(loadHead):
        joints.asymmetric.append(headJoint)

    for direction in directions:
        for joint in joints.symmetric:
            for axis in joint.axes:
                motor_name = f"RM_{axis.axis}_{direction}_{joint.name}"
                data = MotorData(
                    motor=Motor(motor_name),
                    currentPos=axis.currentPos,
                    maxPos=axis.maxPos * (3.14159265359/180),
                    minPos=axis.minPos * (3.14159265359/180),
                    defaultPos=(-1*(axis.minPos))/(axis.maxPos-axis.minPos)
                )

                if axis.currentPos:
                    pos_sens_name = f"PS_{axis.axis}_{direction}_{joint.name}"
                    pos_sens = PositionSensor(pos_sens_name)
                    pos_sens.enable(timestep)
                    data.positionSensor = pos_sens

                data.motor.setPosition(0.0)
                #data.motor.setVelocity(2.0)
                #data.motor.setAcceleration(2.0)
                
                motors.append(data)

    # INIT ASYMMETRIC JOINTS
    for joint in joints.asymmetric:
        for axis in joint.axes:
            motor_name = f"RM_{axis.axis}_{joint.name}"
            data = MotorData(
                motor=Motor(motor_name),
                currentPos=axis.currentPos,
                maxPos=axis.maxPos * (3.14159265359/180) ,
                minPos=axis.minPos* (3.14159265359/180),
                defaultPos=(-1*(axis.minPos))/(axis.maxPos-axis.minPos)
            )

            if axis.currentPos:
                pos_sens_name = f"PS_{axis.axis}_{joint.name}"
                pos_sens = PositionSensor(pos_sens_name)
                pos_sens.enable(timestep)
                data.positionSensor = pos_sens

            data.motor.setPosition(0.0)
            #data.motor.setVelocity(2.0)
            #data.motor.setAcceleration(2.0)
        

            motors.append(data)
    return motors


# BODY PART INITIALISATION --------

# THE BODY HAS TO BE THE FIRST ONE IN THE ASSYMETRIC COLLECTION !!!!

# Initialize body parts
bodyPartList = BodyPartCollection(
    asymmetric=[
        BodyPart(name="body", doneOnTouch=True, touchReward=None, noTouchReward=None, noTouchRewardDelay=None), # HAS TO BE THE FIRST ONE
        BodyPart(name="head", doneOnTouch=True, touchReward=None, noTouchReward=None, noTouchRewardDelay=None)
    ],
    symmetric=[
        BodyPart(name="upper_arm", doneOnTouch=True, touchReward=None, noTouchReward=None, noTouchRewardDelay=None),
        BodyPart(name="lower_arm", doneOnTouch=True, touchReward=None, noTouchReward=None, noTouchRewardDelay=None),
        BodyPart(name="hand", doneOnTouch=True, touchReward=None, noTouchReward=None, noTouchRewardDelay=None),
        BodyPart(name="upper_leg", doneOnTouch=True, touchReward=None, noTouchReward=None, noTouchRewardDelay=None),
        BodyPart(name="lower_leg", doneOnTouch=True, touchReward=None, noTouchReward=None, noTouchRewardDelay=None),
        BodyPart(name="foot", doneOnTouch=False, touchReward=0.4, noTouchReward=-1, noTouchRewardDelay=150)
    ]
)


def InitBodyParts(robotSupervisor: Supervisor,timestep: float) -> List[BodyPartData]:
    BodyParts: List[BodyPartData] = []
    # Initialize body part data (you'll need to implement this part)
    for bodyPart in bodyPartList.asymmetric:
        # You'll need to get the actual node and field references here
        #thisNode = GetNodeByName(robotNode, bodyPart.name)
        
        touchSens = TouchSensor(f"TS_{bodyPart.name.upper()}")
        touchSens.enable(timestep)
        
        thisNode : Node = robotSupervisor.getFromDevice(touchSens._tag).getParentNode()
        
        
        
        transField = thisNode.getField("translation")
        linearVelocityField = thisNode.getField("linearVelocity")
        angularVelocityField = thisNode.getField("angularVelocity")
        rotField = thisNode.getField("rotation")
        

        BodyParts.append(BodyPartData(node=thisNode, startingPosition=transField.getSFVec3f(), transField=transField, linearVelocityField=linearVelocityField, angularVelocityField=angularVelocityField, touchSensor=touchSens, doneOnTouch=bodyPart.doneOnTouch, rotField=rotField, startingRotation=rotField.getSFRotation(), noTouchReward=bodyPart.noTouchReward, touchReward=bodyPart.touchReward, noTouchRewardDelay=bodyPart.noTouchRewardDelay, lastTouched=-100))


    directions = ["left", "right"]
    for direction in directions:
        for bodyPart in bodyPartList.symmetric:
            touchSens = TouchSensor(f"TS_{direction.upper()}_{bodyPart.name.upper()}")
            touchSens.enable(timestep)
            
            thisNode : Node = robotSupervisor.getFromDevice(touchSens._tag).getParentNode()
            
            
            
            rotField = thisNode.getField("rotation")
            transField = thisNode.getField("translation")
            linearVelocityField = thisNode.getField("linearVelocity")
            angularVelocityField = thisNode.getField("angularVelocity")



            BodyParts.append(BodyPartData(node=thisNode, startingPosition=transField.getSFVec3f(), transField=transField, linearVelocityField=linearVelocityField, angularVelocityField=angularVelocityField, touchSensor=touchSens, doneOnTouch=bodyPart.doneOnTouch, rotField=rotField, startingRotation=rotField.getSFRotation(), noTouchReward=bodyPart.noTouchReward, touchReward=bodyPart.touchReward, noTouchRewardDelay=bodyPart.noTouchRewardDelay, lastTouched=-100))
    return BodyParts