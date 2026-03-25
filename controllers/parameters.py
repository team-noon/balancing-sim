import math

# constants
ShoulderYMotornum = 0
ShoulderZMotornum = 1

ElbowZMotornum = 2

HipXMotorNum = 3
HipYMotorNum = 4
HipZMotorNum = 5

KneeXMotorNum = 6

AnkleXMotornum = 7
AnkleYMotornum = 8

NeckMotornum = 18

# MOTOR PARAMETERS
servoTorque = 20 * 10 * 9.81 # in mNm
servoSpeed = 39/50 * math.pi # in rad/sec

brushlessTorque =  30000 #13750 / 3 # in mNm
brushlessSpeed = 3 * math.pi # in rad/sec