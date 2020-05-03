import math
import time
from Servocomm import *

#Initial Variables

cTibiaLengt = 75                    #Long of Leg in cm
cFemurLength = 50                   #Long of AntLeg in cm
cCoxaLength = 15                    #Long of Coxa

#Those describe position relative of the body
bodyHigh = 50                       #Long of Body from Floor in cm
bodyOffX = 0                        #Offset X form origin
bodyOffY = 0                        #Offset Y form origin

bodyRoll = 0
bodyPitch = 0
bodyYaw = 0

#Those describe movements

turnAngle = 0                       #Angle movement refering origin
moveAngle = 0                       #Angle for linear movement
moveDistance = 0                    #Distance for linear movement
timeScale = 1

state = 0                           #Stop/Walking/From walking to Stop/

homeDistance = 40                  #Inicial distance for legs

#Initial Position and Angle for Legs 
frontLegAngle =45
frontLegStart = [50, 50, 0]

middleLegAngle =0
middleLegStart = [60, 0, 0]

backLegAngle =-45
backLegStart = [50, -50, 0]

totalSteps = 60           #Amount of movement for servos Min=6 and Max=200
control_time = 20
minSteps = 10
maxSteps = 200

rad=(math.pi/180)
grad=(180/math.pi)

class BodyHex:
    def __init__(self,controller):
        self.servosCon = controller

        self.bodyHigh = bodyHigh
        self.bodyOffX = bodyOffX
        self.bodyOffY = bodyOffY

        self.angleRoll = bodyRoll
        self.anglePitch = bodyPitch
        self.angleYaw = bodyYaw

        self.time = control_time
        self.stepSpeedMaxMin = [ minSteps, maxSteps]
        self.stepTimeMaxMin = [ 1, 20]
        
        self.FR  = leg(frontLegStart, frontLegAngle,False)
        self.MR  = leg(middleLegStart, middleLegAngle, False)
        self.BR  = leg(backLegStart, backLegAngle, False)

        self.FL  = leg(frontLegStart, frontLegAngle, True)
        self.ML  = leg(middleLegStart, middleLegAngle, True)
        self.BL  = leg(backLegStart, backLegAngle, True)

        self.legs = [self.FR,
                     self.MR,
                     self.BR,
                     self.FL,
                     self.ML,
                     self.BL]
        
        self.tripodR = [self.FR,self.MR,self.BR]
        self.tripodL = [self.FL,self.ML,self.BL]
        self.BodyIK()

    def updateRPY(self, newAngleRoll, newAnglePitch, newAngleYaw):
        self.angleRoll = newAngleRoll
        self.anglePitch = newAnglePitch
        self.angleYaw = newAngleYaw
        self.BodyIK()
        
    def BodyIK(self):
        SA = math.sin(self.anglePitch*rad)
        CA = math.cos(self.anglePitch*rad)

        SB = math.sin(self.angleRoll*rad)
        CB = math.cos(self.angleRoll*rad)

        SG = math.sin(self.angleYaw*rad)
        CG = math.cos(self.angleYaw*rad)

        for i in range(len(self.legs)):
            BodyIKPosX = ((self.legs[i].homePosition[1]*(SB*SA*CG + CB*SG))
                         +(self.legs[i].homePosition[0]*(SB*SA*SG + CB*CG))
                         -(self.legs[i].homePosition[2]*SB*CA))

            BodyIKPosY = ((self.legs[i].homePosition[1]*CA*CG)
                         -(self.legs[i].homePosition[0]*CA*SG)
                         +(self.legs[i].homePosition[2]*SA))

            BodyIKPosZ = ((self.legs[i].homePosition[1]*(SB*SG - CB*SA*CG))
                         +(self.legs[i].homePosition[0]*(CB*SA*SG + SB*CG))
                         +(self.legs[i].homePosition[2]*CB*CA))
            self.legs[i].updateHomePosition(BodyIKPosX, BodyIKPosY, BodyIKPosZ)

class leg():
    def __init__(self, Position, Rotation, Lflip):
        global rad, grad, cFemurLength, cTibiaLengt, cCoxaLength, homeDistance
        self.Femur = cFemurLength
        self.Tibia = cTibiaLengt
        self.Coxa = cCoxaLength
        self.flip = Lflip
        self.solution = True
        self.warning = False
        self.Error = False
        self.estateactive=True
        self.CoxaAngle = 0
        self.FemurAngle = 0
        self.TibiaAngle = 0
        self.servoAngles = [0, 0, 0]
        self.homePoint = [0, 0, 0]
        self.homeDistance = homeDistance
        self.homePosition = [0, 0, 0]
        if (Lflip == True):
            self.homePosition[0] = -Position[0]
        else:
            self.homePosition[0] = Position[0]
        self.homePosition[1] = Position[1]
        self.homePosition[2] = Position[2]
        self.startedPosition = self.homePosition
        self.updateHomePoint(self.homeDistance, 0, Rotation)
        self.homeAnglePoint = (math.atan2(self.homePoint[1], self.homePoint[0]))*grad
        self.homeRadiusPoint = math.sqrt((self.homePoint[0]*self.homePoint[0])+(self.homePoint[1]*self.homePoint[1])+(self.homePoint[2]*self.homePoint[2]))

    def isactive(self):
        return self.estateactive

    def active(self):
        self.estateactive=True

    def disactive(self):
        self.estateactive=False
    
    def IK2Servo(self):
        if (self.flip == True):
            self.servoAngles[0] = (90 + self.CoxaAngle)
            self.servoAngles[1] = (90 + self.FemurAngle)
            self.servoAngles[2] = (180 + self.TibiaAngle)
        else:
            self.servoAngles[0] = (90 - self.CoxaAngle)
            self.servoAngles[1] = (90 - self.FemurAngle)
            self.servoAngles[2] = (- self.TibiaAngle)

    def updateHomePoint(self, newHomeDistance, newBodyHigh, Rotation):
        if (self.flip == True):
            Rotation = 180 - Rotation
        dirX = math.cos(Rotation*rad)
        dirY = math.sin(Rotation*rad)
        self.homePoint[0] = self.homePosition[0] + (newHomeDistance * dirX)
        self.homePoint[1] = self.homePosition[1] + (newHomeDistance * dirY)
        self.homePoint[2] = -newBodyHigh
        self.LegIK(self.homePoint[0], self.homePoint[1], self.homePoint[2])

    def updateHomePosition(self, posX, posY, posZ):
        self.homePosition[0] = posX
        self.homePosition[1] = posY
        self.homePosition[2] = posZ
        self.LegIK(self.homePoint[0], self.homePoint[1], self.homePoint[2])

    def LegIK(self, IKPosX0, IKPosY0, IKPosZ0):
        global rad,grad
        IKPosX = IKPosX0 - self.homePosition[0]
        IKPosY = IKPosY0 - self.homePosition[1]
        IKPosZ = IKPosZ0 - self.homePosition[2]
        Temp1 = (math.atan2(IKPosY, IKPosX))*grad
        if (self.flip == True):
            if (Temp1 >= 0):
                self.CoxaAngle = 180 - (Temp1)
            else:
                self.CoxaAngle = -180 - (Temp1)
        else:
            self.CoxaAngle = Temp1
        xL = math.sqrt((IKPosX*IKPosX) + (IKPosY*IKPosY)) - self.Coxa
        L = math.sqrt((xL*xL) + (IKPosZ*IKPosZ))
        Alfa1 = math.atan2(xL, abs(IKPosZ))

        Temp1 = (((self.Femur*self.Femur) - (self.Tibia*self.Tibia)) + (L*L))
        Temp2 = (2*self.Femur) * L
        Alfa2 = math.acos(Temp1/Temp2)
        self.FemurAngle = ((Alfa2 + Alfa1)*grad) - 90

        Temp1 = (((self.Femur*self.Femur) + (self.Tibia*self.Tibia)) - (L*L))
        Temp2 = (2*self.Femur*self.Tibia)
        self.TibiaAngle = ((math.acos(Temp1/Temp2))*grad - 180)

        self.solution = False
        self.warning = False
        self.Error = False
    
        if(xL < (self.Femur+self.Tibia-5)):
            self.solution = True
        else:
            if(xL < (self.Femur+self.Tibia)):
                self.warning = True
            else:
                self.Error = True
        self.IK2Servo()

#Testing Body
controller = Controller()
Robot = BodyHex(controller)
print("Printing Angles")
for i in range(len(Robot.legs)):
    Robot.servosCon.set_leg_angles(i, Robot.legs[i].servoAngles)
    print("Leg {0}".format(i))
    print("[{0}, {1}, {2}]".format(Robot.legs[i].CoxaAngle, Robot.legs[i].FemurAngle, Robot.legs[i].TibiaAngle))
    print(Robot.legs[i].servoAngles)


#Robot.updateRPY(0, 0, -45)


''' #Testing only Leg
leg1 = leg(frontLegStart, 45, True)
print("Angles")
print(leg1.CoxaAngle)
print(leg1.FemurAngle)
print(leg1.TibiaAngle)
print("HomePosition")
print(leg1.homePosition)
print("HomePoint")
print(leg1.homePoint)
print("HomeAngle")
print(leg1.homeAngle)
print("HomeRadius")
print(leg1.homeRadius)
leg1.updateHomePosition(frontLegStart[0], frontLegStart[1], -30)
#leg1.LegIK(leg1.homePoint[0], leg1.homePoint[1], leg1.homePoint[2])
print("Angles*")
print(leg1.CoxaAngle)
print(leg1.FemurAngle)
print(leg1.TibiaAngle)
print("HomePosition*")
print(leg1.homePosition)
print("HomePoint*")
print(leg1.homePoint)
print("HomeAngle*")
print(leg1.homeAngle)
print("HomeRadius*")
print(leg1.homeRadius)
'''