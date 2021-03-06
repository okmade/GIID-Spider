import math
import time
from Servocomm import *
#from Module_draw import *

#Initial Variables

cTibiaLengt = 97                    #Long of Leg in cm
cFemurLength = 43                   #Long of AntLeg in cm
cCoxaLength = 28                    #Long of Coxa

#Those describe position relative of the body

bodyHigh = 70                       #Long of Body from Floor in cm
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
timeScaleBody = 0.8
state = 0                           #Stop/Walking/From walking to Stop/
homeDistance = 71                   #Inicial distance for legs
stepHeight = 30
stepPower = 3
stepTime = 1
stepTimeBody = 1

#Initial Position and Angle for Legs 
 
frontLegAngle =  45
frontLegStart =  [40, 86, 0]
middleLegAngle = 0
middleLegStart = [62,  0, 0]
backLegAngle =   -45
backLegStart =   [40,-86, 0]

totalSteps = 60           #Amount of movement for servos Min=6 and Max=200
control_time = 20
minSteps = 10
maxSteps = 200

rad = (math.pi/180)
grad = (180/math.pi)

gait = 0
gaitBeta =      [ 6, 6, 3, 3, 4, 2, 6]
           #[?, millipede-like, fly-like, ?, ?, ant-like]
gaitPath = [[3,2,1,6,5,4],
            [1,3,5,2,4,6],
            [2,1,3,3,2,1],
            [3,2,1,2,1,3],
            [2,1,4,4,3,2],
            [2,1,2,1,2,1],
            [6,6,6,6,6,6]]

'''
gaitPath[0] =   [3,2,1,6,5,4]
                #[3,0,0,0,1,0,0],     #L3     ?
                #[2,0,0,0,0,1,0],     #L2
                #[1,0,0,0,0,0,1],     #L1
                #[6,1,0,0,0,0,0],     #R3
                #[5,0,1,0,0,0,0],     #R2
                #[4,0,0,1,0,0,0]]     #R1

gaitPath[1] =   [1,3,5,2,4,6]
                #[1,0,0,0,0,0,1],     #L3     millipede-like
                #[3,0,0,0,1,0,0],     #L2
                #[5,0,1,0,0,0,0],     #L1
                #[2,0,0,0,0,1,0],     #R3
                #[4,0,0,1,0,0,0],     #R2
                #[6,1,0,0,0,0,0]]     #R1

gaitPath[2] =   [2,1,3,3,2,1]
                #[2,0,1,0,0,1,0],     #L3     fly-like
                #[1,0,0,1,0,0,1],     #L2
                #[3,1,0,0,1,0,0],     #L1
                #[3,1,0,0,1,0,0],     #R3
                #[2,0,1,0,0,1,0],     #R2
                #[1,0,0,1,0,0,1]]     #R1

gaitPath[3] =   [3,2,1,2,1,3]
                #[3,1,0,0,1,0,0],     #L3     ?
                #[2,0,1,0,0,1,0],     #L2
                #[1,0,0,1,0,0,1],     #L1
                #[2,0,1,0,0,1,0],     #R3
                #[1,0,0,1,0,0,1],     #R2
                #[3,1,0,0,1,0,0]]     #R1

gaitPath[4] =   [2,1,4,4,3,2]
                #[2,0,0,1,0,0,0],     #L3     ?
                #[1,0,0,0,1,0,0],     #L2
                #[4,1,0,0,0,1,0],     #L1
                #[4,1,0,0,0,1,0],     #R3
                #[3,0,1,0,0,0,1],     #R2
                #[2,0,0,1,0,0,0]]     #R1

gaitPath[5] =   [2,1,2,1,2,1]
                #[2,1,0,1,0,1,0],     #L3     ant-like
                #[1,0,1,0,1,0,1],     #L2
                #[2,1,0,1,0,1,0],     #L1
                #[1,0,1,0,1,0,1],     #R3
                #[2,1,0,1,0,1,0],     #R2
                #[1,0,1,0,1,0,1]]     #R1

gaitPath[6] =   [1,2,3,4,5,6]
                #[1,0,0,0,0,0,0],     #L3   Base
                #[2,0,0,0,0,0,0],     #L2
                #[3,0,0,0,0,0,0],     #L1
                #[4,0,0,0,0,0,0],     #R3
                #[5,0,0,0,0,0,0],     #R2
                #[6,0,0,0,0,0,0]]     #R1
'''

class BodyHex:
    def __init__(self,controller):
        self.servosCon = controller

        self.bodyHigh = bodyHigh
        self.bodyOffX = bodyOffX
        self.bodyOffY = bodyOffY
        self.bodyHighEnd = bodyHigh
        self.bodyOffXEnd = bodyOffX
        self.bodyOffYEnd = bodyOffY
        self.bodyHighIni = 0
        self.bodyOffXIni = 0
        self.bodyOffYIni = 0

        self.angleRoll = bodyRoll
        self.anglePitch = bodyPitch
        self.angleYaw = bodyYaw
        self.angleRollEnd = bodyRoll
        self.anglePitchEnd = bodyPitch
        self.angleYawEnd = bodyYaw
        self.angleRollIni = 0
        self.anglePitchIni = 0
        self.angleYawIni = 0

        self.stepTime = stepTime
        self.timeScale = timeScale
        self.currentTime = 0
        self.stepTimeBody = stepTimeBody
        self.timeScaleBody = timeScaleBody
        self.currentTimeBody = 0

        self.state = 0
        self.activeGait = gait
        self.newActiveGait = gait
        self.stepGaitBeta = 0
        self.activeGaitBeta = gaitBeta[gait]
        self.dir = True
        self.isHome = True
        self.isHomeBody = False
        self.goHome = False

        self.moveAngle = moveAngle
        self.moveDistance = moveDistance
        self.turnAngle = turnAngle

        self.time = control_time
        self.stepSpeedMaxMin = [ minSteps, maxSteps]
        self.stepTimeMaxMin = [ 1, 20]
        
        self.FR  = leg(frontLegStart, frontLegAngle,False)
        self.MR  = leg(middleLegStart, middleLegAngle, False)
        self.BR  = leg(backLegStart, backLegAngle, False)

        self.FL  = leg(frontLegStart, frontLegAngle, True)
        self.ML  = leg(middleLegStart, middleLegAngle, True)
        self.BL  = leg(backLegStart, backLegAngle, True)

        self.legs = [self.FR, self.MR, self.BR,
                     self.FL, self.ML, self.BL]
        #self.tripodR = [self.FR,self.MR,self.BR]       # No implemented
        #self.tripodL = [self.FL,self.ML,self.BL]       # No implemented
        self.TPCamera = TPCamera(controller, 90,90)
        self.BodyIK(self.timeScaleBody)
    
    def setInit(self):
        if (self.state == 0):
            for x in range(len(self.legs)):
                self.legs[x].resetMov([0,0,0],0)
                self.legs[x].updateMov(0)
                self.servosCon.set_leg_angles(x, self.legs[x].servoAngles)
            self.moveDistance = 0

    def update(self, delta):
        dirMove  = [0, 0]
        if (self.currentTime != self.stepTime):            
            self.currentTime = self.currentTime + (delta/self.timeScale)

        if (self.state == 0):
            if (self.activeGait != self.newActiveGait):
                self.activeGait = self.newActiveGait
                self.activeGaitBeta = gaitBeta[self.newActiveGait]

            if (self.moveDistance != 0 or self.turnAngle != 0):
                self.currentTime = 0
                self.stepGaitBeta = 0
                self.state = 1
                #print("State 1")
            else:
                return
        if (self.state == 1):
            dirMove = [self.moveDistance * math.sin(self.moveAngle*rad),
                        self.moveDistance * math.cos(self.moveAngle*rad)]
            for i in range(0, 6):
                auxDirFactor0 = (gaitPath[self.activeGait][i] - 1) / (self.activeGaitBeta - 1)
                auxDirFactor0X = (2*dirMove[0]*auxDirFactor0) - dirMove[0]
                auxDirFactor0Y = (2*dirMove[1]*auxDirFactor0) - dirMove[1]
                auxDirFactorAng = (2*self.turnAngle*auxDirFactor0) - self.turnAngle
                self.legs[i].resetMov([auxDirFactor0X,auxDirFactor0Y,auxDirFactorAng],1)
            self.dir = True

        if (self.currentTime == self.stepTime):
            self.currentTime = 0

            if (self.state == 1):
                if (self.stepGaitBeta >= (self.activeGaitBeta - 1)):
                    self.state = 2
                    #print("State 2")
                else:
                    self.stepGaitBeta = self.stepGaitBeta + 1
            if (self.state == 2):
                if (self.stepGaitBeta >= (self.activeGaitBeta - 1)):
                    self.stepGaitBeta = 0
                    if (self.moveDistance > 0):
                        self.dir = True
                    else:
                        self.dir = False
                    if (self.moveDistance == 0 and self.turnAngle == 0):
                        for i in range(0, 6):
                            auxDirFactor0 = (gaitPath[self.activeGait][i] - 1) / (self.activeGaitBeta - 1)
                            auxDirFactor0X = (2*self.legs[i].targetMoveVec[0]*auxDirFactor0) - self.legs[i].targetMoveVec[0]
                            auxDirFactor0Y = (2*self.legs[i].targetMoveVec[1]*auxDirFactor0) - self.legs[i].targetMoveVec[1]
                            auxDirFactorAng = (2*self.legs[i].targetTurnAngle*auxDirFactor0) - self.legs[i].targetTurnAngle
                            self.legs[i].resetMov([auxDirFactor0X,auxDirFactor0Y,auxDirFactorAng],-1)
                        self.dir = True
                        self.state = 3
                        #print("State 3")
                    else:
                        dirMove = [self.moveDistance * math.sin(self.moveAngle*rad),
                                   self.moveDistance * math.cos(self.moveAngle*rad)]
                        for i in range(0, 6):
                            self.legs[i].resetMov([dirMove[0],dirMove[1], self.turnAngle])
                else:
                    self.stepGaitBeta = self.stepGaitBeta + 1
            elif (self.state == 3):
                if (self.stepGaitBeta >= (self.activeGaitBeta - 1)):
                    self.state = 0
                    self.stepGaitBeta = 0
                    #print("State 0")
                    return
                else:
                    self.stepGaitBeta = self.stepGaitBeta + 1

        if (self.currentTime > self.stepTime):
            self.currentTime = self.stepTime

        deltaTime = self.currentTime / self.stepTime

        if (not(self.dir)):
            deltaTime = 1 - deltaTime
            auxStepGaitBeta = ((self.activeGaitBeta - 1) - self.stepGaitBeta)
        else:
            auxStepGaitBeta = self.stepGaitBeta

        for i in range(0, 6):
            auxBeta0 = ((gaitPath[self.activeGait][i] - 1) + auxStepGaitBeta)
            if (auxBeta0 >= self.activeGaitBeta):
                auxBeta0 = auxBeta0 - self.activeGaitBeta
            auxBeta1 = auxBeta0 / (self.activeGaitBeta - 1)
            if (auxBeta1 != 1):
                auxDeltaTime = ((deltaTime + auxBeta0) / (self.activeGaitBeta - 1))
                self.legs[i].dirFactor = -1
                if ((self.state == 1) or (self.state == 3)):
                    auxDeltaTime = None
                self.legs[i].isUp = False
            else:
                auxDeltaTime = deltaTime
                self.legs[i].isUp = True
                self.legs[i].dirFactor = 1
            
            self.legs[i].updateMov(auxDeltaTime)

    def updateOffset(self, newBodyHigh, newBodyOffX, newBodyOffY):        
        self.bodyHigh = newBodyHigh
        self.bodyOffX = newBodyOffX
        self.bodyOffY = newBodyOffY
        #self.BodyIK()

    def updateRPY(self, newAngleRoll, newAnglePitch, newAngleYaw):
        self.angleRoll = newAngleRoll
        self.anglePitch = newAnglePitch
        self.angleYaw = newAngleYaw
        #self.BodyIK()
    
    def BodyIK(self, delta):
        self.currentTimeBody = self.currentTimeBody + (delta/self.timeScale)

        if (self.isHomeBody):
            if (self.anglePitch != self.anglePitchEnd or
            self.angleRoll != self.angleRollEnd or
            self.angleYaw != self.angleYawEnd or
            self.bodyOffX != self.bodyOffXEnd or
            self.bodyOffY != self.bodyOffYEnd or
            self.bodyHigh != self.bodyHighEnd):
                self.anglePitchIni = self.anglePitchEnd
                self.angleRollIni = self.angleRollEnd
                self.angleYawIni = self.angleYawEnd
                self.bodyOffXIni = self.bodyOffXEnd
                self.bodyOffYIni = self.bodyOffYEnd
                self.bodyHighIni = self.bodyHighEnd
                self.anglePitchEnd = self.anglePitch
                self.angleRollEnd = self.angleRoll
                self.angleYawEnd = self.angleYaw
                self.bodyOffXEnd = self.bodyOffX
                self.bodyOffYEnd = self.bodyOffY
                self.bodyHighEnd = self.bodyHigh
                self.currentTimeBody = 0
                self.isHomeBody = False
            else:
                return

        deltaTime = self.currentTimeBody / self.stepTimeBody
        deltaAnglePitch =   ((self.anglePitchEnd - self.anglePitchIni) * deltaTime) + self.anglePitchIni
        deltaAngleRoll =    ((self.angleRollEnd - self.angleRollIni) * deltaTime) + self.angleRollIni
        deltaAngleYaw =     ((self.angleYawEnd - self.angleYawIni) * deltaTime) + self.angleYawIni
        deltaBodyOffX =     ((self.bodyOffXEnd - self.bodyOffXIni) * deltaTime) + self.bodyOffXIni
        deltaBodyOffY =     ((self.bodyOffYEnd - self.bodyOffYIni) * deltaTime) + self.bodyOffYIni
        deltaBodyHigh =     ((self.bodyHighEnd - self.bodyHighIni) * deltaTime) + self.bodyHighIni

        SA = math.sin(deltaAnglePitch*rad)
        CA = math.cos(deltaAnglePitch*rad)
        SB = math.sin(deltaAngleRoll*rad)
        CB = math.cos(deltaAngleRoll*rad)
        SG = math.sin(deltaAngleYaw*rad)
        CG = math.cos(deltaAngleYaw*rad)

        for i in range(len(self.legs)):
            BodyIKPosX = ((self.legs[i].startedPosition[1]*(SB*SA*CG + CB*SG))
                        +(self.legs[i].startedPosition[0]*(SB*SA*SG + CB*CG))
                        -(self.legs[i].startedPosition[2]*SB*CA)) + deltaBodyOffX

            BodyIKPosY = ((self.legs[i].startedPosition[1]*CA*CG)
                        -(self.legs[i].startedPosition[0]*CA*SG)
                        +(self.legs[i].startedPosition[2]*SA)) + deltaBodyOffY

            BodyIKPosZ = ((self.legs[i].startedPosition[1]*(SB*SG - CB*SA*CG))
                        +(self.legs[i].startedPosition[0]*(CB*SA*SG + SB*CG))
                        +(self.legs[i].startedPosition[2]*CB*CA)) + deltaBodyHigh
            self.legs[i].updateHomePosition(BodyIKPosX, BodyIKPosY, BodyIKPosZ)

        if (self.currentTimeBody > self.stepTimeBody):
            self.isHomeBody = True
        

class leg():
    def __init__(self, Position, Rotation, Lflip):
        self.Femur = cFemurLength
        self.Tibia = cTibiaLengt
        self.Coxa = cCoxaLength

        self.flip = Lflip
        self.solution = True
        self.warning = False
        self.Error = False
        self.estateactive=True
        self.isUp = False

        self.OldCoxaAngle = 0
        self.OldFemurAngle = 0
        self.OldTibiaAngle = 0
        self.CoxaAngle = 0
        self.FemurAngle = 0
        self.TibiaAngle = 0

        self.stateMov = 0       #
        self.statedir = 1       #
        self.isCustomTarget = False
        self.customTarget = [[0, 0, 0], [0, 0, 0]]
        self.dirFactor = 1
        self.stepHeight= stepHeight
        self.stepPower = stepPower

        self.angleRotation = Rotation
        self.homeDistance = homeDistance
        self.targetMoveVec = [0, 0]
        self.targetTurnAngle = 0

        self.servoAngles = [0, 0, 0]
        self.homePoint = [0, 0, 0]
        self.homePosition = [0, 0, 0]
        self.startedPosition = [0, 0, 0]

        if (Lflip == True):
            self.homePosition[0] = -Position[0]
            self.startedPosition[0] = -Position[0]
        else:
            self.homePosition[0] = Position[0]
            self.startedPosition[0] = Position[0]
        
        self.homePosition[1] = Position[1]
        self.homePosition[2] = Position[2]
        self.startedPosition[1] = Position[1]
        self.startedPosition[2] = Position[2]

        self.updateHomePoint(self.homeDistance, Rotation, 0)

    def isactive(self):
        return self.estateactive

    def active(self):
        self.estateactive=True

    def disactive(self):
        self.estateactive=False

    def IK2Servo(self):
        if (self.flip == True):
            self.servoAngles[0] = (85 - self.CoxaAngle)
            self.servoAngles[1] = (90 - self.FemurAngle)
            self.servoAngles[2] = (180 + (self.TibiaAngle + 5))
        else:
            self.servoAngles[0] = (85 + self.CoxaAngle)
            self.servoAngles[1] = (90 + self.FemurAngle)
            self.servoAngles[2] = (- (self.TibiaAngle + 5))

    def customresetMov(self, newTargetIni, newTargetEnd, typeMov = None):  #TypeMov= Linear, Curve: Only for x,y, z=0 
        self.isCustomTarget = True
        self.customTarget[0] = newTargetIni
        self.customTarget[1] = newTargetEnd

    def resetMov(self, newTargetMovVec, typeMov = None):   #TypeMov: 1=[0,0] to value, 2=value to [0,0], None= Normal
        self.isCustomTarget = False
        self.targetMoveVec[0] = newTargetMovVec[0]
        self.targetMoveVec[1] = newTargetMovVec[1]
        self.targetTurnAngle = newTargetMovVec[2]
        if (typeMov == 1):
            self.isCustomTarget = True
            self.customTarget[0] = [0,0,0]
            self.customTarget[1] = newTargetMovVec
        elif (typeMov == -1):
            self.isCustomTarget = True
            self.customTarget[0] = newTargetMovVec
            self.customTarget[1] = [0,0,0]
            self.targetMoveVec[0] = 0
            self.targetMoveVec[1] = 0

    def updateMov(self, timeP = None):
        if (timeP == None):
            return
        movVecX = 0
        movVecY = 0
        angleVecX = 0
        angleVecY = 0
        movZ = 0
        timeAuxMovZ = 0
        factor = self.dirFactor

        if (self.isUp == True):
            if (timeP < 0.5):
                timeAuxMovZ = 1 - (2 * timeP)
            else:
                timeAuxMovZ = (2 * timeP) - 1
            movZ = (1 - pow(timeAuxMovZ, self.stepPower))* (self.stepHeight)        #Remember to add new movement system
        
        if (self.isCustomTarget):
            timeAux = timeP
            movVecX = -(((self.customTarget[1][0] - self.customTarget[0][0]) * timeAux) + self.customTarget[0][0])
            movVecY = -(((self.customTarget[1][1] - self.customTarget[0][1]) * timeAux) + self.customTarget[0][1])
            angleStep = self.homeAnglePoint - (((self.customTarget[1][2] - self.customTarget[0][2]) * timeAux) + self.customTarget[0][2])
        else:
            timeAux = (2 * timeP) - 1
            movVecX = self.targetMoveVec[0] * timeAux * factor
            movVecY = self.targetMoveVec[1] * timeAux * factor
            angleStep = self.homeAnglePoint + (self.targetTurnAngle * timeAux * factor)

        angleVecX = self.homeRadiusPoint * math.cos(angleStep*rad) - self.homePoint[0]
        angleVecY = self.homeRadiusPoint * math.sin(angleStep*rad) - self.homePoint[1]

        targetPointX = self.homePoint[0] + movVecX + angleVecX
        targetPointY = self.homePoint[1] + movVecY + angleVecY
        targetPointZ = movZ

        self.LegIK(targetPointX, targetPointY, targetPointZ)

    def updateHomePoint(self, newHomeDistance, Rotation = None, posZ = None):
        if (Rotation == None):
            Rotation = self.angleRotation

        if (self.flip == True):
            Rotation = 180 - Rotation

        dirX = math.cos(Rotation*rad)
        dirY = math.sin(Rotation*rad)
        self.homePoint[0] = self.homePosition[0] + (newHomeDistance * dirX)
        self.homePoint[1] = self.homePosition[1] + (newHomeDistance * dirY)
        if (posZ != None):
            self.homePoint[2] = posZ
        
        self.homeAnglePoint = (math.atan2(self.homePoint[1], self.homePoint[0]))*grad
        self.homeRadiusPoint = math.sqrt((self.homePoint[0]*self.homePoint[0])+(self.homePoint[1]*self.homePoint[1]))
        #self.LegIK(self.homePoint[0], self.homePoint[1], self.homePoint[2])

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
        
        try:
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
        except:
            self.FemurAngle = self.OldFemurAngle
            self.TibiaAngle = self.OldTibiaAngle

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
        
        self.OldCoxaAngle = self.CoxaAngle
        self.OldFemurAngle = self.FemurAngle
        self.OldTibiaAngle = self.TibiaAngle

        self.IK2Servo()

    def points2Draw(self):
        pX0 = self.homePosition[0]
        pY0 = self.homePosition[1]
        pZ0 = self.homePosition[2]

        if (self.flip == True):
            pX1 = self.Coxa * math.cos((180 - self.CoxaAngle)*rad)
        else:
            pX1 = self.Coxa * math.cos(self.CoxaAngle*rad)
        pY1 = self.Coxa * math.sin(self.CoxaAngle*rad)
        pZ1 = self.homePosition[2]

        if (self.flip == True):
            pX2 = self.Femur * math.cos((180 - self.CoxaAngle)*rad) + pX1
        else:
            pX2 = self.Femur * math.cos(self.CoxaAngle*rad) + pX1
        pY2 = self.Femur * math.sin(self.CoxaAngle*rad) + pY1
        pZ2 = self.Femur * math.sin(self.FemurAngle*rad) + pZ1

        pX3 = self.homePoint[0]
        pY3 = self.homePoint[1]
        pZ3 = self.homePoint[2]

        return [[pX0, pY0, pZ0], [pX1, pY1, pZ1], [pX2, pY2, pZ2], [pX3, pY3, pZ3]]

class TPCamera():
    def __init__(self, ServController, TiltAngle, PanAngle):
        self.TPangles = [TiltAngle, PanAngle]
        self.ServController = ServController
        self.ServController.set_camera_angles(self.TPangles)

    def updateAngles(self, TiltOffSet, PanOffSet):
        self.TPangles[0]  = self.TPangles[0] - TiltOffSet
        self.TPangles[1]  = self.TPangles[1] - PanOffSet

        for i in range(0,len(self.TPangles)):
            if (self.TPangles[i] > self.ServController.servMaxAngle[6][i]):
                self.TPangles[i] = self.ServController.servMaxAngle[6][i]
            elif (self.TPangles[i] < 0):
                self.TPangles[i] = 0
        
        self.ServController.set_camera_angles(self.TPangles)





'''
#Testing Body
controller = Controller()
Robot = BodyHex(controller)
#Robot.updateRPY(0, 0, 0)

for i in range(len(Robot.legs)):
    Robot.servosCon.set_leg_angles(i, Robot.legs[i].servoAngles)
    print("Prueba")

time.sleep(2)
print("move = 20, Back")
for i in range(0, 2500):
    Robot.update(0.025)
    for j in range(len(Robot.legs)):
        Robot.servosCon.set_leg_angles(j, Robot.legs[j].servoAngles)
    
    if (i == 500):
        print("Stopped")
        Robot.moveDistance = 0
    if (i == 1000):
        print("move = 20, Forward")
        Robot.moveDistance = 20
    if (i == 1500):
        print("Stopped")
        Robot.moveDistance = 0
    if (i == 2000):
        print("move = 40, Back")
        Robot.moveAngle = 180
        Robot.moveDistance = 40
    if (i == 2450):
        print("Stopped")
        Robot.moveDistance = 0
        
'''
