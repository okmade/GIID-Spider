from board import SCL, SDA
import busio
import threading
import time
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

ServPos =      [[  1,   2,   3],     #Leg RB -> pca0
                [  8,   9,  10],     #Leg RM -> pca0
                [ 12,  13,  14],     #Leg LB -> pca0
                [  1,   2,   3],     #Leg RF -> pca1
                [  8,   9,  10],     #Leg LF -> pca1
                [ 12,  13,  14],     #Leg LM -> pca1
                [  5,   6,   0]]     #Camera Tilt and Pan -> pca1


servOffset =   [[ -9, -11, 2.5],    #Leg RF
                [ -5,  -5,  -3],    #Leg RM
                [  8, -11,-2.5],    #Leg RB
                [ -3,   2,   2],    #Leg LF
                [  2,   4,  -1],    #Leg LM
                [  9,-1.5,   5],    #Leg LB
                [  9,  16,   0]]    #Camera Tilt and Pan

'''
servOffset =   [[ -5,   3,  -8],    #Leg RF
                [ -2,   0,   4],    #Leg RM
                [  0,   5,   0],    #Leg RB
                [  5,   6,   6],    #Leg LF
                [  0,   0,   0],    #Leg LM
                [  8, -11,  -8],    #Leg LB
                [  7,   0,   0]]    #Camera Tilt and Pan
'''

servMaxAngle = [[180, 180, 180],    #Leg RM
                [180, 180, 180],    #Leg RF
                [180, 180, 180],    #Leg LF
                [180, 180, 180],    #Leg LB
                [180, 180, 180],    #Leg RB
                [180, 180, 180],    #Leg LM
                [150, 150, 150]]    #Camera Tilt and Pan


class Controller:
    def __init__(self):
        self.i2c = busio.I2C(SCL, SDA)
        self.pca0 = PCA9685(self.i2c, address=0x40)
        self.pca1 = PCA9685(self.i2c, address=0x41)
        self.pca0.frequency = 50
        self.pca1.frequency = 50
        self.servos = {}
        self.servOffset = servOffset
        self.servMaxAngle = servMaxAngle
        for j in range(0, 7):
            for i in range(0, 3):
                if (j<3):
                    self.servos[(j*3) + i] = servo.Servo(self.pca0.channels[ServPos[j][i]], min_pulse=500, max_pulse=2530)
                elif (j== 6):
                    self.servos[(j*3) + i] = servo.Servo(self.pca1.channels[ServPos[j][i]])
                else:
                    self.servos[(j*3) + i] = servo.Servo(self.pca1.channels[ServPos[j][i]], min_pulse=500, max_pulse=2530)
                self.servos[(j*3) + i].actuation_range = servMaxAngle[j][i]

        self.sRF = [self.servos[ 9], self.servos[10], self.servos[11]]
        self.sRM = [self.servos[ 3], self.servos[ 4], self.servos[ 5]]
        self.sRB = [self.servos[ 0], self.servos[ 1], self.servos[ 2]]
        
        self.sLF = [self.servos[12], self.servos[13], self.servos[14]]
        self.sLM = [self.servos[15], self.servos[16], self.servos[17]]
        self.sLB = [self.servos[ 6], self.servos[ 7], self.servos[ 8]]
        
        self.sCamera = [self.servos[18], self.servos[19],self.servos[20]]

        self.sLegs = [self.sRF, self.sRM, self.sRB,
                      self.sLF, self.sLM, self.sLB]
                      
        self.stripodR = [self.sRF,self.sRB,self.sLM]
        self.stripodL = [self.sLF,self.sLB,self.sRM]
        
    def set_pos_init(self):
        for i in range(0,len(self.servos)):
            self.servos[i].angle = 90

    def set_leg_angles(self, leg, angles):
        for i in range(0,len(angles)):
            Temp = angles[i] + self.servOffset[leg][i]
            if (Temp > self.servMaxAngle[leg][i]):
                self.sLegs[leg][i].angle = self.servMaxAngle[leg][i]
            elif (Temp<0):
                self.sLegs[leg][i].angle = 0
            else:
                self.sLegs[leg][i].angle = Temp
    
    def set_camera_angles(self, angles, TP = 6):
        for i in range(0,len(angles)):
            Temp = angles[i] + self.servOffset[TP][i]
            if (Temp > self.servMaxAngle[TP][i]):
                self.sCamera[i].angle = self.servMaxAngle[TP][i]
            elif (Temp<30):
                self.sCamera[i].angle = 30
            else:
                self.sCamera[i].angle = Temp
        
    def set_angle(self, servPos, angle):
        if (angle > 170):
            self.servos[servPos].angle = 170
        elif (angle<0):
            self.servos[servPos].angle = 0
        else:
            self.servos[servPos].angle = angle


'''
prueba = Controller()
offset1 = 0
offset2 = 0
offset3 = 0

prueba.set_leg_angles(4,[85, 90, 180])
prueba.set_leg_angles(2,[40, 90, 90])

prueba.set_leg_angles(4,[85, 85, 85])
prueba.set_leg_angles(5,[85, 85, 85])

prueba.set_leg_angles(0,[130, 170, 85])
prueba.set_leg_angles(3,[85, 170, 85])
prueba.set_leg_angles(2,[40 + offset1, 170 + offset2, 85 + offset3])
'''

'''
prueba = Controller()
for i in range(0, 6):
    prueba.set_leg_angles(i,[90, 90, 90])
prueba.set_camera_angles([90, 90])
#prueba.set_leg_angles(3,[0, 90, 90])
#prueba.set_leg_angles(4,[30, 90, 90])
#prueba.set_leg_angles(5,[120, 90, 90])
#prueba.set_angle(1,90)
'''


'''
while True:
        for i in range(0, 170):
                prueba.set_angle(14,i)
                time.sleep(0.0001)
                if (i == 85):
                        time.sleep(1)
        print("180 Grados")
        time.sleep(3)
        for i in range(170, 0, -1):
                prueba.set_angle(14,i)
                time.sleep(0.0001)
                if (i == 85):
                        time.sleep(1)
        print("0 Grados")
        time.sleep(3)

'''
