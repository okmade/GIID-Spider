import logging

from board import SCL, SDA
import busio
import threading
import time
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

ServPos =[
             9, 10, 11,     #Leg LF -> pca0
             5,  6,  7,     #Leg LM -> pca0
             1,  2,  3,     #Leg LB -> pca0
             5,  6,  7,     #Leg RF -> pca1
             9, 10, 11,     #Leg RM -> pca1
            13, 14, 15      #Leg RB -> pca1
            ]
            
servOffset = [[ -1, -11, -3],    #done
              [-6, -1, -8],     #done
              [-8, 0, -6],
              [-3, 3, 5],       #done   [-3, 5, 5],
              [-10,-1,-10],     #done   [-10,-5,-10],
              [ 0, -3, 0]]      #done   [ 0, -1, 0]] 

logging.basicConfig(
    level=logging.DEBUG,
    format='(%(threadName)-10s) %(message)s',
)
'''
class runUpdate(threading.Thread):

    def __init__(self,function,*args):
        threading.Thread.__init__(self,name='Updating_steps')
        self.function=function
        self.args = args
        #self.lock = threading.Lock()
        self._running = True
        self.start()
    
    def get_active(self):
        return self._running

    def run(self):
        #logging.debug('Acquired lock')
        #self.lock.acquire()
        try:
            self.function(*self.args)
            self._running = False
        finally:
            pass
            #logging.debug('Done')
            #self.lock.release()

class runMovement(threading.Thread):

    def __init__(self,function,*args):
        threading.Thread.__init__(self)
        self.function=function
        self.args = args
        self._running = True
        self.start()

    def terminate(self):  
        self._running = False
    
    def get_active(self):
        return self._running

    def run(self):
        while self._running:
            self.function(*self.args)
'''
class Controller:

    def __init__(self):
        self.i2c = busio.I2C(SCL, SDA)
        self.pca0 = PCA9685(self.i2c, address=0x40)
        self.pca1 = PCA9685(self.i2c, address=0x41)
        self.pca0.frequency = 50
        self.pca1.frequency = 50
        self.servos = {}
        self.servOffset = servOffset
        for i in range(0,len(ServPos)):
            if (i<9):
                self.servos[i] = servo.Servo(self.pca0.channels[ServPos[i]], min_pulse=500, max_pulse=2500)
            else:
                self.servos[i] = servo.Servo(self.pca1.channels[ServPos[i]], min_pulse=500, max_pulse=2500)
            self.servos[i].actuation_range = 170
        
        self.sRF = [self.servos[ 9], self.servos[10], self.servos[11] ]
        self.sRM = [self.servos[12], self.servos[13], self.servos[14] ]
        self.sRB = [self.servos[15], self.servos[16], self.servos[17] ]
        
        self.sLF = [self.servos[ 0], self.servos[ 1], self.servos[ 2] ]
        self.sLM = [self.servos[ 3], self.servos[ 4], self.servos[ 5] ]
        self.sLB = [self.servos[ 6], self.servos[ 7], self.servos[ 8] ]
        
        self.sLegs = [self.sRF,
                      self.sRM,
                      self.sRB,
                      self.sLF,
                      self.sLM,
                      self.sLB]
                      
        self.stripodR = [self.sRF,self.sRB,self.sLM]
        self.stripodL = [self.sLF,self.sLB,self.sRM]
        
    def set_pos_init(self):
        for i in range(0,len(self.servos)):
            self.servos[i].angle = 90

    def set_leg_angles(self, leg, angles):
        for i in range(0,3):
            Temp = angles[i] + self.servOffset[leg][i]
            if (Temp > 170):
                self.sLegs[leg][i].angle = 170
            elif (Temp<0):
                self.sLegs[leg][i].angle = 0
            else:
                self.sLegs[leg][i].angle = Temp
        
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

prueba.set_leg_angles(3,[85, 85, 85])
prueba.set_leg_angles(4,[85, 85, 85])
prueba.set_leg_angles(5,[85, 85, 85])

prueba.set_leg_angles(0,[130, 170, 85])
prueba.set_leg_angles(1,[85, 170, 85])
prueba.set_leg_angles(2,[40 + offset1, 170 + offset2, 85 + offset3])

'''


'''
prueba = Controller()
prueba.set_angle(2,0)
#prueba.set_angle(1,90)


while True:
        for i in range(0, 170):
                prueba.set_angle(2,i)
                time.sleep(0.001)
                if (i == 85):
                        time.sleep(1)
        print("180 Grados")
        time.sleep(3)
        for i in range(170, 0, -1):
                prueba.set_angle(2,i)
                time.sleep(0.001)
                if (i == 85):
                        time.sleep(1)
        print("0 Grados")
        time.sleep(3)

'''

'''
for i in range(0,17):
    prueba.set_angle(i,90)
    prueba.set_angle(i,90)
    prueba.set_angle(i,90)
time.sleep(5000)
'''
'''
for i in range(0,17):
    prueba.set_angle(i,30)
    prueba.set_angle(i,30)
    prueba.set_angle(i,30)
time.sleep(5)

for i in range(0,17):
    prueba.set_angle(i,130)
    prueba.set_angle(i,130)
    prueba.set_angle(i,130)
time.sleep(5)

for i in range(0,17):
    prueba.set_angle(i,90)
time.sleep(5)
'''
'''
for i in range(70,110):
    prueba.set_angle(5,i)
    prueba.set_angle(3,i)
    prueba.set_angle(4,i)
    time.sleep(0.1)


'''
