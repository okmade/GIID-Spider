import logging

from board import SCL, SDA
import busio
import threading
import time
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

ServPos =[
             4,  5,  6,     #Leg LF -> pca0
             0,  1,  2,     #Leg LM -> pca0
             8,  9, 10,     #Leg RF -> pca0
            13, 14, 15,     #Leg LB -> pca1
             8,  9, 10,     #Leg RM -> pca1
             1,  2,  3      #Leg RB -> pca1
            ]

logging.basicConfig(
    level=logging.DEBUG,
    format='(%(threadName)-10s) %(message)s',
)

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

class Controller:

    def __init__(self):
        self.i2c = busio.I2C(SCL, SDA)
        self.pca0 = PCA9685(self.i2c, address=0x40)
        self.pca1 = PCA9685(self.i2c, address=0x41)
        self.pca0.frequency = 50
        self.pca1.frequency = 50
        self.servos = {}
        for i in range(0,len(ServPos)):
            if (i<9):
                self.servos[i] = servo.Servo(self.pca0.channels[ServPos[i]])
            else:
                self.servos[i] = servo.Servo(self.pca1.channels[ServPos[i]])
        
        self.sRF = [self.servos[ 6], self.servos[ 7], self.servos[ 8] ]
        self.sRM = [self.servos[12], self.servos[13], self.servos[14] ]
        self.sRB = [self.servos[15], self.servos[16], self.servos[17] ]
        
        self.sLF = [self.servos[ 0], self.servos[ 1], self.servos[ 2] ]
        self.sLM = [self.servos[ 3], self.servos[ 4], self.servos[ 5] ]
        self.sLB = [self.servos[ 9], self.servos[10], self.servos[11] ]
        
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
            if (angles[i] > 180):
                self.sLegs[leg][i].angle = 180
            elif (angles[i]<0):
                self.sLegs[leg][i].angle = 0
            else:
                self.sLegs[leg][i].angle = angles[i]
        
    def set_angle(self, servPos, angle):
        if (angle > 180):
            self.servos[servPos].angle = 180
        elif (angle<0):
            self.servos[servPos].angle = 0
        else:
            self.servos[servPos].angle = angle


'''  Test
prueba = Controller()
prueba.set_pos_init()
time.sleep(5)
prueba.set_angle(11,180)
prueba.set_angle(15,180)
time.sleep(5)
prueba.set_angle(11,90)
prueba.set_angle(15,90)
time.sleep(5)
prueba.set_angle(11,0)
prueba.set_angle(15,0)

for i in range(0,180):
    prueba.set_angle(11,i)
    prueba.set_angle(15,i)
    time.sleep(0.1)
'''
