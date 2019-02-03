from Module_leg import *
from board import SCL, SDA
import busio
import time
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

#Initial Variables
POS_LEFT = 1															#Position Left of Leg on the body
POS_RIGHT = 2															#Position Right of Leg on the body
LONG_LEG = 70															#Long of Leg in cm
LONG_ANT_LEG = 50														#Long of AntLeg in cm
longftob = 50															#Long of Body from Floor in cm
longinx = 50															#Long between Body and Leg in cm
movinx = 30															#Long of movement over floor in cm
moviny = 30															#Long of movement in altitud of leg in cm
points_total = 30      		        											#Amount of movement for servos Min=6 and Max=200
max_points_total = 200
control_time = 20
max_control_time = 100
BASE_TIME = 1000
CONTROL_BY_TIME = False

#Create instace with system algebraic
all1 = Leg(POS_LEFT,0,LONG_ANT_LEG,LONG_LEG,longftob,0,longinx,60,movinx,moviny,points_total)
all2 = Leg(POS_LEFT,0,LONG_ANT_LEG,LONG_LEG,longftob,0,longinx,90,movinx,moviny,points_total)
all3 = Leg(POS_LEFT,0,LONG_ANT_LEG,LONG_LEG,longftob,0,longinx,120,movinx,moviny,points_total)
alr1 = Leg(POS_RIGHT,0,LONG_ANT_LEG,LONG_LEG,longftob,0,longinx,60,movinx,moviny,points_total)
alr2 = Leg(POS_RIGHT,0,LONG_ANT_LEG,LONG_LEG,longftob,0,longinx,90,movinx,moviny,points_total)
alr3 = Leg(POS_RIGHT,0,LONG_ANT_LEG,LONG_LEG,longftob,0,longinx,120,movinx,moviny,points_total)

# Create a comunication with PCA9685
i2c = busio.I2C(SCL, SDA)
pca0 = PCA9685(i2c, address=0x40)
pca1 = PCA9685(i2c, address=0x41)
pca0.frequency = 50
pca1.frequency = 50

sl1_1 = servo.Servo(pca0.channels[4])
sl1_2 = servo.Servo(pca0.channels[5])
sl1_3 = servo.Servo(pca0.channels[6])

sl2_1 = servo.Servo(pca0.channels[0])
sl2_2 = servo.Servo(pca0.channels[1])
sl2_3 = servo.Servo(pca0.channels[2])

sl3_1 = servo.Servo(pca1.channels[13])
sl3_2 = servo.Servo(pca1.channels[14])
sl3_3 = servo.Servo(pca1.channels[15])

sr1_1 = servo.Servo(pca0.channels[8])
sr1_2 = servo.Servo(pca0.channels[9])
sr1_3 = servo.Servo(pca0.channels[10])

sr2_1 = servo.Servo(pca1.channels[8])
sr2_2 = servo.Servo(pca1.channels[9])
sr2_3 = servo.Servo(pca1.channels[10])

sr3_1 = servo.Servo(pca1.channels[1])
sr3_2 = servo.Servo(pca1.channels[2])
sr3_3 = servo.Servo(pca1.channels[3])

def set_pos_init():
    for i in range(0,16):
	    servoall1 = servo.Servo(pca0.channels[i])
	    servoall2 = servo.Servo(pca1.channels[i])
	    servoall1.angle = 90
	    servoall2.angle = 90

def set_velocity(vel):
    global control_time
    global points_total
    if (vel != 'None'):
        new_vel = int(vel)
        if (CONTROL_BY_TIME == True):
            if (new_vel > max_control_time):
                new_vel = max_control_time
            elif (new_vel < 0):
                new_vel = 0
            control_time = new_vel
        else:
            if (new_vel > max_points_total):
                new_vel = max_points_total
            elif (new_vel < 10):
                new_vel = 10
            all1.update_tpoints(new_vel)
            all2.update_tpoints(new_vel)
            all3.update_tpoints(new_vel)
            alr1.update_tpoints(new_vel)
            alr2.update_tpoints(new_vel)
            alr3.update_tpoints(new_vel)
            points_total = new_vel

def go_front(vel):
    #print("Run Ahead")
    for x in range(0,points_total):
        v1,v2,v3=alr1.getanglesforservos(x)
        sr1_1.angle = v3
        sr1_2.angle = v2
        sr1_3.angle = v1 - 45
    
        v1,v2,v3=all2.getanglesforservos(x)
        sl2_1.angle = v3
        sl2_2.angle = v2
        sl2_3.angle = v1
    
        v1,v2,v3=alr3.getanglesforservos(x)
        sr3_1.angle = v3
        sr3_2.angle = v2
        sr3_3.angle = v1 + 45
    
        x2 = x+((points_total/2))
        v1,v2,v3=all1.getanglesforservos(x2)
        sl1_1.angle = v3
        sl1_2.angle = v2
        sl1_3.angle = v1 + 45
    
        v1,v2,v3=alr2.getanglesforservos(x2)
        sr2_1.angle = v3
        sr2_2.angle = v2
        sr2_3.angle = v1
    
        v1,v2,v3=all3.getanglesforservos(x2)
        sl3_1.angle = v3
        sl3_2.angle = v2
        sl3_3.angle = v1 - 45
        if (CONTROL_BY_TIME == True):
            time.sleep(control_time/BASE_TIME)

def go_back(vel):
    #print("Run Revert")
    for x in range(points_total-1,-1,-1):
        v1,v2,v3=alr1.getanglesforservos(x)
        sr1_1.angle = v3
        sr1_2.angle = v2
        sr1_3.angle = v1 - 45
    
        v1,v2,v3=all2.getanglesforservos(x)
        sl2_1.angle = v3
        sl2_2.angle = v2
        sl2_3.angle = v1
    
        v1,v2,v3=alr3.getanglesforservos(x)
        sr3_1.angle = v3
        sr3_2.angle = v2
        sr3_3.angle = v1 + 45
    
        x2 = x+((points_total/2))
        v1,v2,v3=all1.getanglesforservos(x2)
        sl1_1.angle = v3
        sl1_2.angle = v2
        sl1_3.angle = v1 + 45
    
        v1,v2,v3=alr2.getanglesforservos(x2)
        sr2_1.angle = v3
        sr2_2.angle = v2
        sr2_3.angle = v1
    
        v1,v2,v3=all3.getanglesforservos(x2)
        sl3_1.angle = v3
        sl3_2.angle = v2
        sl3_3.angle = v1 - 45
        if (CONTROL_BY_TIME == True):
            time.sleep(control_time/BASE_TIME)

def go_left(vel):
    print("Run Left")
    for x in range(0,points_total):
        
        v1,v2,v3=alr1.getanglesforservos(x)
        sr1_1.angle = v3
        sr1_2.angle = v2
        sr1_3.angle = v1 - 45
        
        v1,v2,v3=alr2.getanglesforservos(x+(points_total/2))
        sr2_1.angle = v3
        sr2_2.angle = v2
        sr2_3.angle = v1
    
        v1,v2,v3=alr3.getanglesforservos(x)
        sr3_1.angle = v3
        sr3_2.angle = v2
        sr3_3.angle = v1 + 45
        
        x2 = -x+(points_total-1)
        v1,v2,v3=all1.getanglesforservos(x2)
        sl1_1.angle = v3
        sl1_2.angle = v2
        sl1_3.angle = v1 + 45
    
        v1,v2,v3=all2.getanglesforservos(x2+(points_total/2))
        sl2_1.angle = v3
        sl2_2.angle = v2
        sl2_3.angle = v1
    
        v1,v2,v3=all3.getanglesforservos(x2)
        sl3_1.angle = v3
        sl3_2.angle = v2
        sl3_3.angle = v1 - 45
        if (CONTROL_BY_TIME == True):
            time.sleep(control_time/BASE_TIME)

def go_right(vel):
    print("Run Right")
    for x in range(points_total-1,-1,-1):
        
        v1,v2,v3=alr1.getanglesforservos(x)
        sr1_1.angle = v3
        sr1_2.angle = v2
        sr1_3.angle = v1 - 45
        
        v1,v2,v3=alr2.getanglesforservos(x+(points_total/2))
        sr2_1.angle = v3
        sr2_2.angle = v2
        sr2_3.angle = v1
    
        v1,v2,v3=alr3.getanglesforservos(x)
        sr3_1.angle = v3
        sr3_2.angle = v2
        sr3_3.angle = v1 + 45
        
        x2 = -x+(points_total-1)
        v1,v2,v3=all1.getanglesforservos(x2)
        sl1_1.angle = v3
        sl1_2.angle = v2
        sl1_3.angle = v1 + 45
    
        v1,v2,v3=all2.getanglesforservos(x2+(points_total/2))
        sl2_1.angle = v3
        sl2_2.angle = v2
        sl2_3.angle = v1
    
        v1,v2,v3=all3.getanglesforservos(x2)
        sl3_1.angle = v3
        sl3_2.angle = v2
        sl3_3.angle = v1 - 45
        if (CONTROL_BY_TIME == True):
            time.sleep(control_time/BASE_TIME)
