# Program to control Spider Robot GIID.
# This is using the following systems:
#
#
#
#
#
#
#
#

#Libraries
from Module_leg import *
from Module_draw import *
from board import SCL, SDA
import busio
import time
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

#Initial Variables
longleg = 70															#Long of Leg in cm
longantleg = 50															#Long of AntLeg in cm
longftob = 70															#Long of Body from Floor in cm
longinx = 50															#Long between Body and Leg in cm
movinx = 40																#Long of movement over floor in cm
moviny = 40																#Long of movement in altitud of leg in cm
points_total=10															#Amount of movement for servos
posleft = 1																#Position Left of Leg on the body
posright = 2															#Position Right of Leg on the body


#Create instace with system algebraic
all1 = Leg(posleft,0,longantleg,longleg,longftob,0,longinx,60,movinx,moviny,points_total)
all2 = Leg(posleft,0,longantleg,longleg,longftob,0,longinx,90,movinx,moviny,points_total)
all3 = Leg(posleft,0,longantleg,longleg,longftob,0,longinx,60,movinx,moviny,points_total)

alr1 = Leg(posright,0,longantleg,longleg,longftob,0,longinx,60,movinx,moviny,points_total)
alr2 = Leg(posright,0,longantleg,longleg,longftob,0,longinx,90,movinx,moviny,points_total)
alr3 = Leg(posright,0,longantleg,longleg,longftob,0,longinx,60,movinx,moviny,points_total)


# Create a comunication with PCA9685
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x40)
pca1 = PCA9685(i2c, address=0x41)
pca.frequency = 50
pca1.frequency = 50

for i in range(0,16):
	servoall1 = servo.Servo(pca.channels[i])
	servoall2 = servo.Servo(pca1.channels[i])
	servoall1.angle = 90
	servoall2.angle = 90

time.sleep(3)

sl1_1 = servo.Servo(pca.channels[4])
sl1_2 = servo.Servo(pca.channels[5])
sl1_3 = servo.Servo(pca.channels[6])

sl2_1 = servo.Servo(pca.channels[0])
sl2_2 = servo.Servo(pca.channels[1])
sl2_3 = servo.Servo(pca.channels[2])

sl3_1 = servo.Servo(pca1.channels[13])
sl3_2 = servo.Servo(pca1.channels[14])
sl3_3 = servo.Servo(pca1.channels[15])

sr1_1 = servo.Servo(pca.channels[8])
sr1_2 = servo.Servo(pca.channels[9])
sr1_3 = servo.Servo(pca.channels[10])

sr2_1 = servo.Servo(pca1.channels[8])
sr2_2 = servo.Servo(pca1.channels[9])
sr2_3 = servo.Servo(pca1.channels[10])

sr3_1 = servo.Servo(pca1.channels[1])
sr3_2 = servo.Servo(pca1.channels[2])
sr3_3 = servo.Servo(pca1.channels[3])


while True:
    #print("Start Movement")
    for x in range(0,points_total):
        v1,v2,v3=alr1.getanglesforservos(x)
        sr1_1.angle = v3
        sr1_2.angle = v2
        sr1_3.angle = v1
        
        v1,v2,v3=all2.getanglesforservos(x)
        sl2_1.angle = v3
        sl2_2.angle = v2
        sl2_3.angle = v1
        
        v1,v2,v3=alr3.getanglesforservos(x)
        sr3_1.angle = v3
        sr3_2.angle = v2
        sr3_3.angle = v1
        
        x2 = x+((points_total/2))
        v1,v2,v3=all1.getanglesforservos(x2)
        sl1_1.angle = v3
        sl1_2.angle = v2
        sl1_3.angle = v1
        
        v1,v2,v3=alr2.getanglesforservos(x2)
        sr2_1.angle = v3
        sr2_2.angle = v2
        sr2_3.angle = v1
        
        v1,v2,v3=all3.getanglesforservos(x2)
        sl3_1.angle = v3
        sl3_2.angle = v2
        sl3_3.angle = v1
        
        print("Valor point",x,"    Valor point2",x2)
        #input("Press Enter to continue...")
        time.sleep(0.1)
    #print("Stop Movement")

#def animate(i):
    #j=i%(points_total+1)
    #draw(Leg1_1.getpointstodraw(j))

#ani = animation.FuncAnimation(fig, animate, interval=2000)
#plt.show()
