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
points_total=50															#Amount of movement for servos
posleft = 1																#Position Left of Leg on the body
posright = 2															#Position Right of Leg on the body


#Create instace with system algebraic
AnglesLeg1_1 = Leg(posleft,0,longantleg,longleg,longftob,0,longinx,60,movinx,moviny,points_total)
AnglesLeg1_2 = Leg(posleft,0,longantleg,longleg,longftob,0,longinx,90,movinx,moviny,points_total)
AnglesLeg1_3 = Leg(posleft,0,longantleg,longleg,longftob,0,longinx,120,movinx,moviny,points_total)

AnglesLeg2_1 = Leg(posright,0,longantleg,longleg,longftob,0,longinx,60,movinx,moviny,points_total)
AnglesLeg2_2 = Leg(posright,0,longantleg,longleg,longftob,0,longinx,90,movinx,moviny,points_total)
AnglesLeg2_3 = Leg(posright,0,longantleg,longleg,longftob,0,longinx,120,movinx,moviny,points_total)


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

servoleg = 		servo.Servo(pca.channels[8])
servoantleg = 	servo.Servo(pca.channels[9])
servocodo = 	servo.Servo(pca.channels[10])

servoleg1 = 	servo.Servo(pca.channels[4])
servoantleg1 = 	servo.Servo(pca.channels[5])
servocodo1 = 	servo.Servo(pca.channels[6])

servoleg2 = 	servo.Servo(pca.channels[0])
servoantleg2 = 	servo.Servo(pca.channels[1])
servocodo2 = 	servo.Servo(pca.channels[2])

while True:
    #print("Start Movement")
    for x in range(0,points_total):
        v1,v2,v3=AnglesLeg2_1.getanglesforservos(x)
        servoleg.angle = v3
        servoantleg.angle = v2
        servocodo.angle = v1
        v1,v2,v3=AnglesLeg1_1.getanglesforservos(x)
        servoleg1.angle = v3
        servoantleg1.angle = v2
        servocodo1.angle = v1
        servoleg2.angle = v3
        servoantleg2.angle = v2
        servocodo2.angle = v1
        #print(Leg1_1.getanglesforservosrigth(x))
        time.sleep(0.01)
    #print("Stop Movement")

#def animate(i):
    #j=i%(points_total+1)
    #draw(Leg1_1.getpointstodraw(j))

#ani = animation.FuncAnimation(fig, animate, interval=2000)
#plt.show()
