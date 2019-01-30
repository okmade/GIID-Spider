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
from Module_func import *
from Module_draw import *
#from board import SCL, SDA
#import busio
#import time
#from adafruit_pca9685 import PCA9685
#from adafruit_motor import servo



set_pos_init()

while True:
    for j in range(0,10):
	    go_front(15)
    for j in range(0,10):
	    go_right(15)
    for j in range(0,10):
	    go_left(15)
    for j in range(0,10):
	    go_back(15)
