#!/usr/bin/python
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

import threading
import time
from Module_leg import *
from Module_func import *
from Module_draw import *

ANALOG_DIGITAL_CONTROL = '1'
IS_ACTIVE = 1

def data_web():
    global ANALOG_DIGITAL_CONTROL
    global IS_ACTIVE
    print("Task 1")
    while (IS_ACTIVE == 1):
        ANALOG_DIGITAL_CONTROL = input("Prueba")
        print ("Valor Task 1:   ", ANALOG_DIGITAL_CONTROL)

def control_spider():
    global ANALOG_DIGITAL_CONTROL
    global IS_ACTIVE
    print("Task 2")
    while (IS_ACTIVE == 1):
        print ("Valor Task 2:   ", ANALOG_DIGITAL_CONTROL)
        if (ANALOG_DIGITAL_CONTROL == '1'):
            for j in range(0,4):
                go_front(15)
            for j in range(0,4):
                go_right(15)
            for j in range(0,4):
                go_left(15)
            for j in range(0,4):
                go_back(15)


if __name__=="__main__":
    try:
        set_pos_init()
        task_web = threading.Thread(target=data_web)
        task_spider = threading.Thread(target=control_spider)
        task_web.start()
        task_spider.start()
    except KeyboardInterrupt:
        IS_ACTIVE == 0
