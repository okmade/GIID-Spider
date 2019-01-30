#!/usr/bin/env python
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

from flask import Flask, render_template, Response, request
from camera_pi import Camera
import cv2
import socket
import io

app = Flask(__name__)

##########   Spider Rutines   ##########
global new_data_spider
new_data_spider = False

global data_spider
data_spider = ['N','N','N',512,512]
#	data_spider[0] = "MODE", 'A' = Analogo, 'D' = Digital, 'N' = None
#	data_spider[1] = If MODE == 'D',
#	 					'N' = None, 'U' = Up, 'D' = Dowm, 'L' = Left, 'R' = Rigth, 'S' = Stop
#					 If MODE == 'A', Left Joystick Vertical 0 to 1023
#	data_spider[2] = If MODE == 'A', Left Joystick Horizontal 0 to 1023
#	data_spider[3] = Right Joystick Vertical 0 to 1023
#	data_spider[4] = Right Joystick Horizontal 0 to 1023


def mode_analog():
    global data_spider
    print("ANALOG MODE")

def mode_digital():
    if (data_spider[1] == 'U'):
        for j in range(0,8):
            go_front(15)
#             print("Front  ",j)
    if (data_spider[1] == 'D'):
        for j in range(0,8):
            go_back(15)
#             print("Back  ",j)
    if (data_spider[1] == 'R'):
        for j in range(0,8):
            go_right(15)
#             print("Right  ",j)
    if (data_spider[1] == 'L'):
        for j in range(0,8):
            go_left(15)
#             print("Left  ",j)

##########     Web Routine    ##########
@app.before_first_request
def activate_spider():
    def run_control_spider():
        global new_data_spider
        global data_spider
        while True:
            if (new_data_spider == True):
                if (data_spider[0] == 'A'):
                    mode_analog()
                elif (data_spider[0] == 'D'):
                    mode_digital()
                new_data_spider = False
            #time.sleep(2)

    thread = threading.Thread(target=run_control_spider)
    thread.start()

@app.route('/')
def index():
    """Video streaming"""
    return render_template('index.html')

def gen(camera):
    """Video streaming generator function."""
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen(Camera()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/mov')
def mov():
    global data_spider
    global new_data_spider
    a_d = request.args.get("mode")
    j1_v = request.args.get("j1_v")
    j1_h = request.args.get("j1_h")
    j2_v = request.args.get("j2_v")
    j2_h = request.args.get("j2_h")
    data_spider = [str(a_d),str(j1_v),str(j1_h),str(j2_v),str(j2_h)]
    new_data_spider = True
    
    print ("Received mode " + str(a_d))
    print ("Received j1_v " + str(j1_v))
    print ("Received j1_h " + str(j1_h))
    print ("Received j2_v " + str(j2_v))
    print ("Received j2_h " + str(j2_h))
    return render_template('index.html')



if __name__ == '__main__':
    set_pos_init()
    app.run(host='0.0.0.0', debug=False, threaded=True)
