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
#import time
from Module_leg import *
from Module_func import *

from flask import Flask, render_template, Response, request
from camera_pi import Camera
import socket
import io

app = Flask(__name__)

##########   Spider Rutines   ##########
global new_data_spider
new_data_spider = False

global spider_is_running
spider_is_running = False

global data_spider
data_spider = ['N','N','N',512,512]
    #	data_spider[0] = "MODE", 'A' = Analogo, 'D' = Digital, 'N' = None
    #	data_spider[1] = If MODE == 'D', 'N' = None, 'F' = Forward, 'B' = Back, 'L' = Left, 'R' = Rigth, 'S' = Stop
    #					 If MODE == 'A', Lateral Movement Velocity Control 0 to 1023
    #	data_spider[2] = If MODE == 'D', Velocity Control 0 to 1023  (Forward or Back)
    #                    If MODE == 'A', Velocity Control 0 to 1023  (Forward or Back)
    #	data_spider[3] = Right Joystick Vertical 0 to 1023
    #	data_spider[4] = Right Joystick Horizontal 0 to 1023

""" Analogo Mode """
def mode_analog():
    global spider_is_running
    spider_is_running = True
    print("ANALOG MODE")
    spider_is_running = False

""" Digital Mode """
def mode_digital():
    global spider_is_running
    spider_is_running = True
    if (data_spider[1] == 'F'):
            go_front(15)
    if (data_spider[1] == 'B'):
            go_back(15)
    if (data_spider[1] == 'R'):
            go_right(15)
    if (data_spider[1] == 'L'):
            go_left(15)
    spider_is_running = False

##########     Web Routine    ##########

""" Main function loop """
@app.before_first_request
def activate_spider():
    def run_control_spider():
        global new_data_spider
        global data_spider
        while True:
            if (new_data_spider == True):
                set_velocity(data_spider[2])
                new_data_spider = False

            if (data_spider[0] == 'A'):
                mode_analog()
            elif (data_spider[0] == 'D'):
                mode_digital()

    thread = threading.Thread(target=run_control_spider)
    thread.start()

""" Control by WebSite """
@app.route('/')
def index():
    return render_template('index.html')

""" Video streaming generator function """
def gen(camera):
    while True:
        frame = camera.get_frame()
        #print ("Frame Size: " + str(len(frame)))
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n'
               b'Content-Length: ' + str.encode(str(len(frame))) + b'\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')   #change by stream_video
def video_feed():
    return Response(gen(Camera()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

""" Snapshoots generator function """
def gen1(camera):
    frame = camera.get_frame()
    return frame

@app.route('/video_feed2')  #change by snapshoots_video
def video_feed2(): 
    image_binary = gen1(Camera())
    response = app.make_response(image_binary)
    response.headers.set('Content-Type', 'image/jpeg')
    return response


""" Data to conrol Spider """
@app.route('/mov', methods=['GET','POST','DELETE'])
def mov():
    global data_spider
    global new_data_spider
    global spider_is_running

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
    return "Received"



if __name__ == '__main__':
    set_pos_init()
    app.run(host='0.0.0.0', debug=False, threaded=True)
