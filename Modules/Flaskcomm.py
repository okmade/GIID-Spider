import time
import sys
import threading
from flask import Flask, render_template, Response, request
from Camera_pi import Camera
import socket
import io


app = Flask(__name__)


##########     Web Routine    ##########

""" Control by WebSite """
@app.route('/')
def index():
    return render_template('index.html')


""" Video streaming generator function """
@app.route('/video_feed')   #change by stream_video
def video_feed():
    return Response(gen_stream(Camera()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def gen_stream(camera):
    while True:
        frame = camera.get_frame()
        #print ("Frame Size: " + str(len(frame)))
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n'
               b'Content-Length: ' + str.encode(str(len(frame))) + b'\r\n\r\n' + frame + b'\r\n')


""" Snapshoots generator function """
@app.route('/video_feed2')  #change by snapshoots_video
def video_feed2(): 
    image_binary = gensnapshoot(Camera())
    response = app.make_response(image_binary)
    response.headers.set('Content-Type', 'image/jpeg')
    return response

def gensnapshoot(camera):
    frame = camera.get_frame()
    return frame


""" Data to conrol Robot """
@app.route('/mov', methods=['GET','POST','DELETE'])
def mov():
    a_d = request.args.get("mode")
    j1_v = request.args.get("j1_v")
    j1_h = request.args.get("j1_h")
    j2_v = request.args.get("j2_v")
    j2_h = request.args.get("j2_h")
    data = [str(a_d),str(j1_v),str(j1_h),str(j2_v),str(j2_h)]
    control_data.set_new_data(data)

    #print ("Received mode " + str(a_d))
    #print ("Received j1_v " + str(j1_v))
    #print ("Received j1_h " + str(j1_h))
    #print ("Received j2_v " + str(j2_v))
    #print ("Received j2_h " + str(j2_h))
    
    return "Received"
