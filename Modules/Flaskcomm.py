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
@app.route('/data', methods=['GET','POST','DELETE'])
def data():
    a_d = request.args.get("mode")
    newMoveDistance = request.args.get("moveDistance")
    newMoveAngle = request.args.get("moveAngle")
    newTurnAngle = request.args.get("turnAngle")
    newTimeScale = request.args.get("timeScale")

    newOffSetZ = request.args.get("offSetZ")
    newOffSetX = request.args.get("offSetX")
    newOffSetY = request.args.get("offSetY")

    newRoll = request.args.get("roll")
    newPitch = request.args.get("pitch")
    newYaw = request.args.get("yaw")

    newHomeDistance = request.args.get("homeDistance")

    if (str(a_d) == 'Full'):
        aux = 1
    else:
        aux = 0
    
    data = [aux, newMoveDistance, newMoveAngle,
                newTurnAngle, newTimeScale, newOffSetX,
                newOffSetY, newOffSetZ, newRoll,
                newPitch, newYaw, newHomeDistance]
    control_data.set_new_data(data)
    
    return "Received"
