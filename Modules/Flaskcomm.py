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
@app.route('/video_stream')   #change by stream_video
def video_stream():
    return Response(gen_stream(Camera()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def gen_stream(camera):
    while True:
        frame = camera.get_frame("cv2")
        yield (b'--frame\r\n'
               b'Content-Length: ' + str.encode(str(len(frame))) + b'\r\n\r\n' + frame + b'\r\n')



""" Snapshoots generator function """
@app.route('/video_snapshoots')  #change by snapshoots_video
def video_snapshoots():
    if (video.videoSource == "Flask-TCP"):
        image_binary = gen_snapshoot(Camera())
        response = app.make_response(image_binary)
        response.headers.set('Content-Type', 'image/jpeg')
        return response
    else:
        print("No image")
        return "No Image"

def gen_snapshoot(camera):
    frame = camera.get_frame("cv2")
    return frame



""" Video Selector """
@app.route('/change_video', methods=['GET','POST','DELETE'])
def change_video():
    newVideoSource = request.args.get("video_source")
    video.change_video(str(newVideoSource))
    print(str(newVideoSource))
    return "Received"



""" Connection Cheking """
@app.route('/test', methods=['GET','POST','DELETE'])
def test():
    return "Connection Ok"



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
    newTilt = request.args.get("tilt")
    newPan = request.args.get("pan")
    newGait = request.args.get("gait")
    if (str(a_d) == 'Full'):
        aux = 1
    else:
        aux = 0
    data = [aux, newMoveDistance, newMoveAngle,
                newTurnAngle, newTimeScale, newOffSetX,
                newOffSetY, newOffSetZ, newRoll,
                newPitch, newYaw, newHomeDistance,
                newTilt, newPan, newGait]
    Run_MovementData.set_new_data(data)
    
    return "Received"
