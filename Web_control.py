#!/usr/bin/env python
from flask import Flask, render_template, Response, request
from camera_pi import Camera
import cv2
import socket
import io

app = Flask(__name__)
vc = cv2.VideoCapture(1)

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

@app.route('/mov')
def mov():
    a_d = request.args.get("a_d")
    print ("Received a_d " + str(a_d))
    if (str(a_d) == "analog"):
        j1_v = request.args.get("j1_v")
        j1_h = request.args.get("j1_h")
        print ("Received j1_v " + str(j1_v))
        print ("Received j1_h " + str(j1_h))
    elif (str(a_d) == "digital"):
        j1 = request.args.get("j1")
        print ("Received j1 " + str(j1))
    j2_v = request.args.get("j2_v")
    j2_h = request.args.get("j2_h")
    print ("Received j2_v " + str(j2_v))
    print ("Received j2_h " + str(j2_h))
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen(Camera()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True, threaded=True)
