#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  camera_pi.py
#  
#  
#  
import time
import io
import threading
import picamera
import cv2


class Camera(object):
    thread = None       # background thread that reads frames from camera
    frame = None        # current frame is stored here by background thread
    last_access = 0     # time of last client access to the camera
    camType = "cv2"      #cv2 or picamera

    def initialize(self):
        if Camera.thread is None:
            # start background frame thread
            Camera.thread = threading.Thread(target=self._thread)
            Camera.thread.start()

            # wait until frames start to be available
            while self.frame is None:
                time.sleep(0)

    def get_frame(self,camType="cv2"):
        Camera.camType = camType
        Camera.last_access = time.time()
        self.initialize()
        return self.frame

    @classmethod
    def _thread(cls):
        if (cls.camType == "cv2"):
            cam = cv2.VideoCapture(0)
            cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)#modify to set camera resolution width
            cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)#modify to set camera resolution height
            imageQuality = 90 #1-100 higher = better quality but more data
            flipImage = True
            #print("cv2")

            while True:
                ret,camImage = cam.read()
                if flipImage:
                    #camImage = cv2.flip(camImage,0)
                    camImage = cv2.flip(camImage,-1)
                # reduce size of image for potentially faster streaming. Keep the 'fx' and 'fy' values the same or the image will become skewed.
                #camImage = cv2.resize(camImage, (0,0), fx=0.5, fy=0.5)
                
                cls.frame = bytes(cv2.imencode('.jpg', camImage,[int(cv2.IMWRITE_JPEG_QUALITY), imageQuality])[1].tostring())

                # if there hasn't been any clients asking for frames in
                # the last 10 seconds stop the thread
                if time.time() - cls.last_access > 3:
                    break

        elif (cls.camType == "pi"):
            #print("pi")
            with picamera.PiCamera() as camera:
                # camera setup
                camera.resolution = (640, 480)
                camera.framerate = 30
                camera.hflip = True
                camera.vflip = False

                time.sleep(0.3)

                stream = io.BytesIO()
                for foo in camera.capture_continuous(stream, 'jpeg', quality=90,
                                                    use_video_port=True):
                    # store frame
                    stream.seek(0)
                    cls.frame = stream.read()

                    # reset stream for next frame
                    stream.seek(0)
                    stream.truncate()

                    time.sleep(0.01)

                    # if there hasn't been any clients asking for frames in
                    # the last 10 seconds stop the thread
                    if time.time() - cls.last_access > 3:
                        break
        cls.thread = None
