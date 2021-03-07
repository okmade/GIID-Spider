#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import socket
from Camera_pi import Camera
from threading import Thread
import threading
import tempfile
import subprocess
import os

DIR_MODULE_PATH = os.path.dirname(os.path.abspath(__file__))
MJPG_STREAMER_PATH = DIR_MODULE_PATH + "/mjpg-streamer/mjpg_streamer"
INPUT_PATH = DIR_MODULE_PATH + "/mjpg-streamer/input_raspicam.so -x 640 -y 480 -fps 15 -vs -vf -hf"
OUTPUT_PATH = DIR_MODULE_PATH + "/mjpg-streamer/output_http.so -w " + DIR_MODULE_PATH + "/mjpg-streamer/www"

cmd = '%s -i "%s" -o "%s" &' % (MJPG_STREAMER_PATH, INPUT_PATH, OUTPUT_PATH)

class StreamVideo(threading.Thread):
    def __init__(self, videoSource = "mjpg-streamer"):
        self.videoSource = videoSource                          #source = "Flask-TCP", "Sockets-TCP", "mjpg-streamer"
        self.is_changed = False
        self.camera = Camera()
        self.startVideoSource()
        threading.Thread.__init__(self, name='VideoStreaming')
        self._running = True
        self.start()

    def terminate(self):  
        self._running = False
    
    def get_active(self):
        return self._running
    
    def startVideoSource(self):
        if (self.videoSource == "mjpg-streamer"):
            files = os.listdir('/dev')
            print(cmd)
            video_files = [f for f in files if 'video' in f]
            if not video_files:
                raise IOError("Camera is not connected correctly")
            self.run_command(cmd)
        elif (self.videoSource == "Sockets-TCP"):
            self.camSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def stopVideoSource(self):
        if (self.videoSource == "mjpg-streamer"):
            pid = self.run_command('ps -A | grep mjpg_streamer | grep -v "grep" | head -n 1')
            if pid == '':
                return False
            else:
                self.run_command('sudo kill %s' % pid)
                return True
            print(pid)
            time.sleep(1)
        elif (self.videoSource == "Sockets-TCP"):
            self.camSocket.close()

    def structureByteHeader(self,numberBytes,desiredLength):
        while len(numberBytes) < desiredLength:
            numberBytes +=  str(" ").encode()
        return numberBytes

    def run_command(self,cmd):
        with tempfile.TemporaryFile() as f:
            subprocess.call(cmd, shell=True, stdout=f, stderr=f)
            f.seek(0)
            output = f.read()
        return output

    def change_video(self, new_videoSource):
        if ((new_videoSource == "Flask-TCP") or
            (new_videoSource == "Sockets-TCP") or
            (new_videoSource == "mjpg-streamer")):
            if (new_videoSource != self.videoSource):
                self.is_changed = True
                self.stopVideoSource()
                self.videoSource = new_videoSource
                self.startVideoSource()
                self.is_changed = False

    def run(self):
        while (self._running):
            if (self.videoSource == "Sockets-TCP"):
                self.camSocket.bind(("",8081))
                self.camSocket.listen(2)
                while ((self.is_changed == False) and (self._running)):
                    try:
                        client,address = self.camSocket.accept()
                        byteString = self.camera.get_frame("cv2")
                        fileSize = len(byteString)
                        byteString = self.structureByteHeader(str(fileSize).encode(),8)+byteString
                        client.sendall(byteString)
                        #print("3")
                    except Exception as e:
                        print(e)
            else:
                time.sleep(3)
                pass
        self.stopVideoSource()

if __name__ == "__main__":
    StreamVideo()
