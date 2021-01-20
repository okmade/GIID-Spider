#!/usr/bin/env python

import tempfile
import subprocess
import os


##MJPG_STREAMER_PATH = "/usr/local/bin/mjpg_streamer"
##INPUT_PATH = "/usr/local/lib/mjpg-streamer/input_raspicam.so -x 320 -y 240 -fps 15 -vs -hf"
##OUTPUT_PATH = "/usr/local/lib/mjpg-streamer/output_http.so -w /usr/local/share/mjpg-streamer/www"

DIR_MODULE_PATH = os.path.dirname(os.path.abspath(__file__)) 
MJPG_STREAMER_PATH = DIR_MODULE_PATH + "/mjpg-streamer/mjpg_streamer"
INPUT_PATH = DIR_MODULE_PATH + "/mjpg-streamer/input_raspicam.so -x 640 -y 480 -fps 15 -vs -vf -hf"
OUTPUT_PATH = DIR_MODULE_PATH + "/mjpg-streamer/output_http.so -w " + DIR_MODULE_PATH + "/mjpg-streamer/www"

cmd = '%s -i "%s" -o "%s" &' % (MJPG_STREAMER_PATH, INPUT_PATH, OUTPUT_PATH)

def run_command(cmd):
	with tempfile.TemporaryFile() as f:
		subprocess.call(cmd, shell=True, stdout=f, stderr=f)
		f.seek(0)
		output = f.read()
	return output

def start():
	files = os.listdir('/dev')
	if 'video0' in files:
		run_command(cmd)
	else:
		raise IOError("Camera is not connected correctly")

def start():
	files = os.listdir('/dev')
	print(cmd)
	video_files = [f for f in files if 'video' in f]
	if not video_files:
		raise IOError("Camera is not connected correctly")
	run_command(cmd)

def get_host():
	return run_command('hostname -I')

def stop():
	pid = run_command('ps -A | grep mjpg_streamer | grep -v "grep" | head -n 1')
	if pid == '':
		return False
	else:
		run_command('sudo kill %s' % pid)
		return True

def restart():
	stop()
	start()
	return True

if __name__ == "__main__":
	start()