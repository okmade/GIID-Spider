import time
import sys
sys.dont_write_bytecode = True
sys.path.append('Modules')
from Flaskcomm import *
from Servocomm import *
from Robot import *

""" Main function loop """
#data_spider = ['N','N',30,512,512]
#   data_spider[0] = "MODE", 'A' = Analogo, 'D' = Digital, 'N' = None
#   data_spider[1] = If MODE == 'D', 'N' = None, 'F' = Forward, 'B' = Back, 'L' = Left, 'R' = Rigth, 'S' = Stop
#                    If MODE == 'A', Lateral Movement Velocity Control 0 to 100
#   data_spider[2] = If MODE == 'D', Velocity Control 0 to 100
#                    If MODE == 'A', Velocity Control 0 to 100
#   CAMERA CONTROL NO IMPLEMENTED YET
#   data_spider[3] = Right Joystick Vertical 0 to 1023 to control camera
#   data_spider[4] = Right Joystick Horizontal 0 to 1023 to control camera

class control_data(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self,name='Controling_data')
        self.data = ['N','N',30,512,512]
        self.new_data = False
        self.start()
        
    def validate_data(self,data):
        if (data[0] == 'D'):
            if (data[1] == 'F' or data[1] == 'B' or
                data[1] == 'R' or data[1] == 'L' or
                data[1] == 'S'):
                    return True
        elif (data[0] == 'A'):
            if (data[1] != None and data[2] != None ):
                return True
        return False
        
    def set_new_data(self, data_entry):
        if (self.validate_data(data_entry)):
            self.data = data_entry
            self.new_data = True
        
    def run(self):
        while (True):
            if (self.new_data):
                if (self.data[0] == 'D'):
                    hexapod.set_speed(self.data[2]) #No impemented yet
                    if (self.data[1] == 'F'):
                        hexapod.set_movement('forward')
                    if (self.data[1] == 'B'):
                        hexapod.set_movement('backward')
                    if (self.data[1] == 'R'):
                        hexapod.set_movement('right')
                    if (self.data[1] == 'L'):
                        hexapod.set_movement('left')
                    if (self.data[1] == 'S'):
                        hexapod.set_movement('stop')
                elif (self.data[0] == 'A'):
                    pass
                self.new_data = False
            else:
                time.sleep(0.01) # keeps infinite while loop from killing processor


if __name__ == '__main__':
    controller = Controller()                               # Servo controller
    hexapod = Hexapod(controller)                           # Hexapod Controller
    control_data = control_data()                           # Data Proccess Controller
    __builtins__.hexapod = hexapod
    __builtins__.control_data = control_data
    app.run(host='0.0.0.0', debug=False, threaded=True)     #Run Routines
    del hexapod
    del controller
    print("Quitting!")
