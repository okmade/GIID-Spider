import time
import sys
sys.dont_write_bytecode = True
sys.path.append('Modules')
from Flaskcomm import *
from Servocomm import *
from Robot import *

""" Main function loop """

class runMovement(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self, name='Updating')
        self._running = True
        self.oldCurrentTime = 0
        self.start()

    def terminate(self):  
        self._running = False
    
    def get_active(self):
        return self._running

    def run(self):
        while self._running:
            currentTime = int(round(time.time() * 1000))
            delta = (currentTime - self.oldCurrentTime) / 1000
            self.oldCurrentTime = currentTime
            if (control_data.new_data == False):
                Robot.update(delta)
                for j in range(len(Robot.legs)):
                    Robot.servosCon.set_leg_angles(j, Robot.legs[j].servoAngles)


class control_data(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self,name='Controling_data')
        self.data    = [0, 0, 0, 0, 0, 50, 0, 0, 0, 0, 0, 0]
        self.oldData = [0, 0, 0, 0, 0, 50, 0, 0, 0, 0, 0, 0]
        self.new_data = False
        self.start()
        
    def validate_data(self, oldData, newData):
            if (newData != None):
                return float(newData)
            else:
                return oldData
        
    def set_new_data(self, data_entry):
        self.data = data_entry
        self.new_data = True
        
    def run(self):
        while (True):
            if (self.new_data):
                if (self.data != self.oldData):

                    '''
                    if ((self.data[1] != None) and (self.data[1] != self.oldData[1])):
                        Robot.moveDistance = self.data[1]
                    if ((self.data[2] != None) and (self.data[2] != self.oldData[2])):
                        Robot.moveAngle = self.data[2]
                    if ((self.data[3] != None) and (self.data[3] != self.oldData[3])):
                        Robot.turnAngle = self.data[3]
                    if ((self.data[4] != None) and (self.data[4] != self.oldData[4])):
                        Robot.timeScale = self.data[4]
                    if ((self.data[5] != None) and (self.data[5] != self.oldData[5])):
                        Robot.bodyOffX = self.data[5]
                    if ((self.data[6] != None) and (self.data[6] != self.oldData[6])):
                        Robot.bodyOffY = self.data[6]
                    if ((self.data[7] != None) and (self.data[7] != self.oldData[7])):
                        Robot.bodyHigh = self.data[7]
                    if ((self.data[8] != None) and (self.data[8] != self.oldData[8])):
                        Robot.angleRoll = self.data[8]
                    if ((self.data[9] != None) and (self.data[9] != self.oldData[9])):
                        Robot.anglePitch = self.data[9]
                    if ((self.data[10] != None) and (self.data[10] != self.oldData[10])):
                        Robot.angleYaw = self.data[10]
                    '''

                    Robot.moveDistance = self.validate_data(Robot.moveDistance, self.data[1])
                    Robot.moveAngle = self.validate_data(Robot.moveAngle, self.data[2])
                    Robot.turnAngle = self.validate_data(Robot.turnAngle, self.data[3])
                    Robot.timeScale = self.validate_data(Robot.timeScale, self.data[4])
                    Robot.bodyOffX = self.validate_data(Robot.bodyOffX, self.data[5])
                    Robot.bodyOffY = self.validate_data(Robot.bodyOffY, self.data[6])
                    Robot.bodyHigh = self.validate_data(Robot.bodyHigh, self.data[7])
                    Robot.angleRoll = self.validate_data(Robot.angleRoll, self.data[8])
                    Robot.anglePitch = self.validate_data(Robot.anglePitch, self.data[9])
                    Robot.angleYaw = self.validate_data(Robot.angleYaw, self.data[10])

                    Robot.BodyIK()
                    
                    '''
                    if ((self.data[11] != None) and (self.data[11] != self.oldData[11])):
                        for i in range(len(Robot.legs)):
                            Robot.legs[i].homeDistance = self.data[11]
                            Robot.legs[i].updateHomePoint(self.data[11])
                    '''
                    
                    for i in range(len(Robot.legs)):
                        Robot.legs[i].homeDistance = self.validate_data(Robot.legs[i].homeDistance, self.data[11])
                        if (self.data[11] != None):
                            Robot.legs[i].updateHomePoint(self.data[11])

                    if ((Robot.isHome == True) and (Robot.moveDistance == 0) and (Robot.turnAngle == 0)):
                        for j in range(len(Robot.legs)):
                            Robot.legs[j].LegIK(Robot.legs[j].homePoint[0], Robot.legs[j].homePoint[1], Robot.legs[j].homePoint[2])
                            Robot.servosCon.set_leg_angles(j, Robot.legs[j].servoAngles)
                    
                    self.oldData = self.data
                self.new_data = False
            else:
                pass


if __name__ == '__main__':
    controller = Controller()                               # Servo controller
    Robot = BodyHex(controller)                             # Hexapod Controller
    control_data = control_data()                           # Data Proccess Controller
    Run_Movement = runMovement()
    __builtins__.Robot = Robot
    __builtins__.control_data = control_data
    __builtins__.Run_Movement = Run_Movement
    app.run(host='0.0.0.0', debug=False, threaded=True)     #Run Routines
    del Robot
    del controller
    print("Quitting!")
