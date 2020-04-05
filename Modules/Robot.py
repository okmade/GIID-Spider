import math
import time
from Servocomm import runMovement, runUpdate, Controller

#Initial Variables

POS_LEFT = 1                #Position Left of Leg on the body
POS_RIGHT = 2               #Position Right of Leg on the body
LONG_LEG = 80               #Long of Leg in cm
LONG_ANT_LEG = 45           #Long of AntLeg in cm
longftob = 50               #Long of Body from Floor in cm
longinx = 50                #Long between Body and Leg in cm
movinx = 30                 #Long of movement over floor in cm
moviny = 30                 #Long of movement in altitud of leg in cm

points_total = 60           #Amount of movement for servos Min=6 and Max=200
control_time = 20
min_control_steps = 10
max_control_steps = 200
min_control_time = 1
max_control_time = 100

BASE_TIME = 1000

rad=(math.pi/180)
grad=(180/math.pi)

class Hexapod:
    def __init__(self,con):
        self.con = con
        self.current_address = None
        self.task = None
        self.mode = 'steps'
        self.steps = points_total
        self.current_step = 0
        self.time = control_time
        self.control_steps = [ min_control_steps, max_control_steps]
        self.control_time = [ min_control_time, max_control_time]
        self.QServos = []
        
        self.RF  = leg(POS_RIGHT,0,LONG_ANT_LEG,LONG_LEG,longftob,0,longinx,60,movinx,moviny,points_total)
        self.RM  = leg(POS_RIGHT,0,LONG_ANT_LEG,LONG_LEG,longftob,0,longinx,90,movinx,moviny,points_total)
        self.RB  = leg(POS_RIGHT,0,LONG_ANT_LEG,LONG_LEG,longftob,0,longinx,120,movinx,moviny,points_total)

        self.LF  = leg(POS_LEFT,0,LONG_ANT_LEG,LONG_LEG,longftob,0,longinx,60,movinx,moviny,points_total)
        self.LM  = leg(POS_LEFT,0,LONG_ANT_LEG,LONG_LEG,longftob,0,longinx,90,movinx,moviny,points_total)
        self.LB  = leg(POS_LEFT,0,LONG_ANT_LEG,LONG_LEG,longftob,0,longinx,120,movinx,moviny,points_total)

        self.legs = [self.RF,
                     self.RM,
                     self.RB,
                     self.LF,
                     self.LM,
                     self.LB]
        
        self.tripodR = [self.RF,self.RB,self.LM]
        self.tripodL = [self.LF,self.LB,self.RM]
        
        self.update_angles(self.steps)

    def set_mode_movement(self, mode):                                  #mode = time/steps
        if (mode == 'time'):
            self.mode = 'time'
        elif (mode == 'steps'):
            self.mode = 'steps'

    def set_speed(self, value=25):
        if (value != 'None'):
            new_vel = int(value)
            if (self.mode == 'time'):
                if (new_vel > self.control_time[1]):
                    new_vel = self.control_time[1]
                elif (new_vel < self.control_time[0]):
                    new_vel = self.control_time[0]
                self.time = new_vel
            elif (self.mode == 'steps'):
                if (new_vel > self.control_steps[1]):
                    new_vel = self.control_steps[1]
                elif (new_vel < self.control_steps[0]):
                    new_vel = self.control_steps[0]
                for i in range(0,6):
                    self.legs[i].update_tpoints(new_vel)
                self.update_angles(new_vel)

    def update_angles(self, value=None): 
        task = runUpdate(self.task_update_angles, value)
        task.join()

    def task_update_angles(self, value_steps):
        if (value_steps == None):
            value_steps = self.steps
        QServos_aux = []
        for i in range(0,value_steps):
            QServosLeg_aux = []
            for j in range(0,len(self.legs)):
                Q1, Q2, Q3 = self.legs[j].getanglesforservos(i)
                QServosLeg_aux.append([Q1,Q2,Q3])
            QServos_aux.append(QServosLeg_aux)
        self.QServos = QServos_aux
        
        oldvalue = self.steps
        self.steps = value_steps
        self.current_step = round((self.current_step*value_steps)/oldvalue)

    def set_movement(self, address):
        if (self.task != None):
            self.task.terminate()
        self.current_address = address
        if (address == 'forward'):
            self.task = runMovement(self.go_forward,self.con)
        elif (address == 'backward'):
            self.task = runMovement(self.go_backward,self.con)
        elif (address == 'left'):
            self.task = runMovement(self.go_left,self.con)
        elif (address == 'right'):
            self.task = runMovement(self.go_right,self.con)
        elif (address == 'forward_anag'):
            self.task = runMovement(self.go_forward_anag,self.con)
        elif (address == 'backward_anag'):
            self.task = runMovement(self.go_backward_anag,self.con)
        elif (address == 'stop' and self.task != None):
            self.task.terminate()

#   Movement has not been implemented yet (I need structure to test)
    def go_forward(self, con):
        print ("Forward")
        if (self.current_step >= self.steps):
            self.current_step = 0
        for i in range(0, len(self.legs)):
            self.con.set_leg_angles(i,self.QServos[self.current_step][i])
        self.current_step += 1
        time.sleep(0.005)

    def go_backward(self, con):
        print ("Backward")
        if (self.current_step < 0):
            self.current_step = self.steps - 1
        for i in range(0, len(self.legs)):
            self.con.set_leg_angles(i,self.QServos[self.current_step][i])
        self.current_step -= 1
        time.sleep(0.005)

    def go_left(self, con):
        print ("Left")
        time.sleep(0.2)

    def go_right(self, con):
        print ("Right")
        time.sleep(0.2)

    def go_forward_anag(self, con):
        print ("Forward Analogico")
        time.sleep(0.2)

    def go_backward_anag(self, con):
        print ("Backward Analogico")
        for i in range(180,0,-1):
            if (self.current_address != 'backward_anag'):
                break
            self.con.set_angle(11,i)
            self.con.set_angle(15,i)
            time.sleep(0.1)


class leg():
    def __init__(self,xpos,ypos,longaleg,longleg,altcodo,cab,pfx,Q1cent,movpfx,movpfy,tpoints):
        self.xpos=xpos
        self.ypos=ypos
        self.L1=longaleg
        self.L2=longleg
        self.altcodo=altcodo
        self.cab=cab
        self.pfx=pfx
        self.Q1cent=Q1cent
        self.movpfx=movpfx
        self.movpfy=movpfy
        self.tpoints=tpoints/2
        self.estateactive=True
        if self.checklimits():
            self.disactive()

    def isactive(self):
        return self.estateactive

    def active(self):
        self.estateactive=True

    def disactive(self):
        self.estateactive=False

    def update_tpoints(self,new_tpoints):
        bef_tpoints = self.tpoints
        self.tpoints = new_tpoints/2
        if self.checklimits():
            self.tpoints = bef_tpoints
        self.updatexmov()

    def update_altcodo(self,new_altcodo):
        bef_altcodo = self.altcodo
        self.altcodo = new_altcodo
        if self.checklimits():
            self.altcodo = bef_altcodo
        self.updatexmov()
    
    def update_pfx(self,new_pfx):
        bef_pfx = self.pfx
        if self.checklimits():
            self.pfx = bef_pfx
        self.updatexmov()
    
    def update_movpfx(self,new_movpfx):
        bef_movpfx = self.movpfx
        if self.checklimits():
            self.movpfx = bef_movpfx
        self.updatexmov()

    def update_movpfy(self,new_movpfy):
        bef_movpfy = self.movpfy
        if self.checklimits():
            self.movpfy = bef_movpfy
        self.updatexmov()

    def checklimits(self):
        self.updatexmov()
        #print(self.L1+self.L2)
        #print(math.sqrt((math.pow(self.altcodo,2)+
        #math.pow(self.pfx,2)+math.pow(self.xmovini,2))))

        #print(math.sqrt((math.pow(self.altcodo,2)+
        #math.pow(self.pfx,2)+math.pow(self.xmovend,2))))

        #print(math.sqrt((math.pow((self.altcodo-self.movpfy),2)+
        #math.pow(self.pfx,2)+math.pow((self.xinc * (self.tpoints/2)),2))))

        if ((self.L1 + self.L2) <= 
        math.sqrt((math.pow(self.altcodo,2)+
        math.pow(self.pfx,2)+math.pow(self.xmovini,2)))):
            return True
        elif ((self.L1 + self.L2) <= 
        math.sqrt((math.pow(self.altcodo,2)+
        math.pow(self.pfx,2)+math.pow(self.xmovend,2)))):
            return True
        elif ((self.L1 + self.L2) <= 
        math.sqrt((math.pow((self.altcodo-self.movpfy),2)+
        math.pow(self.pfx,2)+math.pow((self.xinc * (self.tpoints/2)),2)))):
            return True
        else:
            return False

    def updatexmov(self):
        global rad,grad
        if math.sin(self.Q1cent*rad) == 0:
            robini=0
        else:
            robini=self.pfx/math.sin(self.Q1cent*rad)
        xobini=robini*math.cos(self.Q1cent*rad)
        self.xmovini=(xobini-(self.movpfx/2))
        self.xmovend=self.movpfx+self.xmovini
        self.xinc = (self.xmovend-self.xmovini)/self.tpoints

    def getangles(self,point):
        global rad,grad
        if self.estateactive == False:          #It's neccessary to check
            Q1 = 90
            Q2 = 90
            Q3 = 90
            movx = 0
        else:
            self.updatexmov()
            if (point >= self.tpoints*2):
                point = point % self.tpoints
                
            if (point <= self.tpoints):
                movx = (self.xinc*point)+self.xmovini
            else:
                movx = self.xmovend - (self.xinc*(point % self.tpoints))
            Q1 = (math.atan2(self.pfx,movx))*grad
            if math.sin(Q1*rad) == 0:
                rob = 0
            else:
                rob = self.pfx/math.sin(Q1*rad)
            if (point <= self.tpoints):
                zob = (((-self.movpfy)/(math.pow((self.movpfx/2),2))*(math.pow(((movx-self.xmovini)-(self.movpfx/2)),2)))+self.movpfy)
                ladoz = zob-self.altcodo
            else:
                ladoz = -self.altcodo
            Hipotenusa = math.sqrt(math.pow(rob,2)+math.pow(ladoz,2))
            Alfa = (math.atan2(ladoz,rob))*grad
            Beta = (math.acos(((math.pow(self.L1,2))+(math.pow(Hipotenusa,2))-(math.pow(self.L2,2)))/(2*self.L1*Hipotenusa)))*grad
            if self.cab == 0:
                Q2 = (Beta+Alfa)
            else:
                Q2 = (Beta-Alfa)
            Gamma = math.acos((math.pow(self.L1,2)+math.pow(self.L2,2)-math.pow(Hipotenusa,2))/(2*self.L1*self.L2))
            Q3 = (Gamma-(180*rad))*grad
        return Q1,Q2,Q3,movx

    def getanglesforservos(self,point):
        Qa1,Qa2,Qa3,movxa=self.getangles(point)
        if self.xpos == 2:
            Qf1= -Qa1+180
            Qf2= Qa2+90
            Qf3= -Qa3
        else:
            Qf1= Qa1
            Qf2= -Qa2+90
            Qf3= Qa3+180
        return Qf1,Qf2,Qf3

    def getpointstodraw(self,point):
        global rad,grad

        Qa1,Qa2,Qa3,movxa=self.getangles(point)
        print (self.getangles(point))

        PXo= self.xpos
        PYo= self.ypos
        PZo= self.altcodo

        PXa=(self.L1*math.cos(Qa2*rad)*math.cos(Qa1*rad))+self.xpos
        PYa=(self.L1*math.cos(Qa2*rad)*math.sin(Qa1*rad))+self.ypos
        PZa=self.L1*math.sin(Qa2*rad)+self.altcodo
        
        PXb=movxa+PXo
        PYb=self.pfx+PYo
        PZb=(self.L2*math.sin((Qa2+Qa3)*rad))+PZa
        
        return [[PXo,PYo,PZo],[PXa,PYa,PZa],[PXb,PYb,PZb]]  #Fila x Columna


#list_of_lists = [[[ 1,  2,  3], [ 4,  5,  6], [ 7,  8,  9]],
                 #[[11, 12, 13], [14, 15, 16], [17, 18, 19]],
                 #[[21, 22, 23], [24, 25, 26], [27, 28, 29]]]
#print (list_of_lists)
#print (list_of_lists[1])
#print (list_of_lists[2][2])

#controller = Controller() # Servo controller
#hexy = Hexapod(controller)
#hexy.update_angles(20)
#print(hexy.QServos)


#controller = Controller() # Servo controller
#hexy = Hexapod(controller)
#hexy.con.set_pos_init()
#hexy.set_mode_movement('steps')
#hexy.set_speed(100)
#hexy.set_speed(60)
#hexy.update_angles(100)
#hexy.update_angles(60)
#hexy.set_movement("backward")
#time.sleep(4)
#hexy.set_movement("backward_anag")
#time.sleep(3)
#hexy.set_speed(20)
#hexy.set_speed(200)
#time.sleep(6)
#hexy.task.terminate()
#hexy.set_movement("backward")
#time.sleep(4)
#hexy.task.terminate()
