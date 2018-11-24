import math

rad=(math.pi/180)
grad=(180/math.pi)

class Leg():
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
        self.tpoints=tpoints
        self.estateactive=True
        if self.checklimits():
            self.disactive()

    def isactive(self):
        return self.estateactive

    def active(self):
        self.estateactive=True

    def disactive(self):
        self.estateactive=False
    
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
        print(self.L1+self.L2)
        print(math.sqrt((math.pow(self.altcodo,2)+
        math.pow(self.pfx,2)+math.pow(self.xmovini,2))))

        print(math.sqrt((math.pow(self.altcodo,2)+
        math.pow(self.pfx,2)+math.pow(self.xmovend,2))))

        print(math.sqrt((math.pow((self.altcodo-self.movpfy),2)+
        math.pow(self.pfx,2)+math.pow((self.xinc * (self.tpoints/2)),2))))

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
        if self.estateactive == False:
            Q1 = 90
            Q2 = 53
            Q3 = -135
            movx = 0
        else:
            self.updatexmov()
            movx = (self.xinc*point)+self.xmovini
            Q1 = (math.atan2(self.pfx,movx))*grad
            if math.sin(Q1*rad) == 0:
                rob = 0
            else:
                rob = self.pfx/math.sin(Q1*rad)
            zob = (((-self.movpfy)/(math.pow((self.movpfx/2),2))*(math.pow(((movx-self.xmovini)-(self.movpfx/2)),2)))+self.movpfy)
            ladoz = zob-self.altcodo

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
        
        return [[PXo,PYo,PZo],[PXa,PYa,PZa],[PXb,PYb,PZb]]
        #Fila x Columna