from Module_leg import *
from Module_draw import *
import time

points_total=4
Leg1_1 = Leg(0,0,50,111.803,120,0,50,90,50,50,points_total)


while True:
    print("Start Movement")
    for x in range(0,points_total):
        print(Leg1_1.getangles(x))
    print("Stop Movement")
    time.sleep(5)


""" def animate(i):
    j=i%(points_total+1)
    draw(Leg1_1.getpointstodraw(j))

ani = animation.FuncAnimation(fig, animate, interval=2000)
plt.show() """