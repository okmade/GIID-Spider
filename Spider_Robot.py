from Module_leg import *
from Module_draw import *
points_total=10
Leg1_1 = Leg(0,0,50,111.803,120,0,60,90,50,50,points_total)


def animate(i):
    j=i%(points_total+1)
    draw(Leg1_1.getpointstodraw(j))

ani = animation.FuncAnimation(fig, animate, interval=10)
plt.show()