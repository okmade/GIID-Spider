import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

# Fixing random state for reproducibility
np.random.seed(19680801)
fig = plt.figure()
plotxyz = p3.Axes3D(fig)

def draw(data):
    xoa = np.linspace(data[0][0], data[1][0], 50)
    yoa = np.linspace(data[0][1], data[1][1], 50)
    zoa = np.linspace(data[0][2], data[1][2], 50)

    xab = np.linspace(data[1][0], data[2][0], 50)
    yab = np.linspace(data[1][1], data[2][1], 50)
    zab = np.linspace(data[1][2], data[2][2], 50)

    xbc = np.linspace(data[2][0], data[3][0], 50)
    ybc = np.linspace(data[2][1], data[3][1], 50)
    zbc = np.linspace(data[2][2], data[3][2], 50)

    plotxyz.clear()
    plotxyz.plot(xoa, yoa, zoa, zdir='z')
    plotxyz.plot(xab, yab, zab, zdir='z')
    plotxyz.plot(xbc, ybc, zbc, zdir='z')

    plotxyz.set_xlim3d([(min(data[0][0],data[1][0],data[2][0],data[3][0])-50), (max(data[0][0],data[1][0],data[2][0],data[3][0])+50)])
    plotxyz.set_xlabel('X')

    #plotxyz.set_ylim3d([(min(data[0][1],data[1][1],data[2][1])-50), (max(data[0][1],data[1][1],data[2][1])+50)])
    plotxyz.set_ylabel('Y')

    #plotxyz.set_zlim3d([(min(data[0][2],data[1][2],data[2][2])-50), (max(data[0][2],data[1][2],data[2][2])+50)])
    plotxyz.set_zlabel('Z')

    plotxyz.set_xlim3d(-100, 100)
    plotxyz.set_ylim3d(-20, 110)
    plotxyz.set_zlim3d(-20, 150)
    

    plotxyz.set_title('Spider Robot')
    #plotxyz.view_init(elev=20., azim=45)
    plotxyz.view_init(elev=90., azim=45)