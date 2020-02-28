import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import math
import numpy as np
import timeit

# import exposed vscmg envirnment
import VSCMG_ENV

def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]

# create VSCMG satellite object
sat=VSCMG_ENV.Satellite()
I=np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
sat.setInertia(I,0.01,0.01)
# set satellite states with array of(quaternion, body rates, RW velocities, gimbal angles)
IC=np.array([1.0, 0.0, 0.0, 0.0,	0.0, 0.0, 0.0,	1000.0, 1000.0, 1000.0, 1000.0,  0.0, 0.0, 0.0, 0.0])
sat.setState(IC)

# control action array of (4 x RW acerlations, 4 x gimbal rates)
action=np.array([0.0,0.0,0.0,0.0, 0.01,0.001,0.001,0.001])

t = 0	# time
dt = 0.01	# step size

# take dynamical step of size dt with control action as input

states=sat.step(action,t,0.1)

action=action*0
win = pg.GraphicsWindow()
win.setWindowTitle('VSCMG')
p1 = win.addPlot(title='Quaternion')
p1.setYRange(-1, 1, padding=0)

p1.addLegend()    
data1 = np.zeros(shape=(1000,))
data2 = np.zeros(shape=(1000,))
data3 = np.zeros(shape=(1000,))
data4 = np.zeros(shape=(1000,))

curve1 = p1.plot(data1,pen=(0,0,255), name="qe_0")
curve2 = p1.plot(data2,pen=(0,255,0), name="qe_1")
curve3 = p1.plot(data3,pen=(255,255,0), name="qe_2")
curve4 = p1.plot(data4,pen=(255,0,0), name="qe_3")
pg.setConfigOptions(antialias=True)

while True:
    #''' Rolling plots
    obs = sat.step(action,0,0.1)
    data1[:-1] = data1[1:]  # shift data in the array one sample left
    data2[:-1] = data2[1:]  # shift data in the array one sample left
    data3[:-1] = data3[1:]  # shift data in the array one sample left
    data4[:-1] = data4[1:]  # shift data in the array one sample left
    #[z,y,x]=quaternion_to_euler(obs[0],obs[1],obs[2],obs[3])
    data1[-1] = obs[0]
    data2[-1] = obs[1]
    data3[-1] = obs[2]
    data4[-1] = obs[3]
    curve1.setData(data1)
    curve2.setData(data2)
    curve3.setData(data3)
    curve4.setData(data4)
    pg.QtGui.QApplication.processEvents()
