import numpy as np
import timeit

# import exposed vscmg envirnment
import VSCMG_ENV

# create VSCMG satellite object
sat=VSCMG_ENV.Satellite()

# Show Satellite Information
sat.Info()

# Change Satellite Inertia
I=np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
Jg=0.01 # Gimbal Inertia
Jw=0.01 # Reaction Wheel Inertia
sat.setInertia(I,Jg, Jw)
sat.Info()

# set satellite states with array of(quaternion, body rates, RW velocities, gimbal angles)
IC=np.array([1.0, 0.0, 0.0, 0.0,	0.0, 0.0, 0.0,	1000.0, 1000.0, 1000.0, 1000.0,  0.0, 0.0, 0.0, 0.0])
sat.setState(IC)

# Show Satellite Information
sat.Info()

# control action array of (4 x RW acerlations, 4 x gimbal rates)
action=np.array([0.0,0.0,0.0,0.0, 0.01,0.01,0.01,0.01])
t = 0	# time
dt = 0.01	# step size

# take dynamical step of size dt with control action as input

states=sat.step(action,t,dt)
'''
print(states)
for i in range(100000):
    states=sat.step(action,t,dt)
'''    
print(sat.step(action,t,dt))
# Performance
stp="""
import numpy as np
import timeit
import VSCMG_ENV
sat=VSCMG_ENV.Satellite()
IC=np.array([1.0, 0.0, 0.0, 0.0, 0.0 ,0.0, 0.0, 1000.0, 1000.0, 1000.0, 1000.0,  0.0, 0.0, 0.0, 0.0])
sat.setState(IC)
np.set_printoptions(precision=4, suppress=True)
"""
stmt="""
action=np.array([0.0,0.0,0.0,0.0, 0.1,0.1,0.1,0.1])
sat.step(action,0.0,0.01)
"""
N=100000
print(timeit.timeit(stmt,number=N,setup=stp))