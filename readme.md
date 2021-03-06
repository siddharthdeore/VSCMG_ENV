# Dynamical environmet agent wrapper for Variable Speed Comtrol Moment Gyroscope 

[![HitCount](http://hits.dwyl.com/siddharthdeore/VSCMG_ENV.svg)](http://hits.dwyl.com/siddharthdeore/VSCMG_ENV)
[![MIT license](https://img.shields.io/badge/License-MIT-blue.svg)](https://lbesson.mit-license.org/)

Python wrapper for attitude dynamics of VSCMG in pyramid cluster.

For Reinforcement Learning, It is important to have precise environment, responsive to actions taken by agent. Matlab based ODE integrator are easy to implement but slower in performance. Slight improvement  in performace with python odeint integrators, since order of magnitude of steps to be performed by a learning agent are in billions. To improve computation speed C++ based dynamical envirnomet is needed
This dynamical stepper implemented in C++ has shown significantly large improvements in speed without compromising solution accuracy. To simplify access to envirnoment, python wrapper is implemented to expose VSCMG_ENV stepper.
## Binaries
Stable Release (https://github.com/siddharthdeore/VSCMG_ENV/releases/tag/VSCMG_ENV)
Copy VSCMG_ENV.pyd inside *your_python_root/DLLs* directory

## Python Usage
Acess library from python.
```python
import numpy as np
import VSCMG_ENV

# create VSCMG satellite object
sat=VSCMG_ENV.Satellite()

# set satellite states with array of(quaternion, body rates, RW velocities, gimbal angles)
IC=np.array([1.0, 0.0, 0.0, 0.0,	0.0, 0.0, 0.0,	1000.0, 1000.0, 1000.0, 1000.0,  0.0, 0.0, 0.0, 0.0])
sat.setState(IC)

# control action array of (4 x RW acerlations, 4 x gimbal rates)
action=np.array([0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0])
t = 0	# time
dt = 0.01	# step size

# take dynamical step of size dt with control action as input
states=sat.step(action,t,dt)

# step without control action
states=sat.stepNull(t,dt) 

# step without control action
states=sat.stepNull(t,dt) 

# Reset to default Orientation and zero kinetic energy
sat.resetState()

# Show Satellite Information
sat.Info()

# Body Inertia
I=np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
Jg=0.01 # Gimbal Inertia
Jw=0.01 # Reaction Wheel Inertia

# Set body, Gimbal and Reaction wheel Inertia
sat.setInertia(I,Jg, Jw)
sat.Info()

print(states)

```

## Compile Requrements
	Visual Studio C++ 2019 (MSVC 14.23)
	Python 3.7
	Boost 1.72.0 (https://www.boost.org/)
	Armadillo 9.850.1 (http://arma.sourceforge.net/)


## Visual Studio C++ Project Properties
```batch
C/C++ >> Additional Include Directories
	%PYTHON_ROOT%\include;
	%BOOST_ROOT%;
	%ARMADILLO_ROOT%\include;

Linker >> Additional Library Directories
	%PYTHON_ROOT%\libs;
	%BOOST_ROOT%\stage\lib;
	%ARMADILLO_ROOT%\lib_win64;
```


## TODO
CMake Support

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

---
## License
[MIT](https://choosealicense.com/licenses/mit/)
