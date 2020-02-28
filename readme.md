# VSCMG Dynamical Envirnment Wrapper
## Python Usage
Import exposed python class with
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
dt = 0	# step size

# take dynamical step of size dt with control action as input

states=sat.action(action,t,dt)

print(state)

```


## Project Properties
```c
	C/C++ > Additional Include Directories
		%PYTHON_ROOT%\include;
		%BOOST_ROOT%;
		%ARMADILLO_ROOT%\include;

	Linker > Additional Library Directories
		%PYTHON_ROOT%\libs;
		%BOOST_ROOT%\stage\lib;
		%ARMADILLO_ROOT%\lib_win64;
```
