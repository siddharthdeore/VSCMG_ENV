# VSCMG Dynamical Envirnment Wrapper
## Python Usage
Import exposed python class with
```python
import numpy as np
import VSCMG_ENV

sat=VSCMG_ENV.Satellite()
action=np.array([0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0])

sat=VSCMG_ENV.Satellite()
IC=np.array([1.0, 0.0, 0.0, 0.0, 0.0 ,0.0, 0.0, 1000.0, 1000.0, 1000.0, 1000.0,  0.0, 0.0, 0.0, 0.0])
sat.setState(IC)


```


## Project Properties
```C++
	C/C++ > Additional Include Directories
		%PYTHON_ROOT%\include;
		%BOOST_ROOT%;
		%ARMADILLO_ROOT%\include;

	Linker > Additional Library Directories
		%PYTHON_ROOT%\libs;
		%BOOST_ROOT%\stage\lib;
		%ARMADILLO_ROOT%\lib_win64;
```