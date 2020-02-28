# VSCMG Dynamical Envirnment Wrapper

Visual Studio C++ Project Properties
'''C++
	C/C++ > Additional Include Directories
		%PYTHON_ROOT%\include;
		%BOOST_ROOT%;
		%ARMADILLO_ROOT%\include;

	Linker > Additional Library Directories
		%PYTHON_ROOT%\libs;
		%BOOST_ROOT%\stage\lib;
		%ARMADILLO_ROOT%\lib_win64;
'''
