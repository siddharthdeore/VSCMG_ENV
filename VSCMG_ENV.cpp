#define _CRT_SECURE_NO_WARNINGS
#include "VSCMG.h"

namespace bn = boost::python::numpy;

struct Satellite {
	state_type X= { 0.0,0.0,0.0,1.0,   0.0,0.0,0.0, 100.0,100.0,100.0,100.0, 0.0,0.0,0.0,0.0, };
	VSCMG satellite;
	boost::numeric::odeint::runge_kutta_cash_karp54< state_type > stepper;

	void setState(boost::python::numpy::ndarray& input) {
		double* input_ptr = reinterpret_cast<double*>(input.get_data());
		//std::vector<double> v(input_size);
		for (int i = 0; i < 15; ++i) {
			this->X[i] = *(input_ptr + i );
		}

	}
	// Exposed Step Function Take a step and return Numpy Array
	bn::ndarray step(boost::python::numpy::ndarray& input, double t, double dt) {
		double* input_ptr = reinterpret_cast<double*>(input.get_data());
		std::vector<double> v(8);
		for (int i = 0; i < 8; ++i) {
			v[i] = *(input_ptr + i);
		}
		//satellite.controlAction(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
		satellite.controlAction(v);
		stepper.do_step(satellite, this->X, t, dt);

		// result data for python 
		Py_intptr_t shape[1] = { this->X.size() };
		bn::ndarray result = bn::zeros(1, shape, bn::dtype::get_builtin<double>());
		std::copy(this->X.begin(), this->X.end(), reinterpret_cast<double*>(result.get_data()));
		return result;
	}
};

BOOST_PYTHON_MODULE(VSCMG_ENV) {
	boost::python::numpy::initialize();
	using namespace boost::python;
	class_<Satellite>("Satellite")
		.def("step", &Satellite::step)
		.def("setState", &Satellite::setState)
		;


}
/*
int main()
{
	state_type X, dxdt;
	X = { 1,0,0,0,   0,0,0, 100,100,100,100, 0,0,0,0, };
	VSCMG satellite;
	boost::numeric::odeint::runge_kutta_cash_karp54< state_type > stepper;
	double t = 0;
	double dt = 0.1;
	while (t < 1000) {
		//satellite.controlAction(0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0);
		stepper.do_step(satellite,
						X,
						t,
						dt);
		t += dt;
		write_state(X, t);
	}
}
*/