/*
	@author :	Siddharth Deore
	@email	:	deore.1823670@studenti.uniroma1.it
*/

#define _CRT_SECURE_NO_WARNINGS
#include "VSCMG.h"
#include "NumPyArrayData.h"
namespace bn = boost::python::numpy;

struct Satellite {
	state_type X= { 0.0,0.0,0.0,1.0, 0.0,0.0,0.0, 100.0,100.0,100.0,100.0, 0.0,0.0,0.0,0.0, };
	// initialise object
	VSCMG satellite;
	// initialise stepper
	//boost::numeric::odeint::runge_kutta_cash_karp54< state_type > stepper;
	boost::numeric::odeint::runge_kutta_dopri5<state_type> stepper;
	//boost::numeric::odeint::runge_kutta_fehlberg78<state_type> stepper;
	//(Exposed to python) set state of satellite from numpy array 
	void setState(boost::python::numpy::ndarray& input) {
		double* input_ptr = reinterpret_cast<double*>(input.get_data());
		//std::vector<double> v(input_size);
		for (int i = 0; i < 15; ++i) {
			this->X[i] = *(input_ptr + i);
		}

	}
	void setInertia(boost::python::numpy::ndarray& input, double Jg, double Jw) {
		NumPyArrayData<double> data(input);
		arma::mat Jb = {
			{ 0.0220, 0.0000,  0.0000 },
			{ 0.0000, 0.0220,  0.0000 },
			{ 0.0000, 0.0000,  0.0029 },
		};
		/*
			{ 0.0220, 0.0012, -0.0070 },
			{ 0.0012, 0.0220,  0.0012 },
			{-0.0070, 0.0012,  0.0029 },

		*/

		for (short i = 0; i < 3; i++) {
			for (short j = 0; j < 3; j++) {
				Jb(i, j) = data(i, j);
			}
		}
		satellite.setInertia(Jb, Jg, Jw);
	}
	//(Exposed to python) set state of satellite from numpy array 
	void resetState() {
		this->X = { 0.0,0.0,0.0,1.0, 0.0,0.0,0.0, 0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0 };
	}
	// (Exposed to python) Take a dybamical step step and return Numpy Array
	// x = f(x,u,t)
	bn::ndarray step(boost::python::numpy::ndarray& input, double t, double dt) {
		NumPyArrayData<double> d(input);
		std::vector<double> v(8);
		for (int i = 0; i < 8; ++i) {
			v[i] = d(i);
		}
		//satellite.controlAction(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
		satellite.controlAction(v);
		stepper.do_step(satellite, this->X, t, dt);

		Py_intptr_t shape[1] = { this->X.size() };
		bn::ndarray result = bn::zeros(1, shape, bn::dtype::get_builtin<double>());
		std::copy(this->X.begin(), this->X.end(), reinterpret_cast<double*>(result.get_data()));

		double q0 = this->X[0];
		double q1 = this->X[1];
		double q2 = this->X[2];
		double q3 = this->X[3];
		satellite.normalizeQuaternions(q0, q1, q2, q3);
		result[0] = q0;
		result[1] = q1;
		result[2] = q2;
		result[3] = q3;
		return result;
	}
	bn::ndarray stepNull(double t, double dt) {
		std::vector<double> v{0,0,0,0,0,0,0,0};
		//satellite.controlAction(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
		satellite.controlAction(v);
		stepper.do_step(satellite, this->X, t, dt);

		// result data for python 
		Py_intptr_t shape[1] = { this->X.size() };
		bn::ndarray result = bn::zeros(1, shape, bn::dtype::get_builtin<double>());
		std::copy(this->X.begin(), this->X.end(), reinterpret_cast<double*>(result.get_data()));
		return result;
	}
	void Info() {
		satellite.Info();
	}
};

// Expose VSCMG_ENV Class to Python

BOOST_PYTHON_MODULE(VSCMG_ENV) {
	boost::python::numpy::initialize();
	using namespace boost::python;
	class_<Satellite>("Satellite")
		.def("step", &Satellite::step)
		.def("stepNull", &Satellite::stepNull)
		.def("setState", &Satellite::setState)
		.def("resetState", &Satellite::resetState)
		.def("setInertia", &Satellite::setInertia)
		.def("Info", &Satellite::Info)
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