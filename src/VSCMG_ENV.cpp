/*
	@author :	Siddharth Deore
	@email	:	deore.1823670@studenti.uniroma1.it
*/

#define _CRT_SECURE_NO_WARNINGS

#include "VSCMG.h"
#include "NumPyArrayData.h"
namespace bn = boost::python::numpy;
namespace bo = boost::numeric::odeint;

typedef bo::runge_kutta_cash_karp54< state_type > error_stepper_type;
typedef bo::controlled_runge_kutta< error_stepper_type > controlled_stepper_type;
struct Satellite {
	state_type X= { 0.0,0.0,0.0,1.0, 0.0,0.0,0.0, 100.0,100.0,100.0,100.0, 0.0,0.0,0.0,0.0, };
	double Kp = 0.11;
	double Kd = 0.22;
	// initialise object
	VSCMG satellite;
	// initialise stepper
	//boost::numeric::odeint::runge_kutta_cash_karp54< state_type > stepper;
	//boost::numeric::odeint::runge_kutta_dopri5<state_type> stepper;
	//bo::runge_kutta_fehlberg78<state_type> stepper;
	//bo::controlled_runge_kutta<state_type> bo::make_controlled(bo::runge_kutta_dopri5< state_type > stepper);
	controlled_stepper_type stepper;
	//(Exposed to python) set state of satellite from numpy array 
	void setState(bn::ndarray& input) {
		double* input_ptr = reinterpret_cast<double*>(input.get_data());
		//std::vector<double> v(input_size);
		for (int i = 0; i < 15; ++i) {
			this->X[i] = *(input_ptr + i);
		}

	}
	void setInertia(bn::ndarray& input, double Jg, double Jw) {
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
	bn::ndarray getAction() {
		// States
		double q0 = X[0]; double q1 = X[1]; double q2 = X[2]; double q3 = X[3];
		arma::vec omega = { X[4], X[5] , X[6] };
		satellite.normalizeQuaternions(q0, q1, q2, q3);
		/*
		arma::mat omega_skew = {
			{	0,  X[6], -X[5], X[4] },
			{-X[6],     0,  X[4], X[5] },
			{ X[5], -X[4],     0, X[6] },
			{-X[4], -X[5], -X[6],   0 },
		};
		arma::vec qq = { X[0],X[1] ,X[2] ,X[3] };
		arma::vec q_dot = 0.5 * omega_skew * qq;
		dXdt[0] = q_dot(0);
		dXdt[1] = q_dot(1);
		dXdt[2] = q_dot(2);
		dXdt[3] = q_dot(3);
		*/

		// Error Quaternion (Might be usefull in future )
		//*
		double  qe0, qe1, qe2, qe3;
		qe0 = q0 * satellite.qd[3] + q1 * satellite.qd[2] - q2 * satellite.qd[1] - q3 * satellite.qd[0];
		qe1 = q2 * satellite.qd[0] - q0 * satellite.qd[2] + q1 * satellite.qd[3] - q3 * satellite.qd[1];
		qe2 = q0 * satellite.qd[1] - q1 * satellite.qd[0] + q2 * satellite.qd[3] - q3 * satellite.qd[2];
		qe3 = q0 * satellite.qd[0] + q1 * satellite.qd[1] + q2 * satellite.qd[2] + q3 * satellite.qd[3];
		satellite.normalizeQuaternions(qe0, qe1, qe2, qe3);
		//*/
		arma::mat As(3, 4);
		arma::mat At(3, 4);
		arma::mat Q(3, 8);
		//arma::mat Ag(3, 4); // probabaly not gonna use

		arma::vec u = { 0,0,0 };
		u[0] = Kp * qe0 * qe3 + Kd * omega[0];
		u[1] = Kp * qe1 * qe3 + Kd * omega[1];
		u[2] = Kp * qe2 * qe3 + Kd * omega[2];

		arma::vec Omega = { X[7], X[8], X[9], X[10] };
		arma::mat DiagOmega = arma::diagmat(Omega);
		
		const double d1 = std::acos(std::cos(X[11]));
		const double d2 = std::acos(std::cos(X[12]));
		const double d3 = std::acos(std::cos(X[13]));
		const double d4 = std::acos(std::cos(X[14]));
		const double cb = 0.5773502691896261; // std::cos(_beta);
		const double sb = 0.8164965809277258; // std::sin(_beta);
		const double cd1 = std::cos(d1);
		const double cd2 = std::cos(d2);
		const double cd3 = std::cos(d3);
		const double cd4 = std::cos(d4);
		const double sd1 = std::sin(d1);
		const double sd2 = std::sin(d2);
		const double sd3 = std::sin(d3);
		const double sd4 = std::sin(d4);


		As = { { -cb * cd1, sd2, cb * cd3, -sd4},
				{ -sd1, -cb * cd2, sd3, cb * cd4},
				{ sb * cd1,  sb * cd2, sb * cd3, sb * cd4}
		};
		At = { { -cb * sd1, -cd2, cb * sd3, cd4 },
				{ cd1, -cb * sd2, -cd3, cb * sd4 },
				{ sb * sd1,  sb * sd2, sb * sd3, sb * sd4 }
		};

		Q = arma::join_rows(As, At);
		double m_rw = arma::det(Q * Q.t());
		double alfa = 10000 * exp(-0.0001 * m_rw);
		arma::mat W=arma::eye(8,8);
		W[0, 0] = alfa; W[1, 1] = alfa; W[2, 2] = alfa; W[3, 3] = alfa;
		arma::vec omega_delta(8);
		omega_delta = (W * (Q.t() * arma::inv(Q * (W * Q.t())))) * u;
		
		Py_intptr_t shape[1] = { omega_delta.size() };
		bn::ndarray result = bn::zeros(1, shape, bn::dtype::get_builtin<double>());
		for (short i = 0; i < 8; i++) {
			result[i] = omega_delta[i];
		}
		return result;
	}
	/** (Exposed to python) Intigrate, Take a dybamical step and return Numpy Array
	 * x = f(x,u,t)
	 * @param : input state, python float64 array
	 * @param : time
	 * @param : stepsize
	 */
	bn::ndarray step(bn::ndarray& input, double t, double dt) {
		NumPyArrayData<double> d(input);
		std::vector<double> v(8);
		for (int i = 0; i < 8; ++i) {
			v[i] = d(i);
		}
		//satellite.controlAction(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
		satellite.controlAction(v);
		bo::integrate_adaptive(bo::make_controlled< error_stepper_type >(1.0e-10, 1.0e-6),
							   satellite, this->X, t, dt, dt);

		//stepper.do_step(satellite, this->X, t, dt);

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
		//stepper.do_step(satellite, this->X, t, dt);
		bo::integrate_adaptive(bo::make_controlled< error_stepper_type >(1.0e-6, 1.0e-6),
							   satellite, this->X, t, dt, dt);

		// result data for python 
		Py_intptr_t shape[1] = { this->X.size() };
		bn::ndarray result = bn::zeros(1, shape, bn::dtype::get_builtin<double>());
		std::copy(this->X.begin(), this->X.end(), reinterpret_cast<double*>(result.get_data()));
		return result;
	}
	void setTargetQuaternion(double q0, double q1, double q2, double q3) {
		satellite.setTargetQuaternion(q0, q1, q2, q3);
	}
	void setGains(double q0, double q1) {
		Kp = q0;
		Kd = q1;
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
		.def("Info", &Satellite::Info)
		.def("step", &Satellite::step)
		.def("stepNull", &Satellite::stepNull)
		.def("resetState", &Satellite::resetState)
		.def("setState", &Satellite::setState)
		.def("setInertia", &Satellite::setInertia)
		.def("setTargetQuaternion", &Satellite::setTargetQuaternion)
		.def("getAction", &Satellite::getAction)
		.def("setGains", &Satellite::setGains)
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