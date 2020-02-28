/*	
	@author :	Siddharth Deore
	@email	:	deore.1823670@studenti.uniroma1.it
*/
#define _CRT_SECURE_NO_WARNINGS

#include "VSCMG.h"

void VSCMG::operator()(const state_type& x, state_type& dxdt, const double t) {
	// States
	double q0 = x[0]; double q1 = x[1]; double q2 = x[2]; double q3 = x[3];
	normalizeQuaternions(q0, q1, q2, q3);

	// Attitude Quaternion Kinematic
	dxdt[0] = 0.5 * (q1 * x[6] - q2 * x[5] + q3 * x[4]);
	dxdt[1] = 0.5 * (q2 * x[4] - q0 * x[6] + q3 * x[5]);
	dxdt[2] = 0.5 * (q0 * x[5] - q1 * x[4] + q3 * x[6]);
	dxdt[3] = 0.5 * (-q0 * x[4] - q1 * x[5] - q2 * x[6]);

	// Error Quaternion (Might be usefull in future )
	/*
	double  qe0, qe1, qe2, qe3;
	qe0 = q0 * this->qd[3] + q1 * this->qd[2] - q2 * this->qd[1] - q3 * this->qd[0];
	qe1 = q2 * this->qd[0] - q0 * this->qd[2] + q1 * this->qd[3] - q3 * this->qd[1];
	qe2 = q0 * this->qd[1] - q1 * this->qd[0] + q2 * this->qd[3] - q3 * this->qd[2];
	qe3 = q0 * this->qd[0] + q1 * this->qd[1] + q2 * this->qd[2] + q3 * this->qd[3];
	normalizeQuaternions(qe0, qe1, qe2, qe3);
	*/
	arma::mat As(3, 4);
	arma::mat At(3, 4);
	arma::mat Ag(3, 4); // probabaly not gonna use
	arma::vec omega = { x[4], x[5] , x[6] };
	arma::vec Omega = { x[7], x[8], x[9], x[10] };
	arma::mat DiagOmega = arma::diagmat(Omega);

	const double cb = std::cos(this->_beta);
	const double sb = std::sin(this->_beta);
	const double cd1 = std::cos(x[11]);
	const double cd2 = std::cos(x[12]);
	const double cd3 = std::cos(x[13]);
	const double cd4 = std::cos(x[14]);
	const double sd1 = std::sin(x[11]);
	const double sd2 = std::sin(x[12]);
	const double sd3 = std::sin(x[13]);
	const double sd4 = std::sin(x[14]);

	As = { {-cb * sd1, -cd2, cb * sd3, cd4}, {cd1, -cb * sd2, -cd3, cb * sd4}, {sb * sd1, sb * sd2, sb * sd3, sb * sd4} };
	At = { {-cb * cd1, sd2, cb * cd3, -sd4 }, {-sd1, -cb * cd2, sd3, cb * cd4}, { sb * cd1, sb * cd2, sb * cd3, sb * cd4} };
	Ag = { { sb, 0, -sb, 0 }, {0, sb, 0, -sb}, {cb, cb, cb, cb} };


	arma::vec H_dot = { 0,0,0 };
	arma::vec omega_dot = { 0,0,0 };

	H_dot = -arma::cross(this->_Jw * omega, omega)
		- arma::cross(Ag * Delta_dot, omega)
		- arma::cross(this->_Jw * As * Omega, omega)
		- this->_Jw * As * Omega_dot
		- this->_Jw * At * DiagOmega * Delta_dot;

	omega_dot = arma::inv(this->_Jb) * H_dot;

	dxdt[4] = omega_dot(0);
	dxdt[5] = omega_dot(1);
	dxdt[6] = omega_dot(2);

	dxdt[7] = Omega_dot(0);
	dxdt[8] = Omega_dot(1);
	dxdt[9] = Omega_dot(2);
	dxdt[10] = Omega_dot(3);

	dxdt[11] = Delta_dot(0);
	dxdt[12] = Delta_dot(1);
	dxdt[13] = Delta_dot(2);
	dxdt[14] = Delta_dot(3);
}

void VSCMG::normalizeQuaternions(double& _q0, double& _q1, double& _q2, double& _q3) {
	double norm = std::sqrt(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
	_q0 /= norm;
	_q1 /= norm;
	_q2 /= norm;
	_q3 /= norm;
}
void VSCMG::setInertia(arma::mat Jb, double  Jg, double Jt) {
	this->_Jb = Jb;
	this->_Jg = Jg;
	this->_Jt = Jt;
}
/*
void VSCMG::controlAction(double u0, double u1, double u2, double u3, double u4, double u5, double u6, double u7) {
	Omega_dot = { u0,u1,u2,u3 };
	Delta_dot = { u4,u5,u6,u7 };
}
*/
void VSCMG::controlAction(std::vector<double> v) {
	Omega_dot = { v[0],v[1],v[2],v[3] };
	Delta_dot = { v[4],v[5],v[6],v[7] };
}


void write_state(const state_type& x, double t) {
	printf("%3.2f, %f, %f, %f, %f, %f, %f, %f \n", t, x[0], x[1], x[2], x[3], x[4], x[5], x[6]);
}