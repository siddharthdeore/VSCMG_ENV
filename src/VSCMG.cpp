/*	
	@author :	Siddharth Deore
	@email	:	deore.1823670@studenti.uniroma1.it
*/
#define _CRT_SECURE_NO_WARNINGS

#include "VSCMG.h"
/**
 * VSCMG state space dynamics
 */
void VSCMG::operator()(const state_type& x, state_type& dxdt, const double t) {
	// States
	double q0 = x[0]; double q1 = x[1]; double q2 = x[2]; double q3 = x[3];
	normalizeQuaternions(q0, q1, q2, q3);
	/*
	arma::mat omega_skew = {
		{	0,  x[6], -x[5], x[4] },
		{-x[6],     0,  x[4], x[5] },
		{ x[5], -x[4],     0, x[6] },
		{-x[4], -x[5], -x[6],   0 },
	};
	arma::vec qq = { x[0],x[1] ,x[2] ,x[3] };
	arma::vec q_dot = 0.5 * omega_skew * qq;
	dxdt[0] = q_dot(0);
	dxdt[1] = q_dot(1);
	dxdt[2] = q_dot(2);
	dxdt[3] = q_dot(3);	
	*/

	// Attitude Quaternion Kinematic
	dxdt[0] = 0.5 * ( q1 * x[6] - q2 * x[5] + q3 * x[4]);
	dxdt[1] = 0.5 * ( q2 * x[4] - q0 * x[6] + q3 * x[5]);
	dxdt[2] = 0.5 * ( q0 * x[5] - q1 * x[4] + q3 * x[6]);
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
	//arma::mat Ag(3, 4); // probabaly not gonna use
	arma::vec omega = { x[4], x[5] , x[6] };
	arma::vec Omega = { x[7], x[8], x[9], x[10] };
	arma::mat DiagOmega = arma::diagmat(Omega);
	
	const double d1 = std::acos(std::cos(x[11]));
	const double d2 = std::acos(std::cos(x[12]));
	const double d3 = std::acos(std::cos(x[13]));
	const double d4 = std::acos(std::cos(x[14]));
	const double cb = 0.5773502691896261;//std::cos(this->_beta);
	const double sb = 0.8164965809277258;//std::sin(this->_beta);
	const double cd1 = std::cos(d1);
	const double cd2 = std::cos(d2);
	const double cd3 = std::cos(d3);
	const double cd4 = std::cos(d4);
	const double sd1 = std::sin(d1);
	const double sd2 = std::sin(d2);
	const double sd3 = std::sin(d3);
	const double sd4 = std::sin(d4);


	As = {	{ -cb * cd1, sd2, cb * cd3, -sd4},
			{ -sd1, -cb * cd2, sd3, cb * cd4},
			{ sb * cd1,  sb * cd2, sb * cd3, sb * cd4}
	};
	At = {	{ -cb * sd1, -cd2, cb * sd3, cd4 },
			{ cd1, -cb * sd2, -cd3, cb * sd4 },
			{ sb * sd1,  sb * sd2, sb * sd3, sb * sd4 }
	};
	/*
	As = {	{-cb * sd1, -cd2, cb * sd3, cd4},
			{cd1, -cb * sd2, -cd3, cb * sd4},
			{sb * sd1, sb * sd2, sb * sd3, sb * sd4}
		};
	At = {
		{-cb * cd1, sd2, cb * cd3, -sd4 },
		{-sd1, -cb * cd2, sd3, cb * cd4},
		{ sb * cd1, sb * cd2, sb * cd3, sb * cd4} };
	Ag = { { sb, 0, -sb, 0 }, {0, sb, 0, -sb}, {cb, cb, cb, cb} };
	*/

	arma::vec H_dot = { 0,0,0 };
	arma::vec omega_dot = { 0,0,0 };
	H_dot = - arma::cross(omega, this->_Jw * omega)
			- arma::cross(this->_Jw * (As * Omega), omega)
			- this->_Jw * (As * Omega_dot)
			- this->_Jw * ((At * DiagOmega) * Delta_dot);
		
	     // - arma::cross(Ag * Delta_dot, omega) // if im going to consider Ag Matrix
	
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
void VSCMG::setInertia(arma::mat Jb, double  Jg, double Jw) {
	this->_Jb = Jb;
	this->_Jg = Jg;
	this->_Jw = Jw;
}
void VSCMG::setTargetQuaternion(double q0, double q1, double q2, double q3) {
	this->qd[0] = q0;
	this->qd[1] = q1;
	this->qd[2] = q2;
	this->qd[3] = q3;
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

void VSCMG::Info()
{
	printf("\n Variable Speed Control Moment Gyro Pyramid Cluster \n");
	printf("Body Inertia : \n");
	for (short i = 0; i < 3; i++) {
		for (short j = 0; j < 3; j++) {
			printf("%8.4f ",this->_Jb(i,j));
		}
		printf("\n");
	}
	printf("Reaction Inertia : %8.4f \t Gimbal Inertia %8.4f \n", this->_Jw, this->_Jg);
	printf("\n Target Quaternions (%8.4f, %8.4f, %8.4f, %8.4f)", qd[0], qd[1], qd[2], qd[3]);

}


void write_state(const state_type& x, double t) {
	printf("%3.2f, %f, %f, %f, %f, %f, %f, %f \n", t, x[0], x[1], x[2], x[3], x[4], x[5], x[6]);
}