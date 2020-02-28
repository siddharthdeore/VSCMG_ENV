/*
	@author :	Siddharth Deore
	@email	:	deore.1823670@studenti.uniroma1.it
*/

#pragma once

#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <boost/python/extract.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <armadillo>

typedef boost::array< double, 15 > state_type;
typedef std::vector<double> vec;

struct VSCMG {
	// Single Gimbal VSCMG Pyramid Cluster Configration Class
private:
	// Body Innertia
	arma::mat _Jb = {
		{ 0.0220, 0.0012, -0.0070 },
		{ 0.0012, 0.0220,  0.0012 },
		{-0.0070, 0.0012,  0.0029 },
	};

	double _beta = 0.955316618124509; // Skew Angle beta = atan(sqrt(2));

	double _Jw = 9.7e-6; // Reaction Wheel Inertia
	double _Jg = 1.1e-5; // Gimbal Inertia
	double _Jt = 1.8e-5; // SGCMG Inertia

	state_type _X = { 1,0,0,0,   0,0,0, 100,100,100,100, 0,0,0,0, };

	// Attitude Quaternions
	double _q[4] = { 1.0, 0.0, 0.0, 0.0 };
	// Body Angular Velocities
	double _omega[3] = { 0.0, 0.0, 0.0 };
	// Gimbal Angles
	double _Delta[4] = { 0.0, 0.0, 0.0, 0.0 };
	// Reaction Wheel Rates
	double _Omega[4] = { 0.0, 0.0, 0.0, 0.0 };
public:
	double qd[4] = { 1.0, 0.0, 0.0, 0.0 };
	arma::vec Delta_dot = { 0,0,0,0 }; // controll input delta_dot
	arma::vec Omega_dot = { 0,0,0,0 }; // controll input omega_dot

	void operator()(const state_type& x, state_type& dxdt, const double t);
	void normalizeQuaternions(double& _q0, double& _q1, double& _q2, double& _q3);
	//void controlAction(double u0, double u1, double u2, double u3, double u4, double u5, double u6, double u7);
	void controlAction(std::vector<double> v);
};
