#pragma once

#include "Common.h"
#include "MBD_RecursiveII.h"
#include "Applied_Force.h"

class Integrator
{
public:
	int stepAB3;

	void setMatixVectorSize();

	// Adams-Bashforth 3rd order
	IntegrationData AB3(double t, Eigen::VectorXd Y, Eigen::VectorXd dY, double h);

	// Runge-Kutta 4th order
	IntegrationData RK4(double t, Eigen::VectorXd Y, Eigen::VectorXd dY, double h, MBD_RecursiveII* m_dynamics);

private:
	Eigen::MatrixXd AT;
	Eigen::MatrixXd AT1;
};