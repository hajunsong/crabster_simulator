#include "Integrator.h"

void Integrator::setMatixVectorSize()
{
	int cnt = 13;
	for (int sub = 1; sub <= nSubsystem; sub++)
	{
		cnt = cnt + 2 * (nSubBody[sub] - nJointMotion[sub]);
	}

	stepAB3 = 0;
	AT = Eigen::MatrixXd::Zero(cnt, 2);
	AT1= Eigen::MatrixXd::Zero(cnt, 2);
}

IntegrationData Integrator::AB3(double t, Eigen::VectorXd Y, Eigen::VectorXd dY, double h)
{
	IntegrationData buf;
	buf.Y_next.resize(Y.size());

	if (Y.size() != 0)
	{
		switch (stepAB3)
		{
		case 0:
			// Forward Euler method with 0.25 stepsize_h for initial step
			// use derivative information at 0 step

			buf.Y_next = Y + h * dY / 4.0;
			buf.t_next = t + h / 4.0;

			AT.col(1) = dY;
			AT1.col(1) = dY;
			stepAB3++;
			break;
		case 1:
			// Adams Bashforth 2nd order method with 0.25 stepsize_h for 2nd step
			// use derivative information at 0, h/4 step

			buf.Y_next = Y + h * (3.0 * dY - AT1.col(1)) / 8.0;
			buf.t_next = t + h / 4.0;

			AT1.col(0) = dY;
			stepAB3++;
			break;
		case 2:
			// Adams Bashforth 3rd order method with 0.25 stepsize_h for 3rd step
			// use derivative information at 0, h/4, h/2 step

			buf.Y_next = Y + h * (23.0 * dY - 16.0 * AT1.col(0) + 5.0 * AT1.col(1)) / 48.0;
			buf.t_next = t + h / 4.0;

			AT1.col(1) = AT1.col(0);
			AT1.col(0) = dY;
			stepAB3++;
			break;
		case 3:
			// Adams Bashforth 3rd order method with 0.25 stepsize_h for 4th step
			// use derivative information at h/4, h/2, 3h/4 step

			buf.Y_next = Y + h * (23.0 * dY - 16.0 * AT1.col(0) + 5.0 * AT1.col(1)) / 48.0;
			buf.t_next = t + h / 4.0;

			AT1.col(1) = AT.col(1);
			stepAB3++;
			break;
		case 4:
			// Adams Bashforth 3rd order method with 0.5 stepsize_h for 5th step
			// use derivative information at 0, h / 2, h

			buf.Y_next = Y + h * (23.0 * dY - 16.0 * AT1.col(0) + 5.0 * AT1.col(1)) / 24.0;
			buf.t_next = t + h / 2.0;

			AT.col(0) = dY;
			AT1.col(1) = AT1.col(0);
			AT1.col(0) = dY;
			stepAB3++;
			break;
		case 5:
			// Adams Bashforth 3rd order method with 0.5 stepsize_h for 6th step
			// use derivative information at h/2, h, 3h/2 step

			buf.Y_next = Y + h * (23.0 * dY - 16.0 * AT1.col(0) + 5.0 * AT1.col(1)) / 24.0;
			buf.t_next = t + h / 2.0;

			AT1.col(1) = AT1.col(0);
			AT1.col(0) = dY;
			stepAB3++;
			break;
		case 6:
			// Adams Bashforth 3rd order method with stepsize_h for 7th step
			// use derivative information at 0, h, 2h step

			buf.Y_next = Y + h * (23.0 * dY - 16.0 * AT.col(0) + 5.0 * AT.col(1)) / 12.0;
			buf.t_next = t + h;

			AT.col(1) = AT.col(0);
			AT.col(0) = dY;
			stepAB3++;
			break;
		default:
			// Adams Bashforth 3rd order method with stepsize_h for more than 8th step
			// use derivative information t_current - 2h, t_current - h, t_current step

			buf.Y_next = Y + h * (23.0 * dY - 16.0 * AT.col(0) + 5.0 * AT.col(1)) / 12.0;
			buf.t_next = t + h;

			AT.col(1) = AT.col(0);
			AT.col(0) = dY;
			break;
		}
	}

	return buf;
}


IntegrationData Integrator::RK4(double t, Eigen::VectorXd Y, Eigen::VectorXd dY, double h, MBD_RecursiveII* m_dynamics)
{
	IntegrationData buf;
	buf.Y_next.resize(Y.size());

	Eigen::VectorXd K1(Y.size());
	Eigen::VectorXd K2(Y.size());
	Eigen::VectorXd K3(Y.size());
	Eigen::VectorXd K4(Y.size());

	if (Y.size() != 0)
	{
		K1 = dY;
		K2 = m_dynamics->dynamics_analysis(t + h / 2.0, Y + h * K1 / 2.0);
		K3 = m_dynamics->dynamics_analysis(t + h / 2.0, Y + h * K2 / 2.0);
		K4 = m_dynamics->dynamics_analysis(t + h, Y + h * K3);

		buf.Y_next = Y + h * (K1 + 2.0 * K2 + 2.0 * K3 + K4) / 6.0;
		buf.t_next = t + h;
	}

	return buf;
}