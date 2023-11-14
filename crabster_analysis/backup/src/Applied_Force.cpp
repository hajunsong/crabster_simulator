#include "Applied_Force.h"

Eigen::Vector3d Applied_Force::gravityForce(Eigen::Vector3d axis, double m)
{
	Eigen::Vector3d Fg; Fg.setZero();
	
	//Fg = axis.normalized() * m * g;
	Fg = axis * m * g;

	return Fg;
}

Eigen::Vector3d Applied_Force::buoyancyForce()
{
	// need Dr. Han........
	Eigen::Vector3d Fb;

	Fb.setZero();

	return Fb;
}

Eigen::Vector3d Applied_Force::contactForce(Eigen::Vector3d r, Eigen::Vector3d dr, double pen_z_ref, double k_contact, double c_contact)
{
	// penetration 
	double pen_z = pen_z_ref - r(2);
	double pen_dz = -dr(2);

	// contact force
	Eigen::Vector3d Fc;

	if (pen_z > 0.0)
	{
		Fc(0) = 0.0;
		Fc(1) = 0.0;
		Fc(2) = k_contact * pen_z + c_contact * pen_dz;
	}
	else
	{
		Fc.setZero();
	}

	return Fc;
}

double Applied_Force::RSDATorque(double qj, double dqj, double qj_ini, double k, double c)
{
	double Qj_RSDA = k * (qj - qj_ini) + c * dqj;

	return Qj_RSDA;
}