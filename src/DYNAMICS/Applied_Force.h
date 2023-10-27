#pragma once

#include "Common.h"

class Applied_Force
{
public:
	Eigen::Vector3d gravityForce(Eigen::Vector3d axis, double m);
	Eigen::Vector3d buoyancyForce();
	Eigen::Vector3d contactForce(Eigen::Vector3d rjcp, Eigen::Vector3d drjcp, double pen_z_ref, double k_contact, double c_contact);
	double RSDATorque(double qj, double dqj, double qj_ini, double k, double c);
};