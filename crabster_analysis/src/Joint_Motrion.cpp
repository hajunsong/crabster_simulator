#include "Joint_Motrion.h"

Eigen::Vector3d Joint_Motrion::inputMotion(double t_current, int sub, int body)
{
	Eigen::Vector3d motionData;

	// position
	motionData(0) = std::sin(2.0 * pi * t_current);

	// velocity
	motionData(1) = 2.0 * pi * std::cos(2.0 * pi * t_current);

	// acceleration
	motionData(2) = -4.0 * pi * pi * std::sin(2.0 * pi * t_current);

	return motionData;
}