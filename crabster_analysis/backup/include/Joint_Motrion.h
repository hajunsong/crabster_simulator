#pragma once

#include "Common.h"

class Joint_Motrion
{
public:
	Eigen::Vector3d inputMotion(double t_current, int sub, int body);
};