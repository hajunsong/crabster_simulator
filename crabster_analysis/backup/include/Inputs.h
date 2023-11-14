#pragma once

#include "Common.h"
// #include "json/json.h"
#include <jsoncpp/json/json.h>
#include "MBD_RecursiveII.h"

class Inputs
{
public:
	SimulationData readSimulationParameter(std::string json_dir, MBD_RecursiveII* m_dynamics);
	Eigen::VectorXd readInputData(std::string json_dir, MBD_RecursiveII* m_dynamics);

private:
	BaseBodyData jsonRead_BaseBody(std::string json_dir);
	SubsystemData jsonRead_Subsystem(std::string json_dir, int id_subsystem, int id_body);
};