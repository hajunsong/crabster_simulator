#pragma once

#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>

// base body constraint
// When base body constraints are needed, just active define function
// #define BASEBODY_FIX
#define BASEBODY_TRANS_Z

// analysis method
#define METHOD_CONVENTIONAL		0
#define METHOD_SUBSYSTEM		1

// integrator
#define INTEGRATOR_AB3			0
#define INTEGRATOR_RK4			1

// joint type
#define JOINT_REVOLUTE			0
#define JOINT_TRNSLATIONAL		1

// pi
const double pi = 3.14159265358979323846264338327950288419716939937510582097494459230781;
//const double pi = std::atan(1.0) * 4.0;

// gravity acceleration
extern double g;

// number of subsystems
extern int nSubsystem;

// number of bodies for each subsystem
extern std::map<int, int> nSubBody;

// number of joint motion for each subsystem
extern std::map<int, int> nJointMotion;

// analysis method
extern int analysis_method;

// Y vector
extern Eigen::VectorXd Y;
extern Eigen::VectorXd dY;

// current time stamp
extern double t_current;

// leg contact point
extern std::vector<double> rjf;

// boolean vector
typedef Eigen::Array<bool, Eigen::Dynamic, 1> ArrayXb;

// integration data
typedef struct IntegrationData
{
	Eigen::VectorXd Y_next;
	double t_next;
}IntegrationData;

// input of simulation parameter
typedef struct SimulationData
{
	double start_time;
	double end_time;
	double integration_stepSize;
	double dataSave_stepSize;
	double g;
	double analysis_method;
	double solver;
}SimulationData;

// input of base body
typedef struct BaseBodyData
{
	bool read_ok;
	std::string name;
	bool flag_contact;
	bool flag_motion;
	Eigen::Vector4d p0;
	Eigen::Matrix3d C00;
	Eigen::Vector3d r0;
	Eigen::Vector3d rho0p;
	Eigen::Vector3d w0;
	Eigen::Vector3d dr0;
	double m0;
	Eigen::Matrix3d J0cp;
}BaseBodyData;

// input of subsystem
typedef struct SubsystemData
{
	bool read_ok;
	std::string name;
	int subsystem_id;
	int jointType;
	int parent_id;
	int child_id;
	bool flag_contact;
	bool flag_motion;
	double qj;
	Eigen::Matrix3d Cij;
	Eigen::Matrix3d Cjj;
	Eigen::Vector3d sijp;
	Eigen::Vector3d sjcp;
	Eigen::Vector3d rhojp;
	double dqj;
	double mj;
	Eigen::Matrix3d Jjcp;
	double qj_ini_RSDA;
	double k_RSDA;
	double c_RSDA;
	double pen_z_ref;
	double k_contact;
	double c_contact;
}SubsystemData;

typedef struct OutputData
{
	double t_current;

	struct BaseBody
	{
		Eigen::Vector3d r0c;
		Eigen::Vector4d p0;
		Eigen::Vector3d dr0c;
		Eigen::Vector3d w0;
		Eigen::Vector3d ddr0c;
		Eigen::Vector3d dw0;
	}BaseBody;

	struct Subsystem
	{
		std::map<int, Eigen::VectorXd> qj;
		std::map<int, Eigen::VectorXd> dqj;
		std::map<int, Eigen::VectorXd> ddqj;
		std::map<int, std::vector<Eigen::VectorXd>> Rji;
		std::map<int, std::vector<Eigen::Vector3d>> rjc;
		std::map<int, std::vector<Eigen::Vector4d>> pj;
		std::map<int, std::vector<Eigen::Vector3d>> drjc;
		std::map<int, std::vector<Eigen::Vector3d>> wj;
		std::map<int, std::vector<Eigen::Vector3d>> ddrjc;
		std::map<int, std::vector<Eigen::Vector3d>> dwj;
	}Subsystem;

}OutputData;