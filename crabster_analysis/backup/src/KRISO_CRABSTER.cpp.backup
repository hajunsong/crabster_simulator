// KRISO_CRABSTER.cpp : 이 파일에는 'main' 함수가 포함됩니다. 거기서 프로그램 실행이 시작되고 종료됩니다.
//

#include "Common.h"
#include "Inputs.h"
#include "MBD_RecursiveII.h"
#include "Integrator.h"
#include "Outputs.h"

#include <ros/ros.h>
#include <ros/package.h>

/* ============= global values =================================================== */
// gravity acceleration
double g;

// number of subsystems
int nSubsystem;

// number of bodies for each subsystem
std::map<int, int> nSubBody;

// number of joint motion for each subsystem
std::map<int, int> nJointMotion;

// analysis method
int analysis_method;
/* ============= global values =================================================== */

int main(int argc, char** argv)
{
	ros::init(argc, argv, "crabster_analysis_single");
	ros::NodeHandle nh;

	/* ============= User Input =================================================== */

    // json file direction
	std::string path = ros::package::getPath("crabster_analysis");
    std::string json_dir = path + "/data/input_data/";

	/* ============= User Input =================================================== */

	// class header
	Inputs* m_inputs = new Inputs();
	MBD_RecursiveII* m_dynamics = new MBD_RecursiveII();
	Integrator* m_integrator = new Integrator();
	Outputs* m_outputs = new Outputs();

	// read simulation parameters
	SimulationData SimData = m_inputs->readSimulationParameter(json_dir, m_dynamics);
	g = SimData.g;
	double t_current = SimData.start_time;
	double t_end = SimData.end_time;
	double integrationStep = SimData.integration_stepSize;
	double dataSaveStep = SimData.dataSave_stepSize;
	analysis_method = SimData.analysis_method;
	int solver = SimData.solver;

	// read input data
	Eigen::VectorXd Y = m_inputs->readInputData(json_dir, m_dynamics);
	m_integrator->setMatixVectorSize();

	// dynamics simulation
	double eps = 1e-7;
	int dataSave_count = 0;
	Eigen::VectorXd dY(Y.size());
	IntegrationData integData;

	std::cout << "\nSimulation start!\n" << std::endl;
	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();

	while (std::abs(t_current - (t_end + integrationStep)) > eps)
	{
		// std::cout << "t_current = " << t_current << std::endl;

		dY = m_dynamics->dynamics_analysis(t_current, Y);

		// store output data
		if (std::abs(t_current - dataSave_count * dataSaveStep) < eps)
		{
			m_outputs->storeOutputData(t_current, m_dynamics->getOutputData());
			dataSave_count++;
		}

		switch (solver)
		{
		case INTEGRATOR_AB3:
			integData = m_integrator->AB3(t_current, Y, dY, integrationStep);
			break;
		case INTEGRATOR_RK4:
			integData = m_integrator->RK4(t_current, Y, dY, integrationStep, m_dynamics);
			break;
		}

		Y = integData.Y_next;
		t_current = integData.t_next;
	}

	std::chrono::high_resolution_clock::time_point t_stop = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> t_span = std::chrono::duration_cast<std::chrono::milliseconds>(t_stop - t_start);
	
	std::cout << "\nSimulation finish!" << std::endl;

	m_outputs->csvWrite(path + "data/output_data/");

    delete(m_inputs);
    delete(m_dynamics);
	delete(m_integrator);
	delete(m_outputs);

	switch (analysis_method)
	{
	case METHOD_CONVENTIONAL:
		std::cout << "Analysis method = Conventional method" << std::endl;
		break;
	case METHOD_SUBSYSTEM:
		std::cout << "Analysis method = Subsystem synthesis method" << std::endl;
		break;
	}
	switch (solver)
	{
	case INTEGRATOR_AB3:
		std::cout << "Solver = Adams-Bashforth 3rd order method" << std::endl;
		break;
	case INTEGRATOR_RK4:
		std::cout << "Solver = Runge-Kutta 4th order method" << std::endl;
		break;
	}
	std::cout << "Simulation time = " << std::to_string(t_end) << " [sec]" << std::endl;
	std::cout << "Computation time = " << t_span.count() << " [msec]" << std::endl;

	return 0;
}

