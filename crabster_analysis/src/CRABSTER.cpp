#include "CRABSTER.h"

Crabster::Crabster(std::string name, ros::NodeHandle nh) : as(nh, name, boost::bind(&Crabster::executeCB, this, _1), false), action_name(name)
{
    // json file direction
	path = ros::package::getPath("crabster_analysis");
    json_dir = path + "/data/input_data/";

	as.start();
}

Crabster::~Crabster()
{
}

void Crabster::executeCB(const crabster_msgs::CrabsterSimulationGoalConstPtr &goal)
{

}

void Crabster::run_single()
{
	pthread_create(&run_single_thread, nullptr, run_single_func, this);
}

void* Crabster::run_single_func(void *arg)
{
	Crabster* pThis = static_cast<Crabster*>(arg);

    // class header
	pThis->m_inputs = new Inputs();
	pThis->m_dynamics = new MBD_RecursiveII();
	pThis->m_integrator = new Integrator();
	pThis->m_outputs = new Outputs();

	// read simulation parameters
	SimulationData SimData = pThis->m_inputs->readSimulationParameter(pThis->json_dir, pThis->m_dynamics);
	g = SimData.g;
	double t_current = SimData.start_time;
	double t_end = SimData.end_time;
	double integrationStep = SimData.integration_stepSize;
	double dataSaveStep = SimData.dataSave_stepSize;
	analysis_method = SimData.analysis_method;
	int solver = SimData.solver;

	// read input data
	Y = pThis->m_inputs->readInputData(pThis->json_dir, pThis->m_dynamics);
	pThis->m_integrator->setMatixVectorSize();

	// dynamics simulation
	double eps = 1e-7;
	int dataSave_count = 0;
	Eigen::VectorXd dY(Y.size());
	IntegrationData integData;

    std::cout << "\nSimulation start!\n" << std::endl;
	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();

	while (std::abs(t_current - (t_end + integrationStep)) > eps)
	{
		std::cout << "t_current = " << t_current << std::endl;

		dY = pThis->m_dynamics->dynamics_analysis(t_current, Y);

		// store output data
		if (std::abs(t_current - dataSave_count * dataSaveStep) < eps)
		{
			pThis->m_outputs->storeOutputData(t_current, pThis->m_dynamics->getOutputData());
			dataSave_count++;
		}

		switch (solver)
		{
		case INTEGRATOR_AB3:
			integData = pThis->m_integrator->AB3(t_current, Y, dY, integrationStep);
			break;
		case INTEGRATOR_RK4:
			integData = pThis->m_integrator->RK4(t_current, Y, dY, integrationStep, pThis->m_dynamics);
			break;
		}

		Y = integData.Y_next;
		t_current = integData.t_next;

		usleep(1000000);
	}

	std::chrono::high_resolution_clock::time_point t_stop = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> t_span = std::chrono::duration_cast<std::chrono::milliseconds>(t_stop - t_start);
	
	std::cout << "\nSimulation finish!" << std::endl;

	// m_outputs->csvWrite(path + "data/output_data/");

    delete(pThis->m_inputs);
    delete(pThis->m_dynamics);
	delete(pThis->m_integrator);
	delete(pThis->m_outputs);

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

	return nullptr;
}
