#include "CRABSTER.h"

Crabster::Crabster(std::string name, ros::NodeHandle nh) : as(nh, name, boost::bind(&Crabster::executeCB, this, _1), false), action_name(name)
{
    // json file direction
	path = ros::package::getPath("crabster_analysis");
    json_dir = path + "/data/input_data/";

    // class header
	m_inputs = new Inputs();
	m_dynamics = new MBD_RecursiveII();
	m_integrator = new Integrator();
	m_outputs = new Outputs();

	as.start();
}

Crabster::~Crabster()
{
	delete(m_inputs);
    delete(m_dynamics);
	delete(m_integrator);
	delete(m_outputs);
}

void Crabster::executeCB(const crabster_msgs::CrabsterSimulationGoalConstPtr &goal)
{
	ros::Rate loop_rate(100);
	bool success = true;

	ROS_INFO("%s: Executing, creating crabster simulation", action_name.c_str());
	ROS_INFO("simulation parameter");
	ROS_INFO("simulation time : %f", goal->simulation_time);
	ROS_INFO("itegration step size : %f", goal->integration_stepsize);
	ROS_INFO("gravity : %f", goal->gravity);
	ROS_INFO("analysis method : %s", goal->analysis_method);
	ROS_INFO("sovler : %s", goal->solver);
	ROS_INFO("gravity_axis : %f, %f, %f", goal->gravity_axis[0], goal->gravity_axis[1], goal->gravity_axis[2]);
	ROS_INFO("rotational_axis : %f, %f, %f", goal->rotational_axis[0], goal->rotational_axis[1], goal->rotational_axis[2]);
	ROS_INFO("translational_axis : %f, %f, %f", goal->translational_axis[0], goal->translational_axis[1], goal->translational_axis[2]);

	feedback.time_current = 0;
	feedback.percent_complete = 0;

	result.complete = false;

	

	result.complete = true;

	as.setSucceeded(result);
}

void Crabster::run_single_init()
{
	// read simulation parameters
	SimulationData SimData = m_inputs->readSimulationParameter(json_dir, m_dynamics);
	g = SimData.g;
	t_current = SimData.start_time;
	t_end = SimData.end_time;
	integrationStep = SimData.integration_stepSize;
	dataSaveStep = SimData.dataSave_stepSize;
	analysis_method = SimData.analysis_method;
	solver = SimData.solver;

	// read input data
	Y = m_inputs->readInputData(json_dir, m_dynamics);
	m_integrator->setMatixVectorSize();

	// dynamics simulation
	dataSave_count = 0;
	Eigen::VectorXd dY(Y.size());
}

void Crabster::run_single()
{
	// pthread_create(&run_single_thread, nullptr, run_single_func, this);
	// std::cout << "t_current = " << t_current << std::endl;

	dY = m_dynamics->dynamics_analysis(t_current, Y);

	// store output data
	if (std::abs(t_current - dataSave_count * dataSaveStep) < eps)
	{
		m_outputs->storeOutputData(t_current, m_dynamics->getOutputData());
		dataSave_count++;
	}

	IntegrationData integData;
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
