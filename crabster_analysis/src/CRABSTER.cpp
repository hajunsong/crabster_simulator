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

	pub_joint_command = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

	msg_joint.name.push_back("world_to_BaseBody");

	msg_joint.name.push_back("BaseBody_to_Subsystem11(FR_Yaw_Motor)");
	msg_joint.name.push_back("Subsystem11(FR_Yaw_Motor)_to_Subsystem12(FR_Roll_Arm)");
	msg_joint.name.push_back("Subsystem12(FR_Roll_Arm)_to_Subsystem13(FR_Pitch_Arm)");
	msg_joint.name.push_back("Subsystem13(FR_Pitch_Arm)_to_Subsystem14(FR_Leg)");
	
	msg_joint.name.push_back("BaseBody_to_Subsystem21(MR_Yaw_Motor)");
	msg_joint.name.push_back("Subsystem21(MR_Yaw_Motor)_to_Subsystem22(MR_Roll_Arm)");
	msg_joint.name.push_back("Subsystem22(MR_Roll_Arm)_to_Subsystem23(MR_Pitch_Arm)");
	msg_joint.name.push_back("Subsystem23(MR_Pitch_Arm)_to_Subsystem24(MR_Leg)");
	
	msg_joint.name.push_back("BaseBody_to_Subsystem31(RR_Yaw_Motor)");
	msg_joint.name.push_back("Subsystem31(RR_Yaw_Motor)_to_Subsystem32(RR_Roll_Arm)");
	msg_joint.name.push_back("Subsystem32(RR_Roll_Arm)_to_Subsystem33(RR_Pitch_Arm)");
	msg_joint.name.push_back("Subsystem33(RR_Pitch_Arm)_to_Subsystem34(RR_Leg)");
	
	msg_joint.name.push_back("BaseBody_to_Subsystem41(FL_Yaw_Motor)");
	msg_joint.name.push_back("Subsystem41(FL_Yaw_Motor)_to_Subsystem42(FL_Roll_Arm)");
	msg_joint.name.push_back("Subsystem42(FL_Roll_Arm)_to_Subsystem43(FL_Pitch_Arm)");
	msg_joint.name.push_back("Subsystem43(FL_Pitch_Arm)_to_Subsystem44(FL_Leg)");
	
	msg_joint.name.push_back("BaseBody_to_Subsystem51(ML_Yaw_Motor)");
	msg_joint.name.push_back("Subsystem51(ML_Yaw_Motor)_to_Subsystem52(ML_Roll_Arm)");
	msg_joint.name.push_back("Subsystem52(ML_Roll_Arm)_to_Subsystem53(ML_Pitch_Arm)");
	msg_joint.name.push_back("Subsystem53(ML_Pitch_Arm)_to_Subsystem54(ML_Leg)");
	
	msg_joint.name.push_back("BaseBody_to_Subsystem61(RL_Yaw_Motor)");
	msg_joint.name.push_back("Subsystem61(RL_Yaw_Motor)_to_Subsystem62(RL_Roll_Arm)");
	msg_joint.name.push_back("Subsystem62(RL_Roll_Arm)_to_Subsystem63(RL_Pitch_Arm)");
	msg_joint.name.push_back("Subsystem63(RL_Pitch_Arm)_to_Subsystem64(RL_Leg)");

	ros::Rate loop_rate(10);

	unsigned int n = msg_joint.name.size();
	msg_joint.position.resize(n);
	msg_joint.velocity.resize(n);
	msg_joint.effort.resize(n);

	for (unsigned int i = 0; i < n; i++)
	{
		msg_joint.velocity[i] = 0;
		msg_joint.effort[i] = 0;
		msg_joint.position[i] = 0;
	}

	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();

		msg_joint.header.stamp = ros::Time::now();

		pub_joint_command.publish(msg_joint);
	}
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
	ROS_INFO("%s: Executing, creating crabster simulation", action_name.c_str());
	ROS_INFO("simulation parameter");
	ROS_INFO("simulation time : %f", goal->simulation_time);
	ROS_INFO("itegration step size : %f", goal->integration_stepsize);
	ROS_INFO("datasave step size : %f", goal->datasave_stepsize);
	ROS_INFO("gravity : %f", goal->gravity);
	ROS_INFO("analysis method : %s", goal->analysis_method.data.c_str());
	ROS_INFO("sovler : %s", goal->solver.data.c_str());
	ROS_INFO("gravity_axis : %f, %f, %f", goal->gravity_axis[0], goal->gravity_axis[1], goal->gravity_axis[2]);
	ROS_INFO("rotational_axis : %f, %f, %f", goal->rotational_axis[0], goal->rotational_axis[1], goal->rotational_axis[2]);
	ROS_INFO("translational_axis : %f, %f, %f", goal->translational_axis[0], goal->translational_axis[1], goal->translational_axis[2]);

	feedback.time_current = 0;
	feedback.percent_complete = 0;

	result.complete = false;

	g = goal->gravity;
	t_current = 0;
	t_end = goal->simulation_time;
	integrationStep = goal->integration_stepsize;
	dataSaveStep = goal->datasave_stepsize;
	if (goal->analysis_method.data ==  "Conventional") 
	{ 
		analysis_method = METHOD_CONVENTIONAL;
	}
	else if(goal->analysis_method.data == "Subsystem")
	{ 
		analysis_method = METHOD_SUBSYSTEM;
	}

	if (goal->solver.data == "AB3")
	{
		solver = INTEGRATOR_AB3;
	}
	else if (goal->solver.data == "RK4")
	{
		solver = INTEGRATOR_RK4;
	}
	for (int i = 0; i < 3; i++)
	{
		m_dynamics->gravityAxis(i) = goal->gravity_axis[i];
		m_dynamics->rotAxis(i) = goal->rotational_axis[i];
		m_dynamics->transAxis(i) = goal->translational_axis[i];
	}
	m_dynamics->gravityAxis.normalize();
	m_dynamics->rotAxis.normalize();
	m_dynamics->transAxis.normalize();

	// read input data
	Y = m_inputs->readInputData(json_dir, m_dynamics);
	m_integrator->setMatixVectorSize();

	// dynamics simulation
	dataSave_count = 0;
	Eigen::VectorXd dY(Y.size());

	ros::Rate loop_rate(10);
	bool success = true;

	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
		// ROS_INFO("time : %f", t_current);

		if (as.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name.c_str());
            // set the action state to preempted
            as.setPreempted();
            success = false;
            break;
        }

		feedback.time_current = t_current;
		feedback.percent_complete = (t_current/t_end)*100.0;

		dY = m_dynamics->dynamics_analysis(t_current, Y);

		// store output data
		if (std::abs(t_current - dataSave_count * dataSaveStep) < eps)
		{
			m_outputs->storeOutputData(t_current, m_dynamics->getOutputData());
			dataSave_count++;
		}

		msg_joint.position[0] = Y(2) - 0.5;
		for(unsigned int i = 1; i <= 24; i++){
			msg_joint.position[i] = Y(i + 6);
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

		if (std::abs(t_current - (t_end + integrationStep)) <= eps)
		{
			break;
		}

		as.publishFeedback(feedback);
	}

	if (success)
    {
		result.complete = true;
        ROS_INFO("%s: Succeeded", action_name.c_str());
        // set the action state to succeeded
    }
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

	for(unsigned int i = 1; i <= 6; i++){
		for(unsigned int j = 0; j < 3; j++){
				// ROS_INFO("%d, %d", i, j);
				rjf.push_back(m_dynamics->rjf[i][3](j));
		}
	}

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
