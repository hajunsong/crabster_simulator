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

	std::cout << "\nSimulation start!\n" << std::endl;
	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();

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

		rjf.clear();
		for(unsigned int i = 1; i <= 6; i++){
			for(unsigned int j = 0; j < 3; j++){
				// ROS_INFO("%d, %d", i, j);
				rjf.push_back(m_dynamics->rjf[i][3](j));
				std::cout << m_dynamics->rjf[i][3](j) << " ";
			}
			std::cout << std::endl;
		}
		std::cout << std::endl;

		int y_indx = 13;
		int j_indx = 1;
		int cnt = 0;
		
		if(Y.size() > 0){
			msg_joint.position[0] = Y(2) + 0.5;
			for(int sub = 1; sub <= nSubsystem; sub++){
				cnt = 0;
				for(int i = 0; i < 4; i++){
					if(m_dynamics->flag_motion[sub][i]){
						msg_joint.position[j_indx++] = m_dynamics->m_motion->inputMotion(t_current, sub, i)(0);
					}
					else{
						msg_joint.position[j_indx++] = Y(y_indx++);
					}
				}
				y_indx += (4 - nJointMotion[sub]);
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

		if (std::abs(t_current - (t_end + integrationStep)) <= eps)
		{
			break;
		}

		as.publishFeedback(feedback);
	}
	
	std::chrono::high_resolution_clock::time_point t_stop = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> t_span = std::chrono::duration_cast<std::chrono::milliseconds>(t_stop - t_start);

	std::cout << "Simulation time = " << std::to_string(t_end) << " [sec]" << std::endl;
	std::cout << "Computation time = " << t_span.count() << " [msec]" << std::endl;

	// m_outputs->csvWrite();

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
	std::cout << "t_current = " << t_current << std::endl;

	dY = m_dynamics->dynamics_analysis(t_current, Y);
	for(uint i = 0; i < Y.size(); i++){
		std::cout << Y(i) << " ";
	}
	std::cout << std::endl;

	for(uint i = 0; i < dY.size(); i++){
		std::cout << dY(i) << " ";
	}
	std::cout << std::endl;

	rjf.clear();
	// for(unsigned int i = 1; i <= 6; i++){
	// 	for(unsigned int j = 0; j < 3; j++){
	// 		// ROS_INFO("%d, %d", i, j);
	// 		// rjf.push_back(m_dynamics->rjf[i][3](j));
	// 		// std::cout << m_dynamics->rjf[i][3](j) << " ";
	// 	}
	// 	std::cout << std::endl;
	// }
	// std::cout << std::endl;

	int y_indx = 13;
	int j_indx = 1;
	int cnt = 0;
	
	if(Y.size() > 0){
		msg_joint.position[0] = Y(2) + 0.5;
		for(int sub = 1; sub <= nSubsystem; sub++){
			cnt = 0;
			for(int i = 0; i < 4; i++){
				if(m_dynamics->flag_motion[sub][i]){
					msg_joint.position[j_indx++] = m_dynamics->m_motion->inputMotion(t_current, sub, i)(0);
				}
				else{
					msg_joint.position[j_indx++] = Y(y_indx++);
				}
			}
			y_indx += (4 - nJointMotion[sub]);
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
