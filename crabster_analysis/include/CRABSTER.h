#include "Common.h"
#include "Inputs.h"
#include "MBD_RecursiveII.h"
#include "Integrator.h"
#include "Outputs.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <crabster_msgs/CrabsterSimulationAction.h>
#include <crabster_msgs/CrabsterPose.h>

#include <stdlib.h>
#include <pthread.h>

const double eps = 1e-7;

class Crabster
{
protected:
    actionlib::SimpleActionServer<crabster_msgs::CrabsterSimulationAction> as;
    std::string action_name;
    crabster_msgs::CrabsterSimulationFeedback feedback;
    crabster_msgs::CrabsterSimulationResult result;

public:
    Crabster(std::string name, ros::NodeHandle nh);
    ~Crabster();

    void run_single_init();
    void run_single();
    void executeCB(const crabster_msgs::CrabsterSimulationGoalConstPtr &goal);

    static void *run_single_func(void *arg);


private:
    // json file direction
	std::string path;
    std::string json_dir;

    // class header
	Inputs* m_inputs;
	MBD_RecursiveII* m_dynamics;
	Integrator* m_integrator;
	Outputs* m_outputs;

    pthread_t run_single_thread;

    double t_end, integrationStep, dataSaveStep;
    int solver, dataSave_count;;
};