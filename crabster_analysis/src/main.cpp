#include "CRABSTER.h"

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

// Y vector
Eigen::VectorXd Y;
/* ============= global values =================================================== */

int main(int argc, char** argv)
{
    ros::init(argc, argv, "crabster_analysis_node");
	ros::NodeHandle nh;

    std::string mode;
    nh.getParam("mode", mode);
    ROS_INFO("mode : %s", mode.c_str());

    Crabster crabster("crabster_analysis_server", nh);

    if(mode == "single")
    {
        crabster.run_single();
    }
    else if(mode == "server")
    {

    }

    ros::Publisher pubCrabsterPose = nh.advertise<crabster_msgs::CrabsterPose>("crabster_pose", 1);

    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        // ROS_INFO("Y vector size : %ld", Y.size());

        for(uint i = 0; i < Y.size(); i++){
            
        }



        ros::spinOnce();
        loop_rate.sleep();

    }

	return 0;
}