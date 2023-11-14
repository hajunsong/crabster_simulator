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
Eigen::VectorXd Y, dY;

// current time stamp
double t_current;
/* ============= global values =================================================== */

int main(int argc, char** argv)
{
    ros::init(argc, argv, "crabster_analysis_node");
	ros::NodeHandle nh;

    Crabster crabster("crabster_analysis_server", nh);

    crabster_msgs::CrabsterPose msg;
    ros::Publisher pubCrabsterPose = nh.advertise<crabster_msgs::CrabsterPose>("crabster_pose", 1);

    ros::Rate loop_rate(100);

    // crabster.run_single_init();

    while(ros::ok())
    {
        for(uint i = 0; i < Y.size(); i++){
            msg.pose.push_back(Y(i));
        }

        pubCrabsterPose.publish(msg);

        msg.pose.clear();

        ros::spinOnce();
        loop_rate.sleep();
    }

	return 0;
}