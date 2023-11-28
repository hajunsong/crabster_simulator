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

// leg contact point
std::vector<double> rjf;
/* ============= global values =================================================== */

int main(int argc, char** argv)
{
    ros::init(argc, argv, "crabster_analysis_node");
	ros::NodeHandle nh;

    bool standalone;
    nh.getParam("standalone", standalone);
    ROS_INFO("standalone : %d", (int)standalone);

    Crabster crabster("crabster_analysis_server", nh);

    crabster_msgs::CrabsterPose msg;
    ros::Publisher pubCrabsterPose = nh.advertise<crabster_msgs::CrabsterPose>("crabster_pose", 1);

    ros::Rate loop_rate(100);

    if(standalone)
        crabster.run_single_init();
    rjf.clear();

    while(ros::ok())
    {
        if(standalone)
            crabster.run_single();

        for(uint i = 0; i < Y.size(); i++){
            msg.pose.push_back(Y(i));
        }

        for(uint i = 0; i < 18; i++){
            msg.pose.push_back(rjf[i]);
        }

        pubCrabsterPose.publish(msg);

        msg.pose.clear();

        ros::spinOnce();
        loop_rate.sleep();
    }

	return 0;
}