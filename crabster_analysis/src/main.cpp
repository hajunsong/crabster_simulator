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
std::vector<double> rjf, road_h;

sensor_msgs::JointState msg_joint;
/* ============= global values =================================================== */

void funcSub_foot_contact_z(const std_msgs::Float64MultiArray::ConstPtr &input)
{
    if (input->data.size() == 6)
    {
        // std::cout<< input->data[0] <<" / " << input->data[1] <<" / " << input->data[2] <<" / " << input->data[3] <<" / " << input->data[4] <<" / " << input->data[5] <<" / " << std::endl;
        road_h.clear();
        for(uint i = 0; i < 6; i++){
            road_h.push_back(input->data[i]*0 - 10);
            // std::cout << road_h[i] << " / ";
        }
        // std::cout << std::endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "crabster_analysis_node");
	ros::NodeHandle nh;
	ros::MultiThreadedSpinner spinner(4); // Use 4 threads

    Crabster crabster("crabster_analysis_server", nh);

    crabster_msgs::CrabsterPose msg;
    ros::Publisher pubCrabsterPose = nh.advertise<crabster_msgs::CrabsterPose>("crabster_pose", 1);

    std_msgs::Float64MultiArray array_contact_req;
    ros::Publisher pub_foot_contact_req = nh.advertise<std_msgs::Float64MultiArray>("crabster/foot/contact_req", 18);

    std_msgs::Float64MultiArray array_contact_z;
    ros::Subscriber sub_foot_contact_z;
    sub_foot_contact_z = nh.subscribe<std_msgs::Float64MultiArray>("crabster/foot/contact_z", 6, &funcSub_foot_contact_z);

	bool standalone;
	nh.getParam("standalone", standalone);
	ROS_INFO("standalone : %d", (int)standalone);

    if(standalone)
        crabster.run_single_init();

    rjf.clear();
    road_h.clear();
    road_h.assign(6, -10);

	ros::Publisher pub_joint_command = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

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
		if(standalone)
			crabster.run_single();

        for(uint i = 0; i < Y.size(); i++){
            msg.pose.push_back(Y(i));
        }
        pubCrabsterPose.publish(msg);
        msg.pose.clear();

		msg_joint.header.stamp = ros::Time::now();
		pub_joint_command.publish(msg_joint);

        array_contact_req.data.clear();
        if(rjf.size() == 18){
            for(uint i = 0; i < 18; i++){
                array_contact_req.data.push_back(rjf[i]);
            }
            pub_foot_contact_req.publish(array_contact_req);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

	return 0;
}