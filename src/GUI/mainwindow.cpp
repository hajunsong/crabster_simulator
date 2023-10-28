#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "Common.h"
#include "Inputs.h"
#include "MBD_RecursiveII.h"
#include "Integrator.h"
#include "Outputs.h"

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


MainWindow::MainWindow(int argc, char **argv, QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
	ui->setupUi(this);

	ros::init(argc, argv, "crabster_simulator");
	ros::NodeHandle nh;

	connect(ui->btnLoad, SIGNAL(clicked()), this, SLOT(btnLoadClicked()));
	connect(ui->btnModel, SIGNAL(clicked()), this, SLOT(btnModelClicked()));
	connect(ui->btnRun, SIGNAL(clicked()), this, SLOT(btnRunClicked()));

	m_inputs = new Inputs();
	m_dynamics = new MBD_RecursiveII();
	m_integrator = new Integrator();
	m_outputs = new Outputs();
}

MainWindow::~MainWindow()
{
	delete ui;
	delete rvizRobot;
}

void MainWindow::btnLoadClicked()
{
	 // json file direction
    std::string json_dir = "/mnt/c/Users/hj/Desktop/KRISO/ros_ws/src/crabster_simulator/data/input_data/";
    // std::string json_dir = "/home/keti/Project/KRISO/CRABSTER/ros_ws/src/crabster_simulator/data/input_data/";

	// read simulation parameters
	SimulationData SimData = m_inputs->readSimulationParameter(json_dir, m_dynamics);

	ui->gbSimInfor->setEnabled(true);

	ui->txtEndTime->setText(QString::number(SimData.end_time));
	ui->txtGravity->setText(QString::number(SimData.g));
	ui->txtGravityAxis->setText("[" + QString::number(m_dynamics->gravityAxis(0)) + ", " + QString::number(m_dynamics->gravityAxis(1)) + ", " + QString::number(m_dynamics->gravityAxis(2)) + "]");
	ui->txtIntegStepSize->setText(QString::number(SimData.integration_stepSize));
	ui->txtSaveStepSize->setText(QString::number(SimData.dataSave_stepSize));
	ui->txtSolver->setText(SimData.solver == INTEGRATOR_AB3 ? "AB3" : "RK4");
	ui->txtAnalysisMethod->setText(SimData.analysis_method == METHOD_CONVENTIONAL ? "Convectional" : "Subsystem");

	ui->btnModel->setEnabled(true);
	ui->btnRun->setEnabled(true);

	// base body
	BaseBodyData buf_BaseBody = m_inputs->jsonRead_BaseBody(json_dir);

	std::string urdf_dir = json_dir + "../../urdf/";
	std::string urdf_name = "crabster_description.urdf";
	std::fstream urdf_file;
	urdf_file.open(urdf_dir + urdf_name, std::ios_base::out);
	if(!urdf_file.is_open()){
		std::cout << "Unable to open the file.\n";
		return;
	}

	Eigen::Vector3d rpy;

	std::string urdf = "";
	urdf += "<?xml version=\"1.0\"?>";
	urdf += "\n";
	
	urdf += "<robot\n";
	urdf += "\txmlns:xacro=\"http://wiki.ros.org/xacro\" name=\"crabster\">\n";
	urdf += "\n";

	urdf += "\t<link name=\"world\"></link>\n";

	urdf += "\t<link name=\"" + buf_BaseBody.name + "\">\n";

	urdf += "\t\t<inertial>\n";
	urdf += "\t\t\t<origin\n";
	urdf += "\t\t\t\txyz=\""
		+ std::to_string(buf_BaseBody.rho0p(0)) + " "
		+ std::to_string(buf_BaseBody.rho0p(1)) + " "
		+ std::to_string(buf_BaseBody.rho0p(2)) + "\"\n";
	rpy = mat2rpy(buf_BaseBody.C00);
	
	urdf += "\t\t\t\trpy=\""
		+ std::to_string(rpy(0)) + " "
		+ std::to_string(rpy(1)) + " "
		+ std::to_string(rpy(2)) + "\"/>\n";
	urdf += "\t\t\t<mass\n";
	urdf += "\t\t\t\tvalue=\"" + std::to_string(buf_BaseBody.m0) + "\"/>\n";
	urdf += "\t\t\t<inertia\n";
	urdf += "\t\t\t\tixx=\"" + std::to_string(buf_BaseBody.J0cp(0,0)) + "\"\n";
	urdf += "\t\t\t\tixy=\"" + std::to_string(buf_BaseBody.J0cp(0,1)) + "\"\n";
	urdf += "\t\t\t\tixz=\"" + std::to_string(buf_BaseBody.J0cp(0,2)) + "\"\n";
	urdf += "\t\t\t\tiyy=\"" + std::to_string(buf_BaseBody.J0cp(1,1)) + "\"\n";
	urdf += "\t\t\t\tiyz=\"" + std::to_string(buf_BaseBody.J0cp(1,2)) + "\"\n";
	urdf += "\t\t\t\tizz=\"" + std::to_string(buf_BaseBody.J0cp(2,2)) + "\"/>\n";
	urdf += "\t\t</inertial>\n";
	urdf += "\t\t<visual>\n";
	urdf += "\t\t\t<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
	urdf += "\t\t\t<geometry>\n";
	urdf += "\t\t\t\t<mesh filename=\"package://crabster_simulator/meshes/" + buf_BaseBody.name + ".STL\"/>\n";
	urdf += "\t\t\t</geometry>\n";
	urdf += "\t\t\t<material name=\"\">\n";
	urdf += "\t\t\t\t<color rgba=\"0.8 0.8 0.8 1\"/>\n";
	urdf += "\t\t\t</material>\n";
	urdf += "\t\t</visual>\n"; 

	urdf += "\t\t<collision>\n";
	urdf += "\t\t\t<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
	urdf += "\t\t\t<geometry>\n";
	urdf += "\t\t\t\t<mesh filename=\"package://crabster_simulator/meshes/" + buf_BaseBody.name + ".STL\"/>\n";
	urdf += "\t\t\t</geometry>\n";
	urdf += "\t\t</collision>\n";

	urdf += "\t</link>\n";
	
	urdf += "\t<joint name=\"world_to_" + buf_BaseBody.name + "\" type=\"fixed\">\n";
	urdf += "\t\t<origin\n";
	urdf += "\t\t\txyz=\"" 
		+ std::to_string(buf_BaseBody.r0(0)) + " " 
		+ std::to_string(buf_BaseBody.r0(1)) + " "
		+ std::to_string(buf_BaseBody.r0(2)) + "\"\n";
	
	// Euler parameter scaling
	 buf_BaseBody.p0.normalize();

	// transformation matrix
	double e00 = buf_BaseBody.p0(0);
	Eigen::Vector3d e0;
	e0(0) = buf_BaseBody.p0(1);
	e0(1) = buf_BaseBody.p0(2);
	e0(2) = buf_BaseBody.p0(3);
	Eigen::Matrix3d e0_tilde = skew(e0);

	// base body orientation
	Eigen::MatrixXd E0, G0;
	E0.resize(3, 4);
	G0.resize(3, 4);
	E0.col(0) = -e0;
	E0.block(0, 1, 3, 3) = e0_tilde + e00 * Eigen::Matrix3d::Identity();
	G0.col(0) = -e0;
	G0.block(0, 1, 3, 3) = -e0_tilde + e00 * Eigen::Matrix3d::Identity();

	Eigen::Matrix3d A0 = E0 * G0.transpose();

	rpy = mat2rpy(A0);
	urdf += "\t\t\trpy=\""
		+ std::to_string(rpy(0)) + " " 
		+ std::to_string(rpy(1)) + " "
		+ std::to_string(rpy(2)) + "\"/>\n";
	urdf += "\t\t<parent link=\"world\"/>\n";
	urdf += "\t\t<child link=\"" + buf_BaseBody.name + "\"/>\n";
	urdf += "\t</joint>\n";

	// subsystem
	// int sub_id = 1;
	// int body_id = 1;
	SubsystemData parent;
	for (int sub_id = 1; sub_id <= 6; sub_id++){
		for (int body_id = 1; body_id <= 4; body_id++){
			SubsystemData child = m_inputs->jsonRead_Subsystem(json_dir, sub_id, body_id);

			if(body_id > 1){
				parent = m_inputs->jsonRead_Subsystem(json_dir, sub_id, body_id - 1);
			}
			
			urdf += "\t<link name=\"" + child.name + "\">\n";

			urdf += "\t\t<inertial>\n";
			urdf += "\t\t\t<origin\n";
			urdf += "\t\t\t\txyz=\""
				+ std::to_string(child.rhojp(0)) + " "
				+ std::to_string(child.rhojp(1)) + " "
				+ std::to_string(child.rhojp(2)) + "\"\n";
			rpy = mat2rpy(child.Cjj);
			urdf += "\t\t\t\trpy=\""
				+ std::to_string(rpy(0)) + " " 
				+ std::to_string(rpy(1)) + " "
				+ std::to_string(rpy(2)) + "\"/>\n";
			urdf += "\t\t\t<mass\n";
			urdf += "\t\t\t\tvalue=\"" + std::to_string(child.mj) + "\"/>\n";
			urdf += "\t\t\t<inertia\n";
			urdf += "\t\t\t\tixx=\"" + std::to_string(child.Jjcp(0,0)) + "\"\n";
			urdf += "\t\t\t\tixy=\"" + std::to_string(child.Jjcp(0,1)) + "\"\n";
			urdf += "\t\t\t\tixz=\"" + std::to_string(child.Jjcp(0,2)) + "\"\n";
			urdf += "\t\t\t\tiyy=\"" + std::to_string(child.Jjcp(1,1)) + "\"\n";
			urdf += "\t\t\t\tiyz=\"" + std::to_string(child.Jjcp(1,2)) + "\"\n";
			urdf += "\t\t\t\tizz=\"" + std::to_string(child.Jjcp(2,2)) + "\"/>\n";
			urdf += "\t\t</inertial>\n";

			urdf += "\t\t<visual>\n";
			urdf += "\t\t\t<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
			urdf += "\t\t\t<geometry>\n";
			urdf += "\t\t\t\t<mesh filename=\"package://crabster_simulator/meshes/" + child.name + ".STL\"/>\n";
			urdf += "\t\t\t</geometry>\n";
			urdf += "\t\t\t<material name=\"\">\n";

			switch(body_id){
				case 1:
					urdf += "\t\t\t\t<color rgba=\"0.8 0.1 0.1 1\"/>\n";
					break;
				case 2:
					urdf += "\t\t\t\t<color rgba=\"0.1 0.8 0.1 1\"/>\n";
					break;
				case 3:
					urdf += "\t\t\t\t<color rgba=\"0.1 0.1 0.8 1\"/>\n";
					break;
				case 4:
					urdf += "\t\t\t\t<color rgba=\"0.1 0.1 0.1 1\"/>\n";
					break;
				default:
					urdf += "\t\t\t\t<color rgba=\"0.792156862745098 0.819607843137255 0.933333333333333 1\"/>\n";
					break;
			}

			urdf += "\t\t\t</material>\n";
			urdf += "\t\t</visual>\n"; 

			urdf += "\t\t<collision>\n";
			urdf += "\t\t\t<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
			urdf += "\t\t\t<geometry>\n";
			urdf += "\t\t\t\t<mesh filename=\"package://crabster_simulator/meshes/" + child.name + ".STL\"/>\n";
			urdf += "\t\t\t</geometry>\n";
			urdf += "\t\t</collision>\n";

			urdf += "\t</link>\n";

			if(body_id == 1)
			{
				urdf += "\t<joint name=\"" + buf_BaseBody.name + "_to_" + child.name + "\" type=\"revolute\">\n";
			}
			else{
				urdf += "\t<joint name=\"" + parent.name + "_to_" + child.name + "\" type=\"revolute\">\n";
			}

			urdf += "\t\t<origin\n";
			urdf += "\t\t\txyz=\"" 
				+ std::to_string(child.sijp(0)) + " " 
				+ std::to_string(child.sijp(1)) + " "
				+ std::to_string(child.sijp(2)) + "\"\n";
			rpy = mat2rpy(child.Cij);
			urdf += "\t\t\trpy=\""
				+ std::to_string(rpy(0)) + " "
				+ std::to_string(rpy(1)) + " "
				+ std::to_string(rpy(2)) + "\"/>\n";
			if(body_id == 1)
			{
				urdf += "\t\t<parent link=\"" + buf_BaseBody.name + "\"/>\n";
			}
			else
			{
				urdf += "\t\t<parent link=\"" + parent.name + "\"/>\n";
			}
			urdf += "\t\t<child link=\"" + child.name + "\"/>\n";
			urdf += "\t\t<axis xyz=\"0 0 1\"/>\n";
			urdf += "\t\t<limit\n";
			urdf += "\t\t\tlower=\"-3.14\"\n";
			urdf += "\t\t\tupper=\"3.14\"\n";
			urdf += "\t\t\teffort=\"10\"\n";
			urdf += "\t\t\tvelocity=\"100\"/>\n";
			urdf += "\t</joint>\n";
		}
	}

	urdf += "</robot>";

	urdf_file << urdf;
	urdf_file.close();
}

void MainWindow::btnModelClicked()
{
	rvizRobot = new Rviz();
	rvizRobot->initRvizRobotModel(ui->vlRobotModel);
}

void MainWindow::btnRunClicked()
{

}
