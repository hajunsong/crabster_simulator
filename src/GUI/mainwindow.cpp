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
    // std::string json_dir = "/mnt/c/Users/hj/Desktop/KRISO/ros_ws/src/crabster_simulator/data/input_data/";
    std::string json_dir = "/home/keti/Project/KRISO/CRABSTER/ros_ws/src/crabster_simulator/data/input_data/";

	// read simulation parameters
	SimulationData SimData = m_inputs->readSimulationParameter(json_dir, m_dynamics);

	// base body
	BaseBodyData buf_BaseBody = m_inputs->jsonRead_BaseBody(json_dir);

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

	std::string urdf_dir = json_dir + "../../urdf/";
	std::string urdf_name = "crabster_description.urdf";
	std::fstream urdf_file;
	urdf_file.open(urdf_dir + urdf_name, std::ios_base::out);
	if(!urdf_file.is_open()){
		std::cout << "Unable to open the file.\n";
		return;
	}

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
		+ QString::number(buf_BaseBody.rho0p(0)).toStdString() + " "
		+ QString::number(buf_BaseBody.rho0p(1)).toStdString() + " "
		+ QString::number(buf_BaseBody.rho0p(2)).toStdString() + "\"\n";
	urdf += "\t\t\t\trpy=\"0 0 0\"/>\n";
	urdf += "\t\t\t<mass\n";
	urdf += "\t\t\t\tvalue=\"" + QString::number(buf_BaseBody.m0).toStdString() + "\"/>\n";
	urdf += "\t\t\t<inertia\n";
	urdf += "\t\t\t\tixx=\"" + QString::number(buf_BaseBody.J0cp(0,0)).toStdString() + "\"\n";
	urdf += "\t\t\t\tixy=\"" + QString::number(buf_BaseBody.J0cp(0,1)).toStdString() + "\"\n";
	urdf += "\t\t\t\tixz=\"" + QString::number(buf_BaseBody.J0cp(0,2)).toStdString() + "\"\n";
	urdf += "\t\t\t\tiyy=\"" + QString::number(buf_BaseBody.J0cp(1,1)).toStdString() + "\"\n";
	urdf += "\t\t\t\tiyz=\"" + QString::number(buf_BaseBody.J0cp(1,2)).toStdString() + "\"\n";
	urdf += "\t\t\t\tizz=\"" + QString::number(buf_BaseBody.J0cp(2,2)).toStdString() + "\"/>\n";
	urdf += "\t\t</inertial>\n";

	urdf += "\t\t<visual>\n";
	urdf += "\t\t\t<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
	urdf += "\t\t\t<geometry>\n";
	urdf += "\t\t\t\t<mesh filename=\"package://crabster_simulator/meshes/" + buf_BaseBody.name + ".STL\"/>\n";
	urdf += "\t\t\t</geometry>\n";
	urdf += "\t\t\t<material name=\"\">\n";
	urdf += "\t\t\t\t<color rgba=\"0.792156862745098 0.819607843137255 0.933333333333333 1\"/>\n";
	urdf += "\t\t\t</material>\n";
	urdf += "\t\t</visual>\n"; 

	urdf += "\t\t<collision>\n";
	urdf += "\t\t\t<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
	urdf += "\t\t\t<geometry>\n";
	urdf += "\t\t\t\t<mesh filename=\"package://crabster_simulator/meshes/" + buf_BaseBody.name + ".STL\"/>\n";
	urdf += "\t\t\t</geometry>\n";
	urdf += "\t\t</collision>\n";

	urdf += "\t</link>";
	
	urdf += "\t<joint name=\"world_to_" + buf_BaseBody.name + "\" type=\"fixed\">\n";
	urdf += "\t\t<origin xyz=\"" 
		+ QString::number(buf_BaseBody.r0(0)).toStdString() + " " 
		+ QString::number(buf_BaseBody.r0(1)).toStdString() + " "
		+ QString::number(buf_BaseBody.r0(1)).toStdString() + " "
		+ "\" rpy=\"0 0 0\" />\n";
	urdf += "\t\t<parent link=\"world\" />\n";
	urdf += "\t\t<child link=\"" + buf_BaseBody.name + "\" />\n";
	urdf += "\t</joint>\n";

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
