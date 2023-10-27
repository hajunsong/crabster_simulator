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
	urdf += "\t<visual>\n";
	urdf += "\t\torigin xyz=\"0 0 0"; 
	
	urdf += "\t<joint name=\"BaseBody_Joint\" type=\"fixed\">\n";
	urdf += "\t\t<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />\n";
	urdf += "\t\t<parent link=\"world\" />\n";
	urdf += "\t\t<parent link=\"BaseBody_Joint\" />\n";
	urdf += "\t</joint>";

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
