#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(int argc, char **argv, QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
	ui->setupUi(this);

	ros::init(argc, argv, "crabster_visual");
	ros::NodeHandle nh;

	connect(ui->btnLoad, SIGNAL(clicked()), this, SLOT(btnLoadClicked()));
	connect(ui->btnModel, SIGNAL(clicked()), this, SLOT(btnModelClicked()));
	connect(ui->btnRun, SIGNAL(clicked()), this, SLOT(btnRunClicked()));
}

MainWindow::~MainWindow()
{
	delete ui;
	delete rvizRobot;
}

void MainWindow::btnLoadClicked()
{
	 // json file direction
	std::string path = ros::package::getPath("crabster_analysis");
	std::string data_dir = path + "/data/input_data/";

	std::string sim_json = "AA_Simulation_Parameter.json";
	std::ifstream json_file(data_dir + sim_json, std::ifstream::binary);
	Json::CharReaderBuilder builder;
	builder["collectCommnets"] = false;
	Json::Value data;
	JSONCPP_STRING errs;
	bool ok = parseFromStream(builder, json_file, &data, &errs);

	if(ok){
		try{
			ui->txtEndTime->setText(QString::number(data["end_time"].asDouble()));
			ui->txtGravity->setText(QString::number(data["g"].asDouble()));
			ui->txtGravityAxis->setText(QString::fromStdString("[" 
				+ std::to_string(data["gravity_axis"][0].asDouble()) + ", " 
				+ std::to_string(data["gravity_axis"][1].asDouble()) + ", " 
				+ std::to_string(data["gravity_axis"][2].asDouble()) + "]"));
			ui->txtIntegStepSize->setText(QString::number(data["integration_stepSize"].asDouble()));
			ui->txtSaveStepSize->setText(QString::number(data["dataSave_stepSize"].asDouble()));
			ui->txtSolver->setText(QString::fromStdString(data["solver"].asString()));
			ui->txtAnalysisMethod->setText(QString::fromStdString(data["analysis_method"].asString()));
		}
		catch (Json::Exception &e) {
			ROS_INFO("%s", e.what());
		}
	}

	std::string urdf = "";

	Eigen::Vector3d rpy;
	Eigen::Matrix3d mat;

	std::string base_json = "BaseBody.json";
	std::ifstream json_file_base(data_dir + base_json, std::ifstream::binary);
	Json::CharReaderBuilder builderBase;
	Json::Value dataBase;
	JSONCPP_STRING errsBase;
	bool okBase = parseFromStream(builderBase, json_file_base, &dataBase, &errsBase);

	if(okBase){
		try{
			urdf += "<?xml version=\"1.0\"?>";
			urdf += "\n";

			urdf += "<robot\n";
			urdf += "\txmlns:xacro=\"http://wiki.ros.org/xacro\" name=\"crabster\">\n";
			urdf += "\n";

			urdf += "\t<link name=\"world\"></link>\n";

			urdf += "\t<link name=\"" + dataBase["name"].asString() + "\">\n";

			urdf += "\t\t<inertial>\n";
			urdf += "\t\t\t<origin\n";
			urdf += "\t\t\t\txyz=\"" 
				+ std::to_string(dataBase["rho0p"][0].asDouble()) + " " 
				+ std::to_string(dataBase["rho0p"][1].asDouble()) + " " 
				+ std::to_string(dataBase["rho0p"][2].asDouble()) + "\"\n";
			
			mat(0,0) = dataBase["C00"][0][0].asDouble();
			mat(0,1) = dataBase["C00"][0][1].asDouble();
			mat(0,2) = dataBase["C00"][0][2].asDouble();
			
			mat(1,0) = dataBase["C00"][1][0].asDouble();
			mat(1,1) = dataBase["C00"][1][1].asDouble();
			mat(1,2) = dataBase["C00"][1][2].asDouble();
			
			mat(2,0) = dataBase["C00"][2][0].asDouble();
			mat(2,1) = dataBase["C00"][2][1].asDouble();
			mat(2,2) = dataBase["C00"][2][2].asDouble();
			rpy = mat2rpy(mat);

			urdf += "\t\t\t\trpy=\"" + std::to_string(rpy(0)) + " " + std::to_string(rpy(1)) + " " + std::to_string(rpy(2)) + "\"/>\n";
			urdf += "\t\t\t<mass\n";
			urdf += "\t\t\t\tvalue=\"" + std::to_string(dataBase["m0"].asDouble()) + "\"/>\n";
			urdf += "\t\t\t<inertia\n";
			urdf += "\t\t\t\tixx=\"" + std::to_string(dataBase["J0cp"][0][0].asDouble()) + "\"\n";
			urdf += "\t\t\t\tixy=\"" + std::to_string(dataBase["J0cp"][0][1].asDouble()) + "\"\n";
			urdf += "\t\t\t\tixz=\"" + std::to_string(dataBase["J0cp"][0][2].asDouble()) + "\"\n";
			urdf += "\t\t\t\tiyy=\"" + std::to_string(dataBase["J0cp"][1][1].asDouble()) + "\"\n";
			urdf += "\t\t\t\tiyz=\"" + std::to_string(dataBase["J0cp"][1][2].asDouble()) + "\"\n";
			urdf += "\t\t\t\tizz=\"" + std::to_string(dataBase["J0cp"][2][2].asDouble()) + "\"/>\n";
			urdf += "\t\t</inertial>\n";
			urdf += "\t\t<visual>\n";
			urdf += "\t\t\t<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
			urdf += "\t\t\t<geometry>\n";
			urdf += "\t\t\t\t<mesh filename=\"package://crabster_visual/meshes/" + dataBase["name"].asString() + ".STL\"/>\n";
			urdf += "\t\t\t</geometry>\n";
			urdf += "\t\t\t<material name=\"\">\n";
			urdf += "\t\t\t\t<color rgba=\"0.8 0.8 0.8 1\"/>\n";
			urdf += "\t\t\t</material>\n";
			urdf += "\t\t</visual>\n";

			urdf += "\t\t<collision>\n";
			urdf += "\t\t\t<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
			urdf += "\t\t\t<geometry>\n";
			urdf += "\t\t\t\t<mesh filename=\"package://crabster_visual/meshes/" + dataBase["name"].asString() + ".STL\"/>\n";
			urdf += "\t\t\t</geometry>\n";
			urdf += "\t\t</collision>\n";

			urdf += "\t</link>\n";

			urdf += "\t<joint name=\"world_to_" + dataBase["name"].asString() + "\" type=\"fixed\">\n";
			urdf += "\t\t<origin\n";
			urdf += "\t\t\txyz=\"" 
				+ std::to_string(dataBase["r0"][0].asDouble()) + " " 
				+ std::to_string(dataBase["r0"][1].asDouble()) + " " 
				+ std::to_string(dataBase["r0"][2].asDouble()) + "\"\n";

			// Euler parameter scaling
			Eigen::VectorXd p0 = Eigen::VectorXd(4);
			p0(0) = dataBase["p0"][0].asDouble();
			p0(1) = dataBase["p0"][1].asDouble();
			p0(2) = dataBase["p0"][2].asDouble();
			p0(3) = dataBase["p0"][3].asDouble();
			p0.normalize();

			// transformation matrix
			double e00 = p0(0);
			Eigen::Vector3d e0;
			e0(0) = p0(1);
			e0(1) = p0(2);
			e0(2) = p0(3);
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
			urdf += "\t\t\trpy=\"" + std::to_string(rpy(0)) + " " + std::to_string(rpy(1)) + " " + std::to_string(rpy(2)) + "\"/>\n";
			urdf += "\t\t<parent link=\"world\"/>\n";
			urdf += "\t\t<child link=\"" + dataBase["name"].asString() + "\"/>\n";
			urdf += "\t</joint>\n";

		} 
		catch (Json::Exception &e) {
			ROS_INFO("%s", e.what());
		}
	}

	std::string child_json, parent_json;
	// std::ifstream json_file_base(data_dir + base_json, std::ifstream::binary);
	Json::CharReaderBuilder builderChild, builderParent;
	Json::Value dataChild, dataParent;
	JSONCPP_STRING errsChild, errsParent;
	// bool okBase = parseFromStream(builderBase, json_file_base, &dataBase, &errsBase);
	bool okChild, okParent;

	try{
		for(int sub_id = 1; sub_id <= 6; sub_id++){
			for(int body_id = 1; body_id <= 4; body_id++){
				child_json = "Subsystem" + std::to_string(sub_id) + std::to_string(body_id) + ".json";
				std::ifstream json_file_child(data_dir + child_json, std::ifstream::binary);
				okChild = parseFromStream(builderChild, json_file_child, &dataChild, &errsChild);

				if(body_id > 1)
				{
					parent_json = "Subsystem" + std::to_string(sub_id) + std::to_string(body_id - 1) + ".json";
					std::ifstream json_file_parent(data_dir + parent_json, std::ifstream::binary);
					okParent = parseFromStream(builderParent, json_file_parent, &dataParent, &errsParent);
				}

				urdf += "\t<link name=\"" + dataChild["name"].asString() + "\">\n";

				urdf += "\t\t<inertial>\n";
				urdf += "\t\t\t<origin\n";
				urdf += "\t\t\t\txyz=\"" 
					+ std::to_string(dataChild["rhojp"][0].asDouble()) + " " 
					+ std::to_string(dataChild["rhojp"][1].asDouble()) + " " 
					+ std::to_string(dataChild["rhojp"][2].asDouble()) + "\"\n";

				mat(0,0) = dataChild["Cjj"][0][0].asDouble();
				mat(0,1) = dataChild["Cjj"][0][1].asDouble();
				mat(0,2) = dataChild["Cjj"][0][2].asDouble();
				
				mat(1,0) = dataChild["Cjj"][1][0].asDouble();
				mat(1,1) = dataChild["Cjj"][1][1].asDouble();
				mat(1,2) = dataChild["Cjj"][1][2].asDouble();
				
				mat(2,0) = dataChild["Cjj"][2][0].asDouble();
				mat(2,1) = dataChild["Cjj"][2][1].asDouble();
				mat(2,2) = dataChild["Cjj"][2][2].asDouble();

				rpy = mat2rpy(mat);
				urdf += "\t\t\t\trpy=\"" + std::to_string(rpy(0)) + " " + std::to_string(rpy(1)) + " " + std::to_string(rpy(2)) + "\"/>\n";
				urdf += "\t\t\t<mass\n";
				urdf += "\t\t\t\tvalue=\"" + std::to_string(dataChild["mj"].asDouble()) + "\"/>\n";
				urdf += "\t\t\t<inertia\n";
				urdf += "\t\t\t\tixx=\"" + std::to_string(dataChild["Jjcp"][0][0].asDouble()) + "\"\n";
				urdf += "\t\t\t\tixy=\"" + std::to_string(dataChild["Jjcp"][0][1].asDouble()) + "\"\n";
				urdf += "\t\t\t\tixz=\"" + std::to_string(dataChild["Jjcp"][0][2].asDouble()) + "\"\n";
				urdf += "\t\t\t\tiyy=\"" + std::to_string(dataChild["Jjcp"][1][1].asDouble()) + "\"\n";
				urdf += "\t\t\t\tiyz=\"" + std::to_string(dataChild["Jjcp"][1][2].asDouble()) + "\"\n";
				urdf += "\t\t\t\tizz=\"" + std::to_string(dataChild["Jjcp"][2][2].asDouble()) + "\"/>\n";
				urdf += "\t\t</inertial>\n";

				urdf += "\t\t<visual>\n";
				urdf += "\t\t\t<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
				urdf += "\t\t\t<geometry>\n";
				urdf += "\t\t\t\t<mesh filename=\"package://crabster_visual/meshes/" + dataChild["name"].asString() + ".STL\"/>\n";
				urdf += "\t\t\t</geometry>\n";
				urdf += "\t\t\t<material name=\"\">\n";

				switch (body_id)
				{
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
				urdf += "\t\t\t\t<mesh filename=\"package://crabster_visual/meshes/" + dataChild["name"].asString() + ".STL\"/>\n";
				urdf += "\t\t\t</geometry>\n";
				urdf += "\t\t</collision>\n";

				urdf += "\t</link>\n";

				if (body_id == 1)
				{
					urdf += "\t<joint name=\"" + dataBase["name"].asString() + "_to_" + dataChild["name"].asString() + "\" type=\"revolute\">\n";
				}
				else
				{
					urdf += "\t<joint name=\"" + dataParent["name"].asString() + "_to_" + dataChild["name"].asString() + "\" type=\"revolute\">\n";
				}

				urdf += "\t\t<origin\n";
				urdf += "\t\t\txyz=\"" 
					+ std::to_string(dataChild["sijp"][0].asDouble()) + " " 
					+ std::to_string(dataChild["sijp"][1].asDouble()) + " " 
					+ std::to_string(dataChild["sijp"][2].asDouble()) + "\"\n";

				mat(0,0) = dataChild["Cij"][0][0].asDouble();
				mat(0,1) = dataChild["Cij"][0][1].asDouble();
				mat(0,2) = dataChild["Cij"][0][2].asDouble();
				
				mat(1,0) = dataChild["Cij"][1][0].asDouble();
				mat(1,1) = dataChild["Cij"][1][1].asDouble();
				mat(1,2) = dataChild["Cij"][1][2].asDouble();
				
				mat(2,0) = dataChild["Cij"][2][0].asDouble();
				mat(2,1) = dataChild["Cij"][2][1].asDouble();
				mat(2,2) = dataChild["Cij"][2][2].asDouble();

				rpy = mat2rpy(mat);
				urdf += "\t\t\trpy=\"" + std::to_string(rpy(0)) + " " + std::to_string(rpy(1)) + " " + std::to_string(rpy(2)) + "\"/>\n";
				if (body_id == 1)
				{
					urdf += "\t\t<parent link=\"" + dataBase["name"].asString() + "\"/>\n";
				}
				else
				{
					urdf += "\t\t<parent link=\"" + dataParent["name"].asString() + "\"/>\n";
				}
				urdf += "\t\t<child link=\"" + dataChild["name"].asString() + "\"/>\n";
				urdf += "\t\t<axis xyz=\"0 0 1\"/>\n";
				urdf += "\t\t<limit\n";
				urdf += "\t\t\tlower=\"-3.14\"\n";
				urdf += "\t\t\tupper=\"3.14\"\n";
				urdf += "\t\t\teffort=\"10\"\n";
				urdf += "\t\t\tvelocity=\"100\"/>\n";
				urdf += "\t</joint>\n";
			}
		}
	} 
	catch (Json::Exception &e) {
		ROS_INFO("%s", e.what());
	}

	urdf += "</robot>";

	std::string urdf_dir = ros::package::getPath("crabster_visual") + "/urdf/";
	std::string urdf_name = "crabster_description.urdf";
	std::fstream urdf_file;
	urdf_file.open(urdf_dir + urdf_name, std::ios_base::out);
	if (!urdf_file.is_open())
	{
		std::cout << "Unable to open the file.\n";
		return;
	}

	urdf_file << urdf;
	urdf_file.close();

	ui->gbSimInfor->setEnabled(true);

	ui->btnModel->setEnabled(true);
	ui->btnRun->setEnabled(true);
}

void MainWindow::btnModelClicked()
{
	system("gnome-terminal -- sh -c \"roslaunch crabster_visual display.launch\"");
	// system("roslaunch crabster_visual display.launch && exit");

	std::cout << "=====================================================================================================================" << std::endl;

	rvizRobot = new Rviz();
	rvizRobot->initRvizRobotModel(ui->vlRobotModel);

	ui->btnModel->setEnabled(false);
}

void MainWindow::btnRunClicked()
{

}
