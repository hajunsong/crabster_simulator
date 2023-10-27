#include "Inputs.h"

SimulationData Inputs::readSimulationParameter(std::string json_dir, MBD_RecursiveII* m_dynamics)
{
	std::string file_name = json_dir + "AA_Simulation_Parameter.json";

	std::ifstream json_file(file_name, std::ifstream::binary);

	Json::CharReaderBuilder builder;
	builder["collectComments"] = false;
	Json::Value data;
	JSONCPP_STRING errs;
	bool ok = parseFromStream(builder, json_file, &data, &errs);

	SimulationData buf;
	if (ok)
	{
		buf.start_time = data["start_time"].asDouble();
		buf.end_time = data["end_time"].asDouble();
		buf.integration_stepSize = data["integration_stepSize"].asDouble();
		buf.dataSave_stepSize = data["dataSave_stepSize"].asDouble();
		buf.g = data["g"].asDouble();

		if (data["analysis_method"] == "Conventional") 
		{ 
			buf.analysis_method = METHOD_CONVENTIONAL;
		}
		else if(data["analysis_method"] == "Subsystem")
		{ 
			buf.analysis_method = METHOD_SUBSYSTEM;
		}

		if (data["solver"] == "AB3")
		{
			buf.solver = INTEGRATOR_AB3;
		}
		else if (data["solver"] == "RK4")
		{
			buf.solver = INTEGRATOR_RK4;
		}

		// gravity, rotational, and translational axes
		for (int i = 0; i < 3; i++)
		{
			m_dynamics->gravityAxis(i) = data["gravity_axis"][i].asDouble();
			m_dynamics->rotAxis(i) = data["rotational_axis"][i].asDouble();
			m_dynamics->transAxis(i) = data["translational_axis"][i].asDouble();
		}
		m_dynamics->gravityAxis.normalize();
		m_dynamics->rotAxis.normalize();
		m_dynamics->transAxis.normalize();
	}
	else
	{
		std::cout << "SimulationParameter jsonRead error!!" << std::endl;
	}

	return buf;
}

Eigen::VectorXd Inputs::readInputData(std::string json_dir, MBD_RecursiveII* m_dynamics)
{
	// base body
	BaseBodyData buf_BaseBody = jsonRead_BaseBody(json_dir);

	// subsystem
	int sub_id = 1;
	int body_id = 1;
	int cnt = 0;
	SubsystemData buf;
	std::map<int, std::vector<SubsystemData>> buf_Subsystem;
	while (1)
	{
		buf = jsonRead_Subsystem(json_dir, sub_id, body_id);

		if (buf.ReadOK)
		{
			buf_Subsystem[sub_id].push_back(buf);
			if (buf.flag_motion)
			{
				cnt++;
			}
			body_id++;
		}
		else
		{
			if (body_id == 1)
			{
				break;
			}
			else
			{
				nJointMotion[sub_id] = cnt;
				cnt = 0;
				body_id = 1;
				sub_id++;
			}
		}
	}

	// state vector
	Eigen::VectorXd Y;

	Y.resize(13);
	Y.segment(0, 3) = buf_BaseBody.r0;
	Y.segment(3, 4) = buf_BaseBody.p0;
	Y.segment(7, 3) = buf_BaseBody.dr0;
	Y.segment(10, 3) = buf_BaseBody.w0;

	int idx = 13;
	nSubsystem = buf_Subsystem.size();
	for (int sub = 1; sub <= nSubsystem; sub++)
	{
		nSubBody[sub] = buf_Subsystem[sub].size();
		Y.conservativeResize(idx + 2 * (nSubBody[sub] - nJointMotion[sub]));
		cnt = 0;
		for (int i = 0; i < nSubBody[sub]; i++)
		{
			if (!buf_Subsystem[sub][i].flag_motion)
			{
				Y(idx + cnt) = buf_Subsystem[sub][i].qj;
				Y(idx + cnt + nSubBody[sub] - nJointMotion[sub]) = buf_Subsystem[sub][i].dqj;
				cnt++;
			}
		}
		idx = idx + 2 * (nSubBody[sub] - nJointMotion[sub]);
	}

	// set initial & constant values
	m_dynamics->setData_BaseBody(buf_BaseBody);
	m_dynamics->setData_Subsystem(buf_Subsystem);

	return Y;
}

BaseBodyData Inputs::jsonRead_BaseBody(std::string json_dir)
{
	std::string file_name = json_dir + "BaseBody.json";
	std::ifstream json_file(file_name, std::ifstream::binary);
	Json::CharReaderBuilder builder;
	builder["collectComments"] = false;
	Json::Value data;
	JSONCPP_STRING errs;
	bool ok = parseFromStream(builder, json_file, &data, &errs);

	BaseBodyData buf;
	buf.ReadOK = ok;
	if (ok)
	{
		// check contact & motion
		buf.flag_contact = data["flag contact"].asBool();
		buf.flag_motion = data["flag motion"].asBool();

		// initial position
		for (int i = 0; i < 4; i++)
		{
			buf.p0(i) = data["p0"][i].asDouble();
			if (i < 3)
			{
				buf.r0(i) = data["r0"][i].asDouble();
			}
		}

		// initial velocity
		for (int i = 0; i < 3; i++)
		{
			buf.w0(i) = data["w0"][i].asDouble();
			buf.dr0(i) = data["dr0"][i].asDouble();
		}

		// mass & inertia
		buf.m0 = data["m0"].asDouble();
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				buf.J0cp(i, j) = data["J0cp"][i][j].asDouble();
			}
		}

		// constant data
		for (int i = 0; i < 3; i++)
		{
			buf.rho0p(i) = data["rho0p"][i].asDouble();
			for (int j = 0; j < 3; j++)
			{
				buf.C00(i, j) = data["C00"][i][j].asDouble();
			}
		}

		buf.name = data["name"].asString();
	}
	else
	{
		std::cout << "BaseBody jsonRead error!!" << std::endl;
	}

	return buf;
}

SubsystemData Inputs::jsonRead_Subsystem(std::string json_dir, int id_subsystem, int id_body)
{
	std::string file_name = json_dir + "Subsystem" + std::to_string(id_subsystem) + std::to_string(id_body) + ".json";
	std::ifstream json_file(file_name, std::ifstream::binary);
	Json::CharReaderBuilder builder;
	builder["collectComments"] = false;
	Json::Value data;
	JSONCPP_STRING errs;
	bool ok = parseFromStream(builder, json_file, &data, &errs);
	
	SubsystemData buf;
	buf.ReadOK = ok;
	if (ok)
	{
		// subsystem id
		buf.subsystem_id = data["subsystem id"].asInt();

		// parent id
		buf.parent_id = data["parent id"].asInt();

		// child id
		buf.child_id = data["child id"].asInt();

		// check contact & motion
		buf.flag_contact = data["flag contact"].asBool();
		buf.flag_motion = data["flag motion"].asBool();

		// joint type
		if (data["joint id"] == "revolute")
		{
			buf.jointType = JOINT_REVOLUTE;
		}
		else if (data["joint id"] == "translational")
		{
			buf.jointType = JOINT_TRNSLATIONAL;
		}

		// initial joint position
		buf.qj = data["qj"].asDouble();

		// initial joint velocity
		buf.dqj = data["dqj"].asDouble();

		// mass & inertia
		buf.mj = data["mj"].asDouble();
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				buf.Jjcp(i, j) = data["Jjcp"][i][j].asDouble();
			}
		}

		// constant data
		if (data["sijp"].isNull())
		{
			buf.sijp.setZero();
		}
		else
		{
			for (int i = 0; i < 3; i++)
			{
				buf.sijp(i) = data["sijp"][i].asDouble();
			}
		}

		if (data["sjcp"].isNull())
		{
			buf.sjcp.setZero();
		}
		else
		{
			for (int i = 0; i < 3; i++)
			{
				buf.sjcp(i) = data["sjcp"][i].asDouble();
			}
		}

		if (data["rhojp"].isNull())
		{
			buf.rhojp.setZero();
		}
		else
		{
			for (int i = 0; i < 3; i++)
			{
				buf.rhojp(i) = data["rhojp"][i].asDouble();
			}
		}

		if (data["Cij"].isNull())
		{
			buf.Cij.setZero();
		}
		else
		{
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					buf.Cij(i, j) = data["Cij"][i][j].asDouble();
				}

			}
		}

		if (data["Cjj"].isNull())
		{
			buf.Cjj.setZero();
		}
		else
		{
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					buf.Cjj(i, j) = data["Cjj"][i][j].asDouble();
				}

			}
		}

		// RSDA data
		if (data["qj_ini_RSDA"].isNull())
		{
			buf.qj_ini_RSDA = 0.0;
		}
		else
		{
			buf.qj_ini_RSDA = data["qj_ini_RSDA"].asDouble();
		}

		if (data["k_RSDA"].isNull())
		{
			buf.k_RSDA = 0.0;
		}
		else
		{
			buf.k_RSDA = data["k_RSDA"].asDouble();
		}

		if (data["c_RSDA"].isNull())
		{
			buf.c_RSDA = 0.0;
		}
		else
		{
			buf.c_RSDA = data["c_RSDA"].asDouble();
		}

		// contact data
		if (data["pen_z_ref"].isNull())
		{
			buf.pen_z_ref = 0.0;
		}
		else
		{
			buf.pen_z_ref = data["pen_z_ref"].asDouble();
		}

		if (data["k_contact"].isNull())
		{
			buf.k_contact = 0.0;
		}
		else
		{
			buf.k_contact = data["k_contact"].asDouble();
		}

		if (data["c_contact"].isNull())
		{
			buf.c_contact = 0.0;
		}
		else
		{
			buf.c_contact = data["c_contact"].asDouble();
		}
	}
	else
	{
		std::cout << "Subsystem" << std::to_string(id_subsystem) << std::to_string(id_body) << " jsonRead error!!" << std::endl;
	}

	return buf;
}