#include "Outputs.h"

void Outputs::storeOutputData(double time_step, OutputData outData)
{
	outData.t_current = time_step;
	m_outData.push_back(outData);
}

void Outputs::csvWrite(std::string folder_path)
{
	std::ofstream os_BaseBody;
	std::ofstream os_Subsystem;
	std::string fileName_BaseBody;
	std::string fileName_Subsystem;
	
	std::cout << "\nFile write start!\n" << std::endl;

	// base body file open
	if (analysis_method == METHOD_CONVENTIONAL)
	{
		fileName_BaseBody = "SimResult_ConvMethod_BaseBody.csv";
	}
	else if (analysis_method == METHOD_SUBSYSTEM)
	{
		fileName_BaseBody = "SimResult_SubMethod_BaseBody.csv";
	}

	os_BaseBody.open(folder_path + fileName_BaseBody, std::ios_base::trunc);
	//os_BaseBody.width(3);
	//os_BaseBody.precision(10);
	//os_BaseBody.setf(std::ios_base::fixed, std::ios_base::floatfield);

	// header
	os_BaseBody << "time" << ",";
	os_BaseBody << "r0c(x)" << "," << "r0c(y)" << "," << "r0c(z)" << ","
		<< "p0(e0)" << "," << "p0(e1)" << "," "p0(e2)" << "," "p0(e3)" << ","
		<< "dr0c(x)" << "," << "dr0c(y)" << "," << "dr0c(z)" << ","
		<< "w0(x)" << "," << "w0(y)" << "," << "w0(z)" << ","
		<< "ddr0c(x)" << "," << "ddr0c(y)" << "," << "ddr0c(z)" << ","
		<< "dw0(x)" << "," << "dw0(y)" << "," << "dw0(z)" << ",";
	os_BaseBody << std::endl;

	// data
	for (int num = 0; num < m_outData.size(); num++)
	{
		os_BaseBody << m_outData[num].t_current << ",";
		for (int i = 0; i < 3; i++)
		{
			os_BaseBody << m_outData[num].BaseBody.r0c(i) << ",";
		}
		for (int i = 0; i < 4; i++) 
		{
			os_BaseBody << m_outData[num].BaseBody.p0(i) << ","; 
		}
		for (int i = 0; i < 3; i++) 
		{ 
			os_BaseBody << m_outData[num].BaseBody.dr0c(i) << ","; 
		}
		for (int i = 0; i < 3; i++)
		{ 
			os_BaseBody << m_outData[num].BaseBody.w0(i) << ","; 
		}
		for (int i = 0; i < 3; i++)
		{ 
			os_BaseBody << m_outData[num].BaseBody.ddr0c(i) << ",";
		}
		for (int i = 0; i < 3; i++) 
		{ 
			os_BaseBody << m_outData[num].BaseBody.dw0(i) << ",";
		}
		os_BaseBody << std::endl;
	}

	// base body file close
	os_BaseBody.close();
	std::cout << "Base body file write finish!" << std::endl;

	for (int sub = 1; sub <= nSubsystem; sub++)
	{
		// subsystem file open
		if (analysis_method == METHOD_CONVENTIONAL)
		{
			fileName_Subsystem = "SimResult_ConvMethod_Subsystem_";
		}
		else if (analysis_method == METHOD_SUBSYSTEM)
		{
			fileName_Subsystem = "SimResult_SubMethod_Subsystem_";
		}
		fileName_Subsystem = fileName_Subsystem + std::to_string(sub) + ".csv";

		os_Subsystem.open(folder_path + fileName_Subsystem, std::ios_base::trunc);
		//os_Subsystem.width(3);
		//os_Subsystem.precision(10);
		//os_Subsystem.setf(std::ios_base::fixed, std::ios_base::floatfield);

		// header
		os_Subsystem << "time" << ",";
		for (int i = 0; i < nSubBody[sub]; i++) 
		{ 
			os_Subsystem << "q" << std::to_string(i + 1) << ",";
		}
		for (int i = 0; i < nSubBody[sub]; i++) 
		{ 
			os_Subsystem << "dq" << std::to_string(i + 1) << ",";
		}
		for (int i = 0; i < nSubBody[sub]; i++) 
		{ 
			os_Subsystem << "ddq" << std::to_string(i + 1) << ",";
		}
		for (int i = 0; i < nSubBody[sub]; i++)
		{
			os_Subsystem << "r" << std::to_string(i + 1) << "c(x)" << ","
				<< "r" << std::to_string(i + 1) << "c(y)" << ","
				<< "r" << std::to_string(i + 1) << "c(z)" << ",";
		}
		for (int i = 0; i < nSubBody[sub]; i++)
		{
			os_Subsystem << "p" << std::to_string(i + 1) << "(e0)" << ","
				<< "p" << std::to_string(i + 1) << "(e1)" << ","
				<< "p" << std::to_string(i + 1) << "(e2)" << ","
				<< "p" << std::to_string(i + 1) << "(e3)" << ",";
		}
		for (int i = 0; i < nSubBody[sub]; i++)
		{
			os_Subsystem << "dr" << std::to_string(i + 1) << "c(x)" << ","
				<< "dr" << std::to_string(i + 1) << "c(y)" << ","
				<< "dr" << std::to_string(i + 1) << "c(z)" << ",";
		}
		for (int i = 0; i < nSubBody[sub]; i++)
		{
			os_Subsystem << "w" << std::to_string(i + 1) << "(x)" << ","
				<< "w" << std::to_string(i + 1) << "(y)" << ","
				<< "w" << std::to_string(i + 1) << "(z)" << ",";
		}
		for (int i = 0; i < nSubBody[sub]; i++)
		{
			os_Subsystem << "ddr" << std::to_string(i + 1) << "c(x)" << ","
				<< "ddr" << std::to_string(i + 1) << "c(y)" << ","
				<< "ddr" << std::to_string(i + 1) << "c(z)" << ",";
		}
		for (int i = 0; i < nSubBody[sub]; i++)
		{
			os_Subsystem << "dw" << std::to_string(i + 1) << "(x)" << ","
				<< "dw" << std::to_string(i + 1) << "(y)" << ","
				<< "dw" << std::to_string(i + 1) << "(z)" << ",";
		}
		os_Subsystem << std::endl;

		// data
		for (int num = 0; num < m_outData.size(); num++)
		{
			os_Subsystem << m_outData[num].t_current << ",";
			for (int i = 0; i < nSubBody[sub]; i++)
			{
				os_Subsystem << m_outData[num].Subsystem.qj[sub](i) << ",";
			}
			for (int i = 0; i < nSubBody[sub]; i++)
			{
				os_Subsystem << m_outData[num].Subsystem.dqj[sub](i) << ",";
			}
			for (int i = 0; i < nSubBody[sub]; i++)
			{
				os_Subsystem << m_outData[num].Subsystem.ddqj[sub](i) << ",";
			}
			for (int i = 0; i < nSubBody[sub]; i++)
			{
				os_Subsystem << m_outData[num].Subsystem.rjc[sub][i](0) << ","
					<< m_outData[num].Subsystem.rjc[sub][i](1) << ","
					<< m_outData[num].Subsystem.rjc[sub][i](2) << ",";
			}
			for (int i = 0; i < nSubBody[sub]; i++)
			{
				os_Subsystem << m_outData[num].Subsystem.pj[sub][i](0) << ","
					<< m_outData[num].Subsystem.pj[sub][i](1) << ","
					<< m_outData[num].Subsystem.pj[sub][i](2) << ","
					<< m_outData[num].Subsystem.pj[sub][i](3) << ",";
			}
			for (int i = 0; i < nSubBody[sub]; i++)
			{
				os_Subsystem << m_outData[num].Subsystem.drjc[sub][i](0) << ","
					<< m_outData[num].Subsystem.drjc[sub][i](1) << ","
					<< m_outData[num].Subsystem.drjc[sub][i](2) << ",";
			}
			for (int i = 0; i < nSubBody[sub]; i++)
			{
				os_Subsystem << m_outData[num].Subsystem.wj[sub][i](0) << ","
					<< m_outData[num].Subsystem.wj[sub][i](1) << ","
					<< m_outData[num].Subsystem.wj[sub][i](2) << ",";
			}
			for (int i = 0; i < nSubBody[sub]; i++)
			{
				os_Subsystem << m_outData[num].Subsystem.ddrjc[sub][i](0) << ","
					<< m_outData[num].Subsystem.ddrjc[sub][i](1) << ","
					<< m_outData[num].Subsystem.ddrjc[sub][i](2) << ",";
			}
			for (int i = 0; i < nSubBody[sub]; i++)
			{
				os_Subsystem << m_outData[num].Subsystem.dwj[sub][i](0) << ","
					<< m_outData[num].Subsystem.dwj[sub][i](1) << ","
					<< m_outData[num].Subsystem.dwj[sub][i](2) << ",";
			}
			os_Subsystem << std::endl;
		}

		// subsystem file close
		os_Subsystem.close();
		std::cout << "Subsystem[" << std::to_string(sub) << "] file write finish!" << std::endl;
	}

	m_outData.clear();

	std::cout << "\nFile write finish!\n" << std::endl;
}