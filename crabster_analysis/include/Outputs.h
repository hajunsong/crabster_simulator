#pragma once

#include "Common.h"
#include "MBD_RecursiveII.h"

class Outputs
{
public:
	void storeOutputData(double time_step, OutputData outData);
	void csvWrite(std::string folder_path);

private:
	std::vector<OutputData> m_outData;
};