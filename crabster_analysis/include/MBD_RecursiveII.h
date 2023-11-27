#pragma once

#include "Common.h"
#include "Applied_Force.h"
#include "Joint_Motrion.h"

class MBD_RecursiveII
{
public:
	MBD_RecursiveII();
	~MBD_RecursiveII();

	Eigen::Vector3d gravityAxis;
	Eigen::Vector3d rotAxis;
	Eigen::Vector3d transAxis;
	
	void setData_BaseBody(BaseBodyData buf);
	void setMatixVectorSize_BaseBody();
	void setData_Subsystem(std::map<int, std::vector<SubsystemData>> buf_map);
	void setMatixVectorSize_Subsystem();

	Eigen::VectorXd dynamics_analysis(double t_current, Eigen::VectorXd Y);

	OutputData getOutputData();

	std::map<int, std::vector<Eigen::Vector3d>> rjf;

private:
	Applied_Force* m_force;
	Joint_Motrion* m_motion;

	// joint type
	std::map<int, Eigen::VectorXi> jointType;

	// parent & child list
	std::map<int, Eigen::MatrixXi> pcList;

	// system loop list
	std::map<int, Eigen::MatrixXi> loopList;
	std::map<int, Eigen::VectorXi> subBodySeq;

	// check contact & motion
	std::map<int, ArrayXb> flag_contact;
	std::map<int, ArrayXb> flag_motion;

	// constant value
	std::map<int, std::vector<Eigen::Vector3d>> sijp;
	std::map<int, std::vector<Eigen::Vector3d>> sjcp;
	std::map<int, std::vector<Eigen::Matrix3d>> Cij;
	std::map<int, std::vector<Eigen::Matrix3d>> Cjj;
	std::map<int, std::vector<Eigen::Vector3d>> rhojp;
	std::map<int, Eigen::VectorXd> mj;
	std::map<int, std::vector<Eigen::Matrix3d>> Jjcp;
	std::map<int, Eigen::VectorXd> qj_ini_RSDA;
	std::map<int, Eigen::VectorXd> k_RSDA;
	std::map<int, Eigen::VectorXd> c_RSDA;
	std::map<int, Eigen::VectorXd> pen_z_ref;
	std::map<int, Eigen::VectorXd> k_contact;
	std::map<int, Eigen::VectorXd> c_contact;

	// value
	Eigen::MatrixXd E0;
	Eigen::MatrixXd G0;
	Eigen::Vector4d dp0;
	std::map<int, Eigen::VectorXd> qj;
	std::map<int, Eigen::VectorXd> dqj;
	std::map<int, Eigen::VectorXd> ddqj;
	std::map<int, std::vector<Eigen::Vector4d>> pj;
	std::map<int, std::vector<Eigen::Matrix3d>> Aj;
	std::map<int, std::vector<Eigen::Vector3d>> sij;
	std::map<int, std::vector<Eigen::Vector3d>> dij;
	std::map<int, std::vector<Eigen::Vector3d>> rhoj;
	std::map<int, std::vector<Eigen::Vector3d>> rj;
	std::map<int, std::vector<Eigen::Vector3d>> rjc;
	// std::map<int, std::vector<Eigen::Vector3d>> rjf;
	std::map<int, std::vector<Eigen::Vector3d>> drj;
	std::map<int, std::vector<Eigen::Vector3d>> drjc;
	std::map<int, std::vector<Eigen::Vector3d>> drjf;
	std::map<int, std::vector<Eigen::Vector3d>> Hj;
	std::map<int, std::vector<Eigen::Vector3d>> dHj;
	std::map<int, std::vector<Eigen::Vector3d>> wj;
	std::map<int, std::vector<Eigen::VectorXd>> Bj;
	std::map<int, std::vector<Eigen::VectorXd>> Dj;
	std::map<int, std::vector<Eigen::Vector3d>> ddrj;
	std::map<int, std::vector<Eigen::Vector3d>> ddrjc;
	std::map<int, std::vector<Eigen::Vector3d>> dwj;
	std::map<int, std::vector<Eigen::Matrix3d>> Jjc;
	std::map<int, std::vector<Eigen::MatrixXd>> Mj_hat;
	std::map<int, std::vector<Eigen::VectorXd>> Qj_hat;
	std::map<int, std::vector<Eigen::VectorXd>> Qj_RSDA_hat;
	std::map<int, std::vector<Eigen::MatrixXd>> Kj;
	std::map<int, std::vector<Eigen::VectorXd>> Lj;
	std::map<int, Eigen::MatrixXd> Myy;
	std::map<int, Eigen::MatrixXd> Myq;
	std::map<int, Eigen::MatrixXd> Mqq;
	std::map<int, Eigen::VectorXd> Py;
	std::map<int, Eigen::VectorXd> Pq;
	std::map<int, Eigen::MatrixXd> Me;
	std::map<int, Eigen::VectorXd> Pe;
	std::map<int, std::vector<Eigen::VectorXd>> dYj_hat;
	std::map<int, std::vector<Eigen::VectorXd>> Rji;

	Eigen::MatrixXi removeRowi(Eigen::MatrixXi ref, ArrayXb rowToRemove);
	Eigen::MatrixXd removeRowd(Eigen::MatrixXd ref, ArrayXb rowToRemove);
	Eigen::MatrixXi removeColi(Eigen::MatrixXi ref, ArrayXb colToRemove);
	Eigen::MatrixXd removeCold(Eigen::MatrixXd ref, ArrayXb colToRemove);
	void searchLoop();

	Eigen::Matrix3d skew(Eigen::Vector3d x);
	Eigen::Matrix3d generalRotation(Eigen::Vector3d axis, double rad);
	Eigen::Vector4d TransformationMatrixToEulerParameter(Eigen::Matrix3d R);

	void Y2PosVel(double t_current, Eigen::VectorXd Y);

	void position_BaseBody();
	void velocity_BaseBody();
	void acceleration_BaseBody();
	void massforce_BaseBody();
	
	void position_Subsystem(int sub);
	void velocity_Subsystem(int sub);
	void acceleration_Subsystem(int sub);
	void massforce_Subsystem(int sub);
	void effectiveTerm_Subsystem(int sub);

	void EQM_WholeBody();
	void EQM_BaseBody();
	void EQM_Subsystem(int sub);

	void jointReactionForce(int sub);

	Eigen::VectorXd VelAcc2dY();
};