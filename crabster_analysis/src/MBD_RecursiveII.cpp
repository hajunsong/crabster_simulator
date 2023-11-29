#include "MBD_RecursiveII.h"

MBD_RecursiveII::MBD_RecursiveII()
{
	m_force = new Applied_Force();
	m_motion = new Joint_Motrion();
}

MBD_RecursiveII::~MBD_RecursiveII()
{
	delete(m_force);
	delete(m_motion);
}

void MBD_RecursiveII::setData_BaseBody(BaseBodyData buf)
{
	flag_contact[0].resize(1);
	flag_motion[0].resize(1);
	mj[0].resize(1);

	// check contact
	flag_contact[0](0) = buf.flag_contact;

	// check motion
	flag_motion[0](0) = buf.flag_motion;

	// base body orientation
	E0.resize(3, 4);
	G0.resize(3, 4);

	// C00
	Cjj[0].push_back(buf.C00);

	// rho0p
	rhojp[0].push_back(buf.rho0p);

	// m0
	mj[0](0) = buf.m0;

	// J0cp
	Jjcp[0].push_back(buf.J0cp);

	setMatixVectorSize_BaseBody();
}

void MBD_RecursiveII::setMatixVectorSize_BaseBody()
{
	pj[0].assign(1, Eigen::Vector4d::Zero());
	Aj[0].assign(1, Eigen::Matrix3d::Zero());
	rhoj[0].assign(1, Eigen::Vector3d::Zero());
	rj[0].assign(1, Eigen::Vector3d::Zero());
	rjc[0].assign(1, Eigen::Vector3d::Zero());

	wj[0].assign(1, Eigen::Vector3d::Zero());
	drj[0].assign(1, Eigen::Vector3d::Zero());
	drjc[0].assign(1, Eigen::Vector3d::Zero());

	ddrj[0].assign(1, Eigen::Vector3d::Zero());
	ddrjc[0].assign(1, Eigen::Vector3d::Zero());
	dwj[0].assign(1, Eigen::Vector3d::Zero());

	Jjc[0].assign(1, Eigen::Matrix3d::Zero());
	Mj_hat[0].assign(1, Eigen::MatrixXd::Zero(6, 6));
	Qj_hat[0].assign(1, Eigen::VectorXd::Zero(6));
	Qj_RSDA_hat[0].assign(1, Eigen::VectorXd::Zero(6));

	dYj_hat[0].assign(1, Eigen::VectorXd::Zero(6));
}

void MBD_RecursiveII::setData_Subsystem(std::map<int, std::vector<SubsystemData>> buf_map)
{
	for (int sub = 1; sub <= nSubsystem; sub++)
	{
		jointType[sub].resize(nSubBody[sub]);
		pcList[sub].resize(nSubBody[sub], 2);
		subBodySeq[sub].resize(nSubBody[sub]);
		flag_contact[sub].resize(nSubBody[sub]);
		flag_motion[sub].resize(nSubBody[sub]);
		mj[sub].resize(nSubBody[sub]);
		qj_ini_RSDA[sub].resize(nSubBody[sub]);
		k_RSDA[sub].resize(nSubBody[sub]);
		c_RSDA[sub].resize(nSubBody[sub]);
		pen_z_ref[sub].resize(nSubBody[sub]);
		k_contact[sub].resize(nSubBody[sub]);
		c_contact[sub].resize(nSubBody[sub]);

		for (int i = 0; i < nSubBody[sub]; i++)
		{
			// joint type
			jointType[sub](i) = buf_map[sub][i].jointType;

			// parent & child list
			pcList[sub](i, 0) = buf_map[sub][i].parent_id;
			pcList[sub](i, 1) = buf_map[sub][i].child_id;

			// check contact
			flag_contact[sub](i) = buf_map[sub][i].flag_contact;

			// check motion
			flag_motion[sub](i) = buf_map[sub][i].flag_motion;

			// Cij
			Cij[sub].push_back(buf_map[sub][i].Cij);

			// Cjj
			Cjj[sub].push_back(buf_map[sub][i].Cjj);

			// sijp
			sijp[sub].push_back(buf_map[sub][i].sijp);

			// sjcp
			sjcp[sub].push_back(buf_map[sub][i].sjcp);

			// rhojp
			rhojp[sub].push_back(buf_map[sub][i].rhojp);

			// mj
			mj[sub](i) = buf_map[sub][i].mj;

			// Jjcp
			Jjcp[sub].push_back(buf_map[sub][i].Jjcp);

			// qj_ini_RSDA
			qj_ini_RSDA[sub](i) = buf_map[sub][i].qj_ini_RSDA;

			// k_RSDA
			k_RSDA[sub](i) = buf_map[sub][i].k_RSDA;

			// c_RSDA
			c_RSDA[sub](i) = buf_map[sub][i].c_RSDA;

			// pen_z_ref
			pen_z_ref[sub](i) = buf_map[sub][i].pen_z_ref;

			// k_contact
			k_contact[sub](i) = buf_map[sub][i].k_contact;

			// c_contact
			c_contact[sub](i) = buf_map[sub][i].c_contact;
		}
	}

	searchLoop();

	setMatixVectorSize_Subsystem();

	std::cout << std::endl;
	for (int sub = 1; sub <= subBodySeq.size(); sub++)
	{
		std::cout << "Subsystem[" << std::to_string(sub) << "] body sequence = " << subBodySeq[sub].transpose() << std::endl;
	}

	std::cout << std::endl;
	for (int sub = 1; sub <= subBodySeq.size(); sub++)
	{
		std::cout << "Subsystem[" << std::to_string(sub) << "] check contact = ";
		for (int i = 0; i < nSubBody[sub]; i++)
		{
			std::cout << "body" << std::to_string(i + 1) << "(" << flag_contact[sub](i) << ")  ";
		}
		std::cout << std::endl;
	}

	std::cout << std::endl;
	for (int sub = 1; sub <= subBodySeq.size(); sub++)
	{
		std::cout << "Subsystem[" << std::to_string(sub) << "] check motion = ";
		for (int i = 0; i < nSubBody[sub]; i++)
		{
			std::cout << "body" << std::to_string(i + 1) << "(" << flag_motion[sub](i) << ")  ";
		}
		std::cout << std::endl;
	}
}

void MBD_RecursiveII::setMatixVectorSize_Subsystem()
{
	for (int sub = 1; sub <= nSubsystem; sub++)
	{
		qj[sub] = Eigen::VectorXd::Zero(nSubBody[sub]);
		dqj[sub] = Eigen::VectorXd::Zero(nSubBody[sub]);
		ddqj[sub] = Eigen::VectorXd::Zero(nSubBody[sub]);

		pj[sub].assign(nSubBody[sub], Eigen::Vector4d::Zero());
		Aj[sub].assign(nSubBody[sub], Eigen::Matrix3d::Zero());
		sij[sub].assign(nSubBody[sub], Eigen::Vector3d::Zero());
		dij[sub].assign(nSubBody[sub], Eigen::Vector3d::Zero());
		rhoj[sub].assign(nSubBody[sub], Eigen::Vector3d::Zero());
		rj[sub].assign(nSubBody[sub], Eigen::Vector3d::Zero());
		rjc[sub].assign(nSubBody[sub], Eigen::Vector3d::Zero());
		rjf[sub].assign(nSubBody[sub], Eigen::Vector3d::Zero());

		drj[sub].assign(nSubBody[sub], Eigen::Vector3d::Zero());
		drjc[sub].assign(nSubBody[sub], Eigen::Vector3d::Zero());
		drjf[sub].assign(nSubBody[sub], Eigen::Vector3d::Zero());
		Hj[sub].assign(nSubBody[sub], Eigen::Vector3d::Zero());
		dHj[sub].assign(nSubBody[sub], Eigen::Vector3d::Zero());
		wj[sub].assign(nSubBody[sub], Eigen::Vector3d::Zero());
		Bj[sub].assign(nSubBody[sub], Eigen::VectorXd::Zero(6));
		Dj[sub].assign(nSubBody[sub], Eigen::VectorXd::Zero(6));

		ddrj[sub].assign(nSubBody[sub], Eigen::Vector3d::Zero());
		ddrjc[sub].assign(nSubBody[sub], Eigen::Vector3d::Zero());
		dwj[sub].assign(nSubBody[sub], Eigen::Vector3d::Zero());

		Jjc[sub].assign(nSubBody[sub], Eigen::Matrix3d::Zero());
		Mj_hat[sub].assign(nSubBody[sub], Eigen::MatrixXd::Zero(6, 6));
		Qj_hat[sub].assign(nSubBody[sub], Eigen::VectorXd::Zero(6));
		Qj_RSDA_hat[sub].assign(nSubBody[sub], Eigen::VectorXd::Zero(6));
		Kj[sub].assign(nSubBody[sub], Eigen::MatrixXd::Zero(6, 6));
		Lj[sub].assign(nSubBody[sub], Eigen::VectorXd::Zero(6));

		Myy[sub] = Eigen::MatrixXd::Zero(6, 6);
		Myq[sub] = Eigen::MatrixXd::Zero(6, nSubBody[sub]);
		Mqq[sub] = Eigen::MatrixXd::Zero(nSubBody[sub], nSubBody[sub]);
		Py[sub] = Eigen::VectorXd::Zero(6);
		Pq[sub] = Eigen::VectorXd::Zero(nSubBody[sub]);

		Me[sub] = Eigen::MatrixXd::Zero(6, 6);
		Pe[sub] = Eigen::VectorXd::Zero(6);

		dYj_hat[sub].assign(nSubBody[sub], Eigen::VectorXd::Zero(6));
		Rji[sub].assign(nSubBody[sub], Eigen::VectorXd::Zero(6));
	}
}

Eigen::MatrixXi MBD_RecursiveII::removeRowi(Eigen::MatrixXi ref, ArrayXb rowToRemove)
{
	Eigen::MatrixXi buf(0, ref.cols());

	for (int i = 0; i < ref.rows(); i++)
	{
		if (!rowToRemove(i))
		{
			buf.conservativeResize(buf.rows() + 1, Eigen::NoChange);
			buf.row(buf.rows() - 1) = ref.row(i);
		}
	}

	return buf;
}

Eigen::MatrixXd MBD_RecursiveII::removeRowd(Eigen::MatrixXd ref, ArrayXb rowToRemove)
{
	Eigen::MatrixXd buf(0, ref.cols());

	for (int i = 0; i < ref.rows(); i++)
	{
		if (!rowToRemove(i))
		{
			buf.conservativeResize(buf.rows() + 1, Eigen::NoChange);
			buf.row(buf.rows() - 1) = ref.row(i);
		}
	}

	return buf;
}

Eigen::MatrixXi MBD_RecursiveII::removeColi(Eigen::MatrixXi ref, ArrayXb colToRemove)
{
	Eigen::MatrixXi buf(ref.rows(), 0);

	for (int j = 0; j < ref.cols(); j++)
	{
		if (!colToRemove(j))
		{
			buf.conservativeResize(Eigen::NoChange, buf.cols() + 1);
			buf.col(buf.cols() - 1) = ref.col(j);
		}
	}

	return buf;
}

Eigen::MatrixXd MBD_RecursiveII::removeCold(Eigen::MatrixXd ref, ArrayXb colToRemove)
{
	Eigen::MatrixXd buf(ref.rows(), 0);

	for (int j = 0; j < ref.cols(); j++)
	{
		if (!colToRemove(j))
		{
			buf.conservativeResize(Eigen::NoChange, buf.cols() + 1);
			buf.col(buf.cols() - 1) = ref.col(j);
		}
	}

	return buf;
}

void MBD_RecursiveII::searchLoop()
{
	Eigen::MatrixXi pcList_buf(0, 0);
	Eigen::MatrixXi loopList_buf(0, 0);
	ArrayXb rowToRemove(0);
	int cnt = 0;

	for (int sub = 1; sub <= nSubsystem; sub++)
	{
		// step 1 : search base body loop
		pcList_buf = pcList[sub];
		rowToRemove = ArrayXb::Constant(pcList_buf.rows(), false);
		for (int i = 0; i < pcList_buf.rows(); i++)
		{
			if (pcList_buf(i, 0) == 0)
			{
				loopList_buf.conservativeResize(loopList_buf.rows() + 1, 2);
				loopList_buf(loopList_buf.rows() - 1, 0) = 0;
				loopList_buf(loopList_buf.rows() - 1, 1) = pcList_buf(i, 1);
				rowToRemove(i) = true;
			}
		}
		pcList_buf = removeRowi(pcList_buf, rowToRemove);
		loopList[sub] = loopList_buf;

		// step 2 iterate loop(search each loop)
		while (pcList_buf.rows() != 0)
		{
			loopList_buf.resize(0, loopList[sub].cols() + 1);
			for (int i = 0; i < loopList[sub].rows(); i++)
			{
				cnt = 0;

				if (pcList_buf.rows() == 0)
				{
					loopList_buf.conservativeResize(loopList_buf.rows() + 1, loopList_buf.cols());
					loopList_buf.block(loopList_buf.rows() - 1, 0, 1, loopList[sub].cols()) = loopList[sub].row(i);
					loopList_buf(loopList_buf.rows() - 1, loopList_buf.cols() - 1) = -1;
				}
				else
				{
					rowToRemove = ArrayXb::Constant(pcList_buf.rows(), false);
					for (int j = 0; j < pcList_buf.rows(); j++)
					{
						if (loopList[sub](i, loopList[sub].cols() - 1) == pcList_buf(j, 0))
						{
							loopList_buf.conservativeResize(loopList_buf.rows() + 1, loopList_buf.cols());
							loopList_buf.block(loopList_buf.rows() - 1, 0, 1, loopList[sub].cols()) = loopList[sub].row(i);
							loopList_buf(loopList_buf.rows() - 1, loopList_buf.cols() - 1) = pcList_buf(j, 1);
							rowToRemove(i) = true;
							cnt++;
						}

						if ((j == pcList_buf.rows() - 1) && (cnt == 0))
						{
							loopList_buf.conservativeResize(loopList_buf.rows() + 1, loopList_buf.cols());
							loopList_buf.block(loopList_buf.rows() - 1, 0, 1, loopList[sub].cols()) = loopList[sub].row(i);
							loopList_buf(loopList_buf.rows() - 1, loopList_buf.cols() - 1) = -1;
						}
					}
				}
				pcList_buf = removeRowi(pcList_buf, rowToRemove);
			}
			loopList[sub] = loopList_buf;
		}
		
		// subsystem body sequence
		cnt = 0;
		for (int j = 1; j < loopList[sub].cols(); j++)
		{
			if (loopList[sub](0, j) > 0)
			{
				subBodySeq[sub](cnt) = loopList[sub](0, j);
				cnt++;
			}

			for (int i = 1; i < loopList[sub].rows(); i++)
			{
				if ((loopList[sub](i, j) != loopList[sub](i - 1, j)) && (loopList[sub](i, j) > 0))
				{
					subBodySeq[sub](cnt) = loopList[sub](i, j);
					cnt++;
				}
			}
		}
	}
}

Eigen::Matrix3d MBD_RecursiveII::skew(Eigen::Vector3d x)
{
	Eigen::Matrix3d x_tilde;

	x_tilde(0, 0) = 0.0;	x_tilde(0, 1) = -x(2);	x_tilde(0, 2) = x(1);
	x_tilde(1, 0) = x(2);	x_tilde(1, 1) = 0.0;	x_tilde(1, 2) = -x(0);
	x_tilde(2, 0) = -x(1);	x_tilde(2, 1) = x(0);	x_tilde(2, 2) = 0.0;

	return x_tilde;
}

Eigen::Matrix3d MBD_RecursiveII::generalRotation(Eigen::Vector3d axis, double rad)
{
	Eigen::Matrix3d R;

	if (std::round(axis(0)) == 1)
	{
		// x-axis rotation
		R(0, 0) = 1.0;				R(0, 1) = 0.0;				R(0, 2) = 0.0;
		R(1, 0) = 0.0;				R(1, 1) = std::cos(rad);	R(1, 2) = -std::sin(rad);
		R(2, 0) = 0.0;				R(2, 1) = std::sin(rad);	R(2, 2) = std::cos(rad);
	}
	else if (std::round(axis(1)) == 1)
	{
		// y-axis rotation
		R(0, 0) = std::cos(rad);	R(0, 1) = 0.0;				R(0, 2) = std::sin(rad);
		R(1, 0) = 0.0;				R(1, 1) = 1.0;				R(1, 2) = 0.0;
		R(2, 0) = -std::sin(rad);	R(2, 1) = 0.0;				R(2, 2) = std::cos(rad);
	}
	else if (std::round(axis(2)) == 1)
	{
		// z-axis rotation
		R(0, 0) = std::cos(rad);	R(0, 1) = -std::sin(rad);	R(0, 2) = 0.0;
		R(1, 0) = std::sin(rad);	R(1, 1) = std::cos(rad);	R(1, 2) = 0.0;
		R(2, 0) = 0.0;				R(2, 1) = 0.0;				R(2, 2) = 1.0;
	}

	return R;
}

Eigen::Vector4d MBD_RecursiveII::TransformationMatrixToEulerParameter(Eigen::Matrix3d A)
{
	Eigen::Vector4d p;
	double trA = A.trace();
	double s = 0.0;

	if (trA > 0.0)
	{
		s = 0.5 / std::sqrt(trA + 1.0);
		p(0) = 0.25 / s;
		p(1) = (A(2, 1) - A(1, 2)) * s;
		p(2) = (A(0, 2) - A(2, 0)) * s;
		p(3) = (A(1, 0) - A(0, 1)) * s;
	}
	else
	{
		if ((A(0, 0) > A(1, 1)) && (A(0, 0) > A(2, 2)))
		{
			s = 2.0 * std::sqrt(1.0 + A(0, 0) - A(1, 1) - A(2, 2));
			p(0) = (A(2, 1) - A(1, 2)) / s;
			p(1) = 0.25 * s;
			p(2) = (A(0, 1) + A(1, 0)) / s;
			p(3) = (A(0, 2) + A(2, 0)) / s;
		}
		else if (A(1, 1) > A(2, 2))
		{
			s = 2.0 * std::sqrt(1.0 + A(1, 1) - A(0, 0) - A(2, 2));
			p(0) = (A(0, 2) - A(2, 0)) / s;
			p(1) = (A(0, 1) + A(1, 0)) / s;
			p(2) = 0.25 * s;
			p(3) = (A(1, 2) + A(2, 1)) / s;
		}
		else
		{
			s = 2.0 * std::sqrt(1.0 + A(2, 2) - A(0, 0) - A(1, 1));
			p(0) = (A(1, 0) - A(0, 1)) / s;
			p(1) = (A(0, 2) + A(2, 0)) / s;
			p(2) = (A(1, 2) + A(2, 1)) / s;
			p(3) = 0.25 * s;
		}
	}

	return p;
}

void MBD_RecursiveII::Y2PosVel(double t_current, Eigen::VectorXd Y)
{
	// r0
	rj[0][0] = Y.segment(0, 3);

	// p0
	pj[0][0] = Y.segment(3, 4);

	// dr0
	drj[0][0] = Y.segment(7, 3);

	// w0
	wj[0][0] = Y.segment(10, 3);

	int idx = 13;
	int cnt = 0;
	Eigen::Vector3d motionData;
	for (int sub = 1; sub <= nSubsystem; sub++)
	{
		cnt = 0;
		for (int i = 0; i < nSubBody[sub]; i++)
		{
			if (flag_motion[sub](i))
			{
				motionData = m_motion->inputMotion(t_current, sub, i);

				// qj
				qj[sub](i) = motionData(0);

				// dqj
				dqj[sub](i) = motionData(1);

				// ddqj
				ddqj[sub](i) = motionData(2);
			}
			else
			{
				// qj
				qj[sub](i) = Y(idx + cnt);

				// dqj
				dqj[sub](i) = Y(idx + cnt + nSubBody[sub] - nJointMotion[sub]);

				cnt++;
			}
		}
		idx = idx + 2 * (nSubBody[sub] - nJointMotion[sub]);
	}
}

void MBD_RecursiveII::position_BaseBody()
{
	// Euler parameter scaling
	//p0 = p0 / std::sqrt(p0(0) * p0(0) + p0(1) * p0(1) + p0(2) * p0(2) + p0(3) * p0(3));
	pj[0][0].normalize();

	// transformation matrix
	double e00 = pj[0][0](0);
	Eigen::Vector3d e0;
	e0(0) = pj[0][0](1);
	e0(1) = pj[0][0](2);
	e0(2) = pj[0][0](3);
	Eigen::Matrix3d e0_tilde = skew(e0);

	E0.col(0) = -e0;
	E0.block(0, 1, 3, 3) = e0_tilde + e00 * Eigen::Matrix3d::Identity();
	G0.col(0) = -e0;
	G0.block(0, 1, 3, 3) = -e0_tilde + e00 * Eigen::Matrix3d::Identity();

	Aj[0][0] = E0 * G0.transpose();

	// position vector
	rhoj[0][0] = Aj[0][0] * rhojp[0][0];

	rjc[0][0] = rj[0][0] + rhoj[0][0];

	//std::cout << "\nA0 = \n" << Aj[0][0] << std::endl;
	//std::cout << "\nr0c = \n" << rjc[0][0] << std::endl;
}

void MBD_RecursiveII::velocity_BaseBody()
{
	// angular velocity vector
	dp0 = 0.5 * E0.transpose() * wj[0][0];

	// linear velocity vector
	drjc[0][0] = drj[0][0] + skew(wj[0][0]) * rhoj[0][0];

	//std::cout << "\ndr0c = \n" << drjc[0][0] << std::endl;
}

void MBD_RecursiveII::acceleration_BaseBody()
{
	// angular acceleration
	dwj[0][0] = dYj_hat[0][0].segment(3, 3);

	// linear acceleration
	ddrj[0][0] = dYj_hat[0][0].segment(0, 3) - skew(drj[0][0]) * wj[0][0] - skew(rj[0][0]) * dwj[0][0];
	ddrjc[0][0] = ddrj[0][0] + skew(dwj[0][0]) * rhoj[0][0] + skew(wj[0][0]) * skew(wj[0][0]) * rhoj[0][0];
}

void MBD_RecursiveII::massforce_BaseBody()
{
	// inertia matrix at center of mass with respect to global reference frame 
	Jjc[0][0] = Aj[0][0] * Cjj[0][0] * Jjcp[0][0] * (Aj[0][0] * Cjj[0][0]).transpose();

	// generalized inertia matrix
	Mj_hat[0][0].block(0, 0, 3, 3) = mj[0](0) * Eigen::Matrix3d::Identity();
	Mj_hat[0][0].block(0, 3, 3, 3) = -mj[0](0) * skew(rjc[0][0]);
	Mj_hat[0][0].block(3, 0, 3, 3) = -Mj_hat[0][0].block(0, 3, 3, 3);
	Mj_hat[0][0].block(3, 3, 3, 3) = Jjc[0][0] - mj[0](0) * skew(rjc[0][0]) * skew(rjc[0][0]);

	// gravity force vector at center of mass with respect to global reference frame 
	Eigen::Vector3d F0c_g = m_force->gravityForce(gravityAxis, mj[0](0));

	// buoyancy force/torque vector at center of mass with respect to global reference frame  
	// need Dr.Han.........
	Eigen::Vector3d F0c_b = m_force->buoyancyForce();

	// total force/torque vector at center of mass with respect to global reference frame 
	Eigen::Vector3d F0c = F0c_g + F0c_b;
	Eigen::Vector3d T0c; T0c.setZero();

	// RSDA torque
	Qj_RSDA_hat[0][0].setZero();

	// generalized force vector
	Qj_hat[0][0].segment(0, 3) = F0c + mj[0](0) * skew(drjc[0][0]) * wj[0][0];
	Qj_hat[0][0].segment(3, 3) = T0c + skew(rjc[0][0]) * F0c + mj[0](0) * skew(rjc[0][0]) * skew(drjc[0][0]) * wj[0][0] - skew(wj[0][0]) * Jjc[0][0] * wj[0][0];

	//std::cout << "\nM0_hat = \n" << Mj_hat[0][0] << std::endl;
	//std::cout << "\nQ0_hat = \n" << Qj_hat[0][0] << std::endl;
}

void MBD_RecursiveII::position_Subsystem(int sub)
{
	Eigen::Matrix3d Aijpp;
	Eigen::Vector3d dijpp;
	int curSeq = 0;
	int preSeq = 0;
	
	for (int i = 0; i < nSubBody[sub]; i++)
	{
		// select subsystem body form loopSearch function
		curSeq = subBodySeq[sub](i) - 1;
		preSeq = pcList[sub](curSeq, 0) - 1;

		// value related joint position
		switch (jointType[sub](curSeq))
		{
		case JOINT_REVOLUTE:
			Aijpp = generalRotation(rotAxis, qj[sub](curSeq));
			dijpp.setZero();
			break;
		case JOINT_TRNSLATIONAL:
			Aijpp.setIdentity();
			dijpp = transAxis * qj[sub](curSeq);
			break;
		}

		// transformation matrix
		if (preSeq == -1)
		{
			// Ai = A0
			Aj[sub][curSeq] = Aj[0][0] * Cij[sub][curSeq] * Aijpp;
		}
		else
		{
			// Ai != A0
			Aj[sub][curSeq] = Aj[sub][preSeq] * Cij[sub][curSeq] * Aijpp;
		}
		
		// Euler parameter
		pj[sub][curSeq] = TransformationMatrixToEulerParameter(Aj[sub][curSeq]);

		// position vector
		if (preSeq == -1)
		{
			// ri = r0
			sij[sub][curSeq] = Aj[0][0] * sijp[sub][curSeq];
			dij[sub][curSeq] = Aj[0][0] * Cij[sub][curSeq] * dijpp;
			rj[sub][curSeq] = rj[0][0] + sij[sub][curSeq] + dij[sub][curSeq];
		}
		else
		{
			// ri != r0
			sij[sub][curSeq] = Aj[sub][preSeq] * sijp[sub][curSeq];
			dij[sub][curSeq] = Aj[sub][preSeq] * Cij[sub][curSeq] * dijpp;
			rj[sub][curSeq] = rj[sub][preSeq] + sij[sub][curSeq] + dij[sub][curSeq];
		}

		rhoj[sub][curSeq] = Aj[sub][curSeq] * rhojp[sub][curSeq];
		rjc[sub][curSeq] = rj[sub][curSeq] + rhoj[sub][curSeq];
		if (flag_contact[sub](curSeq))
		{
			rjf[sub][curSeq] = rj[sub][curSeq] + Aj[sub][curSeq] * sjcp[sub][curSeq];
		}

		//std::cout << "\nA" << std::to_string(curSeq + 1) << "[" << std::to_string(sub) << "] = \n" << Aj[sub][curSeq] << std::endl;
		//std::cout << "\nr" << std::to_string(curSeq + 1) << "[" << std::to_string(sub) << "] = \n" << rj[sub][curSeq] << std::endl;
		//std::cout << "\nr" << std::to_string(curSeq + 1) << "c[" << std::to_string(sub) << "] = \n" << rjc[sub][curSeq] << std::endl;
	}
}

void MBD_RecursiveII::velocity_Subsystem(int sub)
{
	Eigen::Vector3d d_dijpp;
	Eigen::Vector3d dij_qj;		// partial derivate of dij vector with respect to qj joint position 
	Eigen::Vector3d d_dij_qj;	// total derivate of dij_qj term
	int curSeq = 0;
	int preSeq = 0;
	
	for (int i = 0; i < nSubBody[sub]; i++)
	{
		// select subsystem body form loopSearch function
		curSeq = subBodySeq[sub](i) - 1;
		preSeq = pcList[sub](curSeq, 0) - 1;

		// value related joint velocity
		switch (jointType[sub](curSeq))
		{
		case JOINT_REVOLUTE:
			if (preSeq == -1)
			{
				Hj[sub][curSeq] = Aj[0][0] * Cij[sub][curSeq] * rotAxis;
				dHj[sub][curSeq] = skew(wj[0][0]) * Hj[sub][curSeq];
			}
			else
			{
				Hj[sub][curSeq] = Aj[sub][preSeq] * Cij[sub][curSeq] * rotAxis;
				dHj[sub][curSeq] = skew(wj[sub][preSeq]) * Hj[sub][curSeq];
			}
			d_dijpp.setZero();
			dij_qj.setZero();
			d_dij_qj.setZero();
			break;
		case JOINT_TRNSLATIONAL:
			Hj[sub][curSeq].setZero();
			dHj[sub][curSeq].setZero();
			d_dijpp = transAxis * dqj[sub](curSeq);
			if (preSeq == -1)
			{
				dij_qj = Aj[0][0] * Cij[sub][curSeq] * transAxis;
				d_dij_qj = skew(wj[0][0]) * dij_qj;
			}
			else
			{
				dij_qj = Aj[sub][preSeq] * Cij[sub][curSeq] * transAxis;
				d_dij_qj = skew(wj[sub][preSeq]) * dij_qj;
			}
			break;
		}

		// angular velocity
		if (preSeq == -1)
		{
			// wi = w0
			wj[sub][curSeq] = wj[0][0] + Hj[sub][curSeq] * dqj[sub](curSeq);
		}
		else
		{
			// wi != w0
			wj[sub][curSeq] = wj[sub][preSeq] + Hj[sub][curSeq] * dqj[sub](curSeq);
		}

		// linear velocity
		if (preSeq == -1)
		{
			// dri = dr0
			drj[sub][curSeq] = drj[0][0] + skew(wj[0][0]) * (sij[sub][curSeq] + dij[sub][curSeq]) + Aj[0][0] * Cij[sub][curSeq] * d_dijpp;
		}
		else
		{
			// dri != dr0
			drj[sub][curSeq] = drj[sub][preSeq] + skew(wj[sub][preSeq]) * (sij[sub][curSeq] + dij[sub][curSeq]) + Aj[sub][preSeq] * Cij[sub][curSeq] * d_dijpp;
		}

		drjc[sub][curSeq] = drj[sub][curSeq] + skew(wj[sub][curSeq]) * rhoj[sub][curSeq];
		if (flag_contact[sub](curSeq))
		{
			drjf[sub][curSeq] = drj[sub][curSeq] + skew(wj[sub][curSeq]) * Aj[sub][curSeq] * sjcp[sub][curSeq];
		}

		// velocity transformation matrix
		Bj[sub][curSeq].segment(0, 3) = dij_qj + skew(rj[sub][curSeq]) * Hj[sub][curSeq];
		Bj[sub][curSeq].segment(3, 3) = Hj[sub][curSeq];

		// velocity coupling term
		Dj[sub][curSeq].segment(0, 3) = (d_dij_qj + skew(drj[sub][curSeq]) * Hj[sub][curSeq] + skew(rj[sub][curSeq]) * dHj[sub][curSeq]) * dqj[sub](curSeq);
		Dj[sub][curSeq].segment(3, 3) = dHj[sub][curSeq] * dqj[sub](curSeq);

		//std::cout << "\nw" << std::to_string(curSeq + 1) << "[" << std::to_string(sub) << "] = \n" << wj[sub][curSeq] << std::endl;
		//std::cout << "\ndr" << std::to_string(curSeq + 1) << "[" << std::to_string(sub) << "] = \n" << drj[sub][curSeq] << std::endl;
		//std::cout << "\ndr" << std::to_string(curSeq + 1) << "c[" << std::to_string(sub) << "] = \n" << drjc[sub][curSeq] << std::endl;
		//std::cout << "\nB" << std::to_string(curSeq + 1) << "[" << std::to_string(sub) << "] = \n" << Bj[sub][curSeq] << std::endl;
		//std::cout << "\nD" << std::to_string(curSeq + 1) << "[" << std::to_string(sub) << "] = \n" << Dj[sub][curSeq] << std::endl;
	}
}

void MBD_RecursiveII::acceleration_Subsystem(int sub)
{
	//Eigen::Vector3d d_dijpp;
	//Eigen::Vector3d dd_dijpp;
	int curSeq = 0;
	int preSeq = 0;

	for (int i = 0; i < nSubBody[sub]; i++)
	{
		// select subsystem body form loopSearch function
		curSeq = subBodySeq[sub](i) - 1;
		preSeq = pcList[sub](curSeq, 0) - 1;

		// subsystem acceleration state 
		if (preSeq == -1)
		{
			dYj_hat[sub][curSeq] = dYj_hat[0][0] + Bj[sub][curSeq] * ddqj[sub](curSeq) + Dj[sub][curSeq];
		}
		else
		{
			dYj_hat[sub][curSeq] = dYj_hat[sub][preSeq] + Bj[sub][curSeq] * ddqj[sub](curSeq) + Dj[sub][curSeq];
		}

		// angular acceleration
		dwj[sub][curSeq] = dYj_hat[sub][curSeq].segment(3, 3);

		// linear acceleration
		ddrj[sub][curSeq] = dYj_hat[sub][curSeq].segment(0, 3) - skew(drj[sub][curSeq]) * wj[sub][curSeq] - skew(rj[0][0]) * dwj[sub][curSeq];
		ddrjc[sub][curSeq] = ddrj[sub][curSeq] + skew(dwj[sub][curSeq]) * rhoj[sub][curSeq] + skew(wj[sub][curSeq]) * skew(wj[sub][curSeq]) * rhoj[sub][curSeq];
	}
}

void MBD_RecursiveII::massforce_Subsystem(int sub)
{
	Eigen::Vector3d Fjc_g_sub;
	Eigen::Vector3d Fjc_b_sub;
	Eigen::Vector3d Fjc_c_sub;
	Eigen::Vector3d Tjc_c_sub;
	Eigen::Vector3d Fjc_sub;
	Eigen::Vector3d Tjc_sub;
	int curSeq = 0;
	int preSeq = 0;
	
	// initialization of composite mass & force -> setZero
	Qj_RSDA_hat[sub].assign(nSubBody[sub], Eigen::VectorXd::Zero(6));
	Kj[sub].assign(nSubBody[sub], Eigen::MatrixXd::Zero(6, 6));
	Lj[sub].assign(nSubBody[sub], Eigen::VectorXd::Zero(6));
	
	for(int i = nSubBody[sub] - 1; i >= 0; i--)
	{
		// select subsystem body form loopSearch function
		curSeq = subBodySeq[sub](i) - 1;
		preSeq = pcList[sub](curSeq, 0) - 1;

		// inertia matrix at center of mass with respect to global reference frame 
		Jjc[sub][curSeq] = Aj[sub][curSeq] * Cjj[sub][curSeq] * Jjcp[sub][curSeq] * (Aj[sub][curSeq] * Cjj[sub][curSeq]).transpose();

		// generalized inertia matrix
		Mj_hat[sub][curSeq].block(0, 0, 3, 3) = mj[sub](curSeq) * Eigen::Matrix3d::Identity();
		Mj_hat[sub][curSeq].block(0, 3, 3, 3) = -mj[sub](curSeq) * skew(rjc[sub][curSeq]);
		Mj_hat[sub][curSeq].block(3, 0, 3, 3) = -Mj_hat[sub][curSeq].block(0, 3, 3, 3);
		Mj_hat[sub][curSeq].block(3, 3, 3, 3) = Jjc[sub][curSeq] - mj[sub](curSeq) * skew(rjc[sub][curSeq]) * skew(rjc[sub][curSeq]);

		// gravity force vector at center of mass with respect to global reference frame 
		Fjc_g_sub = m_force->gravityForce(gravityAxis, mj[sub](curSeq));

		// buoyancy force/torque vector at center of mass with respect to global reference frame  
		// need Dr.Han.........
		Fjc_b_sub = m_force->buoyancyForce();

		// contact force/torque vector at center of mass with respect to global reference frame
		if (flag_contact[sub](curSeq))
		{
			if(road_h.size() == 6){
				pen_z_ref[sub](curSeq) = road_h[sub-1];
			}

			// std::cout << (int)sub << " pen_z_ref : " << pen_z_ref[sub](curSeq);
			// if(sub == 6) std::cout << std::endl;
			Fjc_c_sub = m_force->contactForce(rjf[sub][curSeq], drjf[sub][curSeq], pen_z_ref[sub](curSeq), k_contact[sub](curSeq), c_contact[sub](curSeq));
			Tjc_c_sub = skew(rjf[sub][curSeq] - rjc[sub][curSeq]) * Fjc_c_sub;
		}
		else
		{
			Fjc_c_sub.setZero();
			Tjc_c_sub.setZero();
		}

		// total force/torque vector at center of mass with respect to global reference frame 
		// need Dr.Han.........
		Fjc_sub = Fjc_g_sub + Fjc_b_sub + Fjc_c_sub;
		Tjc_sub = Tjc_c_sub;

		// RSDA torque
		double T_RSDA = m_force->RSDATorque(qj[sub](curSeq), dqj[sub](curSeq), qj_ini_RSDA[sub](curSeq), k_RSDA[sub](curSeq), c_RSDA[sub](curSeq));
		Qj_RSDA_hat[sub][curSeq].segment(3, 3) = Qj_RSDA_hat[sub][curSeq].segment(3, 3) - T_RSDA * Hj[sub][curSeq];
		if (preSeq != -1)
		{
			Qj_RSDA_hat[sub][preSeq].segment(3, 3) = Qj_RSDA_hat[sub][preSeq].segment(3, 3) + T_RSDA * Hj[sub][curSeq];
		}
		else
		{
			Qj_RSDA_hat[0][0].segment(3, 3) = Qj_RSDA_hat[0][0].segment(3, 3) + T_RSDA * Hj[sub][curSeq];
		}

		// generalized force vector
		Qj_hat[sub][curSeq].segment(0, 3) = Fjc_sub + mj[sub](curSeq) * skew(drjc[sub][curSeq]) * wj[sub][curSeq];
		Qj_hat[sub][curSeq].segment(3, 3) = Tjc_sub + skew(rjc[sub][curSeq]) * Fjc_sub + mj[sub](curSeq) * skew(rjc[sub][curSeq]) * skew(drjc[sub][curSeq]) * wj[sub][curSeq]
			- skew(wj[sub][curSeq]) * Jjc[sub][curSeq] * wj[sub][curSeq];
		Qj_hat[sub][curSeq] = Qj_hat[sub][curSeq] + Qj_RSDA_hat[sub][curSeq];

		// composite mass & force
		Kj[sub][curSeq] = Mj_hat[sub][curSeq] + Kj[sub][curSeq];
		Lj[sub][curSeq] = Qj_hat[sub][curSeq] + Lj[sub][curSeq];

		if (preSeq != -1)
		{
			Kj[sub][preSeq] = Kj[sub][preSeq] + Kj[sub][curSeq];
			Lj[sub][preSeq] = Lj[sub][preSeq] + Lj[sub][curSeq] - Kj[sub][curSeq] * Dj[sub][curSeq];
		}

		//std::cout << "\nM" << std::to_string(curSeq + 1) << "_hat[" << std::to_string(sub) << "] = \n" << Mj_hat[sub][curSeq] << std::endl;
		//std::cout << "\nQ" << std::to_string(curSeq + 1) << "_hat[" << std::to_string(sub) << "] = \n" << Qj_hat[sub][curSeq] << std::endl;
		//std::cout << "\nK" << std::to_string(curSeq + 1) << "[" << std::to_string(sub) << "] = \n" << Kj[sub][curSeq] << std::endl;
		//std::cout << "\nL" << std::to_string(curSeq + 1) << "[" << std::to_string(sub) << "] = \n" << Lj[sub][curSeq] << std::endl;
	}

	// compute Myy & Py
	Myy[sub].setZero();
	Py[sub].setZero();

	for (int i = 0; i < loopList[sub].rows(); i++)
	{
		curSeq = loopList[sub](i, 1) - 1;

		if ((i == 0) || (loopList[sub](i, 1) != loopList[sub](i - 1, 1)))
		{
			Myy[sub] = Kj[sub][curSeq] + Myy[sub];
			Py[sub] = Lj[sub][curSeq] - Kj[sub][curSeq] * Dj[sub][curSeq] + Py[sub];
		}
	}

	// compute Myq
	Myq[sub] = Eigen::MatrixXd::Zero(6, nSubBody[sub]);
	for (int j = 0; j < nSubBody[sub]; j++)
	{
		if (flag_motion[sub](j))
		{
			Py[sub] = Py[sub] - Kj[sub][j] * Bj[sub][j] * ddqj[sub](j);
		}
		else
		{
			Myq[sub].col(j) = Kj[sub][j] * Bj[sub][j];
		}
	}
	Myq[sub] = removeCold(Myq[sub], flag_motion[sub]);

	// compute Mqq & Pq
	int row = 0;
	int col = 0;
	std::vector<Eigen::VectorXd> Dj_sum(nSubBody[sub], Eigen::VectorXd::Zero(6));
	Eigen::VectorXd Pq_Moition = Eigen::VectorXd::Zero(nSubBody[sub]);

	Mqq[sub] = Eigen::MatrixXd::Zero(nSubBody[sub], nSubBody[sub]);
	for (int i = 0; i < nSubBody[sub]; i++)
	{
		if (!flag_motion[sub](i))
		{
			Mqq[sub](i, i) = Bj[sub][i].transpose() * Kj[sub][i] * Bj[sub][i];

			Dj_sum[i] = Dj[sub][i];
		}
	}

	for (int j = 1; j < loopList[sub].cols() - 1; j++)
	{
		for (int k = j + 1; k < loopList[sub].cols(); k++)
		{
			for (int i = 0; i < loopList[sub].rows(); i++)
			{
				if ((loopList[sub](i, k) != -1) && ((i == 0) || (loopList[sub](i, k) != loopList[sub](i - 1, k))))
				{
					row = loopList[sub](i, j) - 1;
					col = loopList[sub](i, k) - 1;

					if ((flag_motion[sub](row) == false) && (flag_motion[sub](col) == false))
					{
						Mqq[sub](row, col) = Bj[sub][row].transpose() * Kj[sub][col] * Bj[sub][col];
						Mqq[sub](col, row) = Mqq[sub](row, col);

						Dj_sum[col] = Dj_sum[col] + Dj[sub][row];
					}
					else if ((flag_motion[sub](row) == true) && (flag_motion[sub](col) == false))
					{
						Pq_Moition(col) = Pq_Moition(col) + (Bj[sub][col].transpose() * Kj[sub][col] * Bj[sub][row]).value() * ddqj[sub](row);

						Dj_sum[col] = Dj_sum[col] + Dj[sub][row];
					}
					else if ((flag_motion[sub](row) == false) && (flag_motion[sub](col) == true))
					{
						Pq_Moition(row) = Pq_Moition(row) + (Bj[sub][row].transpose() * Kj[sub][col] * Bj[sub][col]).value() * ddqj[sub](col);
					}
				}
			}
		}
	}
	Mqq[sub] = removeRowd(Mqq[sub], flag_motion[sub]);
	Mqq[sub] = removeCold(Mqq[sub], flag_motion[sub]);

	Pq[sub] = Eigen::VectorXd::Zero(nSubBody[sub]);
	for (int i = 0; i < nSubBody[sub]; i++)
	{
		if (!flag_motion[sub](i))
		{
			//Pq[sub](i) = Bj[sub][i].transpose() * (Lj[sub][i] - Kj[sub][i] * Dj_sum[i]) - Qj_RSDA[sub](i);
			Pq[sub](i) = Bj[sub][i].transpose() * (Lj[sub][i] - Kj[sub][i] * Dj_sum[i]);
		}

	}
	Pq[sub] = Pq[sub] - Pq_Moition;
	Pq[sub] = removeRowd(Pq[sub], flag_motion[sub]);
	

	//std::cout << "\nMyy[" << std::to_string(sub) << "] = \n" << Myy[sub] << std::endl;
	//std::cout << "\nMyq[" << std::to_string(sub) << "] = \n" << Myq[sub] << std::endl;
	//std::cout << "\nMqq[" << std::to_string(sub) << "] = \n" << Mqq[sub] << std::endl;
	//std::cout << "\nPy[" << std::to_string(sub) << "] = \n" << Py[sub] << std::endl;
	//std::cout << "\nPq[" << std::to_string(sub) << "] = \n" << Pq[sub] << std::endl;
}

void MBD_RecursiveII::effectiveTerm_Subsystem(int sub)
{
	Eigen::MatrixXd MyqMqqi(6, nSubBody[sub]);

	if (nSubBody[sub] - nJointMotion[sub] != 0)
	{
		MyqMqqi = Myq[sub] * Mqq[sub].inverse();

		// effective mass
		Me[sub] = Myy[sub] - MyqMqqi * Myq[sub].transpose();

		// effective force
		Pe[sub] = Py[sub] - MyqMqqi * Pq[sub];
	}
	else
	{
		// effective mass
		Me[sub] = Myy[sub];

		// effective force
		Pe[sub] = Py[sub];
	}
}

void MBD_RecursiveII::EQM_WholeBody()
{
	int nSize = 6;
	Eigen::MatrixXd K0(6, 6);
	Eigen::VectorXd L0(6);

	// compute K0 & L0
	K0 = Mj_hat[0][0];
	Qj_hat[0][0] = Qj_hat[0][0] + Qj_RSDA_hat[0][0];
	L0 = Qj_hat[0][0];
	for (int sub = 1; sub <= nSubsystem; sub++)
	{
		K0 = K0 + Myy[sub];
		L0 = L0 + Py[sub];
		nSize = nSize + (nSubBody[sub] - nJointMotion[sub]);
	}

	// compute M & Q
	int cnt = 6;
	Eigen::MatrixXd M(nSize, nSize); M.setZero();
	Eigen::VectorXd Q(nSize);
	M.block(0, 0, 6, 6) = K0;
	Q.segment(0, 6) = L0;
	for (int sub = 1; sub <= nSubsystem; sub++)
	{
		if (nSubBody[sub] - nJointMotion[sub] != 0)
		{
			M.block(0, cnt, 6, nSubBody[sub] - nJointMotion[sub]) = Myq[sub];
			M.block(cnt, 0, nSubBody[sub] - nJointMotion[sub], 6) = Myq[sub].transpose();
			M.block(cnt, cnt, nSubBody[sub] - nJointMotion[sub], nSubBody[sub] - nJointMotion[sub]) = Mqq[sub];

			Q.segment(cnt, nSubBody[sub] - nJointMotion[sub]) = Pq[sub];

			cnt = cnt + nSubBody[sub] - nJointMotion[sub];
		}
	}

#ifdef BASEBODY_FIX
	M.block(0, 0, 6, 6).setIdentity();
	M.block(0, 6, 6, nSize - 6).setZero();
	M.block(6, 0, nSize - 6, 6).setZero();
	Q.segment(0, 6).setZero();
#endif

#ifdef BASEBODY_TRANS_Z
	M.block(0, 0, 6, 6).setIdentity();
	M(2, 2) = K0(2, 2);
	M.block(0, 6, 2, nSize - 6).setZero();
	M.block(3, 6, 3, nSize - 6).setZero();
	M.block(6, 0, nSize - 6, 2).setZero();
	M.block(6, 3, nSize - 6, 3).setZero();
	Q.segment(0, 6).setZero();
	Q(2) = L0(2);
#endif
	
	// solve whole body EQM
	Eigen::VectorXd sol = M.inverse() * Q;

	// base body acceleration state 
	dYj_hat[0][0] = sol.segment(0, 6);

	int idx = 6;
	for (int sub = 1; sub <= nSubsystem; sub++)
	{
		cnt = 0;
		for (int i = 0; i < nSubBody[sub]; i++)
		{
			// subsystem joint acceleration
			if (!flag_motion[sub](i))
			{
				ddqj[sub](i) = sol(idx + cnt);
				cnt++;
			}
		}
		idx = idx + nSubBody[sub] - nJointMotion[sub];
	}

	//std::cout << "\nK0 = \n" << K0 << std::endl;
	//std::cout << "\nL0 = \n" << L0 << std::endl;
	//for (int sub = 1; sub <= nSubsystem; sub++)
	//{
	//	std::cout << "\nddqj[" << std::to_string(sub) << "] = \n" << ddqj[sub] << std::endl;
	//}
}

void MBD_RecursiveII::EQM_BaseBody()
{
	// summation of effective mass & force
	Eigen::MatrixXd M_sum(6, 6); M_sum.setZero();
	Eigen::VectorXd P_sum(6); P_sum.setZero();
	for (int i = 1; i <= nSubsystem; i++)
	{
		M_sum = M_sum + Me[i];
		P_sum = P_sum + Pe[i];
	}

	// solve base body EQM
	Qj_hat[0][0] = Qj_hat[0][0] + Qj_RSDA_hat[0][0];
	dYj_hat[0][0] = (Mj_hat[0][0] + M_sum).inverse() * (Qj_hat[0][0] + P_sum);

#ifdef BASEBODY_FIX
	dYj_hat[0][0].setZero();
#endif

#ifdef BASEBODY_TRANS_Z
	dYj_hat[0][0].segment(0, 2).setZero();
	dYj_hat[0][0].segment(3, 3).setZero();
#endif
}

void MBD_RecursiveII::EQM_Subsystem(int sub)
{
	// solve subsystem EQM
	Eigen::VectorXd sol;
	int cnt = 0;
	if (nSubBody[sub] - nJointMotion[sub] != 0)
	{
		sol = Mqq[sub].inverse() * (Pq[sub] - Myq[sub].transpose() * dYj_hat[0][0]);

		for (int i = 0; i < nSubBody[sub]; i++)
		{
			if (!flag_motion[sub](i))
			{
				ddqj[sub](i) = sol(cnt);
				cnt++;
			}
		}
	}
}

void MBD_RecursiveII::jointReactionForce(int sub)
{
	int row = 0;
	int col = 0;
	std::vector<Eigen::VectorXd> KjBjddqj_sum(nSubBody[sub], Eigen::VectorXd::Zero(6));
	for (int j = 1; j < loopList[sub].cols() - 1; j++)
	{
		for (int k = j + 1; k < loopList[sub].cols(); k++)
		{
			for (int i = 0; i < loopList[sub].rows(); i++)
			{
				if ((loopList[sub](i, k) != -1) && ((i == 0) || (loopList[sub](i, k) != loopList[sub](i - 1, k))))
				{
					row = loopList[sub](i, j) - 1;
					col = loopList[sub](i, k) - 1;

					KjBjddqj_sum[row] = KjBjddqj_sum[row] + Kj[sub][col] * Bj[sub][col] * ddqj[sub](col);
				}
			}
		}
	}

	Eigen::VectorXd Rji_hat(6);
	for (int i = 0; i < nSubBody[sub]; i++)
	{
		if (flag_motion[sub](i))
		{
			Rji_hat = -Lj[sub][i] + Kj[sub][i] * dYj_hat[sub][i] + KjBjddqj_sum[i];
			Rji[sub][i].segment(0, 3) = Rji_hat.segment(0, 3);
			Rji[sub][i].segment(3, 3) = -skew(rj[sub][i]) * Rji_hat.segment(0, 3) + Rji_hat.segment(3, 3);

			Rji[sub][i].segment(0, 3) = Aj[sub][i].transpose() * Rji[sub][i].segment(0, 3);
			Rji[sub][i].segment(3, 3) = Aj[sub][i].transpose() * Rji[sub][i].segment(3, 3);
		}
		else
		{
			Rji[sub][i].setZero();
		}
	}
}

Eigen::VectorXd MBD_RecursiveII::VelAcc2dY()
{
	Eigen::VectorXd dY(13);

	dY.segment(0, 3) = drj[0][0];
	dY.segment(3, 4) = dp0;
	dY.segment(7, 3) = ddrj[0][0];
	dY.segment(10, 3) = dwj[0][0];

	int idx = 13;
	int cnt = 0;
	for (int sub = 1; sub <= nSubsystem; sub++)
	{
		dY.conservativeResize(idx + 2 * (nSubBody[sub] - nJointMotion[sub]));
		cnt = 0;
		for (int i = 0; i < nSubBody[sub]; i++)
		{
			if (!flag_motion[sub](i))
			{
				dY(idx + cnt) = dqj[sub](i);
				dY(idx + cnt + nSubBody[sub] - nJointMotion[sub]) = ddqj[sub](i);
				cnt++;
			}
		}
		idx = idx + 2 * (nSubBody[sub] - nJointMotion[sub]);
	}

	return dY;
}

Eigen::VectorXd MBD_RecursiveII::dynamics_analysis(double t_current, Eigen::VectorXd Y)
{
	// state vector
	Y2PosVel(t_current, Y);

	// base body analysis
	position_BaseBody();
	velocity_BaseBody();
	massforce_BaseBody();

	// subsystem analysis
	for (int sub = 1; sub <= nSubsystem; sub++)
	{
		position_Subsystem(sub);
		velocity_Subsystem(sub);
		massforce_Subsystem(sub);
		if (analysis_method == METHOD_SUBSYSTEM)
		{
			effectiveTerm_Subsystem(sub);
		}
	}

	// EQM
	switch (analysis_method)
	{
	case METHOD_CONVENTIONAL:
		EQM_WholeBody();
		break;
	case METHOD_SUBSYSTEM:
		EQM_BaseBody();
		for (int sub = 1; sub <= nSubsystem; sub++)
		{
			EQM_Subsystem(sub);
		}
		break;
	}

	// acceleration and joint reaction force
	acceleration_BaseBody();
	for (int sub = 1; sub <= nSubsystem; sub++)
	{
		acceleration_Subsystem(sub);
		jointReactionForce(sub);
	}
	
	// dot state vector
	Eigen::VectorXd dY = VelAcc2dY();

	return dY;
}

OutputData MBD_RecursiveII::getOutputData()
{
	OutputData outData;

	// base body data
	outData.BaseBody.r0c = rjc[0][0];
	outData.BaseBody.p0 = pj[0][0];
	outData.BaseBody.dr0c = drjc[0][0];
	outData.BaseBody.w0 = wj[0][0];
	outData.BaseBody.ddr0c = ddrjc[0][0];
	outData.BaseBody.dw0 = dwj[0][0];

	// subsystem data
	for (int sub = 1; sub <= nSubsystem; sub++)
	{
		outData.Subsystem.qj[sub] = qj[sub];
		outData.Subsystem.dqj[sub] = dqj[sub];
		outData.Subsystem.ddqj[sub] = ddqj[sub];
		outData.Subsystem.Rji[sub] = Rji[sub];
		outData.Subsystem.rjc[sub] = rjc[sub];
		outData.Subsystem.pj[sub] = pj[sub];
		outData.Subsystem.drjc[sub] = drjc[sub];
		outData.Subsystem.ddrjc[sub] = ddrjc[sub];
		outData.Subsystem.wj[sub] = wj[sub];
		outData.Subsystem.dwj[sub] = dwj[sub];
	}

	return outData;
}
