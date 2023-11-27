#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QStringList>
#include <QProcess>

#include <ros/ros.h>
#include <ros/package.h>
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/tool_manager.h>

#include "rviz.h"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <jsoncpp/json/json.h>
#include <iostream>
#include <fstream>

#include <crabster_msgs/CrabsterPose.h>
#include <actionlib/client/simple_action_client.h>
#include <crabster_msgs/CrabsterSimulationAction.h>

#include <sensor_msgs/JointState.h>

namespace Ui { class MainWindow; }

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    
    Rviz *rvizRobot;

    ros::NodeHandle *nh;

    crabster_msgs::CrabsterPose crabsterPose;

    void CrabsterPoseCB(const crabster_msgs::CrabsterPose &msg);
    void CrabsterCompleteCB(const actionlib::SimpleClientGoalState &state, const crabster_msgs::CrabsterSimulationResultConstPtr &result);
    void CrabsterFeedbackCB(const crabster_msgs::CrabsterSimulationFeedbackConstPtr &feedback);
    void CrabsterActiveCB();

    ros::Subscriber subCrabsterPose;
    actionlib::SimpleActionClient<crabster_msgs::CrabsterSimulationAction> *ac;
    crabster_msgs::CrabsterSimulationGoal goal;

public slots:
    void btnLoadClicked();
    void btnModelClicked();
    void btnRunClicked();

};
#endif // MAINWINDOW_H

inline Eigen::Vector3d mat2rpy(Eigen::Matrix3d mat){
    Eigen::Vector3d rpy;

    rpy(0) = atan2(mat(2,1), mat(2,2));
    rpy(1) = atan2(-mat(2,0), sqrt(pow(mat(2,1), 2.0) + pow(mat(2,2), 2.0)));
    rpy(2) = atan2(mat(1,0), mat(0,0));

    return rpy;
}

inline Eigen::Matrix3d skew(Eigen::Vector3d x)
{
	Eigen::Matrix3d x_tilde;

	x_tilde(0, 0) = 0.0;	x_tilde(0, 1) = -x(2);	x_tilde(0, 2) = x(1);
	x_tilde(1, 0) = x(2);	x_tilde(1, 1) = 0.0;	x_tilde(1, 2) = -x(0);
	x_tilde(2, 0) = -x(1);	x_tilde(2, 1) = x(0);	x_tilde(2, 2) = 0.0;

	return x_tilde;
}