#include "rviz.h"
#include "ui_mainwindow.h"

Rviz::Rviz()
{
}

void Rviz::initRvizRobotModel(void *_layout)
{
  	QVBoxLayout *layout = static_cast<QVBoxLayout *>(_layout);

	m_RvizRenderPanel = new rviz::RenderPanel();

	layout->addWidget(m_RvizRenderPanel);

	m_RvizManager = new rviz::VisualizationManager(m_RvizRenderPanel);

	m_RvizRenderPanel->initialize(m_RvizManager->getSceneManager(), m_RvizManager);
	m_RvizManager->initialize();
	m_RvizManager->startUpdate();

	setTopicRobot();
}

void Rviz::setTopicRobot()
{
	m_RvizManager->setFixedFrame("world");

	m_RvizGrid = m_RvizManager->createDisplay("rviz/Grid", "adjustable grid", true);
	m_RvizGrid->subProp("Line Style")->setValue("Billboards");

	m_RvizRobotModel = m_RvizManager->createDisplay("rviz/RobotModel", "robotmodel", true);
  	m_RvizRobotModel->subProp("Robot Description")->setValue("robot_description");
}
