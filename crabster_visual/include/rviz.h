#pragma once

#include <ros/ros.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>

class Rviz{

public:
    Rviz();

    void initRvizRobotModel(void *_ui);

private:
    rviz::VisualizationManager  *m_RvizManager;
    rviz::RenderPanel           *m_RvizRenderPanel;

    rviz::Display               *m_RvizGrid;
    rviz::Display               *m_RvizSetFixedFrame;
    rviz::Display               *m_RvizRobotModel;

    void setTopicRobot();
};

