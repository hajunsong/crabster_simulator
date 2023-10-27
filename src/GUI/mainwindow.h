#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/tool_manager.h>

#include "rviz.h"

#include "Common.h"
#include "Inputs.h"
#include "MBD_RecursiveII.h"
#include "Integrator.h"
#include "Outputs.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    Rviz *rvizRobot;

    Inputs* m_inputs;
	MBD_RecursiveII* m_dynamics;
	Integrator* m_integrator;
	Outputs* m_outputs;

public slots:
    void btnLoadClicked();
    void btnModelClicked();
    void btnRunClicked();

};
#endif // MAINWINDOW_H
