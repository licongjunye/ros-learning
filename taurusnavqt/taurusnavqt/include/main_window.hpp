/**
 * @file /include/taurusnavqt/main_window.hpp
 *
 * @brief Qt based gui for taurusnavqt.
 *
 * @date November 2010
 **/
#ifndef taurusnavqt_MAIN_WINDOW_H
#define taurusnavqt_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QImage>
#include <QProcess>
#include <QComboBox>
#include <QSpinBox>
#include <QDebug>
#include <QKeyEvent>
#include "qrviz.hpp"
#include "roboItem.h"
#include "RobotAlgorithm.h"
#include "qnode.hpp"


/*****************************************************************************
** Namespace
*****************************************************************************/

namespace taurusnavqt {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
    
private:
    void initRviz();
    void initGraphicsView();
    void connections();
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);

protected:
    void keyPressEvent (QKeyEvent *event) override;

 Q_SIGNALS:
    void signal_robotstatus(float,float,float);

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);
    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void slot_treewidget_value_change(QString);
    void slot_display_grid(int);
    void slot_display_tf(int);
    void slot_display_laser(int);
    void slot_display_RobotModel(int);
    void slot_display_Map(int);
    void slot_display_Path(int);
    void slot_set_start_pose();
    void slot_set_goal_pose();
    void slot_display_local_map(int state);
    void slot_display_global_map(int state);
    void slot_update_pos(double,double,double);
    void slot_set_return_pos();
    void slot_return();

    //显示图像
    void slot_pubImageMapTimeOut();
    void slot_updateCursorPos(QPointF pos);
    void slot_updateRobotStatus(RobotStatus);
    void slot_updateRoboPos(QPointF);

private slots:
    void on_pB_send_clicked();

private:
    Ui::MainWindowDesign ui;
    QNode qnode;
    qrviz* myrviz;
    QComboBox* fixed_box;
    QSpinBox * Cell_Count_Box;
    QComboBox* Grid_Color_Box;
    QComboBox* Laser_Topic_box;
    QComboBox* Map_Topic_box;
    QComboBox* Map_Color_Scheme_box;
    QComboBox* Path_Topic_box;
    QComboBox* Path_Color_box;
    //Navigate
    QComboBox* Global_CostMap_Topic_box;
    QComboBox* GlobalMapColorScheme_box;
    QComboBox* Local_CostMap_Topic_box;
    QComboBox* LocalMapColorScheme_box;
    QComboBox* Global_Planner_Topic_box;
    QComboBox* Global_Planner_Color_box;
    QComboBox* Local_Planner_Topic_box;
    QComboBox* Local_Planner_Color_box;

    double return_x;
    double return_y;
    double return_z;

    QTimer *m_timerChart;
    QTimer *m_timerPubImageMap;
    QTimer *m_timerCurrentTime;
    QGraphicsScene *m_qgraphicsScene = NULL;
    roboItem *m_roboItem = NULL;
    QPoint m_lastPos;
    bool isPressedWidget;

};

}  // namespace taurusnavqt

#endif // taurusnavqt_MAIN_WINDOW_H
