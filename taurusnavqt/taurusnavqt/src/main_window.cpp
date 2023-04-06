/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace taurusnavqt {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }

    initRviz();
    initGraphicsView();
    // 设置比例5:1
    QList<int> sizes;
    // 在保持比例的情况下，绝对值要尽量大
    sizes << 30000 << 10000;
    ui.splitter->setSizes(sizes);
    connections();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

void MainWindow::initRviz()
{
    //header
    ui.treeWidget->setHeaderLabels(QStringList()<<"key"<<"value");
    ui.treeWidget->setHeaderHidden(true);
    ui.treeWidget->header()->setSectionResizeMode(QHeaderView::ResizeToContents);
    //GLobal Options
    QTreeWidgetItem* Global=new QTreeWidgetItem(QStringList()<<"Global Options");
    Global->setIcon(0,QIcon("://images/options.png"));

    ui.treeWidget->addTopLevelItem(Global);
    Global->setExpanded(true);
    //FixFrame
    QTreeWidgetItem* Fixed_frame=new QTreeWidgetItem(QStringList()<<"Fixed Frame");
    fixed_box=new QComboBox();
    fixed_box->addItem("map");
    fixed_box->setMaximumWidth(150);
    fixed_box->setEditable(true);
    connect(fixed_box,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_value_change(QString)));
    Global->addChild(Fixed_frame);


    ui.treeWidget->setItemWidget(Fixed_frame,1,fixed_box);

    //Grid
    QTreeWidgetItem* Grid=new QTreeWidgetItem(QStringList()<<"Grid");
    //设置图标
    Grid->setIcon(0,QIcon("://images/classes/Grid.png"));
    //checkbox
    QCheckBox* Grid_Check=new QCheckBox();
    connect(Grid_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_grid(int)));
    //添加top节点
    ui.treeWidget->addTopLevelItem(Grid);
    //添加checkbox
    ui.treeWidget->setItemWidget(Grid,1,Grid_Check);
    //设置grid默认展开状态
    Grid->setExpanded(true);

    //添加Cell Count子节点
    QTreeWidgetItem* Cell_Count=new QTreeWidgetItem(QStringList()<<"Plane Cell Count");
    Grid->addChild(Cell_Count);
    //CellCount添加SpinBox
    Cell_Count_Box=new QSpinBox();
    Cell_Count_Box->setValue(13);
    //设置Spinbox的宽度
    Cell_Count_Box->setMaximumWidth(150);
    ui.treeWidget->setItemWidget(Cell_Count,1,Cell_Count_Box);

    //添加color子节点
    QTreeWidgetItem* Grid_Color=new QTreeWidgetItem(QStringList()<<"Color");
    Grid->addChild(Grid_Color);
    //Color添加ComboBox
    Grid_Color_Box=new QComboBox();
    Grid_Color_Box->addItem("160;160;160");
    //设置Comboox可编辑
    Grid_Color_Box->setEditable(true);
    //设置Combox的宽度
    Grid_Color_Box->setMaximumWidth(150);
    ui.treeWidget->setItemWidget(Grid_Color,1,Grid_Color_Box);

    //TF ui
    QTreeWidgetItem* TF=new QTreeWidgetItem(QStringList()<<"TF");
    //设置图标
    TF->setIcon(0,QIcon("://images/classes/TF.png"));
    //checkbox
    QCheckBox* TF_Check=new QCheckBox();
    connect(TF_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_tf(int)));
    //向Treewidget添加TF Top节点
    ui.treeWidget->addTopLevelItem(TF);
    //向TF添加checkbox
    ui.treeWidget->setItemWidget(TF,1,TF_Check);

    //LaserScan
    QTreeWidgetItem* LaserScan=new QTreeWidgetItem(QStringList()<<"LaserScan");
    //设置图标
    LaserScan->setIcon(0,QIcon("://images/classes/LaserScan.png"));
    //checkbox
    QCheckBox* Laser_Check=new QCheckBox();
    connect(Laser_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_laser(int)));
    //向Treewidget添加TF Top节点
    ui.treeWidget->addTopLevelItem(LaserScan);
    //向TF添加checkbox
    ui.treeWidget->setItemWidget(LaserScan,1,Laser_Check);
    //laser topic
    QTreeWidgetItem* LaserTopic=new QTreeWidgetItem(QStringList()<<"Topic");
    Laser_Topic_box=new QComboBox();
    Laser_Topic_box->addItem("/scan");
    Laser_Topic_box->setEditable(true);
    Laser_Topic_box->setMaximumWidth(150);
    LaserScan->addChild(LaserTopic);
    ui.treeWidget->setItemWidget(LaserTopic,1,Laser_Topic_box);

    //RobotModel
    QTreeWidgetItem* RobotModel=new QTreeWidgetItem(QStringList()<<"RobotModel");
    //设置图标
    RobotModel->setIcon(0,QIcon("://images/classes/RobotModel.png"));
    //checkbox
    QCheckBox* RobotModel_Check=new QCheckBox();
    connect(RobotModel_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_RobotModel(int)));
    //向Treewidget添加TF Top节点
    ui.treeWidget->addTopLevelItem(RobotModel);
    //向TF添加checkbox
    ui.treeWidget->setItemWidget(RobotModel,1,RobotModel_Check);


    //Map
    QTreeWidgetItem* mymap=new QTreeWidgetItem(QStringList()<<"Map");
    //设置图标
    mymap->setIcon(0,QIcon("://images/classes/Map.png"));
    //checkbox
    QCheckBox* Map_Check=new QCheckBox();
    connect(Map_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_Map(int)));
    //向Treewidget添加Map Top节点
    ui.treeWidget->addTopLevelItem(mymap);
    //向Map添加checkbox
    ui.treeWidget->setItemWidget(mymap,1,Map_Check);
    //Map topic
    QTreeWidgetItem* MapTopic=new QTreeWidgetItem(QStringList()<<"Topic");
    Map_Topic_box=new QComboBox();
    Map_Topic_box->addItem("/map");
    Map_Topic_box->setEditable(true);
    Map_Topic_box->setMaximumWidth(150);
    mymap->addChild(MapTopic);
    ui.treeWidget->setItemWidget(MapTopic,1,Map_Topic_box);
    //Map color scheme
    QTreeWidgetItem* MapColorScheme=new QTreeWidgetItem(QStringList()<<"Color Scheme");
    Map_Color_Scheme_box=new QComboBox();
    Map_Color_Scheme_box->addItem("map");
    Map_Color_Scheme_box->addItem("costmap");
    Map_Color_Scheme_box->addItem("raw");
    Map_Color_Scheme_box->setMaximumWidth(150);
    mymap->addChild(MapColorScheme);
    ui.treeWidget->setItemWidget(MapColorScheme,1,Map_Color_Scheme_box);

    //Path
    QTreeWidgetItem* Path=new QTreeWidgetItem(QStringList()<<"Path");
    //设置图标
    Path->setIcon(0,QIcon("://images/classes/Path.png"));
    //checkbox
    QCheckBox* Path_Check=new QCheckBox();
    connect(Path_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_Path(int)));
    //向Treewidget添加Path Top节点
    ui.treeWidget->addTopLevelItem(Path);
    //向Path添加checkbox
    ui.treeWidget->setItemWidget(Path,1,Path_Check);
    //Path topic
    QTreeWidgetItem* PathTopic=new QTreeWidgetItem(QStringList()<<"Topic");
    Path_Topic_box=new QComboBox();
    Path_Topic_box->addItem("/move_base/DWAPlannerROS/local_plan");
    Path_Topic_box->setEditable(true);
    Path_Topic_box->setMaximumWidth(150);
    Path->addChild(PathTopic);
    ui.treeWidget->setItemWidget(PathTopic,1,Path_Topic_box);
    //Path color scheme
    QTreeWidgetItem* PathColorScheme=new QTreeWidgetItem(QStringList()<<"Color");
    Path_Color_box=new QComboBox();
    Path_Color_box->addItem("0;12;255");
    Path_Color_box->setEditable(true);
    Path_Color_box->setMaximumWidth(150);
    Path->addChild(PathColorScheme);
    ui.treeWidget->setItemWidget(PathColorScheme,1,Path_Color_box);

    //机器人Navigate 相关UI********************************
    //Golabal Map***************************************
    QTreeWidgetItem* GlobalMap=new QTreeWidgetItem(QStringList()<<"Global Map");
    GlobalMap->setIcon(0,QIcon("://images/default_package_icon.png"));
    QCheckBox* GlobalMap_Check=new QCheckBox();
    connect(GlobalMap_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_global_map(int)));
    ui.treeWidget->addTopLevelItem(GlobalMap);
    ui.treeWidget->setItemWidget(GlobalMap,1,GlobalMap_Check);

    //Global CostMap
    QTreeWidgetItem* Global_CostMap=new QTreeWidgetItem(QStringList()<<"Costmap");
    //设置图标
    Global_CostMap->setIcon(0,QIcon("://images/classes/Map.png"));
    //Global Map添加子节点
    GlobalMap->addChild(Global_CostMap);
    //Map topic
    QTreeWidgetItem* Global_CostMap_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
    Global_CostMap_Topic_box=new QComboBox();
    Global_CostMap_Topic_box->addItem("/move_base/global_costmap/costmap");
    Global_CostMap_Topic_box->setEditable(true);
    Global_CostMap_Topic_box->setMaximumWidth(150);
    Global_CostMap->addChild(Global_CostMap_Topic);
    ui.treeWidget->setItemWidget(Global_CostMap_Topic,1,Global_CostMap_Topic_box);
    //Map color scheme
    QTreeWidgetItem* GlobalMapColorScheme=new QTreeWidgetItem(QStringList()<<"Color Scheme");
    GlobalMapColorScheme_box=new QComboBox();
    GlobalMapColorScheme_box->addItem("costmap");
    GlobalMapColorScheme_box->addItem("map");
    GlobalMapColorScheme_box->addItem("raw");
    GlobalMapColorScheme_box->setMaximumWidth(150);
    Global_CostMap->addChild(GlobalMapColorScheme);
    ui.treeWidget->setItemWidget(GlobalMapColorScheme,1,GlobalMapColorScheme_box);

    //Global Planner
    QTreeWidgetItem* Global_Planner=new QTreeWidgetItem(QStringList()<<"Planner");
    //设置图标
    Global_Planner->setIcon(0,QIcon("://images/classes/Path.png"));
    //向TGlobal Map添加Path Top节点
    GlobalMap->addChild(Global_Planner);

    //Path topic
    QTreeWidgetItem* Global_Planner_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
    Global_Planner_Topic_box=new QComboBox();
    Global_Planner_Topic_box->addItem("/move_base/DWAPlannerROS/global_plan");
    Global_Planner_Topic_box->setEditable(true);
    Global_Planner_Topic_box->setMaximumWidth(150);
    Global_Planner->addChild(Global_Planner_Topic);
    ui.treeWidget->setItemWidget(Global_Planner_Topic,1,Global_Planner_Topic_box);
    //Path color scheme
    QTreeWidgetItem* Global_Planner_Color_Scheme=new QTreeWidgetItem(QStringList()<<"Color");
    Global_Planner_Color_box=new QComboBox();
    Global_Planner_Color_box->addItem("255;0;0");
    Global_Planner_Color_box->setEditable(true);
    Global_Planner_Color_box->setMaximumWidth(150);
    Global_Planner->addChild(Global_Planner_Color_Scheme);
    ui.treeWidget->setItemWidget(Global_Planner_Color_Scheme,1,Global_Planner_Color_box);

    //Local Map***********************************************
    QTreeWidgetItem* LocalMap=new QTreeWidgetItem(QStringList()<<"Local Map");
    LocalMap->setIcon(0,QIcon("://images/default_package_icon.png"));
    QCheckBox* LocalMap_Check=new QCheckBox();
    connect(LocalMap_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_local_map(int)));
    ui.treeWidget->addTopLevelItem(LocalMap);
    ui.treeWidget->setItemWidget(LocalMap,1,LocalMap_Check);

    //Local CostMap
    QTreeWidgetItem* Local_CostMap=new QTreeWidgetItem(QStringList()<<"Costmap");
    //设置图标
    Local_CostMap->setIcon(0,QIcon("://images/classes/Map.png"));
    //Local Map添加子节点
    LocalMap->addChild(Local_CostMap);
    //Map topic
    QTreeWidgetItem* Local_CostMap_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
    Local_CostMap_Topic_box=new QComboBox();
    Local_CostMap_Topic_box->addItem("/move_base/local_costmap/costmap");
    Local_CostMap_Topic_box->setEditable(true);
    Local_CostMap_Topic_box->setMaximumWidth(150);
    Local_CostMap->addChild(Local_CostMap_Topic);
    ui.treeWidget->setItemWidget(Local_CostMap_Topic,1,Local_CostMap_Topic_box);
    //Map color scheme
    QTreeWidgetItem* LocalMapColorScheme=new QTreeWidgetItem(QStringList()<<"Color Scheme");
    LocalMapColorScheme_box=new QComboBox();
    LocalMapColorScheme_box->addItem("costmap");
    LocalMapColorScheme_box->addItem("map");
    LocalMapColorScheme_box->addItem("raw");
    LocalMapColorScheme_box->setMaximumWidth(150);
    Local_CostMap->addChild(LocalMapColorScheme);
    ui.treeWidget->setItemWidget(LocalMapColorScheme,1,LocalMapColorScheme_box);

    //Local Planner
    QTreeWidgetItem* Local_Planner=new QTreeWidgetItem(QStringList()<<"Planner");
    //设置图标
    Local_Planner->setIcon(0,QIcon("://images/classes/Path.png"));
    //向TLocal Map添加Path Top节点
    LocalMap->addChild(Local_Planner);

    //Path topic
    QTreeWidgetItem* Local_Planner_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
    Local_Planner_Topic_box=new QComboBox();
    Local_Planner_Topic_box->addItem("/move_base/DWAPlannerROS/local_plan");
    Local_Planner_Topic_box->setEditable(true);
    Local_Planner_Topic_box->setMaximumWidth(150);
    Local_Planner->addChild(Local_Planner_Topic);
    ui.treeWidget->setItemWidget(Local_Planner_Topic,1,Local_Planner_Topic_box);
    //Path color scheme
    QTreeWidgetItem* Local_Planner_Color_Scheme=new QTreeWidgetItem(QStringList()<<"Color");
    Local_Planner_Color_box=new QComboBox();
    Local_Planner_Color_box->addItem("0;12;255");
    Local_Planner_Color_box->setEditable(true);
    Local_Planner_Color_box->setMaximumWidth(150);
    Local_Planner->addChild(Local_Planner_Color_Scheme);
    ui.treeWidget->setItemWidget(Local_Planner_Color_Scheme,1,Local_Planner_Color_box);

    //set start pose
    connect(ui.set_start_btn,SIGNAL(clicked()),this,SLOT(slot_set_start_pose()));
    connect(ui.set_goal_btn,SIGNAL(clicked()),this,SLOT(slot_set_goal_pose()));
    connect(ui.set_return_pos_btn,SIGNAL(clicked()),this,SLOT(slot_set_return_pos()));
    connect(ui.return_btn,SIGNAL(clicked()),this,SLOT(slot_return()));
    connect(&qnode,SIGNAL(position(double,double,double)),this,SLOT(slot_update_pos(double,double,double)));
}

void MainWindow::initGraphicsView()
{
 //时间动态显示
  m_timerCurrentTime = new QTimer;
  m_timerCurrentTime->setInterval(100);
  m_timerCurrentTime->start();
  // ui.centralwidget->hide();
  //视图场景加载
  m_qgraphicsScene =
      new QGraphicsScene;  //要用QGraphicsView就必须要有QGraphicsScene搭配着用
  m_qgraphicsScene->clear();
  //创建item
  m_roboItem = new roboItem();
  //视图添加item
  m_qgraphicsScene->addItem(m_roboItem);
  //设置item的坐标原点与视图的原点重合（默认为视图中心）
  // widget添加视图
  ui.mapViz->setScene(m_qgraphicsScene);

  ui.horizontalLayout_4->setSpacing(0);
  ui.horizontalLayout_4->setMargin(0);
}

void MainWindow::connections()
{
    connect(&qnode, SIGNAL(updateMap(QImage)), m_roboItem,
          SLOT(paintMaps(QImage)));
  connect(&qnode, SIGNAL(plannerPath(QPolygonF)), m_roboItem,
          SLOT(paintPlannerPath(QPolygonF)));
  connect(&qnode, SIGNAL(updateRoboPose(RobotPose)), m_roboItem,
          SLOT(paintRoboPos(RobotPose)));
  connect(&qnode, SIGNAL(updateLaserScan(QPolygonF)), m_roboItem,
          SLOT(paintLaserScan(QPolygonF)));
  connect(m_roboItem, SIGNAL(cursorPos(QPointF)), this,
          SLOT(slot_updateCursorPos(QPointF)));
  connect(m_roboItem,SIGNAL(signalPub2DPose(QPointF,QPointF)),&qnode,SLOT(pub2DPose(QPointF,QPointF)));
  connect(m_roboItem,SIGNAL(signalPub2DGoal(QPointF,QPointF)),&qnode,SLOT(pub2DGoal(QPointF,QPointF)));
  //设置2D goal
  connect(ui.set_goal_btn, SIGNAL(clicked()), m_roboItem, SLOT(slot_set2DGoal()));
  connect(this, SIGNAL(signalSetMoveCamera()), m_roboItem,
          SLOT(slot_setMoveCamera()));

   connect(m_roboItem,SIGNAL(signalSendPos(QPointF)),this,SLOT(slot_updateRoboPos(QPointF)));
   connect(this,SIGNAL(signal_robotstatus(float,float,float)),&qnode,SLOT(slot_robotstatus(float,float,float)));
}

void MainWindow::mousePressEvent(QMouseEvent *event)
{
    m_lastPos = event->globalPos();
    isPressedWidget = true;  // 当前鼠标按下的即是QWidget而非界面上布局的其它控件
}

void MainWindow::mouseMoveEvent(QMouseEvent *event)
{
    if (isPressedWidget) {
    this->move(this->x() + (event->globalX() - m_lastPos.x()),
               this->y() + (event->globalY() - m_lastPos.y()));
    m_lastPos = event->globalPos();
  }
}

void MainWindow::mouseReleaseEvent(QMouseEvent *event)
{
    // 其实这里的mouseReleaseEvent函数可以不用重写
    m_lastPos = event->globalPos();
    isPressedWidget = false;  // 鼠标松开时，置为false
}


void MainWindow::keyPressEvent(QKeyEvent *event)
{

     if(event->modifiers() == Qt::ShiftModifier){ // 是否按下Ctrl键
         ui.textEdit_keyboard->clear();
        ui.textEdit_keyboard->setText(QKeySequence(event->key()).toString());
        qDebug()<<QKeySequence(event->key()).toString()<<endl;
    }
    else QWidget::keyPressEvent(event);
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
            ui.treeWidget->setEnabled(true);
            myrviz = new qrviz(ui.layout_rviz);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
            ui.treeWidget->setEnabled(true);
            myrviz = new qrviz(ui.layout_rviz);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
    //ui.line_edit_topic->setEnabled(enabled);
}

void MainWindow::slot_treewidget_value_change(QString)
{
    myrviz->Set_FixedFrame(fixed_box->currentText());
}

void MainWindow::slot_display_grid(int state)
{
    qDebug()<<"state"<<state<<endl;
    bool enable=state>1?true:false;
    QStringList qli=Grid_Color_Box->currentText().split(";");
    QColor color=QColor(qli[0].toInt(),qli[1].toInt(),qli[2].toInt());
    myrviz->Display_Grid(Cell_Count_Box->text().toInt(),color,enable);
}

void MainWindow::slot_display_tf(int state)
{
    bool enable=state>1?true:false;
    myrviz->Display_TF(enable);
}

void MainWindow::slot_display_laser(int state)
{
    bool enable=state>1?true:false;
    myrviz->Display_LaserScan(Laser_Topic_box->currentText(),enable);
}

void MainWindow::slot_display_RobotModel(int state)
{
    bool enable=state>1?true:false;
    myrviz->Display_RobotModel(enable);
}

void MainWindow::slot_display_Map(int state)
{
    bool enable=state>1?true:false;
    myrviz->Display_Map(Map_Topic_box->currentText(),Map_Color_Scheme_box->currentText(),enable);

}

void MainWindow::slot_display_Path(int state)
{
    bool enable=state>1?true:false;
    QStringList qli=Path_Color_box->currentText().split(";");
    QColor color=QColor(qli[0].toInt(),qli[1].toInt(),qli[2].toInt());
    myrviz->Display_Path(Path_Topic_box->currentText(),color,enable);
}

void MainWindow::slot_set_start_pose()
{
    myrviz->Set_Start_Pose();
}

void MainWindow::slot_set_goal_pose()
{
    myrviz->Set_Goal_Pose();
}

void MainWindow::slot_display_local_map(int state)
{
    bool enable=state>1?true:false;
    QStringList qli=Local_Planner_Color_box->currentText().split(";");
    QColor color=QColor(qli[0].toInt(),qli[1].toInt(),qli[2].toInt());
    myrviz->Display_Local_Map(Local_CostMap_Topic_box->currentText(),LocalMapColorScheme_box->currentText(),Local_Planner_Topic_box->currentText(),color,enable);
}

void MainWindow::slot_display_global_map(int state)
{
    bool enable=state>1?true:false;
    QStringList qli=Global_Planner_Color_box->currentText().split(";");
    QColor color=QColor(qli[0].toInt(),qli[1].toInt(),qli[2].toInt());
    myrviz->Display_Global_Map(Global_CostMap_Topic_box->currentText(),GlobalMapColorScheme_box->currentText(),Global_Planner_Topic_box->currentText(),color,enable);
}

void MainWindow::slot_update_pos(double x, double y, double z)
{
    ui.pos_x->setText(QString::number(x));
    ui.pos_y->setText(QString::number(y));
    ui.pos_z->setText(QString::number(z));
}

void MainWindow::slot_set_return_pos()
{
    ui.return_x->setText(ui.pos_x->text());
    ui.return_y->setText(ui.pos_y->text());
    ui.return_z->setText(ui.pos_z->text());
}

void MainWindow::slot_return()
{
    // qnode.set_goal(ui.return_x->text().toDouble(),ui.return_y->text().toDouble(),ui.return_z->text().toDouble());
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "taurusnavqt");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "taurusnavqt");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

void MainWindow::slot_pubImageMapTimeOut() {
  QImage image(600, 600, QImage::Format_RGB888);
  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing);
  m_qgraphicsScene->render(&painter);
  qnode.pub_imageMap(image);
}

void MainWindow::slot_updateCursorPos(QPointF pos)
{
    QPointF mapPos = qnode.transScenePoint2Word(pos);
    ui.label_pos_map->setText("x: " + QString::number(mapPos.x()).mid(0, 4) +
                            "  y: " + QString::number(mapPos.y()).mid(0, 4));
    ui.label_pos_scene->setText("x: " + QString::number(pos.x()).mid(0, 4) +
                              "  y: " + QString::number(pos.y()).mid(0, 4));
}
void MainWindow::slot_updateRobotStatus(RobotStatus)
{

}
void MainWindow::slot_updateRoboPos(QPointF scenePos)
{
    QPointF worldPos = qnode.transScenePoint2Word(scenePos);
    qDebug()<<"robopos"<<worldPos.x()<<worldPos.y()<<endl;
    ui.textEdit_tgx->setText(QString::number(worldPos.x()).mid(0, 4));
    ui.textEdit_tgy->setText(QString::number(worldPos.y()).mid(0, 4));
}
} // namespace taurusnavqt

void taurusnavqt::MainWindow::on_pB_send_clicked()
{
    emit signal_robotstatus(ui.textEdit_keyboard->toPlainText().toFloat(),
                           ui.textEdit_tgx->toPlainText().toFloat(),
                           ui.textEdit_tgy->toPlainText().toFloat());
}
