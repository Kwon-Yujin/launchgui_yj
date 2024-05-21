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
#include "../include/launchgui_yj/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace launchgui_yj {

    int ros_topic_data;
    bool ros_status_flag = 0;
    bool ros_cmd_flag = 0;

    QString q_command_string;

    extern int State[10];
    extern int Arm_State[5];
    extern int Ready[2];
    extern QImage qt_front_image;
    extern QImage qt_gripper_image;

    using namespace Qt;

    /*****************************************************************************
    ** Implementation [MainWindow]
    *****************************************************************************/

    MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv) {
        ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

        dialog = new Sc_Dialog;

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

        /*******************************
        ** QNode Update event
        ********************************/
        QObject::connect(&qnode, SIGNAL(statusUpdated()), this, SLOT(updateState()));
        QObject::connect(&qnode, SIGNAL(statusUpdated_Sc()), this, SLOT(updateState_Sc()));
        QObject::connect(&qnode, SIGNAL(statusUpdated()), this, SLOT(getReady()));

        /************************************
        ** Execute pkg event - explicit way
        *************************************/
        //QObject::connect(ui.button_left, SIGNAL(clicked()), this, SLOT(moveLeft()));
        // 'getmission' pkg
        //QObject::connect(ui.Button_getmission, SIGNAL(clicked()), this, SLOT(launch_getmission()));

        // Communication
        // 3. Web server available - 양방향 통신 성공 여부 확인
        QObject::connect(ui.Button_Html, SIGNAL(clicked()), this, SLOT(Html()));
        //QObject::connect(ui.Button_Html_Chrome, SIGNAL(clicked()), this, SLOT(Html_Chrome()));
        // 2. Execute rosbridge_websocket.launch
        QObject::connect(ui.Button_Connect_PC_websocket, SIGNAL(clicked()), this, SLOT(Connect_PC_websocket()));
        // 1. Execute .html websocket server file editor
        QObject::connect(ui.Button_Edit_html, SIGNAL(clicked()), this, SLOT(Edit_html()));

        // Start and Stop
        QObject::connect(ui.Button_Start, SIGNAL(clicked()), this, SLOT(Start()));
        QObject::connect(ui.Button_All_stop, SIGNAL(clicked()), this, SLOT(All_stop()));

        // #1 Joystick pkg
        QObject::connect(ui.Button_Joystick, SIGNAL(clicked()), this, SLOT(Joystick()));
        QObject::connect(ui.Button_Joystick_OFF, SIGNAL(clicked()), this, SLOT(Joystick_OFF()));

        // #2 MD driver pkg
        QObject::connect(ui.Button_MD_driver, SIGNAL(clicked()), this, SLOT(MD_driver()));
        QObject::connect(ui.Button_MD_driver_OFF, SIGNAL(clicked()), this, SLOT(MD_driver_OFF()));

        // #3 Manipulator pkg
        QObject::connect(ui.Button_Arm_Ctrl, SIGNAL(clicked()), this, SLOT(Arm_Ctrl()));
        QObject::connect(ui.Button_Arm_Ctrl_OFF, SIGNAL(clicked()), this, SLOT(Arm_Ctrl_OFF()));

        // #4 Dynamixel that control front camera pose
        QObject::connect(ui.Button_Cam_CtrlDXL, SIGNAL(clicked()), this, SLOT(Cam_CtrlDXL()));
        QObject::connect(ui.Button_Cam_CtrlDXL_OFF, SIGNAL(clicked()), this, SLOT(Cam_CtrlDXL_OFF()));

        // #5 Front camera drive
        QObject::connect(ui.Button_FrontCam_drive, SIGNAL(clicked()), this, SLOT(FrontCam_drive()));
        QObject::connect(ui.Button_FrontCam_drive_OFF, SIGNAL(clicked()), this, SLOT(FrontCam_drive_OFF()));

        // #6 Web_video_server: NUC에서 web_video_server pkg 시작
        QObject::connect(ui.Button_web_vd_server, SIGNAL(clicked()), this, SLOT(Web_vd_server()));
        QObject::connect(ui.Button_web_vd_server_OFF, SIGNAL(clicked()), this, SLOT(Web_vd_server_OFF()));

        // #7 Run RVIZ
        //QObject::connect(ui.Button_Velodyne_Lidar, SIGNAL(clicked()), this, SLOT(Velodyne_Lidar()));
        //QObject::connect(ui.Button_Velodyne_Lidar_OFF, SIGNAL(clicked()), this, SLOT(Velodyne_Lidar_OFF()));

        /*********************
        ** Label
        **********************/
        m_lightimg[0].load(":/images/led-off.png");     // 0: OFF
        m_lightimg[1].load(":/images/led-on.png");      // 1: ON

        m_readyimg[0].load(":/images/switch2.jpg");     // 0: OFF
        m_readyimg[1].load(":/images/switch1.jpg");     // 1: ON (Green-colored switch)

        // Banner image below the window-KUDOS-
        //KUDOS_img.load(":/images/KUDOS2.png");
        cat1_img.load(":/images/cat.jpg");
        //cat2_img.load(":/images/cat2.jpg");

        // 각 사진을 UI 레이블에 표시
        //ui.label->setPixmap(cat1_img);
        //ui.Label_FrontImage->setPixmap(cat1_img);
        //ui.label_2->setPixmap(KUDOS_img);

        //QPixmap cat1_img(":/images/cat.jpg");
        ui.Label_FrontImage->setPixmap(cat1_img);

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

    /*
    void MainWindow::showButtonTestMessage() {
        QMessageBox msgBox;
        msgBox.setText("Button test ...");
        msgBox.exec();
        //close();
    }
    */

    /*
     * These triggers whenever the button is clicked, regardless of whether it
     * is already checked or not.
     */

    void MainWindow::on_button_connect_clicked(bool check) {
            if ( ui.checkbox_use_environment->isChecked() ) {
                if ( !qnode.init() ) {
                    showNoMasterMessage();
                } else {
                    ui.button_connect->setEnabled(false);
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
            }
        }
    }

    /*
    void MainWindow::on_button_test_clicked(bool check ) {
        showButtonTestMessage();
    }
    */

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

    /*
    void MainWindow::moveLeft() {
        logging_model = qnode.loggingModel();
        logging_model->insertRows(logging_model->rowCount(), 1);
        std::stringstream logging_model_msg;
        logging_model_msg << "move to left ...";
        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model->setData(logging_model->index(logging_model->rowCount()-1), new_row);

        std::cout << logging_model->rowCount() << std::endl;
        std::cout << logging_model_msg.str().c_str() << std::endl;
    }
    */

    // 모든 패키지의 status 항목을 취합해 스위치 상태를 업데이트하는 함수가 updateState() 함수이다.
    // get_guicmd에서 GUI 상 발행된 ros_topic_data 메시지를 받아 응답한 경우, LED-on 상태로 돌아감.
    void MainWindow::updateState()
    {
        // Joystick pkg enabled or disabled status
        if (State[1] == 1)
            ui.Label_State_1->setPixmap(m_lightimg[1]);
        else
            ui.Label_State_1->setPixmap(m_lightimg[0]);

        // MD_driver pkg status
        if (State[2] == 1)
            ui.Label_State_2->setPixmap(m_lightimg[1]);
        else
            ui.Label_State_2->setPixmap(m_lightimg[0]);

        // Manipulator control pkg status
        if (State[3] == 1)
            ui.Label_State_3->setPixmap(m_lightimg[1]);
        else
            ui.Label_State_3->setPixmap(m_lightimg[0]);

        // Cam dynamixel control pkg status
        if (State[4] == 1)
            ui.Label_State_4->setPixmap(m_lightimg[1]);
        else
            ui.Label_State_4->setPixmap(m_lightimg[0]);

        // Webcam drive pkg status
        if (State[5] == 1)
            ui.Label_State_5->setPixmap(m_lightimg[1]);
        else
            ui.Label_State_5->setPixmap(m_lightimg[0]);

        // web_video_server pkg status
        if (State[6] == 1)
            ui.Label_State_6->setPixmap(m_lightimg[0]);
        else
            ui.Label_State_6->setPixmap(m_lightimg[1]);

        // Velodyne lidar pkgs status
        //if (State[7] == 1)
        //    ui.Label_State_7->setPixmap(m_lightimg[1]);
        //else
        //    ui.Label_State_7->setPixmap(m_lightimg[0]);

        // Sending front camera's video out
        ui.Label_FrontImage->setPixmap(QPixmap::fromImage(qt_front_image));
        ui.Label_FrontImage->resize(ui.Label_FrontImage->pixmap()->size());
    }

    void MainWindow::updateState_Sc() {
        dialog->setWindowTitle("NUC Screen");
        dialog->show(); // add
        dialog->show_screenshot(); // add
    }

    // GUI signal(graphical/visible) update function
    void MainWindow::getReady() {
        if (Ready[1] == 1)
            ui.Label_Get_ready->setPixmap(m_readyimg[1]);
        else
            ui.Label_Get_ready->setPixmap(m_readyimg[0]);
    }

    /*****************************************************************************
    ** Implemenation for topic publishment [Slots][manually connected]
    *****************************************************************************/
    // Communication
    void MainWindow::Html()
    {
        //ROS_INFO("Html");
        //std::string command_html = "gnome-terminal -- firefox ~/catkin_ws/src/roslibjs/examples/HW_test_server_0927_1.html --new-window --width=1080 --height=720";
        //std::string command_html = "gnome-terminal -- google-chrome --new-window --width=1080 --height=720 ~/catkin_ws/src/roslibjs/examples/HW_test_server_0927_1.html";
        std::string command_html = "gnome-terminal -- google-chrome --new-window --width=1080 --height=720 ~/catkin_ws/src/roslibjs/examples/HW_test_server.html";
        const char *c_html = command_html.c_str();
        system(c_html);
    }

    void MainWindow::Connect_PC_websocket()
    {
        std::string command_web = "gnome-terminal -- roslaunch rosbridge_server rosbridge_websocket.launch";
        const char *c_web = command_web.c_str();
        system(c_web);
    }

    void MainWindow::Edit_html()
    {
        //std::string command_edit = "gedit ~/catkin_ws/src/roslibjs/examples/HW_test_server_0927_1.html";
        std::string command_edit = "gedit ~/catkin_ws/src/roslibjs/examples/HW_test_server.html";
        const char *c_edit = command_edit.c_str();
        system(c_edit);
    }

    // Start and Stop
    void MainWindow::Start()
    {
        ros_topic_data = 0;
        ros_status_flag = true;
    }
    void MainWindow::All_stop() {
        ros_topic_data = 100;
        ros_status_flag = true;
    }

    // Joystick
    void MainWindow::Joystick()
    {
        ros_topic_data = 1;
        ros_status_flag = true;
    }
    void MainWindow::Joystick_OFF()
    {
        ros_topic_data = 101;
        ros_status_flag = true;
    }

    // MD driver
    void MainWindow::MD_driver()
    {
        ros_topic_data = 2;
        ros_status_flag = true;
    }
    void MainWindow::MD_driver_OFF()
    {
        ros_topic_data = 102;
        ros_status_flag = true;
    }

    // Manipulator control
    void MainWindow::Arm_Ctrl()
    {
        ros_topic_data = 3;
        ros_status_flag = true;
    }
    void MainWindow::Arm_Ctrl_OFF()
    {
        ros_topic_data = 103;
        ros_status_flag = true;
    }

    // Dynamixel control of camera pose
    void MainWindow::Cam_CtrlDXL()
    {
        ros_topic_data = 4;
        ros_status_flag = true;
    }
    void MainWindow::Cam_CtrlDXL_OFF()
    {
        ros_topic_data = 104;
        ros_status_flag = true;
    }

    // Front cam drive
    void MainWindow::FrontCam_drive()
    {
        ros_topic_data = 5;
        ros_status_flag = true;
    }
    void MainWindow::FrontCam_drive_OFF()
    {
        ros_topic_data = 105;
        ros_status_flag = true;
    }

    // web_video_server
    void MainWindow::Web_vd_server()
    {
        ros_topic_data = 6;
        ros_status_flag = true;
        //std::string command_html = "gnome-terminal -- firefox \"http://223.171.62.1:8080/stream?topic=/usb_cam/image_raw&type=ros_compressed\" --new-window --width=640 --height=480";
        std::string command_html = "gnome-terminal -- google-chrome --new-window --window-size=640,480 \"http://223.171.62.1:8080/stream?topic=/usb_cam/image_raw&type=ros_compressed\"";
        //std::string command_html = "gnome-terminal -- google-chrome --new-window --width=640 --height=480 --app=\"http://223.171.62.1:8080/stream?topic=/usb_cam/image_raw&type=ros_compressed\"";
        const char *c_html = command_html.c_str();
        system(c_html);
    }
    void MainWindow::Web_vd_server_OFF()
    {
        ros_topic_data = 106;
        ros_status_flag = true;
    }

    // Sceenshot
    void MainWindow::NUC1_screenshot_clicked(bool checked)
    {
        if(checked == true) {
            ros_topic_data = 1000;
            ros_status_flag = true;
            //dialog->show();   //add
        }
        else {
            dialog->close();    //add
        }
    }

    void MainWindow::NUC2_screenshot_clicked(bool checked)
    {
        if(checked == true){
            ros_topic_data = 2000;
            ros_status_flag = true;
            //dialog->show();   //add
        }
        else{
            dialog->close();    //add
        }
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
        QSettings settings("Qt-Ros Package", "launchgui_yj");
        restoreGeometry(settings.value("geometry").toByteArray());
        restoreState(settings.value("windowState").toByteArray());
        QString master_url = settings.value("master_url", QString("http://192.168.10.2:11311/")).toString();
        QString host_url = settings.value("host_url", QString("192.168.10.2")).toString();
        //QString master_url = settings.value("master_url",QString("http://10.60.3.186:11311/")).toString();
        //QString host_url = settings.value("host_url", QString("10.60.3.186")).toString();
        ////QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
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
        QSettings settings("Qt-Ros Package", "launchgui_yj");
        settings.setValue("master_url",ui.line_edit_master->text());
        settings.setValue("host_url",ui.line_edit_host->text());
        //settings.setValue("topic_name",ui.line_edit_topic->text());
        settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
        settings.setValue("geometry", saveGeometry());
        settings.setValue("windowState", saveState());
        settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
    }

    void MainWindow::closeEvent(QCloseEvent *event) {
        WriteSettings();
        QMainWindow::closeEvent(event);
    }

    /*
    Browser::Browser() {
        this->resize(640, 480);

        // Create a web view
        web_view = new QWebView(this);
        setCentralWidget(web_view);

        // Create a button
        button = new QPushButton("Open Browser", this);
        button->move(10, 10);

        // Connect the button click event to open the browser
        connect(button, SIGNAL(clicked()), this, SLOT(openBrowser()));
    }

    Browser::openBrowser() {
        // Load URL when the button is clicked
        QUrl url("http://www.naver.com");
        web_view->setFixedSize(640, 480);
        web_view->load(url);
    }
    */

}  // namespace launchgui_yj

