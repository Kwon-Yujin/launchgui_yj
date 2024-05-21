/**
 * @file /include/launchgui_yj/main_window.hpp
 *
 * @brief Qt based gui for launchgui_yj.
 *
 * @date November 2010
 **/
#ifndef launchgui_yj_MAIN_WINDOW_H
#define launchgui_yj_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include <QPixmap>
#include <QWidget>

#include "ui_main_window.h"
#include "qnode.hpp"
#include "sc_dialog.hpp"

#include <stdlib.h>
#include <string>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace launchgui_yj {

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

        void ReadSettings();    // Load up qt program settings at startup
        void WriteSettings();   // Save qt program settings when closing

        void closeEvent(QCloseEvent *event);    // Overloaded function
        void showNoMasterMessage();
        //void showButtonTestMessage();

    public Q_SLOTS:
        /******************************************
        ** Auto-connections (connectSlotsByName())
        *******************************************/
        void on_actionAbout_triggered();
        void on_button_connect_clicked(bool check);
        //void on_button_test_clicked(bool check);
        void on_checkbox_use_environment_stateChanged(int state);

        /******************************************
        ** Manual connections
        *******************************************/
        void updateLoggingView();   // no idea why this can't connect automatically
        //void moveLeft();
        //void moveRight();

        /******************************************
        ** QNode Update
        *******************************************/
        void updateState();     // GUI signal update function
        void updateState_Sc();
        void getReady();

        /******************************************
        ** SLOTS of DroK5
        *******************************************/
        // Communication with Robot PC
        void Html();
        //void Html_Chrome();
        void Connect_PC_websocket();
        void Edit_html();

        // Start and Stop
        void Start();
        void All_stop();

        // Package execute
        void Joystick();
        void Joystick_OFF();
        void MD_driver();
        void MD_driver_OFF();
        void Arm_Ctrl();
        void Arm_Ctrl_OFF();
        void Cam_CtrlDXL();
        void Cam_CtrlDXL_OFF();
        void FrontCam_drive();
        void FrontCam_drive_OFF();
        void Web_vd_server();
        void Web_vd_server_OFF();

        // Sc
        void NUC1_screenshot_clicked(bool checked);
        void NUC2_screenshot_clicked(bool checked);

    private:
        Ui::MainWindowDesign ui;
        Sc_Dialog *dialog;
        QNode qnode;

        QPixmap m_lightimg[2];
        QPixmap m_readyimg[2];

        QPixmap KUDOS_img;
        QPixmap cat1_img;
        QPixmap cat2_img;

        QLineEdit qline;

  //QStringListModel* logging_model;
    };

    /*
    class Browser : public QMainWindow {
        Q_OBJECT
    public:
        Browser();

    private Q_SLOTS:
        void openBrowser();

    private:
        QWebView *web_view;
        QPushButton *button;
    };
    */

}  // namespace launchgui_yj

#endif // launchgui_yj_MAIN_WINDOW_H
