/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/launchgui_yj/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

QImage qt_image_screenshot;

namespace launchgui_yj {

    /*****************************************************************************
    ** Local variables
    *****************************************************************************/

    extern int ros_topic_data;
    extern bool ros_status_flag;
    extern bool ros_cmd_flag;

    extern QString q_command_string;

    // State[0]: , [1]:MD_driver , [2]: , [3]: , [4]: , [5]: ,
    int State[10];
    int Arm_State[5];
    int Ready[2];
    QImage qt_front_image;
    QImage qt_gripper_image;

    /*****************************************************************************
    ** Topic msg declaration
    *****************************************************************************/
    // QNode Publish msgs
    std_msgs::UInt16 exec_pkg_msg;
    std_msgs::String cmd_msg;

    /*****************************************************************************
    ** Implementation
    *****************************************************************************/

    QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
    {}

    QNode::~QNode() {
        if (ros::isStarted()) {
            ros::shutdown();    // explicitly needed since we use ros::start();
            ros::waitForShutdown();
        }
        wait();
    }

    bool QNode::init() {
        ros::init(init_argc, init_argv, "launchgui_yj");
        if ( ! ros::master::check() ) {
            return false;
        }
        ros::start(); // explicitly needed since our nodehandle is going out of scope.
        ros::NodeHandle n;

        /*****************************************************************************
        ** ROS communication definition
        *****************************************************************************/
        // Publisher
        Exec_pkg_publisher = n.advertise<std_msgs::UInt16>("exec_pkg", 1);
        Command_publisher = n.advertise<std_msgs::String>("gui_terminal_command", 1);

        // Subscriber
        NUC1_getReady_subscriber = n.subscribe("get_guiCmd_ready1", 100, &QNode::NUC1_getReady_Callback, this);
        NUC2_getReady_subscriber = n.subscribe("get_guiCmd_ready2", 100, &QNode::NUC2_getReady_Callback, this);

        Joystick_state_subscriber = n.subscribe("ros_joystick_state", 1000, &QNode::Joystick_state_Callback, this);
        MD_driver_state_subscriber = n.subscribe("md_driver_state", 1000, &QNode::MD_driver_state_Callback, this);
        Arm_Ctrl_state_subscriber = n.subscribe("arm_ctrl_state", 1000, &QNode::Arm_Ctrl_state_Callback, this);
        Cam_CtrlDXL_state_subscriber = n.subscribe("cam_ctrlDxl_state", 1000, &QNode::Cam_CtrlDXL_state_Callback, this);
        FrontImage_subscriber = n.subscribe("/usb_cam/image_raw/compressed", 1000, &QNode::FrontImage_Callback, this);
        Web_vd_server_state_subscriber = n.subscribe("web_vd_server_state", 1000, &QNode::Web_vd_server_state_Callback, this);

        // Sc
        Screenshot_subscriber = n.subscribe("/screenshot/image_raw", 1000, &QNode::Screenshot_Callback, this);

        start();
        return true;
    }

    bool QNode::init(const std::string &master_url, const std::string &host_url) {
        std::map<std::string,std::string> remappings;
        remappings["__master"] = master_url;
        remappings["__hostname"] = host_url;
        ros::init(remappings,"launchgui_yj");
        if ( ! ros::master::check() ) {
            return false;
        }
        ros::start(); // explicitly needed since our nodehandle is going out of scope.
        ros::NodeHandle n;

        /*****************************************************************************
        ** ROS communication definition
        *****************************************************************************/
        // Publisher
        Exec_pkg_publisher = n.advertise<std_msgs::UInt16>("exec_pkg", 1);
        Command_publisher = n.advertise<std_msgs::String>("gui_terminal_command", 1);

        // Subscriber
        NUC1_getReady_subscriber = n.subscribe("get_guiCmd_ready1", 100, &QNode::NUC1_getReady_Callback, this);
        NUC2_getReady_subscriber = n.subscribe("get_guiCmd_ready2", 100, &QNode::NUC2_getReady_Callback, this);

        Joystick_state_subscriber = n.subscribe("ros_joystick_state", 1000, &QNode::Joystick_state_Callback, this);
        MD_driver_state_subscriber = n.subscribe("md_driver_state", 1000, &QNode::MD_driver_state_Callback, this);
        Arm_Ctrl_state_subscriber = n.subscribe("arm_ctrl_state", 1000, &QNode::Arm_Ctrl_state_Callback, this);
        Cam_CtrlDXL_state_subscriber = n.subscribe("cam_ctrlDxl_state", 1000, &QNode::Cam_CtrlDXL_state_Callback, this);
        FrontImage_subscriber = n.subscribe("/usb_cam/image_raw/compressed", 1000, &QNode::FrontImage_Callback, this);
        Web_vd_server_state_subscriber = n.subscribe("web_vd_server_state", 1000, &QNode::Web_vd_server_state_Callback, this);

        // Sc
        Screenshot_subscriber = n.subscribe("/screenshot/image_raw", 1000, &QNode::Screenshot_Callback, this);

        start();
        return true;
    }

    void QNode::run() {
        ros::Rate loop_rate(30);
        ros::NodeHandle n;

        /*****************************************************************************
        ** ROS subscriber -Subscribing-
        *****************************************************************************/
        NUC1_getReady_subscriber = n.subscribe("get_guiCmd_ready1", 100, &QNode::NUC1_getReady_Callback, this);
        NUC2_getReady_subscriber = n.subscribe("get_guiCmd_ready2", 100, &QNode::NUC2_getReady_Callback, this);

        Joystick_state_subscriber = n.subscribe("ros_joystick_state", 1000, &QNode::Joystick_state_Callback, this);
        MD_driver_state_subscriber = n.subscribe("md_driver_status", 1000, &QNode::MD_driver_state_Callback, this);
        Arm_Ctrl_state_subscriber = n.subscribe("arm_ctrl_state", 1000, &QNode::Arm_Ctrl_state_Callback, this);
        Cam_CtrlDXL_state_subscriber = n.subscribe("cam_ctrlDxl_state", 1000, &QNode::Cam_CtrlDXL_state_Callback, this);
        FrontImage_subscriber = n.subscribe("/usb_cam/image_raw/compressed", 1000, &QNode::FrontImage_Callback, this);
        Web_vd_server_state_subscriber = n.subscribe("web_vd_server_state", 1000, &QNode::Web_vd_server_state_Callback, this);

        //Sc
        Screenshot_subscriber = n.subscribe("/screenshot/image_raw", 1000, &QNode::Screenshot_Callback, this);

        /*****************************************************************************
        ** ROS publisher -Publish-
        *****************************************************************************/

        int count = 0;
        while ( ros::ok() ) {
            /*
            std_msgs::String msg;
            std::stringstream ss;
            ss << "hello world " << count;
            msg.data = ss.str();
            chatter_publisher.publish(msg);
            log(Info,std::string("I sent: ")+msg.data);
            ros::spinOnce();
            loop_rate.sleep();
            ++count;*/

            // "Execute package" publish
            if (ros_status_flag == true) {
                exec_pkg_msg.data = ros_topic_data;
                Exec_pkg_publisher.publish(exec_pkg_msg);
                ros_status_flag = false;
            }
            // "GUI terminal command input" publish
            if (ros_cmd_flag == true) {
                cmd_msg.data = q_command_string.toStdString();
                Command_publisher.publish(cmd_msg);
                ros_cmd_flag = false;
            }
            // LED 신호는 실행된 패키지에서 각각 발행된 state 토픽에 의해 켜짐.
            // 매 순간 패키지 실행 상태를 확인할 수 있어야 하기에 주기마다 blackout 필요.
            blackout(0);

            ros::spinOnce();
            loop_rate.sleep();
        }

        State[0] = 0;

        std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
        Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
    }

    /*****************************************************************************
    ** ROS Callback function definition
    *****************************************************************************/
    // get_guicmd가 GUI의 명령에 반응 메시지를 발행했을 때, 이 상태 메시지state_msg를 받아 실행되는 콜백 함수들.

    void QNode::NUC1_getReady_Callback(const std_msgs::UInt16& ready) {
        Ready[0] = ready.data;
        ROS_INFO("NUC1 get_guicmd ready!");
        Q_EMIT statusUpdated();
    }

    void QNode::NUC2_getReady_Callback(const std_msgs::UInt16& ready) {
        Ready[1] = ready.data;
        ROS_INFO("NUC2 get_guicmd ready!");
        Q_EMIT statusUpdated();
    }

    void QNode::Joystick_state_Callback(const std_msgs::UInt16& state_msg) {
        State[1] = state_msg.data;
        Q_EMIT statusUpdated();
    }

    void QNode::MD_driver_state_Callback(const std_msgs::UInt16& state_msg) {
        State[2] = state_msg.data;
        Q_EMIT statusUpdated();
    }

    void QNode::Arm_Ctrl_state_Callback(const std_msgs::UInt16& state_msg) {
        State[3] = state_msg.data;
        Q_EMIT statusUpdated();
    }

    void QNode::Cam_CtrlDXL_state_Callback(const std_msgs::UInt16& state_msg) {
        State[4] = state_msg.data;
        Q_EMIT statusUpdated();
    }

    void QNode::FrontImage_Callback(const sensor_msgs::CompressedImage::ConstPtr& front_img) {
        State[5] = 1;
        cv::Mat image;
        try {
            image = cv::imdecode(cv::Mat(front_img->data), 1);
            //cv::flip(image, image, -1);
            cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
            //cv::imshow("Compressed Image", image);
            //cv::waitKey(1);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat frame = image;
        int frame_width = 640;
        int frame_height = 480;
        qt_front_image = QImage((const unsigned char*)(frame.data), frame.cols, frame.rows, QImage::Format_RGB888).scaled\
            (frame_width, frame_height, Qt::KeepAspectRatio, Qt::SmoothTransformation);
        Q_EMIT statusUpdated();
    }

    void QNode::Web_vd_server_state_Callback(const std_msgs::UInt16& state_msg) {
        State[6] = state_msg.data;
        Q_EMIT statusUpdated();
    }

    void QNode::blackout(int a) {
        for (int i = 0; i < 10; i++)
            State[i] = a;
        for (int i = 0; i < 5; i++)
            Arm_State[i] = a;

        Q_EMIT statusUpdated();
    }

    void QNode::Screenshot_Callback(const sensor_msgs::Image& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        cv::Mat main_frame = cv_ptr->image;
        int WIDTH = 320 * 3;
        int HEIGHT = 180 * 3;
        qt_image_screenshot = QImage((const unsigned char *)(main_frame.data), main_frame.cols, main_frame.rows, QImage::Format_RGB888).scaled(WIDTH, HEIGHT, Qt::KeepAspectRatio, Qt::SmoothTransformation);
        Q_EMIT statusUpdated_Sc();
    }

    void QNode::log( const LogLevel &level, const std::string &msg) {
        logging_model.insertRows(logging_model.rowCount(),1);
        std::stringstream logging_model_msg;
        switch ( level ) {
            case(Debug) : {
                ROS_DEBUG_STREAM(msg);
                logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
                break;
            }
            case(Info) : {
                ROS_INFO_STREAM(msg);
                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
                break;
            }
            case(Warn) : {
                ROS_WARN_STREAM(msg);
                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
                break;
            }
            case(Error) : {
                ROS_ERROR_STREAM(msg);
                logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
                break;
            }
            case(Fatal) : {
                ROS_FATAL_STREAM(msg);
                logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
                break;
            }
        }
        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
        //Q_EMIT loggingUpdated(); // used to readjust the scrollbar
    }

}  // namespace launchgui_yj
