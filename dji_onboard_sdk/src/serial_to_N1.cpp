/*
 ============================================================================
 Name        : dji_sdk_node.c
 Author      : Ying Jiahang, Wu Yuwei
 Modify      : Gao Wenliang
 Version     :
 Copyright   : Your copyright notice
 Description :
 ============================================================================
*/
/*
here is the suggested procedure to operate the multirotor by DJI Onboard SDK.

-->[SDK or user] code to Read level1/level2 APP infomation
         from a .txt file or others. to load infomation about used uav.
         such as serial port info, onboard sdk APP info.

-->[SDK] code to Open serial port

-->[SDK] code to Activation
--->[SDK or user] code to ack activation

-->[SDK] code to Obtain control
--->[SDK or user] code to ack obtain control

//this is recommended to do in another thread
loop()
{
    -->[SDK] code to Get flight msg data from flight_ctrl
    --->[SDK or user] code to Send out flight msg (send to other thread
                 by ROS msg,by socket.....

    -->[SDK or user] code to Get control command (get from other thread
                by ROS msg,by socket.....
    --->[SDK] code to Send control command to N1
    ...
}
-->[SDK] code to Release control
*/

/* ROS */
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include <dji_onboard_sdk/uav_sensor_data.h>
#include <dji_onboard_sdk/uav_ctrl_data.h>
#include <dji_onboard_sdk/gimbal_ctrl_data.h>

/* SDK */
#include "dji_sdk_lib/DJI_Pro_Codec.h"
#include "dji_sdk_lib/DJI_Pro_Hw.h"
#include "dji_sdk_lib/DJI_Pro_Link.h"
#include "dji_sdk_lib/DJI_Pro_App.h"
#include "dji_sdk_lib/DJI_Pro_Rmu.h"

#include "dji_info_lib/dji_sdk_app_info.h"

/* MATH for_example */
#include <math.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <iostream>

/* parameter */
#define C_EARTH (double) 6378137.0
#define C_PI	(double) 3.141592653589793

using namespace ros;
using namespace std;

//This class is the suggested procedure
class dji_uav_onboard
{
public:

    int activiton_times;
    activate_data_t activation_msg;

    class dji_sdk_app_info *user_info;

    int loadUserInfo(std::string info);

    int openSerialPort();

    int setupFcLink();

    int activationFc();
    int AUTO_activation();

    int controlObtain();
    int controlRelease();

    int takeOffCtrl();
    int langCtrl();
    int goHomeCtrl();

private:
    /* activation param */
};

int dji_uav_onboard::loadUserInfo(std::string info)
{
    user_info->loadAppInfo(info);
    return 1;
}

int dji_uav_onboard::openSerialPort()
{
    char uart_name[32];
    strcpy(uart_name, (char *)user_info->serial_port.serial_name.c_str());

    int tmp_flag = Pro_Hw_Setup(uart_name,
                                user_info->serial_port.baud_rate);

    if (0!= tmp_flag)
    {
        cout<<BOLDRED<<"FAIL to open serial poart."<<RESETCOLOR<<endl;
        return 0;
    }
    else
        return 1;
}

int dji_uav_onboard::setupFcLink()
{
    //Setup flight ctrl link
    DJI_Pro_Setup(NULL);

    //Pro_Link_Setup();
    //Pro_App_Recv_Set_Hook(DJI_Pro_App_Recv_Req_Data);

    return 0;
}

int dji_uav_onboard::activationFc()
{
    char temp_buf[65];

    activation_msg.app_id 		= (uint32_t)user_info->app_info.app_id;
    activation_msg.app_api_level 	= (uint32_t)user_info->app_info.app_api_level;
    activation_msg.app_ver		= (uint32_t)user_info->app_info.app_version;
    memcpy(activation_msg.app_bundle_id, user_info->app_info.app_bundle_id.c_str(), 32);

    activation_msg.app_key = temp_buf;
    strcpy(activation_msg.app_key, user_info->app_info.app_enc_key.c_str());

    DJI_Pro_Activate_API(&activation_msg,NULL);
    return 1;
}

int dji_uav_onboard::AUTO_activation()
{
    cout<<"CALL AUTO_activation "<<activiton_times<<" times."<<endl;
    for(int i = 1; i<= activiton_times; i++)
    {
        activationFc();

        // soft time delay
        for(int jj = 0;jj<4096;jj++);
    }
    return 1;
}

int dji_uav_onboard::controlObtain()
{
    cout<<BLUE<<"Obtain control."<<RESETCOLOR<<endl;

    DJI_Pro_Control_Management(1,NULL);

    return 1;
}

int dji_uav_onboard::controlRelease()
{
    cout<<BLUE<<"Release control."<<RESETCOLOR<<endl;

    DJI_Pro_Control_Management(0,NULL);

    return 1;
}

int dji_uav_onboard::takeOffCtrl()
{
    DJI_Pro_Status_Ctrl(4,0);
    printf("[STATUS_CMD] send [TAKE OFF]\n");
}

int dji_uav_onboard::langCtrl()
{
    DJI_Pro_Status_Ctrl(6,0);
    printf("[STATUS_CMD] send [LAND]\n");
}

int dji_uav_onboard::goHomeCtrl()
{
    DJI_Pro_Status_Ctrl(1,0);
    printf("[STATUS_CMD] send [GO HOME]\n");
}

/* ros sub from serial */
ros::Subscriber obtain_ctrl_sub;
ros::Subscriber ctrl_data_sub;
ros::Subscriber flight_status_request_sub;
ros::Subscriber gimbal_ctrl_sub;
//ros::Subscriber activation_sub;

/* ros pub for webserver */
ros::Publisher obtain_ctrl_status_pub;
ros::Publisher sensor_pub;
//ros::Publisher activation_status_pub;

/* ros timer */
ros::Timer simple_task_timer;
std::string user_sdk_info;

dji_uav_onboard my_n1;

//-------------define type------------------

#define  STATUS_STAND_BY     1
#define  STATUS_TAKE_OFF     2
#define  STATUS_IN_AIR 	     3
#define  STATUS_LANDING      4
#define  STATUS_POST_LANDING 5
#define  STATUS_GO_HOME	     6

#define API_GO_HOME  1.0
#define API_TAKE_OFF 4.0
#define API_LANDING  6.0

void flight_status_request_callback(const std_msgs::Float32::ConstPtr& msg)
{
    float status_request_cmd;
    status_request_cmd = msg->data;

    if(status_request_cmd == API_GO_HOME)
    {
        my_n1.goHomeCtrl();
    }
    else
        if(status_request_cmd == API_TAKE_OFF)
        {
            my_n1.takeOffCtrl();
        }
        else
            if(status_request_cmd == API_LANDING)
            {
                my_n1.langCtrl();
            }
}

void ros_obtain_ctrl_callback(const std_msgs::Float32::ConstPtr& msg)
{
    int data = (uint8_t)msg->data;

    if(data == 1)
        my_n1.controlObtain();
    else
        if(data == 0)
            my_n1.controlRelease();
}

//-------------define type------------------

#define API_VERT_VEL 0
#define API_VERT_POS 1
#define API_VERT_TRU 2

#define API_HORI_ATTI   0
#define API_HORI_VEL    1
#define API_HORI_POS    2

#define API_YAW_ANG  0
#define API_YAW_RATE 1

#define GROUND_LEVEL 0
#define BODY_LEVEL   1
#define REF_LEVEL    2

#define GROUND_TORSION 0
#define BODY_TORSION   1

void ros_ctrl_data_callback(const dji_onboard_sdk::uav_ctrl_data::ConstPtr& msg)
{	
    attitude_data_t send_data = {0};

    if(msg->horiz_mode == API_HORI_ATTI)
    {
        send_data.ctrl_flag |= HORIZ_ATT;
    }
    else if(msg->horiz_mode == API_HORI_VEL)
    {
        send_data.ctrl_flag |= HORIZ_VEL;
    }
    else if(msg->horiz_mode == API_HORI_POS)
    {
        send_data.ctrl_flag |= HORIZ_POS;
    }

    if(msg->vert_mode == API_VERT_VEL)
    {
        send_data.ctrl_flag |= VERT_VEL;
    }
    else if(msg->vert_mode == API_VERT_POS)
    {
        send_data.ctrl_flag |= VERT_POS;
    }
    else if(msg->vert_mode == API_VERT_TRU)
    {
        send_data.ctrl_flag |= VERT_TRU;
    }

    if(msg->yaw_mode == API_YAW_ANG)
    {
        send_data.ctrl_flag |= YAW_ANG;
    }
    else if(msg->yaw_mode == API_YAW_RATE)
    {
        send_data.ctrl_flag |= YAW_RATE;
    }

    if(msg->level_frame == GROUND_LEVEL)
    {
        send_data.ctrl_flag |= HORIZ_GND;
    }
    else if(msg->level_frame == BODY_LEVEL)
    {
        send_data.ctrl_flag |= HORIZ_BODY;
    }

    if(msg->torsion_frame == GROUND_TORSION)
    {
        send_data.ctrl_flag |= YAW_GND;
    }
    else if(msg->torsion_frame == BODY_TORSION)
    {
        send_data.ctrl_flag |= YAW_BODY;
    }

    send_data.roll_or_x = msg->ctrl_data.roll_or_x;
    send_data.pitch_or_y = msg->ctrl_data.pitch_or_y;
    send_data.yaw = msg->ctrl_data.yaw;
    send_data.thr_z = msg->ctrl_data.thr_z;

    // DJI_Pro_Attitude_Control(&send_data);
    DJI_Pro_App_Send_Data(0,1, MY_CTRL_CMD_SET, API_CTRL_REQUEST,
                          (unsigned char *)&send_data,sizeof(send_data),
                          0,0,1);
}

#define CAMERA_STANDBY 0
#define CAMERA_SHOT 1
#define VIDIO_START 2
#define VIDIO_STOP 3
#define GIMBAL_ANGLE_CTRL 1
#define GIMBAL_SPEED_CTRL 2
void gimbal_request_callback(const dji_onboard_sdk::gimbal_ctrl_data::ConstPtr& msg)
{
    switch(msg->camera_ctrl)
    {
    case CAMERA_STANDBY:
        DJI_Pro_Camera_Control(API_CAMERA_VIDEO_STOP);
        break;
    case CAMERA_SHOT:
        DJI_Pro_Camera_Control(API_CAMERA_SHOT);
        break;
    case VIDIO_START:
        DJI_Pro_Camera_Control(API_CAMERA_VIDEO_START);
        break;
    case VIDIO_STOP:
        DJI_Pro_Camera_Control(API_CAMERA_VIDEO_STOP);
        break;
    default:;
        break;
    }

    switch (msg->gimbal_mode)
    {
    case GIMBAL_ANGLE_CTRL:
    {
        gimbal_custom_control_angle_t gimbal_angle_ctrl = {0};
        gimbal_angle_ctrl.pitch_angle = 10*msg->pitch;
        gimbal_angle_ctrl.roll_angle = 10*msg->roll;
        gimbal_angle_ctrl.yaw_angle = 10*msg->yaw;
        gimbal_angle_ctrl.duration = 10*msg->duration;
        /////////////////////////////////////////////////////////
        /// here need to calcu speed is less than 400 degree/s //
        /////////////////////////////////////////////////////////
        gimbal_angle_ctrl.ctrl_byte.base = 0;
        gimbal_angle_ctrl.ctrl_byte.yaw_cmd_ignore = 0;
        gimbal_angle_ctrl.ctrl_byte.roll_cmd_ignore = 0;
        gimbal_angle_ctrl.ctrl_byte.pitch_cmd_ignore = 0;

        DJI_Pro_App_Send_Data(0,0,MY_CTRL_CMD_SET,API_GIMBAL_CTRL_ANGLE_REQUEST,
                              (uint8_t*)&gimbal_angle_ctrl,sizeof(gimbal_angle_ctrl),
                              NULL,0,0);
        break;
    }
    case GIMBAL_SPEED_CTRL:
    {
        gimbal_custom_speed_t gimbal_speed_ctrl = {0};
        gimbal_speed_ctrl.yaw_angle_rate = 10*msg->pitch;
        gimbal_speed_ctrl.roll_angle_rate = 10*msg->roll;
        gimbal_speed_ctrl.pitch_angle_rate = 10*msg->yaw;
        gimbal_speed_ctrl.ctrl_byte.ctrl_switch = 1;
        /////////////////////////////////////////////////////////
        /// here need to calcu speed is less than 400 degree/s //
        /////////////////////////////////////////////////////////
        DJI_Pro_App_Send_Data(0,1,MY_CTRL_CMD_SET,API_GIMBAL_CTRL_SPEED_REQUEST,
                              (uint8_t*)&gimbal_speed_ctrl,sizeof(gimbal_speed_ctrl),
                              NULL,0,0);
        break;
    }
    default: ;
        break;
    }
}

/*
  * timer spin_function 50Hz
  */
static sdk_std_msg_t recv_sdk_std_msgs;
dji_onboard_sdk::uav_sensor_data sensorMsg;
void spin_callback(const ros::TimerEvent& e)
{
    // get msg from flight ctrl
    DJI_Pro_Get_Broadcast_Data(&recv_sdk_std_msgs);

    // publish ROS msgs about all flight data
    // ros time
    sensorMsg.header.stamp =ros::Time::now();
    // q_eb
    sensorMsg.orientation.q0 = recv_sdk_std_msgs.q.q0;
    sensorMsg.orientation.q1 = recv_sdk_std_msgs.q.q1;
    sensorMsg.orientation.q2 = recv_sdk_std_msgs.q.q2;
    sensorMsg.orientation.q3 = recv_sdk_std_msgs.q.q3;
    // linear_v_e
    sensorMsg.linear_v.v_x = recv_sdk_std_msgs.v.x; // m/s
    sensorMsg.linear_v.v_y = recv_sdk_std_msgs.v.y;
    sensorMsg.linear_v.v_z = recv_sdk_std_msgs.v.z;
    // linear_a_e
    sensorMsg.linear_a.a_x = recv_sdk_std_msgs.a.x; // m/s^2
    sensorMsg.linear_a.a_y = recv_sdk_std_msgs.a.y;
    sensorMsg.linear_a.a_z = recv_sdk_std_msgs.a.z;
    // angular_v_b
    sensorMsg.angular_v.w_x = recv_sdk_std_msgs.w.x; // degree/s
    sensorMsg.angular_v.w_y = recv_sdk_std_msgs.w.y;
    sensorMsg.angular_v.w_z = recv_sdk_std_msgs.w.z;
    // gps_e data
    sensorMsg.gps_pos.height = recv_sdk_std_msgs.pos.height; // m
    sensorMsg.gps_pos.lati = recv_sdk_std_msgs.pos.lati;    // rad
    sensorMsg.gps_pos.longti = recv_sdk_std_msgs.pos.longti;    // rad
    sensorMsg.gps_pos.alti = recv_sdk_std_msgs.pos.alti;    // m
    //cout<<"lati: "<<sensorMsg.gps_pos.lati<<" rad, "<<sensorMsg.gps_pos.lati*180.0f/C_PI<<" degree."<<endl;
    //cout<<"longti: "<<sensorMsg.gps_pos.longti<<" rad, "<<sensorMsg.gps_pos.longti*180.0f/C_PI<<" degree."<<endl;
    // gimbal
    sensorMsg.gimbal.roll = recv_sdk_std_msgs.gimbal.x;
    sensorMsg.gimbal.pitch = recv_sdk_std_msgs.gimbal.y;
    sensorMsg.gimbal.yaw = recv_sdk_std_msgs.gimbal.z;
    // status
    sensorMsg.status = recv_sdk_std_msgs.status;
    // battery
    sensorMsg.battery = recv_sdk_std_msgs.battery_remaining_capacity;   // %

    sensor_pub.publish(sensorMsg);
}


/*
  * main_function
  */
int main (int argc, char** argv)
{

    cout<<"DJI onboard sdk on ROS start run."<<endl;

    /* initialize ros */
    ros::init(argc, argv, "serial_to_N1");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // ros param set sdk info poart
    nh_private.param("user_sdk_info", user_sdk_info, std::string("/home/g/ws/src/dji_onboard_sdk/src/app1.txt"));
    // ros param about activiton times
    nh_private.param("activiton_times", my_n1.activiton_times, 2);

    /* start ros subscriber */
    obtain_ctrl_sub      = nh.subscribe("/nav_request", 10, ros_obtain_ctrl_callback);
    ctrl_data_sub		= nh.subscribe("/api_ctrl", 10, ros_ctrl_data_callback);
    flight_status_request_sub = nh.subscribe("/status_request", 1, flight_status_request_callback);
    gimbal_ctrl_sub = nh.subscribe("/gimbal_request",5,gimbal_request_callback);

    /* start ros publisher */
    obtain_ctrl_status_pub 	= nh.advertise<std_msgs::Float32>("/nav_status", 10);
    sensor_pub		= nh.advertise<dji_onboard_sdk::uav_sensor_data>("/uav_sensor", 10);
    //activation_status_pub   = nh.advertise<std_msgs::Float32>("/activation_status", 10);

    /* ros timer 50Hz */
    simple_task_timer 	= nh.createTimer(ros::Duration(1.0/50.0), spin_callback);

    // load user infomation
    my_n1.loadUserInfo(user_sdk_info);

    // open serial port & link to flight control
    my_n1.openSerialPort();

    // setup flight-control link
    my_n1.setupFcLink();

    // auto activation for times
    my_n1.AUTO_activation();

    /* ros spin for timer */
    ros::spin();

    my_n1.controlRelease();
    cout<<BOLDBLUE<<"DJI onboard sdk ctrl Done."<<RESETCOLOR<<endl;
    return 0;
}
