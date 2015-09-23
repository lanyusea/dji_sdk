/*
 ============================================================================
 Name        : dji_sdk_node.c
 Author      : Ying Jiahang, Wu Yuwei
 Modify      : Gao Wenliang
 Version     :
 Copyright   : Your copyright notice
 Description :
 ============================================================================

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

/* SDK */
#include "sdk_lib/DJI_Pro_Codec.h"
#include "sdk_lib/DJI_Pro_Hw.h"
#include "sdk_lib/DJI_Pro_Link.h"
#include "sdk_lib/DJI_Pro_App.h"
#include "sdk_lib/sdk_info_load.h"

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

#define NO_AUTHORITY 8

using namespace ros;
using namespace std;


/* ros sub from serial */
ros::Subscriber obtain_ctrl_sub;
ros::Subscriber ctrl_data_sub;
ros::Subscriber flight_status_request_sub;
//ros::Subscriber activation_sub;
/* ros pub for webserver */
ros::Publisher obtain_ctrl_status_pub;
ros::Publisher sensor_pub;
//ros::Publisher activation_status_pub;

/* ros timer */
ros::Timer simple_task_timer;

/* enc_key */
static char *enc_key;
/* req_id for nav closed by app msg */
static req_id_t nav_force_close_req_id = {0};
/* std msg from uav */
static sdk_std_msg_t recv_sdk_std_msgs = {0};

/* ros launch param */
std::string sdk_app_info;

dji_sdk_app_info sdk_app;

/* activation */
static activation_data_t activation_msg = {14,2,1,""};
int activiton_times;

/*
  * table of sdk req data handler
  */
int16_t sdk_std_msgs_handler(uint8_t cmd_id,uint8_t* pbuf,uint16_t len,req_id_t req_id);
int16_t	nav_force_close_handler(uint8_t cmd_id,uint8_t* pbuf,uint16_t len,req_id_t req_id);
/* cmd id table */
cmd_handler_table_t cmd_handler_tab[] = 
{
    {0x00,sdk_std_msgs_handler				},
    {0x01,nav_force_close_handler			},
    {ERR_INDEX,NULL							}
};
/* cmd set table */
set_handler_table_t set_handler_tab[] =
{
    {0x02,cmd_handler_tab					},
    {ERR_INDEX,NULL							}
};

// sdk_req_data_callback
int16_t nav_force_close_handler(uint8_t cmd_id,uint8_t* pbuf,uint16_t len,req_id_t req_id)
{
    if(len != sizeof(uint8_t))
        return -1;
    uint8_t msg;
    memcpy(&msg, pbuf, sizeof(msg));
    /* test session ack */
    nav_force_close_req_id.sequence_number = req_id.sequence_number;
    nav_force_close_req_id.session_id      = req_id.session_id;
    nav_force_close_req_id.reserve	       = 1;

    printf("WARNING nav close by app %d !!!!!!!!!!!!!! \n", msg);
    return 0;

}

#define _recv_std_msgs(_flag, _enable, _data, _buf, _datalen) \
    if( (_flag & _enable))\
{\
    memcpy((uint8_t *)&(_data),(uint8_t *)(_buf)+(_datalen), sizeof(_data));\
    _datalen += sizeof(_data);\
    }

int16_t sdk_std_msgs_handler(uint8_t cmd_id,uint8_t* pbuf,uint16_t len,req_id_t req_id)
{
    uint16_t *msg_enable_flag = (uint16_t *)pbuf;
    uint16_t data_len = MSG_ENABLE_FLAG_LEN;

    _recv_std_msgs( *msg_enable_flag, ENABLE_MSG_TIME	, recv_sdk_std_msgs.time_stamp		, pbuf, data_len);
    _recv_std_msgs( *msg_enable_flag, ENABLE_MSG_Q		, recv_sdk_std_msgs.q				, pbuf, data_len);
    _recv_std_msgs( *msg_enable_flag, ENABLE_MSG_A		, recv_sdk_std_msgs.a				, pbuf, data_len);
    _recv_std_msgs( *msg_enable_flag, ENABLE_MSG_V		, recv_sdk_std_msgs.v				, pbuf, data_len);
    _recv_std_msgs( *msg_enable_flag, ENABLE_MSG_W		, recv_sdk_std_msgs.w				, pbuf, data_len);
    _recv_std_msgs( *msg_enable_flag, ENABLE_MSG_POS	, recv_sdk_std_msgs.pos				, pbuf, data_len);
    _recv_std_msgs( *msg_enable_flag, ENABLE_MSG_MAG	, recv_sdk_std_msgs.mag				, pbuf, data_len);
    _recv_std_msgs( *msg_enable_flag, ENABLE_MSG_RC		, recv_sdk_std_msgs.rc				, pbuf, data_len);
    _recv_std_msgs( *msg_enable_flag, ENABLE_MSG_GIMBAL	, recv_sdk_std_msgs.gimbal			, pbuf, data_len);
    _recv_std_msgs( *msg_enable_flag, ENABLE_MSG_STATUS	, recv_sdk_std_msgs.status			, pbuf, data_len);
    _recv_std_msgs( *msg_enable_flag, ENABLE_MSG_BATTERY	, recv_sdk_std_msgs.battery_remaining_capacity	, pbuf, data_len);
    _recv_std_msgs( *msg_enable_flag, ENABLE_MSG_DEVICE	, recv_sdk_std_msgs.ctrl_device			, pbuf, data_len);

    return 0;
}


/* test cmd agency */
static uint8_t test_cmd_send_flag = 1;
static uint8_t test_cmd_is_resend = 0;
void cmd_callback_test_fun(uint16_t *ack)
{
    char result[6][50]={
        {"REQ_TIME_OUT"},
        {"REQ_REFUSE"},
        {"CMD_RECIEVE"},
        {"STATUS_CMD_EXECUTING"},
        {"STATUS_CMD_EXE_FAIL"},
        {"STATUS_CMD_EXE_SUCCESS"}};

    uint16_t recv_ack = *ack;
    printf("[DEBUG] recv_ack %#x \n", recv_ack);
    printf("[TEST_CMD] Cmd result: %s \n", *(result+recv_ack));
    test_cmd_send_flag = 1;
    if(recv_ack != STATUS_CMD_EXE_SUCCESS)
    {
        test_cmd_is_resend = 1;
    }

    /* for debug */
    if(recv_ack != STATUS_CMD_EXE_SUCCESS)
    {
        test_cmd_send_flag  = 0;
        printf("[ERROR] APP LAYER NOT STATUS_CMD_EXE_SUCCESS !!!!!!!!!!!!!!!!!!\n");
    }

} 


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
    uint8_t send_data = 0;

    if(status_request_cmd == API_GO_HOME)
    {
        send_data = 1;
    }
    else
        if(status_request_cmd == API_TAKE_OFF)
        {
            send_data = 4;
        }
        else
            if(status_request_cmd == API_LANDING)
            {
                send_data = 6;
            }

    App_Complex_Send_Cmd(send_data, cmd_callback_test_fun);
    printf("[TEST_CMD] send %d \n",send_data);
}


/* activation */
void activation_func_ack_cmd_callback(ProHeader *header)
{
    uint16_t ack_data;
    printf("Sdk_ack_cmd0_callback,sequence_number=%d,session_id=%d,data_len=%d\n", header->sequence_number, header->session_id, header->length - EXC_DATA_SIZE);
    memcpy((uint8_t *)&ack_data,(uint8_t *)&header->magic, (header->length - EXC_DATA_SIZE));

    if( is_sys_error(ack_data))
    {
        printf("[DEBUG] SDK_SYS_ERROR!!! \n");
        std_msgs::Float32 msg;
        msg.data = NO_AUTHORITY;
        //activation_status_pub.publish(msg);
    }
    else
    {
        char result[][50]={
            {"ACTIVATION_SUCCESS"},
            {"PARAM_ERROR"},
            {"DATA_ENC_ERROR"},
            {"NEW_DEVICE_TRY_AGAIN"},
            {"DJI_APP_TIMEOUT"},
            {" DJI_APP_NO_INTERNET"},
            {"SERVER_REFUSED"},
            {"LEVEL_ERROR"}};

        printf("[ACTIVATION] Activation result: %s \n", *(result+ack_data));
        std_msgs::Float32 msg;
        msg.data = (float)ack_data;
        //activation_status_pub.publish(msg);

        if(ack_data == 0)
        {
            Pro_Config_Comm_Encrypt_Key(enc_key);
            printf("[ACTIVATION] set key %s\n",enc_key);
        }
        else if(ack_data == 3)
        {
            /* new device, try again when activation is failed */
            alarm(2);
        }
    }
}

void activation_func(void)
{
    App_Send_Data( 2, 0,
                   MY_ACTIVATION_SET, API_USER_ACTIVATION,(uint8_t*)&activation_msg,sizeof(activation_msg),
                   activation_func_ack_cmd_callback,
                   1000, 1);
    printf("[ACTIVATION] send acticition msg: %d %d \n", activation_msg.app_id, activation_msg.app_api_level);
}


void nav_open_ack_callback(ProHeader *header)
{
    uint16_t ack_data;
    printf("call %s\n",__func__);
    printf("Recv ACK,sequence_number=%d,session_id=%d,data_len=%d\n", header->sequence_number, header->session_id, header->length - EXC_DATA_SIZE);
    memcpy((uint8_t *)&ack_data,(uint8_t *)&header->magic, (header->length - EXC_DATA_SIZE));

    std_msgs::Float32 msg;
    if( is_sys_error(ack_data))
    {
        printf("[DEBUG] SDK_SYS_ERROR!!! \n");
        msg.data = NO_AUTHORITY;
        //activation_status_pub.publish(msg);
    }
    else
    {
        msg.data = (float)ack_data;
        obtain_ctrl_status_pub.publish(msg);
    }
}

void ros_obtain_ctrl_callback(const std_msgs::Float32::ConstPtr& msg)
{
    uint8_t send_data = (uint8_t)msg->data;
    printf("send open nav %d\n",send_data);
    App_Send_Data(1, 1,
                  MY_CTRL_CMD_SET, API_OPEN_SERIAL, (uint8_t*)&send_data, sizeof(send_data),
                  nav_open_ack_callback,
                  1000, 0);
}

//-------------define type------------------

#define API_VERT_VEL 0
#define API_VERT_POS 1

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
    api_ctrl_without_sensor_data_t send_data = {0};

    if(msg->horiz_mode == API_HORI_ATTI)
    {
        send_data.ctrl_flag |= 0x00;
    }
    else if(msg->horiz_mode == API_HORI_VEL)
    {
        send_data.ctrl_flag |= 0x40;
    }
    else if(msg->horiz_mode == API_HORI_POS)
    {
        send_data.ctrl_flag |= 0x80;
    }

    if(msg->vert_mode == API_VERT_VEL)
    {
        send_data.ctrl_flag |= 0x00;
    }
    else if(msg->vert_mode == API_VERT_POS)
    {
        send_data.ctrl_flag |= 0x10;
    }

    if(msg->yaw_mode == API_YAW_ANG)
    {
        send_data.ctrl_flag |= 0x00;
    }
    else if(msg->yaw_mode == API_YAW_RATE)
    {
        send_data.ctrl_flag |= 0x08;
    }

    if(msg->level_frame == GROUND_LEVEL)
    {
        send_data.ctrl_flag |= 0x00;
    }
    else if(msg->level_frame == BODY_LEVEL)
    {
        send_data.ctrl_flag |= 0x02;
    }
    else if(msg->level_frame == REF_LEVEL)
    {
        send_data.ctrl_flag |= 0x04;
    }

    if(msg->torsion_frame == GROUND_TORSION)
    {
        send_data.ctrl_flag |= 0x00;
    }
    else if(msg->torsion_frame == BODY_TORSION)
    {
        send_data.ctrl_flag |= 0x01;
    }

    send_data.roll_or_x = msg->ctrl_data.roll_or_x;
    send_data.pitch_or_y = msg->ctrl_data.pitch_or_y;
    send_data.yaw = msg->ctrl_data.yaw;
    send_data.thr_z = msg->ctrl_data.thr_z;

    App_Send_Data(0, 0,
                  MY_CTRL_CMD_SET, API_CTRL_REQUEST, (uint8_t*)&send_data, sizeof(send_data),
                  NULL, 0, 0);
} 



/*
  * timer spin_function 50Hz
  */
dji_onboard_sdk::uav_sensor_data sensorMsg;
void spin_callback(const ros::TimerEvent& e)
{
    std_msgs::Float32 msg;

    //ROS_INFO("STD_MSGS:");
    //printf("[STD_MSGS] time_stamp %d \n",recv_sdk_std_msgs.time_stamp);
    //printf("[STD_MSGS] q %f %f %f %f \n",recv_sdk_std_msgs.q.q0,recv_sdk_std_msgs.q.q1,recv_sdk_std_msgs.q.q2,recv_sdk_std_msgs.q.q3);
    //printf("[STD_MSGS] a %f %f %f\n",recv_sdk_std_msgs.a.x,recv_sdk_std_msgs.a.y,recv_sdk_std_msgs.a.z);
    //printf("[STD_MSGS] v %f %f %f\n",recv_sdk_std_msgs.v.x,recv_sdk_std_msgs.v.y,recv_sdk_std_msgs.v.z);
    //printf("[STD_MSGS] w %f %f %f\n",recv_sdk_std_msgs.w.x,recv_sdk_std_msgs.w.y,recv_sdk_std_msgs.w.z);
    //printf("[STD_MSGS] pos %f %f %f %f \n",recv_sdk_std_msgs.pos.lati, recv_sdk_std_msgs.pos.longti, recv_sdk_std_msgs.pos.alti, recv_sdk_std_msgs.pos.height);
    //printf("[STD_MSGS] mag %d %d %d \n",recv_sdk_std_msgs.mag.x,recv_sdk_std_msgs.mag.y,recv_sdk_std_msgs.mag.z);
    //printf("[STD_MSGS] rc %d %d %d %d %d\n",recv_sdk_std_msgs.rc.roll, recv_sdk_std_msgs.rc.pitch, recv_sdk_std_msgs.rc.yaw, recv_sdk_std_msgs.rc.throttle,recv_sdk_std_msgs.rc.mode);
    //printf("[STD_MSGS] gimbal %f %f %f\n",recv_sdk_std_msgs.gimbal.x, recv_sdk_std_msgs.gimbal.y,recv_sdk_std_msgs.gimbal.z);
    //printf("[STD_MSGS] status %d\n",recv_sdk_std_msgs.status);
    //printf("[STD_MSGS] battery %d\n",recv_sdk_std_msgs.battery_remaining_capacity);
    //printf("[STD_MSGS] ctrl_device %d\n",recv_sdk_std_msgs.ctrl_device);

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

    // status
    sensorMsg.status = recv_sdk_std_msgs.status;
    // battery
    sensorMsg.battery = recv_sdk_std_msgs.battery_remaining_capacity;   // %

    sensor_pub.publish(sensorMsg);

    /* test session ack for force close */
    if(nav_force_close_req_id.reserve == 1)
    {
        std_msgs::Float32 msg2;
        msg2.data = 4;
        obtain_ctrl_status_pub.publish(msg2);
        nav_force_close_req_id.reserve = 0;

        uint16_t ack = 0x0001;
        printf("Ack close send %d !!!!!!!!!!! \n", ack);
        App_Send_Ack(nav_force_close_req_id, (uint8_t *)&ack, sizeof(ack));
    }
}

void Activation_Alrm(int sig)
{
    printf("Activation try again\n");
    activation_func();
}

void AUTO_activation(int act_num)
{
    activation_msg.app_id 		= (uint32_t)sdk_app.app_info.app_id;
    activation_msg.app_api_level 	= (uint32_t)sdk_app.app_info.app_api_level;
    activation_msg.app_ver		= (uint32_t)sdk_app.app_info.app_version;
    memcpy(activation_msg.app_bundle_id, sdk_app.app_info.app_bundle_id.c_str(), 32);
    enc_key = (char*)sdk_app.app_info.app_enc_key.c_str();

    cout<<"CALL AUTO_activation "<<act_num<<" times."<<endl;
    for(int i = 0; i<= act_num; i++)
    {
        activation_func();

        // soft time delay
        for(int jj = 0;jj<2048;jj++);
    }
}

int serialPortOpen()
{
    int tmp_flag = Pro_Hw_Setup((char *)sdk_app.serial_port.serial_name.c_str(),
                           sdk_app.serial_port.baud_rate);
    if (0!= tmp_flag)
    {
        cout<<"FAIL to open serial poart."<<endl;
        return 0;
    }
    else
        return 1;
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
    nh_private.param("sdk_app_info", sdk_app_info, std::string("/home/g/ws/src/dji_onboard_sdk/src/app1.txt"));
    sdk_app.loadAppInfo(sdk_app_info);

    // ros param about activiton times
    nh_private.param("activiton_times", activiton_times, 2);


    /* start ros subscriber */
    obtain_ctrl_sub      = nh.subscribe("/nav_request", 10, ros_obtain_ctrl_callback);
    ctrl_data_sub		= nh.subscribe("/api_ctrl", 10, ros_ctrl_data_callback);
    flight_status_request_sub = nh.subscribe("/status_request", 1, flight_status_request_callback);

    /* start ros publisher */
    obtain_ctrl_status_pub 	= nh.advertise<std_msgs::Float32>("/nav_status", 10);
    sensor_pub		= nh.advertise<dji_onboard_sdk::uav_sensor_data>("/uav_sensor", 10);
    //activation_status_pub   = nh.advertise<std_msgs::Float32>("/activation_status", 10);

    /* ros timer 50Hz */
    simple_task_timer 	= nh.createTimer(ros::Duration(1.0/50.0), spin_callback);

    // open serial port & link to flight control
    serialPortOpen();

    Pro_Link_Setup();
    App_Recv_Set_Hook(App_Recv_Req_Data);
    App_Set_Table(set_handler_tab, cmd_handler_tab);

    CmdStartThread();

    // setup a timer
    signal(SIGALRM, Activation_Alrm);

    AUTO_activation(activiton_times);

    /* ros spin for timer */
    ros::spin();

    return 0;
}
