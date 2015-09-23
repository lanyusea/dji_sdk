#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <stdlib.h>//itoa

#include <SerialStream.h>

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"

#include "serial_to_uav/Battery.h"
#include "serial_to_uav/api_sensor_data.h"
#include "serial_to_uav/api_ctrl_data.h"
#include "data_interface2.h"

#include <math.h>

using namespace std;
using namespace LibSerial;

SerialStream serial_port;

#define C_EARTH (double) 6378137.0
#define C_PI	(double) 3.141592653589793
// send uav_ctrl info to serial port when is_write_on
bool is_write_on;

int16_t yaw_rate = 0, pitch = 0, roll = 0;
int16_t vel = 0;
static int16_t status = 3;	// if we have control data, then assume the uav is in status FLIGHT_STATUS_INAIR

int read_counter = 0;

int buffer_counter = 0;
int float_counter = 0;

ros::Subscriber api_ctrl_sub;
ros::Subscriber api_request_status_sub;


ros::Publisher imu_pub;
ros::Publisher battery_pub;
ros::Publisher status_pub;

//declear protocal global varible:
int16_t pose_handler(uint16_t cmd_id,uint8_t* pbuf,uint16_t len);
int16_t battery_handler(uint16_t cmd_id,uint8_t* pbuf,uint16_t len);
int16_t ack_handler(uint16_t cmd_id,uint8_t* pbuf,uint16_t len);
//table of protocol:
cmd_handler_table_t flight_cmd_tab[3]={
	{0x00,pose_handler	}, /* */
	{0x01,battery_handler	},
	{0x02,ack_handler	}
};

dev_handler_table_t g_handler_tab[2]={
	{0x00,flight_cmd_tab}
};

fmu_api_sensor_data pose_msg;
fmu_api_battery_data battery_msg;
uint8_t msg_buf[200];
//protocol handler function:
int16_t pose_handler(uint16_t cmd_id,uint8_t* pbuf,uint16_t len)
{
	//printf("pose_handler\n");

	//static tf::TransformBroadcaster br;
    	//tf::Transform tran;
	if(len != sizeof(fmu_api_sensor_data))
	{
		cout<< "len "<< len << "|" << sizeof(fmu_api_sensor_data)<<endl;
		return 0;
	}
	serial_to_uav::api_sensor_data msg;

	memcpy(&pose_msg,pbuf,sizeof(fmu_api_sensor_data));

	//update new pose ros message
	// q
	msg.orientation.w = pose_msg.q0;
	msg.orientation.x = pose_msg.q1;
	msg.orientation.y = pose_msg.q2;
	msg.orientation.z = pose_msg.q3;
	// v
	msg.linear_v.x = pose_msg.vgx * 100;
	msg.linear_v.y = pose_msg.vgy * 100;
	msg.linear_v.z = pose_msg.vgz * 100;
	// a
	msg.linear_a.x = pose_msg.agx * 100;
	msg.linear_a.y = pose_msg.agy * 100;
	msg.linear_a.z = pose_msg.agz * 100;
	// w
	msg.angular_v.x = pose_msg.wx;
	msg.angular_v.y = pose_msg.wy;
	msg.angular_v.z = pose_msg.wz;
	// gps
	msg.gps.x = pose_msg.lati;
	msg.gps.y = pose_msg.longti;
	msg.gps.z = pose_msg.height;

	msg.status = (float)pose_msg.status;
	msg.header.stamp =ros::Time::now();
	imu_pub.publish(msg);

	//tran.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	//tran.setRotation( tf::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w));
	//br.sendTransform( tf::StampedTransform(tran, ros::Time::now(), "world","uav_frame"));

}
int16_t battery_handler(uint16_t cmd_id,uint8_t* pbuf,uint16_t len)
{
	//printf("battery_handler\n");
	if(len != sizeof(fmu_api_battery_data))
		return 0;
	serial_to_uav::Battery msg;
	memcpy(&battery_msg,pbuf,sizeof(battery_msg));
	msg.full_charge_capacity = battery_msg.full_charge_capacity;
	msg.remaining_capacity = battery_msg.remaining_capacity;
	msg.pack_voltage = battery_msg.pack_voltage;
	msg.current = battery_msg.current;
	msg.average_current = battery_msg.average_current;
	msg.temperature = battery_msg.temperature;
	msg.capacity_percentage = battery_msg.capacity_percentage;
    msg.right = battery_msg.right;
	battery_pub.publish(msg);
	//debug msg:
	//cout<<"battery"<<" LEN:"<<len+00<<"\tfull_capacity:"<<(battery_msg.full_charge_capacity+00)<<"\tremaining_capacity:"<<(battery_msg.remaining_capacity+00)<<"\ncurrent_voltage:"<<(battery_msg.current+00)<<"\ncurrent:"<<(battery_msg.average_current+00)<<"\tcapacity_percentage:"<<(battery_msg.capacity_percentage+00)<<"\tright"<<battery_msg.right+0<<"\ttemp:"<<battery_msg.temperature+0<<endl;

}

int16_t ack_handler(uint16_t cmd_id,uint8_t* pbuf,uint16_t len)
{
	if(len != sizeof(fmu_api_ack_data))
		return 0;
	fmu_api_ack_data ack_msg;
	memcpy(&ack_msg, pbuf, sizeof(ack_msg));
	printf("ack_command  %d  %d\n",ack_msg.ack_command,ack_msg.ack_result);

}

uint8_t serial_write(uint8_t* pbuf,uint8_t len)
{
	for(int i=0;i<len;i++)
	{
        serial_port << pbuf[i];
	}
}

void serial_interrupt(int sig){
	if (serial_port.IsOpen())
		serial_port.Close();
	ros::shutdown();
	cout << "node terminated" << endl;
}

void get_ctrl_test_mode1_and_mode2(bool &is_init)
{
	static int cnt;
	if(!is_init)
	{
		cnt = 0;
		is_init= true;
	}
	else
	{
		api_ctrl_data_t send_data = {0};
		//send_data.ctrl_flag = 0x90 ;
		//send_data.ctrl_flag = 0x00;
        send_data.ctrl_flag = 0x0b;
		if(cnt < 10*50)
		{
			send_data.roll_or_x = 0;
			send_data.pitch_or_y = 0;//10/180*C_PI;
			//send_data.thr_z = 2; //m
			send_data.thr_z = 0; //m/s
			send_data.yaw = -200;
			cout<< "1" <<endl;
		}
		else if(cnt < 20*50)
		{
			send_data.roll_or_x = 0;
			send_data.pitch_or_y = 0;
            //send_data.thr_z = 10; //m
			send_data.thr_z = 0; //m/s
			send_data.yaw = 0;
			cout<< "2" <<endl;
		}
		else
		{
			cnt = 0;
		}
        cout << "r p y vel"
             << send_data.roll_or_x <<"|"
             << send_data.pitch_or_y<<"|"
             << send_data.yaw<<"|"
             << send_data.thr_z<<endl;
		send_data.sensor_flag = 0;
		//send_data.pos_x = 0.0;
		//send_data.pos_y = 0.0;
		send_data.pos_z = pose_msg.height;
		send_data.vel_x = -10.0;
		send_data.vel_y = 10.0;
		send_data.vel_z = -pose_msg.vgz;	// add "-" for g_navi_data
		//printf("sent ctrl %d %f %f %f %f\n",send_data.ctrl_flag, send_data.roll_or_x, send_data.pitch_or_y, send_data.thr_z, send_data.yaw);
		dji_api_pack_data2(MY_DEV_ID,API_CTRL_REQUEST,(uint8_t*)&send_data,sizeof(send_data));

		cnt++;
	}
}

void get_ctrl_test_mode3_and_mode4(bool &is_init)
{
	static int cnt;
	if(!is_init)
	{
		cnt = 0;
		is_init = true;


	}
	else
	{
		api_ctrl_data_t send_data = {0};
		send_data.ctrl_flag = 0x40; // mode 4 or mode 3
		if(cnt < 10*50)
		{
            send_data.roll_or_x = 0;
			send_data.pitch_or_y = -1;
			//send_data.thr_z = 2; //m
			send_data.thr_z = 0; //m/s
			send_data.yaw = 0;
			cout<< "1" <<endl;
		}
		else if(cnt < 20*50)
		{
			send_data.roll_or_x = 0;
			send_data.pitch_or_y = 0;
			//send_data.thr_z = 10; //m
			send_data.thr_z = 0; //m/s
			send_data.yaw = 0;
			cout<< "2" <<endl;
		}
		else
		{
			cnt = 0;
		}
		cout << "height: " << pose_msg.height << endl;
        cout << "vel_z:  " << pose_msg.vgz <<endl;
		send_data.sensor_flag = 0;
		//send_data.pos_x = 0.0;
		//send_data.pos_y = 0.0;
		send_data.pos_z = 0.0;
		send_data.vel_x = pose_msg.vgx;
		send_data.vel_y = pose_msg.vgy;
		send_data.vel_z = -pose_msg.vgz;	// add "-" for g_navi_data
		//printf("sent ctrl %d %f %f %f %f\n",send_data.ctrl_flag, send_data.roll_or_x, send_data.pitch_or_y, send_data.thr_z, send_data.yaw);
		dji_api_pack_data2(MY_DEV_ID,API_CTRL_REQUEST,(uint8_t*)&send_data,sizeof(send_data));

		cnt++;
	}
}


void get_ctrl_test_mode5_and_mode6(bool &is_init)
{
	static float delta_t;
	static float init_lat;
	static float init_lon;
	if(!is_init)
	{
		delta_t = 0;
		init_lat = pose_msg.lati;
		init_lon = pose_msg.longti;
		is_init = true;
    }
	else
	{
		api_ctrl_data_t send_data = {0};
		send_data.ctrl_flag = 0x80; // mode 5 or mode 6
		if(delta_t < 10*50)
		{

			float tgt_pos_x = 0; //m
			float tgt_pos_y = 0;

			float tgt_pos_lat = init_lat + tgt_pos_x/C_EARTH;
			float tgt_pos_lon = init_lon + tgt_pos_y/C_EARTH*cos(init_lat);

			float pos_x_offset = (tgt_pos_lat - pose_msg.lati) * C_EARTH;
			float pos_y_offset = (tgt_pos_lon - pose_msg.longti) * C_EARTH * cos(pose_msg.lati);

			float tgt_yaw_angle = atan2(pos_y_offset, pos_x_offset);

            send_data.roll_or_x = 0;
			send_data.pitch_or_y = 0;
			//send_data.thr_z = 2; //m
			send_data.thr_z = 1; //m/s
			send_data.yaw = 0;
			cout<< "up" <<endl;
		}
		else if(delta_t < 20*50)
		{
			float tgt_pos_x = 0; //m
			float tgt_pos_y = 0;

			float tgt_pos_lat = init_lat + tgt_pos_x/C_EARTH;
			float tgt_pos_lon = init_lon + tgt_pos_y/C_EARTH*cos(init_lat);

			float pos_x_offset = (tgt_pos_lat - pose_msg.lati) * C_EARTH;
			float pos_y_offset = (tgt_pos_lon - pose_msg.longti) * C_EARTH * cos(pose_msg.lati);

			float tgt_yaw_angle = atan2(pos_y_offset, pos_x_offset);

            send_data.roll_or_x = 0;
			send_data.pitch_or_y = 0;
			//send_data.thr_z = 2; //m
			send_data.thr_z = 0; //m/s
			send_data.yaw = 0;
			cout<< "stop" <<endl;
		}
		else
		{
			delta_t = 0;
		}
		float my_vg_z = pose_msg.vgz + 0.05 * ((rand())%10);
		cout << "vg_z:   " << pose_msg.vgz << endl;
		cout << "my_vg_z:"<< my_vg_z << endl;
		//cout << "vel_z:    " <<  pose_msg.vgz <<endl;
		send_data.sensor_flag = 1;
		//send_data.pos_x = 0.0;
		//send_data.pos_y = 0.0;
		send_data.pos_z = 0;
		send_data.vel_x = 0;
		send_data.vel_y = 0;
		send_data.vel_z = -my_vg_z;	// add "-" for g_navi_data
		//printf("sent ctrl %d %f %f %f %f\n",send_data.ctrl_flag, send_data.roll_or_x, send_data.pitch_or_y, send_data.thr_z, send_data.yaw);
		dji_api_pack_data2(MY_DEV_ID,API_CTRL_REQUEST,(uint8_t*)&send_data,sizeof(send_data));

		delta_t++;

	}

}

void get_ctrl_test_mode7_and_mode8(bool &is_init)
{

}

void get_ctrl_test_mode9_and_mode10(bool &is_init)
{

}

void get_ctrl_test_mode11_and_mode12(bool &is_init)
{
	static float delta_t;
	static float init_lat;
	static float init_lon;
	static float init_height;
	if(!is_init)
	{
		delta_t = 0;
		init_lat = pose_msg.lati;
		init_lon = pose_msg.longti;
		init_height = pose_msg.height;
		is_init = true;
    }
	else
	{
		api_ctrl_data_t send_data = {0};
		//send_data.ctrl_flag = 0x99; // mode 11 or mode 12
        send_data.ctrl_flag = 0x98;
		if(delta_t < 10*50)
		{

			float tgt_pos_x = 5; //m
			float tgt_pos_y = 0;

			float tgt_pos_lat = init_lat + tgt_pos_x/C_EARTH;
			float tgt_pos_lon = init_lon + tgt_pos_y/C_EARTH*cos(init_lat);

			float pos_x_offset = (tgt_pos_lat - pose_msg.lati) * C_EARTH;
			float pos_y_offset = (tgt_pos_lon - pose_msg.longti) * C_EARTH * cos(pose_msg.lati);

			float tgt_yaw_angle = 0;//atan2(pos_y_offset, pos_x_offset);

            send_data.roll_or_x = 1;//pos_x_offset;
			send_data.pitch_or_y = 0;//pos_y_offset;
			send_data.thr_z = 2; //m
			send_data.yaw = tgt_yaw_angle;
			cout<< "1" <<endl;
		}
		else if(delta_t < 20*50)
		{
			float tgt_pos_x = 0; //m
			float tgt_pos_y = 0;

			float tgt_pos_lat = init_lat + tgt_pos_x/C_EARTH;
			float tgt_pos_lon = init_lon + tgt_pos_y/C_EARTH*cos(init_lat);

			float pos_x_offset = (tgt_pos_lat - pose_msg.lati) * C_EARTH;
			float pos_y_offset = (tgt_pos_lon - pose_msg.longti) * C_EARTH * cos(pose_msg.lati);

			float tgt_yaw_angle = 0;//atan2(pos_y_offset, pos_x_offset);

            send_data.roll_or_x = 0;//pos_x_offset;
			send_data.pitch_or_y = 1;//pos_y_offset;
			send_data.thr_z = 2; //m
			send_data.yaw = tgt_yaw_angle;
			cout<< "2" <<endl;
		}
		else
		{
			delta_t = 0;
		}
		//float my_height = pose_msg.height+ 30 + 0.1*((rand())%10);
		float my_height = pose_msg.height-init_height;
		cout << "height:   " << pose_msg.height << endl;
		cout << "my_height:"<< my_height << endl;
		//cout << "vel_z:    " <<  pose_msg.vgz <<endl;
		send_data.sensor_flag = 0;
		//send_data.pos_x = 0.0;
		//send_data.pos_y = 0.0;
		send_data.pos_z = my_height;
		send_data.vel_x = 0;
		send_data.vel_y = 0;
		send_data.vel_z = 0;	// add "-" for g_navi_data
		//printf("sent ctrl %d %f %f %f %f\n",send_data.ctrl_flag, send_data.roll_or_x, send_data.pitch_or_y, send_data.thr_z, send_data.yaw);
		dji_api_pack_data2(MY_DEV_ID,API_CTRL_REQUEST,(uint8_t*)&send_data,sizeof(send_data));

		delta_t++;

	}

}

void get_ctrl_test_circle(bool &is_init)
{
	static float delta_t;
	static float init_lat;
	static float init_lon;
	static float init_h;
	if(!is_init)
	{
		delta_t = 0;
		init_lat = pose_msg.lati;
		init_lon = pose_msg.longti;
		init_h = pose_msg.height;
		is_init = true;

    }
	else
	{
		float tgt_pos_x = 5.0f*sin(delta_t/50.0f*30/180*C_PI);
		float tgt_pos_y = 5.0f*cos(delta_t/50.0f*30/180*C_PI);

		float tgt_pos_lat = init_lat + tgt_pos_x/C_EARTH;
		float tgt_pos_lon = init_lon + tgt_pos_y/C_EARTH*cos(init_lat);

		float pos_x_offset = (tgt_pos_lat - pose_msg.lati) * C_EARTH;
		float pos_y_offset = (tgt_pos_lon - pose_msg.longti) * C_EARTH * cos(pose_msg.lati);

		float tgt_yaw_angle = atan2(pos_y_offset, pos_x_offset);

		//api_ctrl_data_t send_data = {0};
		api_ctrl_without_sensor_data_t send_data = {0};
		send_data.ctrl_flag = 0x90 ;

		send_data.roll_or_x = pos_x_offset;
		send_data.pitch_or_y = pos_y_offset;
		send_data.thr_z = init_h;
		send_data.yaw = tgt_yaw_angle;

		printf("init_lati,init_longti %f,%f \n", init_lat,init_lon);
		printf("tgt_pos_x,tgt_pos_y %f,%f \n", tgt_pos_x,tgt_pos_y);
		printf("pose_msg.lati,pose_msg.longti %f,%f \n", pose_msg.lati,pose_msg.longti);

		printf("x %f y %f z %f yaw %f \n", pos_x_offset,pos_y_offset,init_h,tgt_yaw_angle);



		//send_data.sensor_flag = 0;
		////send_data.pos_x = 0.0;
		////send_data.pos_y = 0.0;
		//send_data.pos_z = -5.0;
		//send_data.vel_x = 0.0;
		//send_data.vel_y = 0.0;
        //send_data.vel_z = 0.0;
		//printf("sent ctrl %d %f %f %f %f\n",send_data.ctrl_flag, send_data.roll_or_x, send_data.pitch_or_y, send_data.thr_z, send_data.yaw);
		dji_api_pack_data2(MY_DEV_ID,API_CTRL_REQUEST,(uint8_t*)&send_data,sizeof(send_data));

		delta_t++;

	}
}

void get_ctrl_test_line(bool &is_init)
{
	static float delta_t;
	static float init_lat;
	static float init_lon;
	static float init_h;
	if(!is_init)
	{
		delta_t = 0;
		init_lat = pose_msg.lati;
		init_lon = pose_msg.longti;
		init_h = pose_msg.height;
		is_init = true;
    }
	else
	{
		float tgt_pos_x = 15; //m
		float tgt_pos_y = 0;

		float tgt_pos_lat = init_lat + tgt_pos_x/C_EARTH;
		float tgt_pos_lon = init_lon + tgt_pos_y/C_EARTH*cos(init_lat);

		float pos_x_offset = (tgt_pos_lat - pose_msg.lati) * C_EARTH;
		float pos_y_offset = (tgt_pos_lon - pose_msg.longti) * C_EARTH * cos(pose_msg.lati);

		float tgt_yaw_angle = atan2(pos_y_offset, pos_x_offset);

		api_ctrl_data_t send_data = {0};
		send_data.ctrl_flag = 0x98 ;

		send_data.roll_or_x = pos_x_offset;
		send_data.pitch_or_y = pos_y_offset;
		send_data.thr_z = init_h;
		send_data.yaw = tgt_yaw_angle;
		delta_t++;
		printf("init_lati,init_longti %f,%f \n", init_lat,init_lon);
		printf("tgt_pos_x,tgt_pos_y %f,%f \n", tgt_pos_x,tgt_pos_y);
		printf("pose_msg.lati,pose_msg.longti %f,%f \n", pose_msg.lati,pose_msg.longti);

		printf("x %f y %f z %f yaw %f \n", pos_x_offset,pos_y_offset,init_h,tgt_yaw_angle);



		send_data.sensor_flag = 0;
		//send_data.pos_x = 0.0;
		//send_data.pos_y = 0.0;
		send_data.pos_z = -5.0;
		send_data.vel_x = 0.0;
		send_data.vel_y = 0.0;
        send_data.vel_z = 0.0;
		//printf("sent ctrl %d %f %f %f %f\n",send_data.ctrl_flag, send_data.roll_or_x, send_data.pitch_or_y, send_data.thr_z, send_data.yaw);
		dji_api_pack_data2(MY_DEV_ID,API_CTRL_REQUEST,(uint8_t*)&send_data,sizeof(send_data));
	}
}


void get_ctrl_test_vicon(bool &is_init)
{
	static float delta_t;
	static float init_lat;
	static float init_lon;
	static float init_height;
	if(!is_init)
	{
		delta_t = 0;
		init_lat = pose_msg.lati;
		init_lon = pose_msg.longti;
		init_height = pose_msg.height;
		is_init = true;
    }
	else
	{
		api_ctrl_data_t send_data = {0};
		send_data.ctrl_flag = 0x5a; // mode 10
		if(delta_t < 10*50)
		{

            send_data.roll_or_x = 0;
			send_data.pitch_or_y = 0;
			//send_data.thr_z = 2; //m
			send_data.thr_z = 0; //m
			send_data.yaw = 0;
			cout<< "1" <<endl;
		}
		else if(delta_t < 20*50)
		{

            send_data.roll_or_x = 0;
			send_data.pitch_or_y = 0;
			//send_data.thr_z = 2; //m
			send_data.thr_z = 1; //m
			send_data.yaw = 0;
			cout<< "2" <<endl;
		}
		else
		{
			delta_t = 0;
		}
		//float my_height = pose_msg.height+ 30 + 0.1*((rand())%10);
		float my_height = pose_msg.height-init_height;
		cout << "height:   " << pose_msg.height << endl;
		cout << "my_height:"<< my_height << endl;
		//cout << "vel_z:    " <<  pose_msg.vgz <<endl;
		send_data.sensor_flag = 1;
		//send_data.pos_x = 0.0;
		//send_data.pos_y = 0.0;
		send_data.pos_z = my_height;
		send_data.vel_x = pose_msg.vgx;
		send_data.vel_y = pose_msg.vgy;
		send_data.vel_z = 0;	// add "-" for g_navi_data
		//printf("sent ctrl %d %f %f %f %f\n",send_data.ctrl_flag, send_data.roll_or_x, send_data.pitch_or_y, send_data.thr_z, send_data.yaw);
		dji_api_pack_data2(MY_DEV_ID,API_CTRL_REQUEST,(uint8_t*)&send_data,sizeof(send_data));

		delta_t++;

	}

}

void get_ctrl_test_all(const std_msgs::Float32::ConstPtr& msg)
{
	static bool init_flag;
	static ros::Time time = ros::Time::now();
	if(ros::Time::now().toSec()- time.toSec() < 1)
	{
		time = ros::Time::now();
	}
	else
	{
		init_flag = false;
		time = ros::Time::now();
	}
	//cout << "time "<< time.toSec() - ros::Time::now().toSec() << endl;
	switch((int)msg->data)
	{
		case 0:
		get_ctrl_test_mode1_and_mode2(init_flag);
		break;
		case 1:
		get_ctrl_test_mode3_and_mode4(init_flag);
		break;
		case 2:
		get_ctrl_test_mode5_and_mode6(init_flag);
		break;
		case 3:
		get_ctrl_test_mode7_and_mode8(init_flag);
		break;
		case 4:
		get_ctrl_test_mode9_and_mode10(init_flag);
		break;
		case 5:
		get_ctrl_test_mode11_and_mode12(init_flag);
		break;
		case 6:
		get_ctrl_test_circle(init_flag);
		break;
		case 7:
		get_ctrl_test_line(init_flag);
		break;
		case 8:
		get_ctrl_test_vicon(init_flag);
		break;

	}
}

void get_ctrl_data(const serial_to_uav::api_ctrl_data::ConstPtr& msg)
{
/* ctrl msg frame */
/* horiz_mode    vert_mode   yaw_mode   level_frame   torsion_frame | crtl_data x 4*/
/*     2bit          2bit       1bit          2bit          1bit   	float  x 4 */
/* sensor msg fram*/
/*         pos_flag  vel_flag | sensor_data x 6 */
/* 000000     1bit	1bit  	      float x 6 */

	api_ctrl_data_t send_data = {0};

	if(msg->horiz_mode == FMU_API_HORI_ATTI_TILT_ANG)
	{
		send_data.ctrl_flag |= 0x00;
	}
	else if(msg->horiz_mode == FMU_API_HORI_VEL)
	{
		send_data.ctrl_flag |= 0x40;
	}
	else if(msg->horiz_mode == FMU_API_HORI_POS)
	{
		send_data.ctrl_flag |= 0x80;
	}

	if(msg->vert_mode == FMU_API_VERT_VEL)
	{
		send_data.ctrl_flag |= 0x00;
	}
	else if(msg->vert_mode == FMU_API_VERT_POS)
	{
		send_data.ctrl_flag |= 0x10;
	}

	if(msg->yaw_mode == FMU_API_YAW_ANG)
	{
		send_data.ctrl_flag |= 0x00;
	}
	else if(msg->yaw_mode == FMU_API_YAW_RATE)
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

	if(msg->pos_flag == 0)
	{
		send_data.sensor_flag |= 0x00;
	}
	else
	{
		send_data.sensor_flag |= 0x02;
	}

	if(msg->vel_flag == 0)
	{
		send_data.sensor_flag |= 0x00;
	}
	else
	{
		send_data.sensor_flag |= 0x01;
	}

	send_data.roll_or_x = msg->ctrl_data.x;
	send_data.pitch_or_y = msg->ctrl_data.y;
	send_data.thr_z = msg->ctrl_data.z;
	send_data.yaw = msg->ctrl_data.w;
	//send_data.pos_x = msg->pos.x;
	//send_data.pos_y = msg->pos.y;
	send_data.pos_z = msg->pos.z;
	send_data.vel_x = msg->vel.x;
	send_data.vel_y = msg->vel.y;
    send_data.vel_z = msg->vel.z;
	printf("sent ctrl %d %f %f %f %f\n",send_data.ctrl_flag, send_data.roll_or_x, send_data.pitch_or_y, send_data.thr_z, send_data.yaw);
	dji_api_pack_data2(MY_DEV_ID,API_CTRL_REQUEST,(uint8_t*)&send_data,sizeof(send_data));
}

void request_status(const std_msgs::Float32::ConstPtr& msg)
{
/*     cmd_request       */
/* 2->takeoff o 4 landing*/
	fmu_api_status_request_data send_data;
    send_data.status_request =(int16_t)(msg->data);
	printf("sent request %i\n",send_data.status_request);
    dji_api_pack_data2(MY_DEV_ID,API_STATUS_REQUEST,(uint8_t*)&send_data,sizeof(send_data));
}

/* get data via serial port and publish */
void get_data()
{
	char next_byte;
	static int connection_loss_count = 0;
  	static tf::TransformBroadcaster br;
   	tf::Transform tran;

	//debug msg:
	//cout<<"protocol check start!!!"<<endl;
	// if rdbuf is not empty, reset connection_loss_count
	while( serial_port.rdbuf()->in_avail() > 0  )
	{
		connection_loss_count = 0;

		//protocol recive handler
		//caustion:next_byte is a signed int but i change it to unsigned
		serial_port.get(next_byte);
		dji_api_recv_ontick((uint8_t)next_byte,g_handler_tab);
    }
	if (connection_loss_count > 50)
		printf("connection loss!\n");
}

int main (int argc, char** argv)
{
    // Register signals
    signal(SIGINT, serial_interrupt);
    serial_port.Open( "/dev/ttySAC2" ) ;//!!modify ttySAC2
	if ( ! serial_port.good() )
	{
		std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
				  << "Error: Could not open serial port."
				  << std::endl ;
		exit(1) ;
	}
	//
	// Set the baud rate of the serial port.
	//
/*	serial_port.SetBaudRate( SerialStreamBuf::BAUD_115200 ) ;
	if ( ! serial_port.good() )
	{
        std::cerr << "Error: Could not set the baud rate." <<
			std::endl ;
		exit(1) ;
    }*/
	//
	// Set the number of data bits.
	//
	serial_port.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
	if ( ! serial_port.good() )
	{
        std::cerr << "Error: Could not set the character size." <<
			std::endl ;
		exit(1) ;
	}
	//
	// Disable parity.
	//
	serial_port.SetParity( SerialStreamBuf::PARITY_NONE ) ;
	if ( ! serial_port.good() )
	{
        std::cerr << "Error: Could not disable the parity." <<
			std::endl ;
		exit(1) ;
	}
	//
	// Set the number of stop bits.
	//
	serial_port.SetNumOfStopBits( 1 ) ;
	if ( ! serial_port.good() )
	{
		std::cerr << "Error: Could not set the number of stop bits."
				  << std::endl ;
		exit(1) ;
	}
	//
	// Turn off hardware flow control.
	//
	serial_port.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_NONE ) ;
	if ( ! serial_port.good() )
	{
		std::cerr << "Error: Could not use hardware flow control."
				  << std::endl ;
		exit(1) ;
    }
	ros::init(argc, argv, "serial_talker");
	ros::NodeHandle n;
	//control read only or read & is_write_on
	n.param<bool>("is_write_on", is_write_on, true);
	imu_pub = n.advertise<serial_to_uav::api_sensor_data>("uav_imu",100);
	status_pub = n.advertise<std_msgs::Float32>("uav_flight_status",10);
	battery_pub = n.advertise<serial_to_uav::Battery>("uav_battery",10);

	// if read & is_write_on, listen to uav_control topica
	if (is_write_on)
	{
        api_ctrl_sub	       = n.subscribe("/api_ctrl", 10, get_ctrl_data);//get_ctrl_test_all
		api_request_status_sub = n.subscribe("/uav_flight_status_request", 10, request_status);

	}

	ros::Rate loop_rate(50);
	cout << "started, ready to control" << endl;
	//init protocol,express function point
	init_dji_api_protocol(serial_write);
	while (ros::ok())
	{
		get_data();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
