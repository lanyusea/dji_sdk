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

#include "serial_to_uav/UAV.h" // custom msgs
#include "serial_to_uav/Battery.h"
#include "data_interface.h"

using namespace std;
using namespace LibSerial;

SerialStream serial_port;

// send uav_ctrl info to serial port when is_write_on
bool is_write_on;

int16_t yaw_rate = 0, pitch = 0, roll = 0;
int16_t vel = 0;
static int16_t status = 3;	// if we have control data, then assume the uav is in status FLIGHT_STATUS_INAIR

int read_counter = 0;

int buffer_counter = 0;
int float_counter = 0;

ros::Subscriber sub;
ros::Publisher pub;
//battery ROS topic:
ros::Publisher battery_pub;

ros::Subscriber status_sub;
ros::Publisher status_pub;


//declear protocal global varible:
int16_t pose_handler(uint16_t cmd_id,uint8_t* pbuf,uint16_t len);
int16_t battery_handler(uint16_t cmd_id,uint8_t* pbuf,uint16_t len);
//table of protocol:
cmd_handler_table_t flight_cmd_tab[2]={
{0x10,pose_handler},
{0x02,battery_handler}
};
dev_handler_table_t g_handler_tab[2]={
{0x00,flight_cmd_tab}
};
api_updata_t pose_msg;
api_battery_t battery_msg;
uint8_t msg_buf[200];
//protocol handler function:
int16_t pose_handler(uint16_t cmd_id,uint8_t* pbuf,uint16_t len)
{
	#define POSEMSG_LEN 4*15
	static tf::TransformBroadcaster br;
    	tf::Transform tran;
	serial_to_uav::UAV msg;
	std_msgs::Float32 status_msg;
	memcpy(&pose_msg,pbuf,POSEMSG_LEN);
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
	// h
	msg.height = pose_msg.height;
	msg.header.stamp =ros::Time::now();
	pub.publish(msg);
	tran.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	tran.setRotation( tf::Quaternion(msg.orientation.x, 
									 msg.orientation.y, 
									 msg.orientation.z,
									 msg.orientation.w) );
	br.sendTransform(tf::StampedTransform(tran, ros::Time::now(), "world","uav_frame"));

	// uav status publisher
	status_msg.data = pose_msg.status;
	status_pub.publish(status_msg);	
}
int16_t battery_handler(uint16_t cmd_id,uint8_t* pbuf,uint16_t len)
{
	#define BATTERYMSG_LEN 7*2
	serial_to_uav::Battery msg;
	memcpy(&msg.full_charge_capacity,pbuf,BATTERYMSG_LEN);
	memcpy(&battery_msg,pbuf,BATTERYMSG_LEN);
	battery_pub.publish(msg);
	//debug msg:
	//cout<<"battery"<<" LEN:"<<len+00<<"\tfull_capacity:"<<(battery_msg.full_charge_capacity+00)<<"\tremaining_capacity:"<<(battery_msg.remaining_capacity+00)<<"\ncurrent_voltage:"<<(battery_msg.current+00)<<"\ncurrent:"<<(battery_msg.average_current+00)<<"\tcapacity_percentage:"<<(battery_msg.capacity_percentage+00)<<"\tright"<<battery_msg.right+0<<"\ttemp:"<<battery_msg.temperature+0<<endl;
	
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

void get_uav_ctrl(const geometry_msgs::Quaternion::ConstPtr& msg)
{	
	api_send_data_t send_data;

	// load old stuff for control
	send_data.send_yaw = (float)msg->z;
	send_data.send_pitch = (float)msg->x;
	send_data.send_roll = (float)msg->y;
	send_data.send_thr = (float)(msg->w);
	//load_status
	send_data.req_status = 3;
	//printf("yaw %i pitch %i roll %i\n",send_data.send_yaw, send_data.send_pitch,send_data.send_roll);
	dji_api_pack_data2(0x00,0x00,(uint8_t*)&send_data,sizeof(api_send_data_t));
	cout<<"yaw"<<send_data.send_yaw+0x00<<"  pitch"<<send_data.send_pitch+0x00<<" ROLL"<<send_data.send_roll+0x00<<"  VEL"<<send_data.send_thr+0x00<<endl;
	 
} 

void request_status(const std_msgs::Float32::ConstPtr& msg)
{
	api_send_data_t send_data;
	// update global var 'status' based on msg
	send_data.req_status=(uint32_t)(msg->data);	
	send_data.send_yaw=(float)yaw_rate;
	send_data.send_pitch=(float)pitch;
	send_data.send_roll=(float)roll;
	send_data.send_thr=(float )vel;
	printf("sent request %i\n",send_data.req_status);
	dji_api_pack_data2(0x00,0x00,(uint8_t*)&send_data,sizeof(api_send_data_t));
	cout<<send_data.req_status<<endl;
}

/* get imu data via serial port and publish */
void get_pose()
{	
	char next_byte;
	static int connection_loss_count = 0;
  	static tf::TransformBroadcaster br;
   	tf::Transform tran;

	serial_to_uav::UAV msg;
	std_msgs::Float32 status_msg;
	
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
	serial_port.Open( "/dev/ttySAC0" ) ;//!!modify ttySAC2
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
	serial_port.SetBaudRate( SerialStreamBuf::BAUD_115200 ) ;
	if ( ! serial_port.good() )
	{
		std::cerr << "Error: Could not set the baud rate." <<  
			std::endl ;
		exit(1) ;
	}
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
	pub = n.advertise<serial_to_uav::UAV>("uav_imu",100);
	status_pub = n.advertise<std_msgs::Float32>("uav_flight_status",10);
	battery_pub = n.advertise<serial_to_uav::Battery>("uav_battery",10);
	
	// if read & is_write_on, listen to uav_control topic
	if (is_write_on)
	{
		sub=n.subscribe("/board_ctrl", 1000, get_uav_ctrl);
		status_sub = n.subscribe("/uav_flight_status_request", 10, request_status);
	}

	ros::Rate loop_rate(50);
	cout << "started, ready to control" << endl;
	//init protocol,express function point
	init_dji_api_protocol(serial_write);
	while (ros::ok())
	{
		get_pose();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
