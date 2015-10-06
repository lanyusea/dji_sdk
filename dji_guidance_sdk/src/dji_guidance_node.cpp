/*
 * main_sdk0428.cpp
 *
 *  Created on: Apr 29, 2015
 *      Author: craig
 */

#include <stdio.h>
#include <string.h>
#include <iostream>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Vector3.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "guidance_lib/DJI_guidance.h"

e_vbus_index CAMERA_ID = e_vbus5;

#define WIDTH 320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)

using namespace cv;
using namespace std;

ros::Publisher depth_image_pub;
ros::Publisher left_image_pub;
ros::Publisher right_image_pub;
ros::Publisher sonar_pub;

Mat     g_greyscale_image_left(HEIGHT, WIDTH, CV_8UC1);
Mat		g_greyscale_image_right(HEIGHT, WIDTH, CV_8UC1);
Mat		g_depth(HEIGHT,WIDTH,CV_16SC1);

int my_callback(int data_type, int data_len, char *content)
{
    //std::cout<<"\t\tcall back "<<data_type<<std::endl;
    if (e_image == data_type && NULL != content)
    {
        //example
        image_data data;
        memcpy( (char*)&data, content, sizeof(data) );

        memcpy( g_greyscale_image_left.data, data.m_greyscale_image_left[CAMERA_ID], IMAGE_SIZE );
        memcpy( g_greyscale_image_right.data, data.m_greyscale_image_right[CAMERA_ID], IMAGE_SIZE );
        memcpy( g_depth.data, data.m_depth_image[CAMERA_ID], IMAGE_SIZE * 2 );
        /*

        imshow("left", g_greyscale_image_left);
        waitKey(1);


        //publish depth image
        cv_bridge::CvImage mat_16;
        g_depth.copyTo(mat_16.image);
        mat_16.header.frame_id  = "guidance";
        mat_16.header.stamp		= ros::Time::now();
        mat_16.encoding		 	= sensor_msgs::image_encodings::MONO16;
        depth_image_pub.publish(mat_16.toImageMsg());

        // publish left grayscale image
        cv_bridge::CvImage mat_8;
        g_greyscale_image_left.copyTo(mat_8.image);
        mat_8.header.frame_id  = "guidance";
        mat_8.header.stamp		= ros::Time::now();
        mat_8.encoding		 	= sensor_msgs::image_encodings::MONO8;
        left_image_pub.publish(mat_8.toImageMsg());
        */
    }
    if ( e_imu == data_type && NULL != content )
    {
        imu *imu_data = (imu*)content;
        printf( "imu:%f %f %f,%f %f %f %f\n", imu_data->acc_x, imu_data->acc_y, imu_data->acc_z, imu_data->q[0], imu_data->q[1], imu_data->q[2], imu_data->q[3] );
        printf( "frame index:%d,stamp:%d\n", imu_data->frame_index, imu_data->time_stamp );
    }

    if ( e_velocity == data_type && NULL != content )
    {
        velocity *vo = (velocity*)content;
        printf( "vx:%f vy:%f vz:%f\n", 0.001f * vo->vx, 0.001f * vo->vy, 0.001f * vo->vz );
        printf( "frame index:%d,stamp:%d\n", vo->frame_index, vo->time_stamp );
    }

    if ( e_obstacle_distance == data_type && NULL != content )
    {
        obstacle_distance *oa = (obstacle_distance*)content;
        printf( "obstacle distance:" );
        for ( int i = 0; i < CAMERA_PAIR_NUM; ++i )
        {
            printf( " %f ", 0.01f * oa->distance[i] );
        }
        printf( "\n" );
        printf( "frame index:%d,stamp:%d\n", oa->frame_index, oa->time_stamp );
    }

    if ( e_ultrasonic == data_type && NULL != content )
    {
        ultrasonic_data *ultrasonic = (ultrasonic_data*)content;
        geometry_msgs::Vector3 outMsg;
        outMsg.x = ultrasonic->ultrasonic[0] * 0.1f;
        outMsg.y = (int)ultrasonic->reliability[0];
        outMsg.z = 0.0f;

        if(outMsg.y == 1)
        {
            cout<<"ultrasonic distance "<<outMsg.x;
            cout<<" reliable"<<endl;
            sonar_pub.publish(outMsg);
        }
        else
        {
            cout<<"\033[1m\033[31m"<<"ultrasonic distance "<<outMsg.x;
            cout<<"\033[0m"<<endl;
            outMsg.x = 0.0;
            sonar_pub.publish(outMsg);
        }
    }

    return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dji_guidance_node");

    ros::NodeHandle my_node;
    //depth_image_pub = my_node.advertise<sensor_msgs::Image>("/depth_image",1);
    //left_image_pub  = my_node.advertise<sensor_msgs::Image>("/left_image",1);
    //right_image_pub  = my_node.advertise<sensor_msgs::Image>("/right_image",1);
    sonar_pub = my_node.advertise<geometry_msgs::Vector3>("/sonar_height",1);

    reset_config();

    init_transfer();

    //select_greyscale_image(CAMERA_ID, true);
    //select_greyscale_image(CAMERA_ID, false);
    //select_depth_image(CAMERA_ID);

    //select_imu();

    select_ultrasonic();

    //select_obstacle_distance();
    //select_velocity();

    set_sdk_event_handler(my_callback);

    start_transfer();

    std::cout<<"start_transfer"<<std::endl;

    ros::spin();

    stop_transfer();
    //make sure the ack packet from GUIDANCE is received
    sleep(1);
    std::cout<<"release_transfer"<<std::endl;
    release_transfer();

    return 0;
}
