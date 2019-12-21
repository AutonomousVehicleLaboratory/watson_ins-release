/*
 * Watson DMS-SGP02 INS ROS Node
 *
 * Copyright 2019 AVL
 *
 * Please contact AVL <avl@eng.ucsd.edu> for any inquiries.
 */
#include "watson_ins_driver.h"
#include "sensor_msgs/NavSatFix.h"
#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string>

#define LOOP_RATE 5
#define QUEUE_LENGTH 20

int main(int argc, char* argv[])
{
  std::string path;
  int baud;
  int timeout_ms;

  ros::init(argc, argv, "watson_ins");

  ros::NodeHandle n;

  // Host Params
  n.param("SerialPath", path, std::string("/dev/ttyUSB0"));
  n.param("SerialBaud", baud, 9600);
  n.param("SerialTimeout", timeout_ms, 1000);

  WatsonINSDriver INSDrv(path, baud, timeout_ms);

  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu_raw", QUEUE_LENGTH);
  ros::Publisher nav_pub = n.advertise<sensor_msgs::NavSatFix>("/fix", QUEUE_LENGTH);
  ros::Rate loop_rate(LOOP_RATE);
 
  while (ros::ok()) { 
    try {
      bool validIMU, validGPS;
      sensor_msgs::Imu imu_msg;
      sensor_msgs::NavSatFix gps_msg;
      INSDrv.read(imu_msg, gps_msg, validIMU, validGPS);
      if (validIMU) {
        ROS_INFO("Publishing valid IMU message");
        imu_pub.publish(imu_msg);
      }
      if (validGPS) {
        ROS_INFO("Publishing valid GPS message");
        nav_pub.publish(gps_msg);
      }
    } catch (const std::exception& e) {
      ROS_ERROR("Reading from INS driver failed");
    }

    // I don't think this is needed? 
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
