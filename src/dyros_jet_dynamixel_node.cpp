#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include "dxl_lists.h"
#include "rt_ros_service.h"

int main(int argc, char **argv)
{
  if(dxl_initailize() == false) return -1;

  if(dynamixel_motor_init() == false) return -1;


//   ros::init(argc, argv, “dyros_jet_dynamixel_node”);
//   ros::NodeHandle nh;
 //  ROS_INFO(“ssss”);
 /*  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>(“say_hello_world”, 1000);
   ros::Rate loop_rate(10);
   int count = 0;
   while (ros::ok())
   {
       std_msgs::String msg;
       std::stringstream ss;
       ss << “hello world” << count;
       msg.data = ss.str();
       ROS_INFO(“%s”, msg.data.c_str());
       chatter_pub.publish(msg);
       ros::spinOnce();
       loop_rate.sleep();
       ++count;
   }*/
   return 0;
}
