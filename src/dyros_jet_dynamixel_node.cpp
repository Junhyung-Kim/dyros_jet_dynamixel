#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/String.h>
#include <stdio.h>

#include "dynamixel_pro.h"
#include "rt_dynamixel_msgs/JointState.h"
#include "rt_dynamixel_msgs/JointSet.h"
#include "rt_dynamixel_msgs/ModeSetting.h"
#include "rt_dynamixel_msgs/MotorSetting.h"

#include "dxl_lists.h"
#include "rt_ros_service.h"

int main(int argc, char **argv)
{
//  ros::init(argc, argv, "dyros_jet_dynamixel_node");
//  ros::NodeHandle nh;

  if(dxl_initailize() == false) return -1;

  if(dynamixel_motor_init() == false) return -1;

  //motor_test();


  /*
  RTROSPublisher rtRosPublisher(nh);
  RTROSSubscriber rtRosSubscriber(nh);
  RTROSMotorSettingService rtRosMotorSettingService(nh);
*/
//  ros::spin();

  return 0;
}
