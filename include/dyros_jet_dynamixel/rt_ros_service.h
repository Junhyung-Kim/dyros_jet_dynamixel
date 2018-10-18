
#ifndef RT_ROS_SERVICE_H_
#define RT_ROS_SERVICE_H_


#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>

#include <pthread.h>

#include "dynamixel_pro.h"
#include "rt_dynamixel_msgs/JointState.h"
#include "rt_dynamixel_msgs/JointSet.h"
#include "rt_dynamixel_msgs/ModeSetting.h"
#include "rt_dynamixel_msgs/MotorSetting.h"

#include "dxl_lists.h"

using namespace DXL_PRO;

// global ----------------------------------------------
extern DynamixelPro dxlDevice[8];

extern int nTotalMotors;
extern int nDXLCount[8];
extern dxl_inverse dxlID2Addr[60]; // max ID = 50,

extern dxl_pro_data& dxl_from_id(int id);

// ------------------------------------------------------


// rt_task_proc -----------------------------------------
void* publisher_proc(void *arg);
void* subscribe_proc(void *arg);
void* motor_set_proc(void *arg);
// ------------------------------------------------------

class RTROSPublisher
{
    pthread_t TaskPublish;
    pthread_attr_t taskPub;
    struct sched_param param1;
public:
    realtime_tools::RealtimePublisher<rt_dynamixel_msgs::JointState> pubState;

    RTROSPublisher(ros::NodeHandle &nh);
    virtual ~RTROSPublisher()
    {        pthread_detach(TaskPublish);    }
    void start()
    {        pthread_create(&TaskPublish, &taskPub, &publisher_proc, this);    }

};

class RTROSSubscriber
{
    pthread_t TaskSubscriber;
    pthread_attr_t taskSub;
    struct sched_param param2;

private:
    void JointCallback(const rt_dynamixel_msgs::JointSetConstPtr msg);

public:
    ros::Subscriber subSetter;
    rt_dynamixel_msgs::JointSetConstPtr recMsg;
    RTROSSubscriber(ros::NodeHandle &nh);
    virtual ~RTROSSubscriber()
    {        pthread_detach(TaskSubscriber);    }

    void start()
    {        pthread_create(&TaskSubscriber, &taskSub, &subscribe_proc, this);    }

};

class RTROSMotorSettingService
{
private:

    bool modeSwitch(rt_dynamixel_msgs::ModeSettingRequest &req,
                    rt_dynamixel_msgs::ModeSettingResponse &res);

    bool motorSet(rt_dynamixel_msgs::MotorSettingRequest &req,
                    rt_dynamixel_msgs::MotorSettingResponse &res);

public:
    ros::ServiceServer modeServer;
    ros::ServiceServer motorServer;

    rt_dynamixel_msgs::MotorSettingRequest motorRequest;
    rt_dynamixel_msgs::MotorSettingResponse motorResponse;

    RTROSMotorSettingService(ros::NodeHandle &nh);
    virtual ~RTROSMotorSettingService()
    {           }

};



#endif // RT_ROS_SERVICE_H_
