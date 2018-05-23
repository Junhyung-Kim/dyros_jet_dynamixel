
#ifndef RT_ROS_SERVICE_H_
#define RT_ROS_SERVICE_H_


#include <ros/ros.h>
#include <rosrt/rosrt.h>

#include <pthread.h>

#include "dynamixel_pro.h"
#include "rt_dynamixel_msgs/JointState.h"
#include "rt_dynamixel_msgs/JointSet.h"
#include "rt_dynamixel_msgs/ModeSetting.h"
#include "rt_dynamixel_msgs/MotorSetting.h"

#include "dxl_lists.h"

using namespace DXL_PRO;

// global ----------------------------------------------
extern DynamixelPro dxlDevice[4];

extern int nTotalMotors;
extern int nDXLCount[4];
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
    pthread_t rttTaskObject;
    pthread_attr_t rt_taskPub;
    struct sched_param param1;
public:
    rosrt::Publisher<rt_dynamixel_msgs::JointState> pubState;
    rt_dynamixel_msgs::JointStatePtr jointMsg;


    RTROSPublisher(ros::NodeHandle &nh);
    virtual ~RTROSPublisher()
    {        pthread_detach(rttTaskObject);    }
    void start()
    {        pthread_create(&rttTaskObject, &rt_taskPub, &publisher_proc, 0);    }

};

class RTROSSubscriber
{
    pthread_t rttSubscriber;
    pthread_attr_t rt_taskSub;
    struct sched_param param2;
public:
    rosrt::Subscriber<rt_dynamixel_msgs::JointSet> subSetter;

    RTROSSubscriber(ros::NodeHandle &nh);
    virtual ~RTROSSubscriber()
    {        pthread_detach(rttSubscriber);    }

    void start()
    {        pthread_create(&rttSubscriber, &rt_taskSub, &subscribe_proc, 0);    }

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
