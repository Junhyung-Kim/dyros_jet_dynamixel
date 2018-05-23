    #include "rt_ros_service.h"

//RTIME control_period = 25e5;

RTROSPublisher::RTROSPublisher(ros::NodeHandle &nh)
{
    pubState.initialize(nh.advertise<rt_dynamixel_msgs::JointState>("rt_dynamixel/joint_state",1),
                        1, rt_dynamixel_msgs::JointState());

    pthread_attr_init(&rt_taskPub);
    pthread_attr_setschedpolicy(&rt_taskPub, SCHED_FIFO);
    param1.sched_priority = 2;
    pthread_attr_setschedparam(&rt_taskPub, &param1);

    jointMsg = pubState.allocate();

    jointMsg->id.resize(nTotalMotors);
    jointMsg->angle.resize(nTotalMotors);
    jointMsg->velocity.resize(nTotalMotors);
    jointMsg->current.resize(nTotalMotors);
    jointMsg->updated.resize(nTotalMotors);

    int _cnt=0;
    for(int i=0;i<4;i++)
        for(int j=0;j<nDXLCount[i];j++)
        {
            jointMsg->id[_cnt++] = dxlLists[i][j].id;
        }
}

RTROSSubscriber::RTROSSubscriber(ros::NodeHandle &nh)
{
    pthread_attr_init(&rt_taskSub);
    pthread_attr_setschedpolicy(&rt_taskSub, SCHED_FIFO);
    param2.sched_priority = 1;
    pthread_attr_setschedparam(&rt_taskSub, &param2);

    subSetter.initialize(3,nh,"rt_dynamixel/joint_set");
}


RTROSMotorSettingService::RTROSMotorSettingService(ros::NodeHandle &nh)
{
    modeServer = nh.advertiseService("rt_dynamixel/mode",&RTROSMotorSettingService::modeSwitch,this);
    motorServer = nh.advertiseService("rt_dynamixel/motor_set",&RTROSMotorSettingService::motorSet,this);
}

bool RTROSMotorSettingService::modeSwitch(rt_dynamixel_msgs::ModeSettingRequest &req,
                rt_dynamixel_msgs::ModeSettingResponse &res)
{

    res.result = -1;
    switch (req.mode)
    {
    case rt_dynamixel_msgs::ModeSettingRequest::CONTROL_RUN:
        for(int i=0;i<4;i++)
        {
            dxlDevice[i].bControlWriteEnable = true;
        }
        res.result =  rt_dynamixel_msgs::ModeSettingRequest::CONTROL_RUN;
        break;

    case rt_dynamixel_msgs::ModeSettingRequest::DISABLE:
        for(int i=0;i<4;i++)
        {
            dxlDevice[i].bControlWriteEnable = false;
        }

        // wait for process end
        for(int i=0;i<4;i++)
            while(dxlDevice[i].bControlLoopProcessing) {}
        res.result =  rt_dynamixel_msgs::ModeSettingRequest::DISABLE;
        break;

    case rt_dynamixel_msgs::ModeSettingRequest::SETTING:
        for(int i=0;i<4;i++)
        {
            dxlDevice[i].bControlWriteEnable = false;
        }

        // wait for process end
        for(int i=0;i<4;i++)
            while(dxlDevice[i].bControlLoopProcessing) {}
        res.result =  rt_dynamixel_msgs::ModeSettingRequest::SETTING;

        break;

    default:
        break;
    }


    return true;
}


bool RTROSMotorSettingService::motorSet(rt_dynamixel_msgs::MotorSettingRequest &req,
                rt_dynamixel_msgs::MotorSettingResponse &res)
{
    for(int i=0; i<4; i++)
    {
        dxlDevice[i].bControlLoopEnable = false;
    }
    for(int i=0; i<4; i++)
    {
        while(dxlDevice[i].bControlLoopProcessing) {}
    }


    pthread_t rttMotorSetTask;
    pthread_attr_t rt_motorSet;
    struct sched_param param3;

    pthread_attr_init(&rt_motorSet);
    pthread_attr_setschedpolicy(&rt_motorSet, SCHED_FIFO);
    param3.sched_priority = 0;
    pthread_attr_setschedparam(&rt_motorSet, &param3);

    pthread_create(&rttMotorSetTask, &rt_motorSet, &motor_set_proc, 0);
    motorResponse.result = -1;

    motorRequest = req;
    pthread_detach(rttMotorSetTask);

    res = motorResponse;
    //res.result = req.mode;

    for(int i=0; i<4; i++)
    {
        dxlDevice[i].bControlLoopEnable = true;
    }
    return true;
}

//////////////////////////////////////////////////////
/// \brief publisher_proc
/// \param arg
///
///
///

void* publisher_proc(void *arg)
{
    RTROSPublisher* pObj = (RTROSPublisher*)arg;
    int i,j;
    struct periodic_info info1;

    dynamixel_packet::make_periodic(5000, &info1);
    // 1e6 -> 1ms   5e5 -> 500us

    while (1)
    {
         dynamixel_packet::wait_period(&info1); //wait for next cycle

        // Data set
        for(i=0;i<4;i++)
        {//JH
         //   dxlDevice[i].mutex_acquire();
        }

        int _cnt = 0;
        for(i=0;i<4;i++)
        {
            for(j=0;j<nDXLCount[i];j++)
            {
                // SI
                pObj->jointMsg->angle[_cnt] = dxlDevice[i][j].position_rad();
                pObj->jointMsg->velocity[_cnt] = dxlDevice[i][j].velocity_radsec();
                pObj->jointMsg->current[_cnt] = dxlDevice[i][j].current_amp();
                pObj->jointMsg->updated[_cnt] = dxlDevice[i][j].updated;
                _cnt++;
            }
        }

        for(i=0;i<4;i++)
        {//JH
           // dxlDevice[i].mutex_release();
        }

        pObj->pubState.publish(pObj->jointMsg);
    }
}



void* subscribe_proc(void *arg)
{
    RTROSSubscriber* rtsub = (RTROSSubscriber*)arg;
    struct periodic_info info2;
    dynamixel_packet::make_periodic(1000, &info2);
        // 1e6 -> 1ms
    int i;
    while(1)
    {
        dynamixel_packet::wait_period(&info2); //wait for next cycle

        rt_dynamixel_msgs::JointSetConstPtr rcvMsg = rtsub->subSetter.poll();
        if(rcvMsg) // if message recieved ( if not rcvMsg == NULL )
        {
            // Data set
            // ROS_INFO("Sub ")
            for(i=0;i<4;i++)
            {//jh
              //  dxlDevice[i].mutex_acquire();
            }

            for(i=0;i< (int)rcvMsg->id.size();i++)
            {
                if(check_vaild_dxl_from_id(rcvMsg->id[i]))
                {
                    dxl_from_id(rcvMsg->id[i]).aim_radian = rcvMsg->angle[i];
                }
            }

            for(i=0;i<4;i++)
            {//JH
              //  dxlDevice[i].mutex_release();
            }
        }
    }
}

void* motor_set_proc(void *arg)
{
    RTROSMotorSettingService *pObj = (RTROSMotorSettingService*)arg;
    int channel;
    int index;
    uint8_t error;
    struct timespec tim;
    tim.tv_sec = 0;
    tim.tv_nsec = 5000000;
    switch (pObj->motorRequest.mode)
    {
    case rt_dynamixel_msgs::MotorSettingRequest::SET_TORQUE_ENABLE:
        for (int i=0; i<4; i++)
        {
            dxlDevice[i].setAllTorque(pObj->motorRequest.value);
            pObj->motorResponse.result = pObj->motorRequest.mode;
        }
        break;

    case rt_dynamixel_msgs::MotorSettingRequest::SET_GOAL_POSITION:
        if(check_vaild_dxl_from_id(pObj->motorRequest.id))
        {
            channel = dxlID2Addr[pObj->motorRequest.id].channel;
            index = dxlID2Addr[pObj->motorRequest.id].index;
            dxlDevice[channel].setAimRadian(index,pObj->motorRequest.fvalue,&error);
            pObj->motorResponse.result = pObj->motorRequest.mode;
        }
        break;

    case rt_dynamixel_msgs::MotorSettingRequest::GET_HOMING_OFFSET:
        if(check_vaild_dxl_from_id(pObj->motorRequest.id))
        {
            channel = dxlID2Addr[pObj->motorRequest.id].channel;
            index = dxlID2Addr[pObj->motorRequest.id].index;
            dxlDevice[channel].getHomingOffset(index,pObj->motorRequest.value,&pObj->motorResponse.value,&error);
            pObj->motorResponse.result = pObj->motorRequest.mode;
            //pObj->motorResponse
        }
        break;

    case rt_dynamixel_msgs::MotorSettingRequest::SET_HOMING_OFFSET:
        if(check_vaild_dxl_from_id(pObj->motorRequest.id))
        {
            channel = dxlID2Addr[pObj->motorRequest.id].channel;
            index = dxlID2Addr[pObj->motorRequest.id].index;
            pObj->motorResponse.result = pObj->motorRequest.mode;
        }
        break;

    default:
        break;
    }
    nanosleep(&tim, NULL);
}
