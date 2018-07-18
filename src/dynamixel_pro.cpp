  /*
   * dynamixel.cpp
   *
   *  Created on: 2012. 12. 26.
   *      Author: zerom
   */

  #include <stdio.h>	//TODO: TEST CODE
  #include <stdlib.h>
  #include <string.h>
  #include <iostream>
  #include "dynamixel_pro.h"

  using namespace DXL_PRO;

  dynamixel_packet::dynamixel_packet()
  {
      ComPort = new PortHandlerLinux(" ");
      packetHandler = PacketHandler::getPacketHandler(2.0);

  }

  dynamixel_packet::dynamixel_packet(const char* port_name)
  {
      ComPort = new PortHandlerLinux(port_name);
      packetHandler = PacketHandler::getPacketHandler(2.0);
  }

  dynamixel_packet::~dynamixel_packet()
  {
      Disconnect();
  }

  bool dynamixel_packet::Connect()
  {
    if(ComPort->openPort() == false)
        {
            printf("Open error");
            // TODO: error print
            return false;
        }

    if (ComPort->setBaudRate(BAUDRATE))
        {
          printf("Succeeded to change the baudrate!\n");
        }
    else
        {
          printf("Failed to change the baudrate!\n");
          printf("Press any key to terminate...\n");
          return false;
        }
    return true;

  }

  void dynamixel_packet::Disconnect()
  {
      ComPort->closePort();
  }

  int dynamixel_packet::make_periodic(unsigned int period, struct periodic_info *info)
  {
    int ret;
    unsigned int ns;
    unsigned int sec;
    int fd;
    struct itimerspec itval;

    /* Create the timer */
    fd = timerfd_create(CLOCK_MONOTONIC, 0);
    info->wakeups_missed = 0;
    info->timer_fd = fd;
    if (fd == -1)
      return fd;

    /* Make the timer periodic */
    sec = period/1000000;
    ns = (period - (sec * 1000000)) * 1000;
    itval.it_interval.tv_sec = sec;
    itval.it_interval.tv_nsec = ns;
    itval.it_value.tv_sec = sec;
    itval.it_value.tv_nsec = ns;
    ret = timerfd_settime(fd, 0, &itval, NULL);
    return ret;
  }

  void dynamixel_packet::wait_period(struct periodic_info *info)
  {
    unsigned long long missed;
    int ret;

    /* Wait for the next timer event. If we have missed any the
       number is written to "missed" */
    ret = read (info->timer_fd, &missed, sizeof (missed));
    if (ret == -1)
    {
      perror ("read timer");
      return;
    }

    /* "missed" should always be >= 1, but just to be sure, check it is not 0 anyway */
   // std::cout << "wakeups_missed" << info->wakeups_missed << std::endl;
    if (missed > 0)
      info->wakeups_missed += (missed - 1);
  }

  bool DynamixelPro::checkControlLoopEnabled(const char *szSetName)
  {
      if(bControlLoopEnable)
      {
          printf("[Error] Cannot set the %s. Control loop is running.\n", szSetName);
          return true;
      }
      while(bControlLoopProcessing) {} // Wait...
      return false;
  }

  void DynamixelPro::setIDList(int motorNum, dxl_pro_data * motorList)
  {
      nMotorNum = motorNum;
      for (int i = 0; i < nMotorNum; i++)
      {
          vMotorData[i] = motorList[i];
          vMotorData[i].position = 0;
          vMotorData[i].velocity = 0;
          vMotorData[i].current = 0;
          pucIDList[i] = motorList[i].id;
      }
  }

  void DynamixelPro:: setEachRadian(double * pdRadians)
  {
      GroupSyncWrite groupposwrite(ComPort,packetHandler,setPosition_address,4);
      unsigned int _nParam = 0;
      uint8_t _pbParams[500];
      for (int i = 0; i<nMotorNum; i++)
      {
          if (vMotorData[i].type == H54)
          {
              int nH54Position = (int)(pdRadians[i] * RAD_TO_H54);
              _pbParams[_nParam++] = DXL_LOBYTE(DXL_LOWORD(nH54Position));
              _pbParams[_nParam++] = DXL_HIBYTE(DXL_LOWORD(nH54Position));
              _pbParams[_nParam++] = DXL_LOBYTE(DXL_HIWORD(nH54Position));
              _pbParams[_nParam++] = DXL_HIBYTE(DXL_HIWORD(nH54Position));
          }
          else
          {
              int nH42Position = (int)(pdRadians[i] * RAD_TO_H42);
              _pbParams[_nParam++] = DXL_LOBYTE(DXL_LOWORD(nH42Position));
              _pbParams[_nParam++] = DXL_HIBYTE(DXL_LOWORD(nH42Position));
              _pbParams[_nParam++] = DXL_LOBYTE(DXL_HIWORD(nH42Position));
              _pbParams[_nParam++] = DXL_HIBYTE(DXL_HIWORD(nH42Position));
          }
          groupposwrite.addParam(vMotorData[i].id, _pbParams);
      }
      groupposwrite.txPacket();
      groupposwrite.clearParam();
  }

  int DynamixelPro::getAllStatus()
  {   GroupSyncRead groupstatread(ComPort,packetHandler,pres_pos,size_12);
      int error;
      int nReceived = 0;
      for (int i = 0; i < nMotorNum; i++)
      {
        groupstatread.addParam(vMotorData[i].id);
      }
      error = groupstatread.txRxPacket();
      mutex_acquire();
      for (int i = 0; i< nMotorNum ;i++)
      {
         uint8_t error1[1];

         groupstatread.getError(vMotorData[i].id, error1);
         vMotorData[i].updated = error1[0];
         if(vMotorData[i].updated == 0)
         {
           vMotorData[i].position = groupstatread.getData(vMotorData[i].id,pres_pos,size_4);

           vMotorData[i].velocity = groupstatread.getData(vMotorData[i].id,pres_pos+4,size_4);

           vMotorData[i].current = groupstatread.getData(vMotorData[i].id,pres_pos+4+4+2,size_2);

           nReceived++;
         }
         else
         {
       //   std::cout << "[WARN] Failed to receieve the response. (ID: " << (int)vMotorData[i].id << ")" << std::endl;
         }
      }
      mutex_release();
      return nReceived;
  }

  void DynamixelPro::setReturnDelayTime(uint8_t nValue)
  {    
    dynamixel::GroupSyncWrite groupDelayWrite(ComPort, packetHandler, returnDelay, size_1);
    int a;
    if(checkControlLoopEnabled("return delay time"))  { return; }
    unsigned int _nParam = 0;
    uint8_t _pbParams[1];

    for (int i = 0; i<2; i++)
    {
        for (int j = 0; j<nMotorNum; j++)
        {
            _pbParams[_nParam] = nValue;
            groupDelayWrite.addParam(vMotorData[j].id,_pbParams);
        }
    }
    a=groupDelayWrite.txPacket();
    groupDelayWrite.clearParam();
  }

  void DynamixelPro::setAllAcceleration(uint32_t nValue)
  {
    dynamixel::GroupSyncWrite groupAccelWrite(ComPort, packetHandler,setAccel, size_4);

    unsigned int _nParam = 0;
    uint8_t _pbParams[4];
      if(checkControlLoopEnabled("Acceleration"))  { return; }
      for (int i = 0; i<2; i++)
      {
          // Set Goal Acc
          for (int j = 0; j<nMotorNum; j++)
          {
            _nParam = 0;
            _pbParams[_nParam++] = DXL_LOBYTE(DXL_LOWORD(nValue));
            _pbParams[_nParam++] = DXL_HIBYTE(DXL_LOWORD(nValue));
            _pbParams[_nParam++] = DXL_LOBYTE(DXL_HIWORD(nValue));
            _pbParams[_nParam++] = DXL_HIBYTE(DXL_HIWORD(nValue));
            groupAccelWrite.addParam(vMotorData[j].id,_pbParams);
          }
      }
      int a;
      groupAccelWrite.txPacket();
      groupAccelWrite.clearParam();

  }

  void DynamixelPro::setAllVelocity(uint32_t nValue)
  {
    dynamixel::GroupSyncWrite groupVelWrite(ComPort, packetHandler,setVel, size_4);

    unsigned int _nParam = 0;
    uint8_t _pbParams[4];
      if(checkControlLoopEnabled("Acceleration"))  { return; }
      for (int i = 0; i<2; i++)
      {
          // Set Goal Acc
          for (int j = 0; j<nMotorNum; j++)
          {
            _nParam = 0;
            _pbParams[_nParam++] = DXL_LOBYTE(DXL_LOWORD(nValue));
            _pbParams[_nParam++] = DXL_HIBYTE(DXL_LOWORD(nValue));
            _pbParams[_nParam++] = DXL_LOBYTE(DXL_HIWORD(nValue));
            _pbParams[_nParam++] = DXL_HIBYTE(DXL_HIWORD(nValue));
            groupVelWrite.addParam(vMotorData[j].id,_pbParams);
          }
      }
      groupVelWrite.txPacket();
      groupVelWrite.clearParam();
  }

  void DynamixelPro::setAllTorque(uint8_t nValue)
  {
    dynamixel::GroupSyncWrite groupTorWrite(ComPort, packetHandler,setTor, size_1);

    unsigned int _nParam = 0;
    uint8_t _pbParams[1];

      if(checkControlLoopEnabled("Acceleration"))  { return; }
      for (int i = 0; i<2; i++)
      {   _nParam = 0;
          // Set Goal Acc
          for (int j = 0; j<nMotorNum; j++)
          {
            _pbParams[_nParam] = nValue;
            groupTorWrite.addParam(vMotorData[j].id,_pbParams);
          }
      }
      int err;
      err= groupTorWrite.txPacket();
      groupTorWrite.clearParam();
  }

  int DynamixelPro::setPositionGain(int index, int nPositionPGain, uint8_t* error)
  {
      int posGain;
      uint16_t pos_gain;
      pos_gain = nPositionPGain;
      if(checkControlLoopEnabled("position gain"))  { return 1; }
      posGain=packetHandler->write2ByteTxRx(ComPort, vMotorData[index].id, posgain_address, pos_gain, error);
     std::cout << "id " << index << " dd " << (int)vMotorData[index].id<< std::endl;
      return posGain;
  }

  int DynamixelPro::setVelocityGain(int index, int nVelocityIGain, int nVelocityPGain, uint8_t* error)
  {
      int velGain;
      uint16_t velo_pgain, velo_igain;
      velo_pgain = nVelocityPGain;
      velo_igain = nVelocityIGain;
      uint32_t vel_gain = ((uint32_t)velo_igain << 16)| velo_pgain;
      std::cout << "Setvel" << std::endl;
      if(checkControlLoopEnabled("get velocity gain"))  { return 1; }
      velGain = packetHandler->write4ByteTxRx(ComPort, vMotorData[index].id, velgain_address, vel_gain, error);
      return velGain;
  }

  int DynamixelPro::getHomingOffset(int index, int nValue, long *value, uint8_t* error)
  {
      uint32_t *data_1;
      int getHomeOffset;
      if(checkControlLoopEnabled("homing offset"))  { return 1; }
      getHomeOffset = packetHandler->read4ByteTxRx(ComPort, vMotorData[index].id, getHomeOffset_address, data_1, error);
      value = (long *)data_1;
      return getHomeOffset;//ReadDWord(vMotorData[index].id, 13, value, error);
  }

  int DynamixelPro::setAimRadian(int index, double radian, uint8_t *error)
  {
      int setAim;
      if(checkControlLoopEnabled("aim radian"))  { return 1; }
      vMotorData[index].aim_radian = radian;
      uint32_t position;
      if (vMotorData[index].type == H54)
      {
          position = (uint32_t)(radian * RAD_TO_H54);
      }
      else if (vMotorData[index].type == H42)
      {
          position = (uint32_t)(radian * RAD_TO_H42);
      }
      setAim = packetHandler->write4ByteTxRx(ComPort, vMotorData[index].id, setPosition_address, position, error);
      return setAim;
  }

  int DynamixelPro::setStatusReturn()
  {
    int err;
    for (int i = 0; i<2; i++)
    {
        for (int j = 0; j<nMotorNum; j++)
        {
            err= packetHandler->write1ByteTxOnly(ComPort, vMotorData[j].id, statusReturn, 1);
        }
    }
  }

  unsigned long get_real_time()
  {
    struct timespec time;
    unsigned long current_time;
    if (clock_gettime(CLOCK_REALTIME, &time) == -1)
    {
      printf ("clock get time error");
    }
    return ((unsigned long)time.tv_sec * 1000.0 + (unsigned long)time.tv_nsec * 0.001 * 0.001);
  }

  void* dxl_control(void *parent)
  {
      DynamixelPro *pDynamixelObj = (DynamixelPro*)parent;
      int i, motorNum =  pDynamixelObj->getMotorNum();
      double pdRadians[10] = {0, };
      struct periodic_info info3;
  //    std::cout << "motorNum" << motorNum << std::endl;
      dynamixel_packet::make_periodic(50000, &info3);
        while(1)
        {
          if(pDynamixelObj->bControlLoopEnable)
          { int a = get_real_time();
          //  std::cout << "wait" << a << std::endl;
            dynamixel_packet::wait_period(&info3);
            a = get_real_time();
       //     std::cout << "Wait Finisti" << a << std::endl;
            pDynamixelObj->bControlLoopProcessing = true;
            pDynamixelObj->LoopStartTime = get_real_time();
            pDynamixelObj->LoopTimeoutTime = pDynamixelObj->LoopStartTime + 4.7; //4.7ms
          }

         if(pDynamixelObj->bControlWriteEnable)
          {
              pDynamixelObj->mutex_acquire();
              for(i=0;i<motorNum;i++)
              { // std::cout <<"dsafsdfa" << std::endl;
                 pdRadians[i] = (*pDynamixelObj).vMotorData[i].aim_radian;
                 std::cout <<"dsafsdfa    "<< i << "  " << pdRadians[i]<< std::endl;

              }
              pDynamixelObj->mutex_release();
              pDynamixelObj->setEachRadian(pdRadians);
        //  std::cout << " ssssss" << std::endl;
          }
          pDynamixelObj->getAllStatus();
          pDynamixelObj->bControlLoopProcessing = false;
        }
  }

