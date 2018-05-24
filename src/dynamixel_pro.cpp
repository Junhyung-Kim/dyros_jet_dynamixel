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
      ComPort = new PortHandlerLinux();
      packetHandler = PacketHandler::getPacketHandler(2.0);
  }

  dynamixel_packet::dynamixel_packet(const char* port_name)
  {
      ComPort = new PortHandlerLinux(port_name);
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


  int dynamixel_packet::make_periodic(int unsigned period, struct periodic_info *info)
  {
    static int next_sig;
    int ret;
    unsigned int ns;
    unsigned int sec;
    struct sigevent sigev;
    timer_t timer_id;
    struct itimerspec itval;
    /* Initialise next_sig first time through. We can't use static
       initialisation because SIGRTMIN is a function call, not a constant */
    if (next_sig == 0)
      next_sig = SIGRTMIN;
    /* Check that we have not run out of signals */
    if (next_sig > SIGRTMAX)
      return -1;
    info->sig = next_sig;
    next_sig++;
    /* Create the signal mask that will be used in wait_period */
    sigemptyset(&(info->alarm_sig));
    sigaddset(&(info->alarm_sig), info->sig);
    /* Create a timer that will generate the signal we have chosen */
    sigev.sigev_notify = SIGEV_SIGNAL;
    sigev.sigev_signo = info->sig;
    sigev.sigev_value.sival_ptr = (void *)&timer_id;
    ret = timer_create(CLOCK_MONOTONIC, &sigev, &timer_id);
    if (ret == -1)
      return ret;
    /* Make the timer periodic */
    sec = period / 1000000;
    ns = (period - (sec * 1000000)) * 1000;
    itval.it_interval.tv_sec = sec;
    itval.it_interval.tv_nsec = ns;
    itval.it_value.tv_sec = sec;
    itval.it_value.tv_nsec = ns;
    ret = timer_settime(timer_id, 0, &itval, NULL);
    return ret;
  }


  void dynamixel_packet::wait_period(struct periodic_info *info)
  {
    int sig;
    sigwait(&(info->alarm_sig), &sig);
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

  void DynamixelPro::setEachRadian(double * pdRadians)
  {
      GroupSyncWrite groupposwrite(ComPort,packetHandler,596,4);
      unsigned int _nParam = 0;
      uint8_t _pbParams[4];
      for (int i = 0; i<nMotorNum; i++)
      {
     //     _pbParams[_nParam++] = vMotorData[i].id;
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
          std::cout << vMotorData[i].id << std::endl;
          std::cout << _pbParams << std::endl;
          groupposwrite.addParam(vMotorData[i].id, _pbParams);
      }
      groupposwrite.txPacket();
  }

  int DynamixelPro::getAllStatus()
  {   GroupSyncRead groupstatread(ComPort,packetHandler,pres_pos,size_12);
      int error;
      int nReceived = 0;
      bool dxl_getdata_result;

      for (int i = 0; i < nMotorNum; i++)
      {
          groupstatread.addParam(vMotorData[i].id);
      }

      error = groupstatread.txRxPacket();
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
         }
      }
      return nReceived;
  }

  void DynamixelPro::dxl_control()
  {
      int i, motorNum = getMotorNum();
      double pdRadians[10] = {0, };
          if(bControlLoopEnable)
          {
              bControlLoopProcessing = true;

              if(bControlWriteEnable)
              {
                  // copy rads
                  for(i=0;i<motorNum;i++)
                  {
                      pdRadians[i] = vMotorData[i].aim_radian;
                  }
                  // release
                  setEachRadian(pdRadians);
              }
              getAllStatus();
              bControlLoopProcessing = false;
          }
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
    std::cout << "sss" << a << std::endl;
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

  void DynamixelPro::setAllTorque( uint8_t nValue)
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
      return posGain;
  }

  int DynamixelPro::setVelocityGain(int index, int nVelocityIGain, int nVelocityPGain, uint8_t* error)
  {
      int velGain;
      uint16_t velo_pgain, velo_igain;
      velo_pgain = nVelocityPGain;
      velo_igain = nVelocityIGain;
      std::cout <<"p"<<std::hex<<velo_pgain <<std::endl;
      std::cout <<"i"<<std::hex<<velo_igain <<std::endl;
      uint32_t vel_gain = ((uint32_t)velo_pgain << 16)| velo_igain;

      std::cout <<std::hex <<vel_gain << std::endl;
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

  long get_real_time()
  {
    struct timespec time;
    long current_time;
    if (clock_gettime(CLOCK_REALTIME, &time) == -1)
    {
      printf ("clock get time error");
    }
    return ((long)time.tv_sec * 1000.0 + (long)time.tv_nsec * 0.001 * 0.001);
  }
