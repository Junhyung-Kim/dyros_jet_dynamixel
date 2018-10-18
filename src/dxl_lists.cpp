
#include "dxl_lists.h"
#include <iostream>

using namespace DXL_PRO;

// Defines (Debug, Test, ...)
// #define DXL_TEST_SET

#ifdef DXL_TEST_SET
dxl_pro_data dxlLists[4][10] = {
    {
        // Index: 0
        {1, H54},
    },    {
        // Index: 1
    },    {
        // Index: 2
    },    {
        // Index: 3
    }
};   // Max 4channels, 10 motors
#else
dxl_pro_data dxlLists[4][10] = {
    {
        // Index: 0: 1-Right Upper body
        {1, H54},
        {3, H54},
        {5, H54},
        {7, H54},
        {9, H54},
        {11, H42}, // Warning
        {13, H42},


    },    {
        // Index: 1: 2-Left Upper body
        {2, H54},
        {4, H54},
        {6, H54},
        {8, H54},
        {10, H54},
        {12, H42},
        {14, H42},
        {32, H42},

    },    {
        // Index: 2: 3-Right Lower body
        {15, H54},    // Fatal
        {17, H54},    // Fatal
        {19, H54},    // Warning
        {21, H54},    // Warning
        {23, H54},
        {25, H54},
        {27, H54}

    },    {
   // Index: 3: 4-Left Lower body
        {16, H54},
        {18, H54},
        {20, H54},
        {22, H54},
        {24, H54},
        {26, H54},
        {28, H54},

    }
};   // Max 4channels, 10 motors
#endif
// Position P, Velocity P, Velocity I
dxl_gains dxlGains[4][10] =
{
    {
        // Index: 0

        {1, 30,256,10},
        {3, 30,256,10},
        {5, 30,256,10},
        {7, 30,256,10},
        {9, 30,256,10},
        {11, 30,256,10},
        {13, 30,256,10},
       //{31, 15,-1,-1}
    },    {
        // Index: 1
        {2, 30,256,10},
        {4, 30,256,10},
        {6, 30,256,10},
        {8, 30,256,10},
        {10, 30,256,10},
        {12, 30,256,10},
        {14, 30,256,10},
        {32, 30,256,10}
    },    {
        {15,32,500,10},
        {17,32,500,0},
        {19, 32,500,0},
        {21, 32,500,0},
        {23, 32,500,0},
        {25, 32,500,0},
        {27, 64,500,20},
    },    {
        // Index: 3
        {16, 32,500,10},
        {18, 32,500,0},
        {20, 32,500,0},
        {22, 32,500,0},
        {24, 32,500,0},
        {26, 32,500,0},
        {28, 32,500,20},
  }
};

int nTotalMotors;
int nDXLCount[4] = {0, };
dxl_inverse dxlID2Addr[60] = {0, };
DynamixelPro dxlDevice[4] = {0, 1, 2, 3};

void make_dxl_count()
{
    int i,j;
    for(i=0;i<4;i++)
    {
        for(j=0;j<10;j++)
        {
            if(dxlLists[i][j].type == 0)  { break; }
        }
        nDXLCount[i] = j;
    }

    nTotalMotors =  nDXLCount[0] + nDXLCount[1] + nDXLCount[2] + nDXLCount[3];
}

void make_inverse_access_data()
{
    int i,j;
    for(i=0;i<50;i++)
    {
        dxlID2Addr[i].channel = -1;
        dxlID2Addr[i].index = -1;
    }
    for(i=0;i<4;i++)
    {
        for(j=0;j<nDXLCount[i];j++)
        {
            dxlID2Addr[dxlLists[i][j].id].channel = i;
            dxlID2Addr[dxlLists[i][j].id].index = j;
        }
    }
}

bool dxl_initailize()
{
    make_dxl_count();
    make_inverse_access_data();
    dxlDevice[0].ComPort->setPortName("/dev/ttyCTI0");
    dxlDevice[1].ComPort->setPortName("/dev/ttyCTI1");
    dxlDevice[2].ComPort->setPortName("/dev/ttyCTI2");
    dxlDevice[3].ComPort->setPortName("/dev/ttyCTI3");

    int i;
    bool error = 0;

    for(i=0; i<4;i++)
    {
       dxlDevice[i].setIDList(nDXLCount[i], dxlLists[i]);
       error = dxlDevice[i].Connect();
       if(error == false)
       {
           ROS_ERROR("Error on openning serial device: ttyCTI%d",i);
           return false;
        }
        dxlDevice[i].startThread();
    }
    return true;
}

bool check_vaild_dxl_from_id(int id)
{
    if ( dxlID2Addr[id].channel == -1 ) return false;

    return true;
}

dxl_pro_data& dxl_from_id(int id)
{
    return dxlDevice[dxlID2Addr[id].channel][dxlID2Addr[id].index];
}


bool dynamixel_motor_init()
{
    bool isDone;
    uint8_t err;
    motion_init_proc(&isDone);
    return isDone;

}

void motion_init_proc(bool *isDone)
{
  *isDone = true;
  bool isUpdateComplete[4] = {false, };
  int error;
  int nRecv[4] = {0, };
  struct timespec tim, tim1, tim2; //5E6 = 5MS
  tim.tv_sec = 0;
  tim.tv_nsec = 5000000;
  tim1.tv_sec = 0;
  tim1.tv_nsec = 50000000;
  tim2.tv_sec = 0;
  tim2.tv_nsec = 500000000;

  for(int c=0; c<2; c++) //2
  {
    for(int i=0; i<4; i++) //4
    {
      dxlDevice[i].setReturnDelayTime(0);
      nanosleep(&tim,NULL);
      dxlDevice[i].setAllAcceleration(0);
      nanosleep(&tim,NULL);
      dxlDevice[i].setAllVelocity(0);
      nanosleep(&tim,NULL);
      dxlDevice[i].setAllTorque(0);
      nanosleep(&tim,NULL);
      for(int j=0; j<nDXLCount[i]; j++)
      {
        uint8_t err;
        if(dxlDevice[i][j].id == dxlGains[i][j].id)
        {
          if(dxlGains[i][j].position_p_gain  < 0) continue;
          if(dxlGains[i][j].velocity_i_gain  < 0) continue;
          if(dxlGains[i][j].velocity_p_gain  < 0) continue;
           dxlDevice[i].setPositionGain(j,dxlGains[i][j].position_p_gain,&err);
          nanosleep(&tim,NULL);

          dxlDevice[i].setVelocityGain(j,dxlGains[i][j].velocity_i_gain,dxlGains[i][j].velocity_p_gain, &err);
          nanosleep(&tim,NULL);
        }
        else
        {
          ROS_ERROR("No match between Devices ID and Gain datas");
        }

      }
    }
  }
  for(int i=0;i<4;i++) //4
      {
          dxlDevice[i].setStatusReturn();
          ROS_INFO("chennal... %d",i);
          for(int c=0; c<10;c++)
          {
              nRecv[i] = dxlDevice[i].getAllStatus();
              if(nRecv[i] == dxlDevice[i].getMotorNum())
              {
                  isUpdateComplete[i] = true;
              }
              else
              {
                ROS_INFO("ID: %d Motor seems to be dead?",dxlDevice[i][nRecv[i]].id);
              }
               nanosleep(&tim1,NULL);
          }
           nanosleep(&tim1,NULL);
      }

  // check all motor read
      for(int i=0;i<4;i++)
          if(isUpdateComplete[i] == false)
          {
            *isDone = false;
             return ;
          }

      // Update all radian angle
      for(int i=0;i<4; i++)
          for(int j=0;j<dxlDevice[i].getMotorNum(); j++)
          {
                dxlDevice[i][j].aim_radian = dxlDevice[i][j].position_rad();

              //  std::cout << "device" << (double)dxlDevice[i][j].aim_radian << std::endl;
          }
      *isDone = true;
}
