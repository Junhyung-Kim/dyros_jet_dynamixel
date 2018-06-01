
#ifndef DYNAMIXEL_PRO_H_
#define DYNAMIXEL_PRO_H_

#include <sys/time.h>
#include <sys/signal.h>
#include <stdint.h>
#include <pthread.h>
#include "dynamixel_sdk/port_handler_linux.h"
#include "dynamixel_sdk/protocol2_packet_handler.h"
#include "dynamixel_sdk/group_sync_write.h"
#include "dynamixel_sdk/group_sync_read.h"

#define BAUDRATE        3000000
#define posgain_address  594
#define velgain_address  586
#define setPosition_address 596
#define getHomeOffset_address 13
#define pres_pos 611
#define returnDelay 9
#define statusReturn 891
#define setAccel 606
#define setVel 600
#define setTor 562
#define size_1  1
#define size_2  2
#define size_4  4
#define size_12  12

using namespace dynamixel;

struct periodic_info {

  int sig;

  sigset_t alarm_sig;

};

long get_real_time();

namespace DXL_PRO {
    enum dxl_pro_type { H54 = 1, H42 = 2};

    class dynamixel_packet
        {
        private:
            double PacketStartTime;
            double PacketWaitTime;
            double ByteTransferTime;
/*
            int GetBaudrate(int baud_num);

            virtual bool IsPacketTimeout();

            unsigned short UpdateCRC(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
            void AddStuffing(unsigned char *packet);
            void RemoveStuffing(unsigned char *packet);

            int TxP acket(unsigned char *txpacket);
            int RxPacket*(unsigned char *rxpacket);
            int TxRxPacket(unsigned char *txpacket, unsigned char *rxpacket, int *error);
*/
        public:
            PortHandler *ComPort;
            PacketHandler *packetHandler;

            dynamixel_packet();
            dynamixel_packet(const char* port_name);
            virtual ~dynamixel_packet();


            bool Connect();
            void Disconnect();
  /*          bool SetBaudrate(int baud);

            int Ping(int id, int *error);
            int Ping(int id, PingInfo *info, int *error);
            int BroadcastPing(std::vector<PingInfo>& vec_info);
            int Reboot(int id, int *error);
            int FactoryReset(int id, int option, int *error);

            int Read(int id, int address, int length, unsigned char* data, int *error);
            int ReadByte(int id, int address, int *value, int *error);
            int ReadWord(int id, int address, int *value, int *error);
            int ReadDWord(int id, int address, long *value, int *error);

            int Write(int id, int address, int length, unsigned char* data, int *error);
            int WriteByte(int id, int address, int value, int *error);
            int WriteWord(int id, int address, int value, int *error);
            int WriteDWord(int id, int address, long value, int *error);
*/
     //       int SyncRead(int start_addr, int data_length, unsigned char id_length, unsigned char *read_ids, SyncReadData* rxdata, unsigned char *received);
            //void SyncDataFree(unsigned char id_length, PSyncData rxdata);
/*
            int BulkRead(std::vector<BulkReadData>& data);
*/
            static int make_periodic(int unsigned period, struct periodic_info *info);
            static void wait_period(struct periodic_info *info);
        };


        const int H54_180_DEG = 250950; // H54 Resolution per 180 degree
        const int H42_180_DEG = 151900; // H42 Resolution per 180 degree

        const double H54_GEAR_RATIO = 502.0;
        const double H42_GEAR_RATIO = 304.0;

        const double PI = 3.1415926535897932384626433832795;
        const double DEG_TO_H54 = H54_180_DEG / 180.0;
        const double DEG_TO_H42 = H42_180_DEG / 180.0;
        const double RAD_TO_H54 = H54_180_DEG / PI;
        const double RAD_TO_H42 = H42_180_DEG / PI;

        const double H54_TO_DEG = 180.0 / H54_180_DEG;
        const double H42_TO_DEG = 180.0 / H42_180_DEG;
        const double H54_TO_RAD = PI / H54_180_DEG;
        const double H42_TO_RAD = PI / H42_180_DEG;

    struct dxl_gains
    {
        int id;
        int position_p_gain;
        int velocity_p_gain;
        int velocity_i_gain;
    };
    struct dxl_pro_data
        {
            uint8_t id;
            dxl_pro_type type;
            uint8_t updated;
            bool write_enable;
            int32_t position;
            int32_t velocity;
            int16_t current;

            enum update { UPDATED=0, LOST=128 };

            double aim_radian;

            const double position_rad()
            {
                if (type == H54) return position * H54_TO_RAD;
                if (type == H42) return position * H42_TO_RAD;
            }
            const double position_deg()
            {
                if (type == H54) return position * H54_TO_DEG;
                if (type == H42) return position * H42_TO_DEG;
            }
            double velocity_radsec()
            {
                if (type == H54) return (velocity / H54_GEAR_RATIO) * PI * 2 / 60;
                if (type == H42) return (velocity / H42_GEAR_RATIO) * PI * 2 / 60;
            }
            double velocity_degsec()
            {
                if (type == H54) return (velocity / H54_GEAR_RATIO) * 360 / 60;
                if (type == H42) return (velocity / H42_GEAR_RATIO) * 360 / 60;
            }
            double velocity_rpm()
            {
                if (type == H54) return velocity / H54_GEAR_RATIO;
                if (type == H42) return velocity / H42_GEAR_RATIO;
            }
            double current_amp()
            {
                if (type == H54) return current * 33.0 / 2048;
                if (type == H42) return current * 8.250 / 2048;
            }
            double current_mA()
            {
                if (type == H54) return current * 33000.0 / 2048;
                if (type == H42) return current * 8250.0 / 2048;
            }
            double torque()
            {
            }
        };
    struct dxl_inverse  ///< For inversed access
        {
            int channel;
            int index;
        };

    class DynamixelPro : public dynamixel_packet
        {
        private:

            unsigned char pucIDList[253];   ///< Motor ID Datas
            int nMotorNum;                  ///< The number of motos
            int a;

            pthread_t TaskObject;          ///< Real-time task object (pthread)
            pthread_attr_t taskObj;

            bool checkControlLoopEnabled(const char *szSetName);

        public:

            pthread_mutex_t DataMutex;

            int nIndex;
            bool bControlLoopEnable;        ///< For enable control loop
            bool bControlWriteEnable;       ///< For enable writing angles
            bool bControlLoopProcessing;

            DynamixelPro(int index) :
                dynamixel_packet(), nIndex(index), bControlLoopEnable(false), bControlWriteEnable(false)
                {
                        char threadName[40] = {0, };
                        char threadName2[40] = {0, };
                        char mutexName[40] = {0, };
                        sprintf(threadName,"dxl ctr thr %d", index);
                        sprintf(mutexName,"dxl dat mtx %d", index);
                        pthread_mutex_init(&DataMutex, NULL);

                }

            ~DynamixelPro()
            {
              pthread_mutex_destroy(&DataMutex);
            }

            void mutex_acquire()
            { pthread_mutex_lock(&DataMutex); }

            void mutex_release()
            { pthread_mutex_unlock(&DataMutex); }

            dxl_pro_data vMotorData[10];
            dxl_pro_data& operator [] (const int& i) { return vMotorData[i]; }

            //std::vector<dxl_pro_data> vMotorData;   ///< Data of all motors
            void setIDList(int motorNum, dxl_pro_data *motorList);
            void dxl_control();
            void setEachRadian(double* pdRadians);
            int getAllStatus();
            int getMotorNum() { return nMotorNum; }
            void setReturnDelayTime(uint8_t nValue);
            void setAllAcceleration(uint32_t nValue);
            void setAllVelocity(uint32_t nValue);
            void setAllTorque(uint8_t nValue);
            int setPositionGain(int index, int nPositionPGain, uint8_t* error);
            int setVelocityGain(int index, int nVelocityIGain, int nVelocityPGain, uint8_t* error);
            int setAimRadian(int index, double radian, uint8_t* error);
            int getHomingOffset(int index, int nValue, long *value, uint8_t* error);
            int setStatusReturn();

    };
}

#endif // RT_SERIAL_PORT_H
