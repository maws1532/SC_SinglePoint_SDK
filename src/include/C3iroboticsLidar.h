/*********************************************************************************
File name:	  C3iroboticsLidar.h
Author:       Kimbo
Version:      V1.7.1
Date:	 	  2017-02-03
Description:  3irobotics lidar sdk
Others:       None

History:
	1. Date:
	Author:
	Modification:
***********************************************************************************/

#ifndef EVEREST_LIDAR_C3IROBOTICSLIDAR_H
#define EVEREST_LIDAR_C3IROBOTICSLIDAR_H

/******************************* Current libs includes ****************************/
#include "CLidarPacket.h"
#include "CLidarUnpacket.h"
#include "CLidarPacketReceiver.h"
#include "CSerialConnection.h"
#include "CSimulateSerial.h"

/******************************* System libs includes *****************************/
#include <vector>


/******************************* Other libs includes ******************************/

namespace dtfeverest
{
	namespace dtfhwdrivers
	{
	    struct TLidarScan
	    {
	        TLidarScan() : distance(), signal(){ }

	        void insert(TLidarScan &scan)
	        {
                this->distance.insert(this->distance.end(), scan.distance.begin(), scan.distance.end());
                this->signal.insert(this->signal.end(), scan.signal.begin(), scan.signal.end());
                this->CurTime.insert(this->CurTime.end(), scan.CurTime.begin(), scan.CurTime.end());
	        }

	        void clear()
	        {
	            distance.clear();
	            signal.clear();
                CurTime.clear();
	        }

            size_t getSize() {return distance.size();}

	        std::vector<float> distance;
	        std::vector<int>   signal;
            std::vector<double> CurTime;
            
	    };
        struct LidarCurrData
        {
            u8          ConfidenceDegr;
            float       dis;
            int         sig;
            double      CTime;
        };
        struct ErrorTimeOut
        {
            bool Invalidflag;
            bool Shieldflag;
        };

        enum TLidarGrabResult
        {
            LIDAR_GRAB_ING = 0,
            LIDAR_GRAB_SUCESS,
            LIDAR_GRAB_ERRO,
            LIDAR_GRAB_ELSE
        };

        enum TLidarCommandID
        {
            I3LIDAR_NEW_DISTANCE = 0x34,
            I3LIDAR_SN = 0x98,
            I3LIDAR_VERSION = 0x99
       };
        enum TLidarVersion
        {
            LIDAR_NONE = 0,
            LIDAR_2_6_K, 
            LIDAR_2_1_K,
            TS12D_LIDAR_3_9_K,
        };

		class C3iroboticsLidar
		{
            public:
                enum TGrabScanState
                {
                    GRAB_SCAN_FIRST = 0,
                    GRAB_SCAN_ELSE_DATA
                };

                enum PWMPolarityState
                {
                    INVERSED = 0,
                    NORMAL
                };

                /* Constructor */
                C3iroboticsLidar();

                /* Destructor */
                ~C3iroboticsLidar();

                /* Set device connect */
                bool initilize(CDeviceConnection *device_connect);

                /* Get scan data */
                TLidarGrabResult getScanData();

                /* Get Lidar Error */
                TLidarError getLidarError() { return m_packet.m_lidar_erro; }

                /* Get lidar scan */
                TLidarScan& getLidarScan() { return m_lidar_scan; }

                /*Get lidar struct data*/
                LidarCurrData getLidardata(){return Current;}

                /* Set data with signal*/
                void setDataWithSignal(bool data_with_signal) {m_data_with_signal = data_with_signal;}

                /* Return true if receive lidar speed */
                bool isReceiveLidarSpeed() {bool flag = m_receive_lidar_speed; m_receive_lidar_speed = false; return flag;}

                /* Get Lidar current speed */
                double getLidarCurrentSpeed() {return m_current_lidar_speed;}

                /*get Lidar expect speed*/
                double GetLidarExpectspeed() {return Expectspeed;}

                /* Enable log when receive timer overs */
                void enableLogWhenReceiveTimeOvers(bool state) {m_receiver.enableLogWhenReceiveTimeOvers(state);}

                /* pwn Write */
                void PwmWriteData(const char *file_name, int64_t data);

                /* pwd Write */
                void PwmWriteData(const char *file_name, const char *data);

                /* pwm Init */
                void PwmInit(PWMPolarityState state);

                /* Control Lidar Speed */
                void controlLidarSpeed();

                /*get pwm*/
                int32_t GetPwm();

                /*get max pwm limit*/
                u8 GetPWMMaxLimit();

                /*set max pwm limit*/
                int SetPWMMaxLimit(u8 limit);
                
                /*Set Lidar Expect speed*/
                int SetLidarExpectSpeed(double speed);

                /*set scan error time*/
                int ScanErrTimeOut(CLidarPacket *packet);

                /*control Lidar Pause*/
                void ControlLidarPause();

                /*connect Liar Start*/
                void ConnectLidarStart();

                /*Set Device Node number*/
                void SetDeviceNodeID(u8 num);

                /*Get Device Node number*/
                u8 GetDeviceNodeID();

                /*Get Lidar SN code*/
                std::string GetLidarSNCode();

                /*Get Lidar SN code length*/
                u8 GetSNCodelen();

                /*get lidar sofware version*/
                std::string GetLidarSoftwareVersion();

                /*Get Lidar Hardware Version*/
                std::string GetLidarHardwareVersion();

                /*Get Lidar Type*/
                std::string GetLidarType();

                /*get lidar information*/
                bool GetDeviceInfo();

                /*set pwm polarity*/
                bool SetPwmpolarity(PWMPolarityState state);

                /*get lidar vesion*/
                std::string  GetLidarversion();

                /*Setlidar versio*/
                void SetLidarversion(TLidarVersion ver);

                /*Get SDK version*/
                std::string  GetSDKVersion(); 

                TLidarGrabResult SendLidarData();



            private:
                /* Analysis packet */
                TLidarGrabResult analysisPacket(CLidarPacket &lidar_packet);

                /* Analysis tooth scan */
                TLidarGrabResult analysisToothScan(CLidarPacket &lidar_packet);

                /* Analysis new tooth scan */
                TLidarGrabResult analysisNewToothScan(CLidarPacket &lidar_packet);

                /* Analysis health info */
                TLidarGrabResult analysisHealthInfo(CLidarPacket &lidar_packet);

                /* Analysis lidar speed */
                TLidarGrabResult analysisLidarSpeed(CLidarPacket &lidar_packet);

                /*Get Lidar SN*/ 
                TLidarGrabResult analysisLidarSN(CLidarPacket &lidar_packet);

                /*Get Lidar Info*/ 
                TLidarGrabResult analysisLidarInfo(CLidarPacket &lidar_packet);

                /* Reset scan grab */
                void resetScanGrab();

                /* Combine scan */
                TLidarGrabResult combineScan(TToothScan &tooth_scan);

                /* Return true if tooth scan is first */
                bool isFirstScan(TToothScan &tooth_scan);

                /* Grab first scan */
                TLidarGrabResult grabFirstScan(TToothScan &tooth_scan);

                /* Change tooth scan to lidar scan */
                void toothScan2LidarScan(TToothScan &tooth_scan, TLidarScan &lidar_scan);

                /* Control Lidar PWM */
                void controlLidarPWM(int32_t percent);//yuy

                /**/
                void LidarTranform(float &Angle, float &Dis);



                /* Params */
                struct TParams
                {
                    /* Constructor */
                    TParams()
                    {
                        packet_wait_time_ms = 100;
                        tooth_number = 16;
                        scan_time_out_ms = 150;
                        speed_time_out = 3500;
                        Shield_time_out = 3500;
                        Invalid_time_out = 3500;
                        GetData_time_out = 5000;
                    }

                    size_t packet_wait_time_ms;
                    size_t scan_time_out_ms;
                    size_t speed_time_out;
                    size_t Shield_time_out;
                    size_t Invalid_time_out;
                    size_t GetData_time_out;
                    int tooth_number;
                };
            public:
                TLidarScan              m_lidar_scan;
                ErrorTimeOut            Error_timeout;
                u8 SNCodelen;
                char pProInfopBuf[256];
                u8 SNCode[256];
                char SoftwareV[96];
                char HardwareV[96];
                char Lidartype[96];
                std::string Lds_str;
                TLidarVersion LidarV;
                LidarCurrData Current;
            private:
                CDeviceConnection       *m_device_connect;
                CLidarPacketReceiver    m_receiver;
                TParams                 m_params;
                //TLidarScan              m_lidar_scan;
                //TLidarError             m_lidar_erro;
                TGrabScanState          m_grab_scan_state;
                PWMPolarityState        m_pwm_polarity_state;
                int                     m_grab_scan_count;
                int                     m_Shield_count;
                float                   m_last_scan_angle;
                CLidarPacket            m_packet;
                TToothScan              m_remainder_tooth_scan;
                bool                    m_remainder_flag;
                bool                    m_data_with_signal;

                bool                    m_receive_lidar_speed;
                double                  m_current_lidar_speed;
                double                  Expectspeed;

                CCountDown              m_data_count_down;
                CCountDown              m_speed_count_down;
                CCountDown              m_stop_count_down;
                CCountDown              m_Shield_count_down;
                CCountDown              m_RangInvalidcount_down;
                CCountDown              m_GetSNcount_down;
                //调整速度相关变量 
                double  error;
                double  last_error;
                double  error_sum;
                int32_t percent;
                bool    speedStableFlag;
                uint8_t countSpeed;
                u8 Node_num;
                u8 MaxPwm;
                std::string SDKVersion;

		};
	}
}

#endif


