/*********************************************************************************
File name:	  CLidarPacketReceiver.h
Author:       Kimbo
Version:      V1.7.1
Date:	 	  2017-02-03
Description:  lidar packet receiver
Others:       None

History:
	1. Date:
	Author:
	Modification:
***********************************************************************************/
#ifndef EVEREST_LIDAR_CLIDARPACKETRECEIVER_H
#define EVEREST_LIDAR_CLIDARPACKETRECEIVER_H

/******************************* Current libs includes ****************************/
#include "CLidarPacket.h"
#include "CLidarUnpacket.h"

/******************************* Current libs includes ****************************/
#include "CCountDown.h"

/******************************* System libs includes *****************************/
#include <vector>
#include <fstream>

namespace dtfeverest
{
	namespace dtfhwdrivers
	{
        class CDeviceConnection;

		class CLidarPacketReceiver
		{
            public:
                /* Constructor */
                CLidarPacketReceiver();

                /* Destructor */
                ~CLidarPacketReceiver();

                /* Receive lidar packet, if return true, it means receive a valid packet */
                bool receivePacket(CLidarPacket *packet);

				/* Sets the device this instance receives packets from */
				void setDeviceConnection(CDeviceConnection *device_connection) { m_device_conn = device_connection; }

				/* Gets the device this instance receives packets from */
				CDeviceConnection *getDeviceConnection(void)  { return m_device_conn; }

                enum TState
                {
                    STATE_HEADER1 = 0,
                    STATE_HEADER2,
                    STATE_LENGHT,
                    STATE_ACQUIRE_DATA
                };

                enum TPacketResult
                {
                    PACKET_ING = 0,
                    PACKET_SUCCESS,
                    PACKET_FAILED
                };

                enum DataState
                {
                    GetData_ING = 0,
                    GetData_SUCCESS,
                    GetData_FAILED
                };
                
                
                /*Set lidar SN flag*/
                void SetSNFlag(DataState state);

                /*Get lidar SN flag*/
                DataState GetSNFlag(void);

                /*Set lidar Info flag*/
                void SetInfoFlag(DataState state);

                /*Get lidar Info flag*/
                DataState GetInfoFlag(void);

                /* Enable log when receive timer overs */
                void enableLogWhenReceiveTimeOvers(bool state) {m_log_when_receive_time_over = state;}

                bool SendContrSNData();

                bool SendContrInfoData();

                bool SendContrContinueRang();

			private:
				/* Read packet, if it return ture, it means read complete packet */
				TPacketResult readPacket(CLidarPacket *packet, u8 ch);

				/* process state header1 */
				TPacketResult processStateHeader1(CLidarPacket *packet, u8 ch);

				/* process state header2 */
				TPacketResult processStateHeader2(CLidarPacket *packet, u8 ch);

				/* Process state length */
				TPacketResult processStateLength(CLidarPacket *packet, u8 ch);

				/* Process acuquire data */
				TPacketResult processStateAcquireData(CLidarPacket *packet, u8 ch);

                void reset();

            public:
                struct TParams
                {
                    /* Constructor */
                    TParams()
                    {
                        packet_max_time_ms = 1000;//3500;
                        packet_wait_time_ms = 100;
                        CRC_max_time_ms = 1000;//3500;
                        Data_Wrong_time_out = 1000;//3500;
                    }

                    size_t packet_max_time_ms;
                    size_t packet_wait_time_ms;
                    size_t CRC_max_time_ms;
                    size_t Data_Wrong_time_out;
                };

            private:
                CDeviceConnection 	*m_device_conn;
                CCountDown          m_count_down;
                CCountDown          Timeout_Data_Wrong;
                bool                m_error_Data_Wrong;
                CCountDown          Timeout_CRC;
                bool                m_error_crc;
                TParams             m_params;
                TState              m_state;
                DataState           m_InfoState;
                DataState           m_SNstate;      
                int                 m_actual_count;
                int                 m_packet_length;
                std::ofstream       m_save_fp;
                size_t              m_counter;
                bool                m_log_when_receive_time_over;
		};
	}
}

#endif


