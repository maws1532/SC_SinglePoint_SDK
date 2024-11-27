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

/********************************* File includes **********************************/
#include "CLidarPacketReceiver.h"

/******************************* Current libs includes ****************************/
#include "CDeviceConnection.h"
#include "CCountDown.h"

/********************************** Name space ************************************/
using namespace dtfeverest;
using namespace dtfeverest::dtfhwdrivers;

#define LIDAR_PACEKT_HEADER1 0xAA
#define LIDAR_PACEKT_HEADER2 0x00

/***********************************************************************************
Function:     CLidarPacketReceiver
Description:  The constructor of CLidarPacketReceiver
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarPacketReceiver::CLidarPacketReceiver()
{
    m_device_conn = NULL;
    m_log_when_receive_time_over = false;
    reset();
    m_counter = 0;
    m_InfoState = GetData_ING;
    m_SNstate = GetData_ING;
}

/***********************************************************************************
Function:     CLidarPacketReceiver
Description:  The destructor of CLidarPacketReceiver
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarPacketReceiver::~CLidarPacketReceiver()
{
    if(m_save_fp)
    {
        m_save_fp.close();
    }
}
/***********************************************************************************
Function:     SendContrSNData
Description:  Send lidar packet,
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool CLidarPacketReceiver::SendContrSNData()
{
    if(m_device_conn == NULL || m_device_conn->getStatus() != CDeviceConnection::STATUS_OPEN)
	{
		printf("[CLidarPacketReceiver] receivePacket: connection not open!\n");
		return false;
	}
    // printf("[CLidarPacketReceiver] Send SN CMD!\n");
    char SNdata[128];
    u8 ByteNum = 0;
    SNdata[ByteNum++] = 0xAA;
    SNdata[ByteNum++] = 0x00;
    SNdata[ByteNum++] = 0x08;
    SNdata[ByteNum++] = 0x10;
    SNdata[ByteNum++] = 0x61;
    SNdata[ByteNum++] = 0x98;
    SNdata[ByteNum++] = 0x00;
    SNdata[ByteNum++] = 0x00;
    SNdata[ByteNum++] = 0x01;
    SNdata[ByteNum++] = 0xBB;
    m_device_conn->write(SNdata, ByteNum);
    return 0;
}
/***********************************************************************************
Function:     SendContrInfoData
Description:  Send lidar packet,
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool CLidarPacketReceiver::SendContrInfoData()
{
    if(m_device_conn == NULL || m_device_conn->getStatus() != CDeviceConnection::STATUS_OPEN)
	{
		printf("[CLidarPacketReceiver] receivePacket: connection not open!\n");
		return false;
	}
    // printf("[CLidarPacketReceiver] Send info CMD!\n");
    char SNdata[128];
    u8 ByteNum = 0;
    SNdata[ByteNum++] = 0xAA;
    SNdata[ByteNum++] = 0x00;
    SNdata[ByteNum++] = 0x08;
    SNdata[ByteNum++] = 0x10;
    SNdata[ByteNum++] = 0x61;
    SNdata[ByteNum++] = 0x99;
    SNdata[ByteNum++] = 0x00;
    SNdata[ByteNum++] = 0x00;
    SNdata[ByteNum++] = 0x01;
    SNdata[ByteNum++] = 0xBC;
    m_device_conn->write(SNdata, ByteNum);
    return 0;
}
/***********************************************************************************
Function:     SendContrInfoData
Description:  Send lidar packet,
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool CLidarPacketReceiver::SendContrContinueRang()
{
    if(m_device_conn == NULL || m_device_conn->getStatus() != CDeviceConnection::STATUS_OPEN)
	{
		printf("[CLidarPacketReceiver] receivePacket: connection not open!\n");
		return false;
	}
    // printf("[CLidarPacketReceiver] Send connect CMD!\n");
    char SNdata[128];
    u8 ByteNum = 0;
    SNdata[ByteNum++] = 0xAA;
    SNdata[ByteNum++] = 0x00;
    SNdata[ByteNum++] = 0x08;
    SNdata[ByteNum++] = 0x10;
    SNdata[ByteNum++] = 0x61;
    SNdata[ByteNum++] = 0x34;
    SNdata[ByteNum++] = 0x00;
    SNdata[ByteNum++] = 0x00;
    SNdata[ByteNum++] = 0x01;
    SNdata[ByteNum++] = 0x57;
    m_device_conn->write(SNdata, ByteNum);
    return 0;
}
/***********************************************************************************
Function:     receivePacket
Description:  Receive lidar packet, if return true, it means receive a valid packet
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool CLidarPacketReceiver::receivePacket(CLidarPacket *packet)
{
	/* Judge whether serial is connecting */
	if (packet == NULL || m_device_conn == NULL || m_device_conn->getStatus() != CDeviceConnection::STATUS_OPEN)
	{
		printf("[CLidarPacketReceiver] receivePacket: connection not open!\n");
		return false;
	}
    /* Read packet */
	m_count_down.setTime((double)m_params.packet_max_time_ms);
	char ch;
	while(1)
	{
		if(m_count_down.isEnd())
		{
			//printf("[CLidarPacketReceiver] Receive packet time %5.2f ms is over!\n", m_count_down.getInputTime());
            //printf("(LINE:%d)-<FUNCTION:%s> revice data over\n",__LINE__,__FUNCTION__);
            packet->m_lidar_erro = LIDAR_ERROR_TIME_OVER;
			if(m_log_when_receive_time_over)
            {
                printf("[CLidarPacketReceiver] Receive packet time is over!\n");
            }
            TPacketResult packet_result = readPacket(packet, ch);
			return false;
		}
        if(LIDAR_ERROR_TIME_OVER == packet->m_lidar_erro)
        {
            packet->m_lidar_erro = LIDAR_ERROR_NONE;
        }
        
        int read_bytes = m_device_conn->read((char *)&ch, 1, 1);
		if(read_bytes == 0)
		{
            continue;
		}
		else if(read_bytes < 0)
		{
		    printf("[CLidarPacketReceiver] finish read data read bytes is %d!\n", read_bytes);
		    return false;
		}
		else
		{
		    TPacketResult packet_result = readPacket(packet, ch);
            // printf("%02x\n", ch);
		    switch(packet_result)
		    {
                case PACKET_ING: break;
		        case PACKET_SUCCESS:
		        {
                    reset();
		            return true;
		        }
		        case PACKET_FAILED:
		        {
                    reset();
		            return false;
		        }
		    }
		}
	}

	printf("[CLidarPacketReceiver] It should not come to here!\n");
	return false;
}

/***********************************************************************************
Function:     readPacket
Description:  Read packet, if it return ture, it means read complete packet or enter
              erro state
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarPacketReceiver::TPacketResult CLidarPacketReceiver::readPacket(CLidarPacket *packet, u8 ch)
{
    TPacketResult packet_result = PACKET_ING;
    switch(m_state)
    {
        case STATE_HEADER1: packet_result = processStateHeader1(packet, ch); break;
        case STATE_HEADER2: packet_result = processStateHeader2(packet, ch); break;
        case STATE_LENGHT: packet_result =  processStateLength(packet, ch); break;
        case STATE_ACQUIRE_DATA: packet_result = processStateAcquireData(packet, ch); break;
        default:
            printf("[CLidarPacketReceiver] Enter erro state %d!\n", m_state);
        break;
    }
    return packet_result;
}

/***********************************************************************************
Function:     processStateHeader1
Description:  Proces state header1
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarPacketReceiver::TPacketResult CLidarPacketReceiver::processStateHeader1(CLidarPacket *packet, u8 ch)
{
    if(ch == LIDAR_PACEKT_HEADER1)
    {
        packet->reset();
        packet->pushBack(ch);
        m_state = STATE_HEADER2;
        m_count_down.setTime(m_params.packet_wait_time_ms);

    }    
    return PACKET_ING;
}

/***********************************************************************************
Function:     processStateHeader2
Description:  Proces state header2
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarPacketReceiver::TPacketResult CLidarPacketReceiver::processStateHeader2(CLidarPacket *packet, u8 ch)
{
    TPacketResult resule = PACKET_ING;
    if(ch == LIDAR_PACEKT_HEADER2)
    {
        packet->pushBack(ch);
        m_state = STATE_LENGHT;
        if(packet->m_error_Data_Wrong)
            packet->m_error_Data_Wrong = false;
    }
    else
    {
        if(!packet->m_error_Data_Wrong)
        {
            packet->m_error_Data_Wrong = true;
            packet->Timeout_Data_Wrong.setTime((double)m_params.Data_Wrong_time_out);
            resule = PACKET_FAILED;
            printf("[CLidarPacketReceiver]-----------m_error_Data_Wrong--------------!\n");
        }
        reset();    
        //printf("[CLidarPacketReceiver] Find erro header2 0x%x!\n", ch);
    }
    return resule;
}

/***********************************************************************************
Function:     processStateLength
Description:  Process state length
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarPacketReceiver::TPacketResult CLidarPacketReceiver::processStateLength(CLidarPacket *packet, u8 ch)
{
#if 1
    /* Limit packet length */
    if(ch < 6 || ch > 255)
    {
        reset();
        printf("[CLidarPacketReceiver] Find erro length is 0x%x!\n", (ch));
        return PACKET_ING;
    }
#endif
    packet->pushBack(ch);

    // Add 2bytes for receive CRC16, sub 3 bytes for header(1bytes) and length(2bytes)
    m_packet_length = (int)ch + 2 - 3;

    m_state = STATE_ACQUIRE_DATA;

    return PACKET_ING;
}

/***********************************************************************************
Function:     processStateAcquireData
Description:  Process state acquire data
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarPacketReceiver::TPacketResult CLidarPacketReceiver::processStateAcquireData(CLidarPacket *packet, u8 ch)
{
    m_actual_count++;
    packet->pushBack(ch);
    if(m_actual_count == m_packet_length)
    {
        reset();
		
        if(packet->verifyCheckSum(packet->getPrototypeCode()))
        {
            if(packet->m_error_crc)
            {
                packet->m_error_crc = false;
            }
            return PACKET_SUCCESS;
        }
        else
        {
            if(!packet->m_error_crc)
            {
                packet->m_error_crc = true;
                packet->Timeout_CRC.setTime((double)m_params.CRC_max_time_ms);
            }
            return PACKET_FAILED;
        }
    }
    return PACKET_ING;
}

/***********************************************************************************
Function:     reset
Description:  Reset
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CLidarPacketReceiver::reset()
{
    m_state = STATE_HEADER1;
    m_actual_count = 0;
    m_packet_length = 0;
}
/***********************************************************************************
Function:     SetInfoFlag
Description:  Set Info Flag
Input:        DataState
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CLidarPacketReceiver::SetInfoFlag(DataState state)
{
    m_InfoState = state;
}
/***********************************************************************************
Function:     GetInfoFlag
Description:  Get Info Flag
Input:        None
Output:       DataState
Return:       None
Others:       None
***********************************************************************************/
CLidarPacketReceiver::DataState CLidarPacketReceiver::GetInfoFlag()
{
    return m_InfoState;
}
/***********************************************************************************
Function:     SetSNFlag
Description:  Set SN Flag
Input:        DataState
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CLidarPacketReceiver::SetSNFlag(DataState state)
{
    m_SNstate = state;
}
/***********************************************************************************
Function:     GetSNFlag
Description:  Get SN Flag
Input:        None
Output:       DataState
Return:       None
Others:       None
***********************************************************************************/
CLidarPacketReceiver::DataState CLidarPacketReceiver::GetSNFlag()
{
    return m_SNstate;
}
