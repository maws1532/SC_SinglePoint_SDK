/*********************************************************************************
File name:	  CLidarUnpacket.h
Author:       Kimbo
Version:      V1.7.1
Date:	 	  2017-02-06
Description:  3irobotics Lidar unpacket
Others:       None

History:
	1. Date:
	Author:
	Modification:
***********************************************************************************/

/********************************* File includes **********************************/
#include "CLidarUnpacket.h"
#include <iostream>
#include <chrono>
/******************************* System libs includes *****************************/
#include <string.h>

/********************************** Name space ************************************/
using namespace dtfeverest;
using namespace dtfeverest::dtfhwdrivers;
/***********************************************************************************
Function:     CLidarUnpacket
Description:  The constructor of CLidarUnpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarUnpacket::CLidarUnpacket()
{
    
}

/***********************************************************************************
Function:     CLidarUnpacket
Description:  The destructor of CLidarUnpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarUnpacket::~CLidarUnpacket()
{

}
/***********************************************************************************
Function:     unpacketLidarScan
Description:  Lidar unpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TToothScan CLidarUnpacket::unpacketLidarScan(CLidarPacket &packet)
{
    TToothScan tooth_scan;
    u16 length = packet.getParamLength();
    u8 *buffer = packet.getParamPtr();
    u16 tooth_angle = 0;
    u16 distance_number = 0;
    u16 distance = 0;

    // Get tooth angle, unit is 0.01 degree
    tooth_angle = CLidarPacket::bufToUByte2(buffer);
    tooth_scan.angle = float(tooth_angle) / 100.0;

    // Get distance number
    // tooth_angle + distance1 + distance2 + ...
    //    2bytes       2bytes      2bytes  + ...
    //                      length - 2(tooth_angle_bytes)
    //  distance_number =   -----------------------------
    //                            2(distance_bytes)
    distance_number = (length - 2) / 2;
    tooth_scan.distance.resize(distance_number);
//    printf("[CLidarUnpacket] angle %5.2f, distance_number %d !\n",tooth_scan.angle, distance_number);

    // Get distance, unit is 0.25mm
    for(size_t i = 0 ; i < distance_number; i++)
    {
        distance = CLidarPacket::bufToUByte2(buffer + 2 * i + 2);

        tooth_scan.distance[i] = float(distance) / 4.0f / 1000.0f;
    }

    return tooth_scan;
}

/***********************************************************************************
Function:     unpacketLidarScan2
Description:  Lidar unpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TToothScan CLidarUnpacket::unpacketLidarScan2(CLidarPacket &packet)
{
    TToothScan tooth_scan;
    u16 length = packet.getParamLength();
    u8 *buffer = packet.getParamPtr();
    u16 tooth_angle = 0;
    u16 distance_number = 0;
    u16 distance = 0;
    u8 signal = 0;

    // Get tooth angle, unit is 0.01 degree
    tooth_angle = CLidarPacket::bufToUByte2(buffer);
    tooth_scan.angle = float(tooth_angle) / 100.0;

    // Get distance number
    // tooth_angle + distance1 + distance2 + ...
    //    2bytes       2bytes      2bytes  + ...
    //                      length - 2(tooth_angle_bytes)
    //  distance_number =   -----------------------------
    //                            2(distance_bytes)
    distance_number = (length - 2) / 3;
    tooth_scan.distance.resize(distance_number);
    tooth_scan.signal.resize(distance_number);
//    printf("[CLidarUnpacket] distance_number %d! packe length %d\n", distance_number, length);

    // Get distance, unit is 0.25mm
//    packet.printHex();
    for(size_t i = 0 ; i < distance_number; i++)
    {
        signal = CLidarPacket::bufToUByte((buffer+2) + 3*i);
        distance = CLidarPacket::bufToUByte2((buffer+2) + 3*i + 1);
        tooth_scan.distance[i] = float(distance) / 4.0f / 1000.0f;
        tooth_scan.signal[i] = int(signal);
//        printf("[CLidarUnpacket]sigal %d! distance %5.2f\n", tooth_scan.signal[i], tooth_scan.distance[i]);
    }
    return tooth_scan;
}

/***********************************************************************************
Function:     unpacketNewLidarScanNoSingal
Description:  Lidar unpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TToothScan CLidarUnpacket::unpacketNewLidarScanNoSingal(CLidarPacket &packet)
{
    TToothScan tooth_scan;
    int head_ptr_offset = 0;
    u16 length = packet.getParamLength();
    u8 *buffer = packet.getParamPtr();
    u16 tooth_angle = 0;
    u8  lidar_speed = 0;
    s16 lidar_offset_angle = 0;
    u16 distance_number = 0;
    u16 distance = 0;

    // Get tooth angle, unit is 0.01 degree
    lidar_speed = CLidarPacket::bufToUByte(buffer + head_ptr_offset);                   head_ptr_offset += 1;
    lidar_offset_angle = CLidarPacket::bufToByte2(buffer + head_ptr_offset);            head_ptr_offset += 2;
    tooth_angle = CLidarPacket::bufToUByte2(buffer + head_ptr_offset);                  head_ptr_offset += 2;

    tooth_scan.angle = float(tooth_angle) * 0.01f;
    tooth_scan.offset_valid = false;
    tooth_scan.offset_angle = lidar_offset_angle * 0.01f;
    tooth_scan.lidar_speed = lidar_speed * 0.05f;

    // Get distance number
    // tooth_angle + distance1 + distance2 + ...
    //    2bytes       2bytes      2bytes  + ...
    //                      length - head_data_bytes(5)
    //  distance_number =   -----------------------------
    //                            2(distance_bytes)
    distance_number = (length - head_ptr_offset) / 2;
    tooth_scan.distance.resize(distance_number);
//    printf("[CLidarUnpacket] angle %5.2f, distance_number %d !\n",tooth_scan.angle, distance_number);

    // Get distance, unit is 0.25mm
    for(size_t i = 0 ; i < distance_number; i++)
    {
        distance = CLidarPacket::bufToUByte2(buffer + 2 * i + head_ptr_offset);

        tooth_scan.distance[i] = float(distance) / 4.0f / 1000.0f;
    }

    return tooth_scan;
}
/***********************************************************************************
Function:     getsystemCurrenttime
Description:  Lidar unpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
double CLidarUnpacket::GetSystemTimeInSeconds()
{
    struct timespec tp;
    if(0 == clock_gettime(CLOCK_MONOTONIC, &tp))
    {
        return (double)(tp.tv_sec + tp.tv_nsec/1e9);
    }
    return -1;
}
/***********************************************************************************
Function:     unpacketLidarScan2
Description:  Lidar unpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TToothScan CLidarUnpacket::unpacketNewLidarScanHasSingal(CLidarPacket &packet)
{
    TToothScan tooth_scan;
    int head_ptr_offset = 7;
    u16 length = packet.getParamLength();
    u8 *buffer = packet.getParamPtr();
    u8  lidar_speed = 0;
    s16 lidar_offset_angle = 0;
    u16 distance_number = 0;
    u16 distance = 0;
    u8 signal = 0;
    int tmpnum = 0;
    tooth_scan.ConfidenceDegr = CLidarPacket::bufToUByte(buffer);
    tooth_scan.Noise = CLidarPacket::bufToUByte2(buffer + 1);
    distance_number = (length - head_ptr_offset) / 3;
    // tooth_scan.distance.resize(distance_number);
    // tooth_scan.signal.resize(distance_number);
    // tooth_scan.CurTime.resize(distance_number);
    // for(size_t i = 0 ; i < distance_number; i++)
    // {
        signal = CLidarPacket::bufToUByte((buffer+ head_ptr_offset));
        distance = CLidarPacket::bufToUByte2((buffer+ head_ptr_offset) + 1);
        tooth_scan.Dis = float(distance);
        tooth_scan.sig = int(signal);
        tooth_scan.CTime = GetSystemTimeInSeconds();
        if(tooth_scan.ConfidenceDegr > 30)
        {
            if(tooth_scan.Dis < 40)//40mm
            {
                tooth_scan.Shield_count++;
                // printf("lidar shield\n");            
            }
        }
        else
        {
            tooth_scan.Dis = 0.0;
        }
        
        // printf("[CLidarUnpacket]sigal %d! distance %5.2f\n", tooth_scan.sig, tooth_scan.Dis);
    return tooth_scan;
}

/***********************************************************************************
Function:     unpacketHealthInfo
Description:  Health info unpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TLidarError CLidarUnpacket::unpacketHealthInfo(CLidarPacket &packet)
{
    u8 health_info = *(packet.getParamPtr());
    return TLidarError(health_info);
}

/***********************************************************************************
Function:     unpacketLidarSpeed
Description:  Health info unpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
int CLidarUnpacket::unpacketLidarSpeed(CLidarPacket &packet)
{
    int lidar_speed = *(packet.getParamPtr());
    return lidar_speed;
}
/***********************************************************************************
Function:     unpacketLidarInformation
Description:  Health info unpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
u8 *CLidarUnpacket::unpacketLidarInformation(CLidarPacket &packet)
{
    u8 *pTemp = packet.getParamPtr();

    return pTemp;
}
/***********************************************************************************
Function:     unpacketLidarversion
Description:  Health info unpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
u8 CLidarUnpacket::UnpackerLidarVersion(CLidarPacket &packet)
{
    u8 ver = packet.getPrototypeCode();
    return ver;
}